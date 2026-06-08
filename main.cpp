#include "can_bus_manager.hpp"
#include "mr72_radar.hpp"
#include "csv_logger.hpp"
#include "drone_state.hpp"
#include "thread_safe_queue.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <string>
#include <cmath>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <algorithm>

// Tích hợp thư viện MAVLink C
#include "../library/c_library_v2/ardupilotmega/mavlink.h"

std::atomic<bool> g_isAppRunning{true};
DroneState g_droneState;

using FrameRelative = std::vector<obstacle_relative_t>;
using FrameAbsolute = std::vector<obstacle_absolute_t>;
using SharedFrameAbsolute = std::shared_ptr<FrameAbsolute>;

ThreadSafeQueue<std::pair<int, FrameRelative>> queue_relative;
ThreadSafeQueue<SharedFrameAbsolute> queue_log;
ThreadSafeQueue<SharedFrameAbsolute> queue_gcs;
ThreadSafeQueue<SharedFrameAbsolute> queue_fc;

void SignalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
    g_isAppRunning = false;
}

long long GetCurrentTimestampMs() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

int CreateUdpSocket(const std::string& ip, int port, struct sockaddr_in& addr) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    return sock;
}

// ---------------------------------------------------------
// THREAD 1: Đọc Radar (Raw -> Relative)
// ---------------------------------------------------------
void ReadDataFromRadarThread(const std::string& canIface) {
    CanBusManager canBus(canIface);
    
    // Khởi tạo 4 radar với các ID 1, 2, 3, 4
    MR72Radar radars[4] = { MR72Radar(1), MR72Radar(2), MR72Radar(3), MR72Radar(4) };
    for (int i = 0; i < 4; i++) radars[i].Init(0.0f);
    
    if (!canBus.Connect()) {
        std::cerr << "ReadDataFromRadarThread: Failed to connect to CAN " << canIface << "\n";
        g_isAppRunning = false;
        return;
    }

    struct can_frame frame;
    while (g_isAppRunning) {
        if (canBus.ReadCanFrame(frame)) {
            // Đẩy dữ liệu vào Parse của cả 4 radar
            for (int i = 0; i < 4; i++) {
                if (radars[i].ParseCanFrame(frame, 0.0f, 0.0f)) {
                    queue_relative.Push({i + 1, radars[i].GetObstaclesRelative()});
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    canBus.Disconnect();
    std::cout << "ReadDataFromRadarThread Exited.\n";
}

struct TrackedObject {
    obstacle_absolute_t data;
    long long last_seen_ms;
};

// ---------------------------------------------------------
// THREAD 2: Xử lý Data (Relative + FC State -> Absolute)
// ---------------------------------------------------------
void DataProcessingThread() {
    std::pair<int, FrameRelative> framePair;
    std::vector<TrackedObject> global_tracked_objects;

    while (g_isAppRunning) {
        if (queue_relative.TryPop(framePair)) {
            long long now = GetCurrentTimestampMs();
            int radar_id = framePair.first;
            FrameRelative& relativeFrame = framePair.second;

            float vx, vy, alt;
            g_droneState.GetState(vx, vy, alt);

            std::vector<obstacle_absolute_t> new_abs_points;
            new_abs_points.reserve(relativeFrame.size());

            // 1. Convert to absolute
            for (const auto& rel : relativeFrame) {
                obstacle_absolute_t abs_obs;
                
                float base_x = 0, base_y = 0;
                float base_vx = 0, base_vy = 0;

                // Xoay toạ độ và vận tốc relative về hệ toạ độ gốc (Front=X+, Right=Y+)
                switch (radar_id) {
                    case 1: // Front
                        base_x = rel.x;       base_y = rel.y;
                        base_vx = rel.v_x;    base_vy = rel.v_y;
                        break;
                    case 2: // Right
                        base_x = -rel.y;      base_y = rel.x;
                        base_vx = -rel.v_y;   base_vy = rel.v_x;
                        break;
                    case 3: // Back
                        base_x = -rel.x;      base_y = -rel.y;
                        base_vx = -rel.v_x;   base_vy = -rel.v_y;
                        break;
                    case 4: // Left
                        base_x = rel.y;       base_y = -rel.x;
                        base_vx = rel.v_y;    base_vy = -rel.v_x;
                        break;
                    default:
                        base_x = rel.x;       base_y = rel.y;
                        base_vx = rel.v_x;    base_vy = rel.v_y;
                        break;
                }

                // Bù trừ vận tốc (Vận tốc gốc so với mặt đất)
                abs_obs.v_x = base_vx + vx;
                abs_obs.v_y = base_vy + vy;
                abs_obs.v_z = rel.v_z;

                // Bù trừ trễ 100ms
                abs_obs.x = base_x + abs_obs.v_x * 0.1f;
                abs_obs.y = base_y + abs_obs.v_y * 0.1f;
                abs_obs.z = rel.z;

                // Chuyển sang Polar
                abs_obs.angle = std::atan2(abs_obs.y, abs_obs.x);
                abs_obs.range = std::sqrt(abs_obs.x * abs_obs.x + abs_obs.y * abs_obs.y);

                new_abs_points.push_back(abs_obs);
            }

            // 2. Data Association (Merge objects < 0.2m)
            for (auto& new_point : new_abs_points) {
                float min_dist = 0.2f; // Ngưỡng 0.2m
                TrackedObject* best_match = nullptr;
                
                for (auto& track : global_tracked_objects) {
                    float dx = track.data.x - new_point.x;
                    float dy = track.data.y - new_point.y;
                    float dz = track.data.z - new_point.z;
                    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    
                    if (dist < min_dist) {
                        min_dist = dist;
                        best_match = &track;
                    }
                }
                
                if (best_match) {
                    // Update existing object: Gán lại ID của object đã tồn tại cho object mới (gộp ID)
                    new_point.id = best_match->data.id;
                    best_match->data = new_point;
                    best_match->last_seen_ms = now;
                } else {
                    // Create new tracked object: Giữ nguyên ID nguyên bản từ radar
                    global_tracked_objects.push_back({new_point, now});
                }
            }

            // 3. Remove old objects (Không xuất hiện trong 500ms)
            global_tracked_objects.erase(
                std::remove_if(global_tracked_objects.begin(), global_tracked_objects.end(),
                               [now](const TrackedObject& t) { return (now - t.last_seen_ms) > 500; }),
                global_tracked_objects.end()
            );

            // 4. Phân phối Frame tổng hợp cho Output
            auto absFrame = std::make_shared<FrameAbsolute>();
            absFrame->reserve(global_tracked_objects.size());
            for (const auto& track : global_tracked_objects) {
                absFrame->push_back(track.data);
            }

            queue_log.Push(absFrame);
            queue_gcs.Push(absFrame);
            queue_fc.Push(absFrame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    std::cout << "DataProcessingThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 3: Ghi Log
// ---------------------------------------------------------
void WriteLogThread(const std::string& logDir) {
    std::string logFilePath = logDir + "/radar_log.csv";
    CsvLogger csvLogger(logFilePath);
    if (!csvLogger.Open()) return;

    SharedFrameAbsolute frame;
    while (g_isAppRunning) {
        if (queue_log.TryPop(frame)) {
            float alt = g_droneState.GetAltitude();
            csvLogger.LogObstacles(GetCurrentTimestampMs(), *frame, alt);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    csvLogger.Close();
    std::cout << "WriteLogThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 4: Gửi GCS (OBSTACLE_DISTANCE_3D)
// ---------------------------------------------------------
void SendDataToGcsThread(const std::string& ip, int port) {
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0) return;

    SharedFrameAbsolute frame;
    long long last_send_ms = 0;

    // Force MAVLink 2 for messages > 255
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    while (g_isAppRunning) {
        if (queue_gcs.TryPop(frame)) {
            long long now = GetCurrentTimestampMs();
            if (now - last_send_ms < 100) continue; // 10Hz limit
            last_send_ms = now;

            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;

            for (const auto& obs : *frame) {
                if (obs.range < 2.0f || obs.range > 40.0f) continue;
                
                uint16_t tracking_id = obs.id;
                
                mavlink_msg_obstacle_distance_3d_pack(
                    1, 195, &msg,
                    now,
                    MAV_DISTANCE_SENSOR_RADAR,
                    MAV_FRAME_BODY_FRD,
                    tracking_id,
                    obs.x, obs.y, obs.z,
                    2.0f, 40.0f
                );
                int len = mavlink_msg_to_send_buffer(buffer, &msg);
                sendto(sock, buffer, len, 0, (struct sockaddr*)&addr, sizeof(addr));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    close(sock);
    std::cout << "SendDataToGcsThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 5: Gửi FC (OBSTACLE_DISTANCE)
// ---------------------------------------------------------
void SendDataToFcThread(const std::string& ip, int port) {
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0) return;

    SharedFrameAbsolute frame;
    long long last_send_ms = 0;

    while (g_isAppRunning) {
        if (queue_fc.TryPop(frame)) {
            long long now = GetCurrentTimestampMs();
            if (now - last_send_ms < 100) continue; // 10Hz limit
            last_send_ms = now;

            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;
            
            uint16_t distances[72];
            for (int i=0; i<72; i++) distances[i] = 4001; // Safe distance

            for (const auto& obs : *frame) {
                float dist_cm = obs.range * 100.0f;
                if (dist_cm < 200.0f || dist_cm > 4000.0f) continue;

                float angle_deg = obs.angle * (180.0f / M_PI);
                while (angle_deg < 0.0f) angle_deg += 360.0f;
                while (angle_deg >= 360.0f) angle_deg -= 360.0f;

                int idx = (int)(angle_deg / 5.0f + 0.5f) % 72;
                if (distances[idx] == 4001 || dist_cm < distances[idx]) {
                    distances[idx] = (uint16_t)dist_cm;
                }
            }

            mavlink_msg_obstacle_distance_pack(
                1, 195, &msg,
                now, 
                MAV_DISTANCE_SENSOR_RADAR, 
                distances,
                5, // angular_width (5 độ mỗi sector)
                200, 4000,
                5.0f,
                0.0f,
                MAV_FRAME_BODY_FRD
            );
            
            int len = mavlink_msg_to_send_buffer(buffer, &msg);
            sendto(sock, buffer, len, 0, (struct sockaddr*)&addr, sizeof(addr));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    close(sock);
    std::cout << "SendDataToFcThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 6: Lắng nghe FC (Cập nhật Drone State)
// ---------------------------------------------------------
void FcListenerThread(int listenPort) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listenPort);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "FC Listener failed to bind port " << listenPort << "\n";
        close(sock);
        return;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buffer[2048];
    while (g_isAppRunning) {
        int n = recv(sock, buffer, sizeof(buffer), 0);
        if (n > 0) {
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                        mavlink_global_position_int_t gpi;
                        mavlink_msg_global_position_int_decode(&msg, &gpi);
                        
                        // FRD conversion
                        float alt_m = (gpi.relative_alt / 1000.0f) * -1.0f;
                        if (gpi.hdg != 65535) {
                            float yaw_rad = (gpi.hdg / 100.0f) * (M_PI / 180.0f);
                            float vx = gpi.vx / 100.0f;
                            float vy = gpi.vy / 100.0f;
                            
                            float forward = vx * std::cos(yaw_rad) + vy * std::sin(yaw_rad);
                            float right = -vx * std::sin(yaw_rad) + vy * std::cos(yaw_rad);
                            
                            g_droneState.Update(forward, right, alt_m);
                        } else {
                            g_droneState.Update(0.0f, 0.0f, alt_m);
                        }
                    }
                }
            }
        }
    }
    close(sock);
    std::cout << "FcListenerThread Exited.\n";
}

void PrintUsage() {
    std::cout << "Usage: radar_app [options]\n"
              << "Options:\n"
              << "  -G <ip>      GCS IP\n"
              << "  -P <port>    GCS Port\n"
              << "  -F <ip>      FC IP (destination)\n"
              << "  -Q <port>    FC Port (destination)\n"
              << "  -L <port>    Local Port to listen FC MAVLink\n"
              << "  -d <dir>     Log directory (Blackbox)\n"
              << "  -a <alt>     Altitude Minimum\n"
              << "  -b <alt>     Altitude Distance\n"
              << "  -r <rate>    Parser rate\n"
              << "  -h           Show this help\n";
}

int main(int argc, char **argv) {
    int opt;
    std::string gcsIp = "192.168.0.29";
    int gcsPort = 14550;
    std::string fcIp = "127.0.0.1";
    int fcPort = 14550;
    int localFcListenPort = 14551; // Port nghe telemetry từ FC
    std::string canInterface = "can0"; 
    std::string logDir = "."; // Mặc định lưu log ở thư mục hiện tại
    int altMin = 601;
    int altDis = 300;
    int parserRate = 50;

    while ((opt = getopt(argc, argv, "G:P:F:Q:L:d:a:b:r:h")) != -1) {
        switch (opt) {
            case 'G': gcsIp = optarg; break;
            case 'P': gcsPort = std::atoi(optarg); break;
            case 'F': fcIp = optarg; break;
            case 'Q': fcPort = std::atoi(optarg); break;
            case 'L': localFcListenPort = std::atoi(optarg); break;
            case 'd': logDir = optarg; break;
            case 'a': altMin = std::atoi(optarg); break;
            case 'b': altDis = std::atoi(optarg); break;
            case 'r': parserRate = std::atoi(optarg); break;
            case 'h': PrintUsage(); return 0;
            default: PrintUsage(); return -1;
        }
    }

    std::signal(SIGINT, SignalHandler);

    std::cout << "Starting Radar App Pipeline...\n"
              << "GCS: " << gcsIp << ":" << gcsPort << "\n"
              << "FC Dest: " << fcIp << ":" << fcPort << "\n"
              << "FC Listen Port: " << localFcListenPort << "\n"
              << "Log Directory: " << logDir << "\n";

    // Khởi tạo 6 Threads
    std::thread t1(ReadDataFromRadarThread, canInterface);
    std::thread t2(DataProcessingThread);
    std::thread t3(WriteLogThread, logDir);
    std::thread t4(SendDataToGcsThread, gcsIp, gcsPort);
    std::thread t5(SendDataToFcThread, fcIp, fcPort);
    std::thread t6(FcListenerThread, localFcListenPort);

    // Join chờ ứng dụng kết thúc
    if (t1.joinable()) t1.join();
    if (t2.joinable()) t2.join();
    if (t3.joinable()) t3.join();
    if (t4.joinable()) t4.join();
    if (t5.joinable()) t5.join();
    if (t6.joinable()) t6.join();

    std::cout << "Application successfully terminated.\n";
    return 0;
}
