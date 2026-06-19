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

#define CYCLE_TIME_MS 100 // Chu kỳ 100ms (10Hz) để gửi tới FC và GCS và ghi log

// Tích hợp thư viện MAVLink C
#include "../library/c_library_v2/ardupilotmega/mavlink.h"

std::atomic<bool> g_isAppRunning{true};
DroneState g_droneState;

using frameRelative_t = std::vector<obstacleRelative_t>;
using frameAbsolute_t = std::vector<obstacleAbsolute_t>;
using sharedFrameAbsolute_t = std::shared_ptr<frameAbsolute_t>;

ThreadSafeQueue<std::pair<int, frameRelative_t>> g_queueRelative;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueLog;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueGcs;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueFc;

void SignalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
    g_isAppRunning = false;
}

long long GetCurrentTimestampMs() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

long long GetCurrentTimestampUsec() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

uint32_t GetTimeBootMs() {
    auto now = std::chrono::steady_clock::now();
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
                    g_queueRelative.Push({i + 1, radars[i].GetObstaclesRelative()});
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    canBus.Disconnect();
    std::cout << "ReadDataFromRadarThread Exited.\n";
}

struct trackedObject_t {
    obstacleAbsolute_t data;
    long long lastSeenMs;
};

// ---------------------------------------------------------
// THREAD 2: Xử lý Data (Relative + FC State -> Absolute)
// ---------------------------------------------------------
void DataProcessingThread() {
    std::pair<int, frameRelative_t> framePair;
    std::vector<trackedObject_t> trackedObjects;

    constexpr float timeDelayThreadSleepSeconds = CYCLE_TIME_MS / 1000.0f; // Bù trừ trễ thời gian thread ngủ để tăng độ chính xác khi tính toán bù trừ (s)
    long long lastPushTimeMs = GetCurrentTimestampMs();

    while (g_isAppRunning) {
        bool gotData = false;
        long long now = GetCurrentTimestampMs();

        // Rút sạch tất cả dữ liệu hiện có trong queue g_queueRelative
        while (g_queueRelative.TryPop(framePair)) {
            gotData = true;
            int radarId = framePair.first;
            frameRelative_t& relativeFrame = framePair.second;

            float vx, vy, alt;
            g_droneState.GetState(vx, vy, alt);

            std::vector<obstacleAbsolute_t> newAbsPoints;
            newAbsPoints.reserve(relativeFrame.size());

            // 1. Convert to absolute
            for (const auto& rel : relativeFrame) {
                obstacleAbsolute_t absObs;
                
                float baseX = 0, baseY = 0;
                float baseVx = 0, baseVy = 0;

                // Xoay toạ độ và vận tốc relative về hệ toạ độ gốc (Front=X+, Right=Y+)
                switch (radarId) {
                    case 1: // Front
                        baseX = rel.x;       baseY = rel.y;
                        baseVx = rel.vx;    baseVy = rel.vy;
                        break;
                    case 2: // Right
                        baseX = -rel.y;      baseY = rel.x;
                        baseVx = -rel.vy;   baseVy = rel.vx;
                        break;
                    case 3: // Back
                        baseX = -rel.x;      baseY = -rel.y;
                        baseVx = -rel.vx;   baseVy = -rel.vy;
                        break;
                    case 4: // Left
                        baseX = rel.y;       baseY = -rel.x;
                        baseVx = rel.vy;    baseVy = -rel.vx;
                        break;
                    default:
                        baseX = rel.x;       baseY = rel.y;
                        baseVx = rel.vx;    baseVy = rel.vy;
                        break;
                }

                // Tạo ID duy nhất bằng cách kết hợp radarId và ID vật thể gốc
                // Mỗi radar MR72 tối đa có 64 vật cản (ID 0..63)
                absObs.id = (radarId - 1) * 64 + rel.id;

                // Bù trừ vận tốc (Vận tốc gốc so với mặt đất)
                absObs.vx = baseVx + vx;
                absObs.vy = baseVy + vy;
                absObs.vz = rel.vz;

                // Bù trừ trễ thời gian thread ngủ 
                absObs.x = baseX + baseVx * timeDelayThreadSleepSeconds;
                absObs.y = baseY + baseVy * timeDelayThreadSleepSeconds;
                absObs.z = rel.z;

                // Chuyển sang Polar
                absObs.angle = atan2f(absObs.y, absObs.x);
                absObs.range = sqrtf(absObs.x * absObs.x + absObs.y * absObs.y);

                // Chỉ lấy trong FOV
                if (std::abs(absObs.angle) > 0.3926991)
                    continue;
                if (absObs.angle < 1.1780972 || absObs.angle > 1.9634954)
                    continue;
                if (std::abs(absObs.angle) > 2.7488935)
                    continue;
                if (absObs.angle < -1.9634954 || absObs.angle > -1.1780972)
                    continue;

                newAbsPoints.push_back(absObs);
            }

            // Cập nhật tọa độ vật cản vào trackedObjects dựa theo ID duy nhất
            for (auto& new_point : newAbsPoints) {
                trackedObject_t* pBestMatch = nullptr;
                
                for (auto& track : trackedObjects) {
                    if (track.data.id == new_point.id) {
                        pBestMatch = &track;
                        break;
                    }
                }
                
                if (pBestMatch) {
                    // Tìm thấy điểm cũ khớp ID -> Cập nhật tọa độ và thời gian cập nhật
                    pBestMatch->data = new_point;
                    pBestMatch->lastSeenMs = now;
                } else {
                    // ID mới hoàn toàn -> Thêm vào danh sách theo dõi
                    trackedObjects.push_back({new_point, now});
                }
            }
        }

        // Định kỳ 100ms (hoặc khi có dữ liệu mới) thực hiện cập nhật và đẩy đi
        if (now - lastPushTimeMs >= CYCLE_TIME_MS || gotData) {
            // Remove old objects (Không xuất hiện trong 300ms)
            trackedObjects.erase(
                std::remove_if(trackedObjects.begin(), trackedObjects.end(),
                               [now](const trackedObject_t& t) { return (now - t.lastSeenMs) > 300; }),
                trackedObjects.end()
            );

            // Phân phối Frame tổng hợp cho Output
            auto pAbsFrame = std::make_shared<frameAbsolute_t>();
            pAbsFrame->reserve(trackedObjects.size());
            for (const auto& track : trackedObjects) {
                pAbsFrame->push_back(track.data);
            }

            g_queueLog.Push(pAbsFrame);
            g_queueGcs.Push(pAbsFrame);
            g_queueFc.Push(pAbsFrame);

            lastPushTimeMs = now;
        }

        if (!gotData) {
            // Ngủ ngắn nếu hàng đợi trống để tiết kiệm tài nguyên CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::cout << "DataProcessingThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 3: Ghi Log
// ---------------------------------------------------------
void WriteLogThread(const std::string& logDir, int altMin, int altDis, bool forceLog) {
    std::string logFilePath = logDir + "/radar_log.csv";
    CsvLogger csvLogger(logFilePath);
    
    bool isLogging = false;
    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    // Nếu chạy chế độ mock/forceLog, bật ghi log ngay từ đầu
    if (forceLog) {
        if (csvLogger.Open()) {
            isLogging = true;
            std::cout << "Force Log mode enabled. Started Blackbox recording immediately.\n";
        }
    }

    auto next_wake_time = std::chrono::steady_clock::now();

    while (g_isAppRunning) {
        // Rút cạn queue để luôn lấy frame mới nhất
        while (g_queueLog.TryPop(pTempFrame)) {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame) {
            float alt = g_droneState.GetAltitude();
            
            // Nếu không dùng forceLog, ghi log phụ thuộc vào độ cao (Hysteresis)
            if (!forceLog) {
                float altCm = alt * 100.0f;
                if (altCm >= altMin) {
                    if (!isLogging) {
                        if (csvLogger.Open()) {
                            isLogging = true;
                            std::cout << "Drone reached target altitude. Started Blackbox recording.\n";
                        }
                    }
                } else if (altCm < altDis) {
                    if (isLogging) {
                        csvLogger.Close();
                        isLogging = false;
                        std::cout << "Drone descended below threshold. Stopped Blackbox recording.\n";
                    }
                }
            }
            
            if (isLogging) {
                csvLogger.LogObstacles(GetCurrentTimestampUsec(), *pLatestFrame, -alt);
            }
        }
        
        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time) next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
    }
    
    if (isLogging) csvLogger.Close();
    std::cout << "WriteLogThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 4: Gửi GCS (OBSTACLE_DISTANCE_3D)
// ---------------------------------------------------------
void SendDataToGcsThread(const std::string& ip, int port) {
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0) return;

    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    // Force MAVLink 2 for messages > 255
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    auto next_wake_time = std::chrono::steady_clock::now();

    while (g_isAppRunning) {
        // Rút cạn queue để đảm bảo luôn lấy được frame mới nhất
        while (g_queueGcs.TryPop(pTempFrame)) {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame) {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;
            uint32_t time_boot_ms = GetTimeBootMs();

            for (const auto& obs : *pLatestFrame) {
                uint16_t tracking_id = obs.id;
                
                mavlink_msg_obstacle_distance_3d_pack(
                    1, 195, &msg,
                    time_boot_ms,
                    MAV_DISTANCE_SENSOR_RADAR,
                    MAV_FRAME_BODY_FRD,
                    tracking_id,
                    obs.x, obs.y, obs.z,
                    2.0f, 40.0f
                );
                int len = mavlink_msg_to_send_buffer(buffer, &msg);
                sendto(sock, buffer, len, 0, (struct sockaddr*)&addr, sizeof(addr));
            }
        }
        
        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time) next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
    }
    close(sock);
    std::cout << "SendDataToGcsThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 5: Gửi FC (OBSTACLE_DISTANCE)
// ---------------------------------------------------------
void SendDataToFcThread(const std::string& ip, int port, int altMin, int altDis) {
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0) return;

    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    auto next_wake_time = std::chrono::steady_clock::now();

    bool isActive = false;

    while (g_isAppRunning) {
        // Rút cạn queue để lấy được frame mới nhất
        while (g_queueFc.TryPop(pTempFrame)) {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame) {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;
            
            float altCm = g_droneState.GetAltitude() * 100.0f;
            if (altCm >= altMin) isActive = true;
            else if (altCm < altDis) isActive = false;
            
            // Khởi tạo mảng 72 phần tử đại diện cho 72 cung (mỗi cung 5 độ).
            // Gán giá trị mặc định là UINT16_MAX (65535) -> Quy ước của MAVLink: Không có vật cản.
            uint16_t distances[72];
            for (uint8_t i=0; i<72; i++) distances[i] = UINT16_MAX;

            // Nếu drone đạt độ cao an toàn, bắt đầu phân tích điểm ảnh radar để chèn vào bản tin
            if (isActive) {
                for (const auto& obs : *pLatestFrame) {
                    float dist_cm = obs.range * 100.0f; // MAVLink yêu cầu đơn vị cm
                    
                    // Đổi góc từ radian sang độ
                    float angle_deg = obs.angle * (180.0f / M_PI);
                    
                    // Chuẩn hoá góc về miền [0, 360) độ
                    // (Vì hàm atan2 có thể trả về góc âm từ -180 đến 0)
                    while (angle_deg < 0.0f) angle_deg += 360.0f;
                    while (angle_deg >= 360.0f) angle_deg -= 360.0f;

                    // Bản tin OBSTACLE_DISTANCE chia không gian 360 độ thành 72 cung (cách nhau 5 độ).
                    // Góc 0 là hướng thẳng mặt drone, tăng dần theo chiều kim đồng hồ.
                    // Công thức sau tính index của mảng dựa trên góc: 
                    uint8_t idx = (uint16_t)(angle_deg / 5.0f + 0.5f) % 72;
                    
                    // Nếu cung này chưa có điểm nào, hoặc điểm hiện tại gần hơn điểm trước đó:
                    // Cập nhật khoảng cách vật cản nhỏ nhất vào cung này.
                    if (distances[idx] == UINT16_MAX || dist_cm < distances[idx]) {
                        distances[idx] = (uint16_t)dist_cm;
                    }
                }
            }

            mavlink_msg_obstacle_distance_pack(
                1, 195, &msg,
                GetCurrentTimestampUsec(), 
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
        }

        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time) next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
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
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Thay vì bind() như server, ta gửi 1 ký tự rác 'X' tới mavlink-routerd 
    // để nó biết địa chỉ và cổng của thread này mà gửi ngược MAVLink về.
    char dummy = 'X';
    if (sendto(sock, &dummy, 1, 0, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "FC Listener failed to send dummy packet to port " << listenPort << "\n";
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
                        
                        // Altitude (Dương = Bay lên)
                        float altM = (gpi.relative_alt / 1000.0f);
                        if (gpi.hdg != 65535) {
                            float yawRad = (gpi.hdg / 100.0f) * (M_PI / 180.0f);
                            float vx = gpi.vx / 100.0f;
                            float vy = gpi.vy / 100.0f;
                            
                            float forward = vx * cosf(yawRad) + vy * sinf(yawRad);
                            float right = -vx * sinf(yawRad) + vy * cosf(yawRad);
                            
                            g_droneState.Update(forward, right, altM);
                        } else {
                            g_droneState.Update(0.0f, 0.0f, altM);
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
              << "  -I <id>      New Radar ID to set (0-7)\n"
              << "  -O <id>      Old Radar ID to change from (0-7)\n"
              << "  -f           Force logging regardless of altitude\n"
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
    std::string logDir = "/usr/local/etc/connect_radar_for_drone/blackbox"; // Thư mục lưu log mặc định
    int altMin = 1001;
    int altDis = 600;
    int parserRate = 50;
    int oldId = -1;
    int newId = -1;
    bool forceLog = false;

    while ((opt = getopt(argc, argv, "G:P:F:Q:L:d:a:b:r:I:O:fh")) != -1) {
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
            case 'I': newId = std::atoi(optarg); break;
            case 'O': oldId = std::atoi(optarg); break;
            case 'f': forceLog = true; break;
            case 'h': PrintUsage(); return 0;
            default: PrintUsage(); return -1;
        }
    }

    // Logic: Nếu đang ở chế độ đổi ID (có cờ -I và -O), đổi ID xong thì thoát.
    if (newId != -1 || oldId != -1) {
        if (newId < 0 || newId > 7 || oldId < 0 || oldId > 7) {
            std::cerr << "Radar ID must be between 0-7\n";
            return -1;
        }
        std::cout << "ID Mode: Changing Radar ID from " << oldId << " to " << newId << "...\n";
        CanBusManager canBus(canInterface);
        if (!canBus.Connect()) {
            std::cerr << "Failed to connect CAN bus for ID change.\n";
            return -1;
        }

        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id = 0x200 + oldId * 0x10;
        frame.can_dlc = 8;
        frame.data[0] = 0x02 | 0x80;
        frame.data[4] = newId & 0xFF;
        frame.data[5] = 0x80;

        if (canBus.WriteCanFrame(frame)) {
            std::cout << "ID change command sent successfully.\n";
        } else {
            std::cerr << "Failed to send ID change command.\n";
        }

        canBus.Disconnect();
        return 0;
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
    std::thread t3(WriteLogThread, logDir, altMin, altDis, forceLog);
    std::thread t4(SendDataToGcsThread, gcsIp, gcsPort);
    std::thread t5(SendDataToFcThread, fcIp, fcPort, altMin, altDis);
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
