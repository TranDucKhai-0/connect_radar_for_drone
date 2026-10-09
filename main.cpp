#include "can_bus_manager.hpp"
#include "mr72_radar.hpp"
#include "u10_radar.hpp"
#include "csv_logger.hpp"
#include "drone_state.hpp"
#include "thread_safe_queue.hpp"
#include "laser_driver.h"

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
#include <sys/un.h>
#include <cstring>
#include <algorithm>

// ---------- Laser Range Finder (LRF) ----------
extern "C" {
#include "laser_driver.h"
}
#include <fcntl.h>              // open(), O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>            // struct termios, cfsetispeed, tcsetattr
#include <dirent.h>             // opendir(), readdir()
#include <cerrno>               // errno

#define NUMBER_FRAME_OUT 3 // số khung hình liên tiếp điểm ma xuất hiện 

#define CYCLE_TIME_MS 100 // Chu kỳ 100ms (10Hz) để gửi tới FC và GCS và ghi log

// Tích hợp thư viện MAVLink C
#include "../library/c_library_v2/ardupilotmega/mavlink.h"

std::atomic<bool> g_isAppRunning{true};
DroneState g_droneState;
std::atomic<bool> g_isSystemActive{false};
std::atomic<uint8_t> g_fcSystemId{1};

using frameRelative_t = std::vector<obstacleRelative_t>;
using frameAbsolute_t = std::vector<obstacleAbsolute_t>;
using sharedFrameAbsolute_t = std::shared_ptr<frameAbsolute_t>;

ThreadSafeQueue<std::pair<int, frameRelative_t>> g_queueRelative;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueLog;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueGcs;
ThreadSafeQueue<sharedFrameAbsolute_t> g_queueFc;

// ---------- LRF Global ----------
static int g_laser_serial_fd = -1;  // File descriptor của Serial port, chỉ Thread LRF truy cập

void SignalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
    g_isAppRunning = false;
}

long long GetCurrentTimestampMs()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

long long GetCurrentTimestampUsec()
{
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

uint32_t GetTimeBootMs()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void SystemdNotifyWatchdog()
{
    const char* socketPath = std::getenv("NOTIFY_SOCKET");
    if (!socketPath)
        return;

    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0)
        return;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;

    size_t pathLen = strlen(socketPath);
    if (pathLen >= sizeof(addr.sun_path))
    {
        close(fd);
        return;
    }

    if (socketPath[0] == '@')
    {
        addr.sun_path[0] = '\0';
        std::strncpy(addr.sun_path + 1, socketPath + 1, pathLen - 1);
    }
    else
    {
        std::strncpy(addr.sun_path, socketPath, pathLen);
    }

    const char* msg = "WATCHDOG=1";
    sendto(fd, msg, strlen(msg), 0, (struct sockaddr*)&addr, sizeof(addr.sun_family) + pathLen);
    close(fd);
}

int CreateUdpSocket(const std::string &ip, int port, struct sockaddr_in &addr)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return -1;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    return sock;
}

// ---------------------------------------------------------
// THREAD 1: Đọc Radar (Raw -> Relative)
// ---------------------------------------------------------
void ReadDataFromRadarThread(const std::string &canIface)
{
    CanBusManager canBus(canIface);

    // Khởi tạo song song cả 2 dòng radar ở 4 vị trí để hỗ trợ cắm-và-chạy (Plug & Play) dựa trên CAN ID khác biệt
    MR72Radar mr72Radars[4] = {MR72Radar(1), MR72Radar(2), MR72Radar(3), MR72Radar(4)};
    U10Radar u10Radars[4] = {U10Radar(1), U10Radar(2), U10Radar(3), U10Radar(4)};
    // U10Radar u10Radars[4] = {U10Radar(1, 0x0D1), U10Radar(2, 0x0D2), U10Radar(3, 0x0D3), U10Radar(4, 0x0D4)};
    

    for (int i = 0; i < 4; i++)
    {
        mr72Radars[i].Init(0.0f);
        u10Radars[i].Init(0.0f);
    }

    if (!canBus.Connect())
    {
        std::cerr << "ReadDataFromRadarThread: Failed to connect to CAN " << canIface << "\n";
        g_isAppRunning = false;
        return;
    }

    struct can_frame frame;
    while (g_isAppRunning)
    {
        if (canBus.ReadCanFrame(frame))
        {
            // Đẩy dữ liệu vào hàm ParseCanFrame của cả 2 loại radar ở tất cả vị trí
            for (int i = 0; i < 4; i++)
            {
                // Parse dữ liệu từ MR72
                if (mr72Radars[i].ParseCanFrame(frame, 0.0f, 0.0f))
                {
                    g_queueRelative.Push({i + 1, mr72Radars[i].GetObstaclesRelative()});
                }
                // Parse dữ liệu từ U10
                if (u10Radars[i].ParseCanFrame(frame, 0.0f, 0.0f))
                {
                    g_queueRelative.Push({i + 1, u10Radars[i].GetObstaclesRelative()});
                }
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    canBus.Disconnect();
    std::cout << "ReadDataFromRadarThread Exited.\n";
}

struct trackedObject_t
{
    obstacleAbsolute_t data;
    uint32_t lastSeenMs;
    int frozenCount = 0;
};

// ---------------------------------------------------------
// THREAD 2: Xử lý Data (Relative + FC State -> Absolute)
// ---------------------------------------------------------
void DataProcessingThread(const std::string &logDir, bool forceLog)
{
    std::pair<int, frameRelative_t> framePair;
    std::vector<trackedObject_t> trackedObjects;

    constexpr float timeDelayThreadSleepSeconds = CYCLE_TIME_MS / 1000.0f; // Bù trừ trễ thời gian thread ngủ để tăng độ chính xác khi tính toán bù trừ (s)
    uint32_t lastPushTimeMs = GetTimeBootMs();

    // Logger cho dữ liệu thô nhận được từ các radar
    CsvLogger rawRadarLogger(logDir + "/raw_radar_log.csv");
    bool isLogging = false;

    // Nếu chạy chế độ mock/forceLog, bật ghi log ngay từ đầu
    if (forceLog)
    {
        if (rawRadarLogger.Open())
        {
            isLogging = true;
            std::cout << "Force Log mode enabled. Started Raw Radar recording immediately.\n";
        }
    }

    while (g_isAppRunning)
    {
        bool gotData = false;
        uint32_t now = GetTimeBootMs();

        // Gửi watchdog ping đến systemd định kỳ mỗi 1 giây để chứng minh luồng chính vẫn hoạt động
        static uint32_t lastWatchdogPingMs = 0;
        if (now - lastWatchdogPingMs >= 1000)
        {
            SystemdNotifyWatchdog();
            lastWatchdogPingMs = now;
        }

        // Quản lý trạng thái log tự động dựa trên độ cao nếu không ở chế độ forceLog
        if (!forceLog)
        {
            if (g_isSystemActive)
            {
                if (!isLogging)
                {
                    if (rawRadarLogger.Open())
                    {
                        isLogging = true;
                        std::cout << "System active. Started Raw Radar recording.\n";
                    }
                }
            }
            else
            {
                if (isLogging)
                {
                    rawRadarLogger.Close();
                    isLogging = false;
                    std::cout << "System inactive. Stopped Raw Radar recording.\n";
                }
            }
        }

        // Rút sạch tất cả dữ liệu hiện có trong queue g_queueRelative
        while (g_queueRelative.TryPop(framePair))
        {
            gotData = true;
            int radarId = framePair.first;
            frameRelative_t &relativeFrame = framePair.second;

            float vx, vy, vz, alt;
            g_droneState.GetState(vx, vy, vz, alt);

            std::vector<obstacleAbsolute_t> newAbsPoints;
            newAbsPoints.reserve(relativeFrame.size());

            // 1. Convert to absolute
            for (const auto &rel : relativeFrame)
            {
                obstacleAbsolute_t absObs;

                float baseX = 0, baseY = 0;
                float baseVx = 0, baseVy = 0;

                // Xoay toạ độ và vận tốc relative về hệ toạ độ gốc (Front=X+, Right=Y+)
                switch (radarId)
                {
                case 1: // Front
                    baseX = rel.x;
                    baseY = rel.y;
                    baseVx = rel.vx;
                    baseVy = rel.vy;
                    break;
                case 2: // Right
                    baseX = -rel.y;
                    baseY = rel.x;
                    baseVx = -rel.vy;
                    baseVy = rel.vx;
                    break;
                case 3: // Back
                    baseX = -rel.x;
                    baseY = -rel.y;
                    baseVx = -rel.vx;
                    baseVy = -rel.vy;
                    break;
                case 4: // Left
                    baseX = rel.y;
                    baseY = -rel.x;
                    baseVx = rel.vy;
                    baseVy = -rel.vx;
                    break;
                default:
                    baseX = rel.x;
                    baseY = rel.y;
                    baseVx = rel.vx;
                    baseVy = rel.vy;
                    break;
                }

                // Tạo ID duy nhất bằng cách kết hợp radarId và ID vật thể gốc
                // Mỗi radar MR72 tối đa có 64 vật cản (ID 0..63)
                absObs.id = (radarId - 1) * 64 + rel.id;
                absObs.radar_id = radarId;

                // Bù trừ vận tốc (Vận tốc gốc so với mặt đất) - ĐÃ COMMENT (Chỉ xoay về hệ trục chung)
                // absObs.vx = baseVx + vx;
                // absObs.vy = baseVy + vy;
                absObs.vx = baseVx;
                absObs.vy = baseVy;
                absObs.vz = rel.vz;

                // Bù trừ trễ thời gian thread ngủ - ĐÃ COMMENT (Chỉ xoay về hệ trục chung)
                // absObs.x = baseX + baseVx * timeDelayThreadSleepSeconds;
                // absObs.y = baseY + baseVy * timeDelayThreadSleepSeconds;
                absObs.x = baseX;
                absObs.y = baseY;
                absObs.z = rel.z;

                // Chuyển sang Polar
                absObs.angle = atan2f(absObs.y, absObs.x);
                absObs.range = sqrtf(absObs.x * absObs.x + absObs.y * absObs.y);

                // // Chỉ lấy trong FOV và khoảng cách hợp lệ của từng radar tương ứng
                // switch (radarId)
                // {
                // case 1: // Front
                //     if (std::abs(absObs.angle) > 0.3490658 || absObs.range < 2.0f || absObs.range > 40.0f) // +- 20 độ
                //         continue;
                //     break;
                // case 2: // Right
                //     if (absObs.angle < 1.2217304 || absObs.angle > 1.9198621 || absObs.range < RANGE_LIMIT_MIN || absObs.range > RANGE_LIMIT_MAX/2) // 70 -> 110 độ
                //         continue;
                //     break;
                // case 3: // Back
                //     if (std::abs(absObs.angle) < 2.7488935 || absObs.range < 2.0f || absObs.range > 40.0f) // 157,5 -> -157,5 độ
                //         continue;
                //     break;
                // case 4: // Left
                //     if (absObs.angle < -1.9634954 || absObs.angle > -1.1780972 || absObs.range < 2.0f || absObs.range > 20.0f) // -112,5 -> -67,5 độ
                //         continue;
                //     break;
                // default:
                //     break;
                // }

                newAbsPoints.push_back(absObs);
            }

            // Ghi log dữ liệu thô (sau khi xoay hệ trục về drone)
            if (isLogging)
            {
                long long timestampUsec = GetCurrentTimestampUsec();
                rawRadarLogger.LogObstacles(timestampUsec, newAbsPoints, -alt);
            }

            // Cập nhật tọa độ vật cản vào trackedObjects dựa theo ID duy nhất
            for (auto &new_point : newAbsPoints)
            {
                trackedObject_t *pBestMatch = nullptr;

                for (auto &track : trackedObjects)
                {
                    if (track.data.id == new_point.id)
                    {
                        pBestMatch = &track;
                        break;
                    }
                }

                if (pBestMatch)
                {
                    // Tính toán chênh lệch tọa độ và vận tốc giữa khung hình cũ và mới
                    // float dx = pBestMatch->data.x - new_point.x;
                    // float dy = pBestMatch->data.y - new_point.y;
                    // float dz = pBestMatch->data.z - new_point.z;
                    // float dist_pos = std::sqrt(dx * dx + dy * dy + dz * dz);

                    // float dvx = pBestMatch->data.vx - new_point.vx;
                    // float dvy = pBestMatch->data.vy - new_point.vy;
                    // float dvz = pBestMatch->data.vz - new_point.vz;
                    // float dist_vel = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

                    // float drone_speed = std::sqrt(vx * vx + vy * vy + vz * vz);
                    // Kiểm tra điều kiện "đóng băng" dữ liệu
                    // if (dist_pos < 0.0009f && dist_vel < 0.09f && drone_speed > 0.3f)
                    // {
                    //     pBestMatch->frozenCount++;
                    // }
                    // else
                    // {
                    //     pBestMatch->frozenCount = 0; // Reset nếu drone đứng yên hoặc có sự dịch chuyển thực tế
                    // }

                    // Tìm thấy điểm cũ khớp ID -> Cập nhật tọa độ và thời gian cập nhật
                    pBestMatch->data = new_point;
                    pBestMatch->lastSeenMs = now;
                }
                else
                {
                    // ID mới hoàn toàn -> Thêm vào danh sách theo dõi
                    trackedObjects.push_back({new_point, now, 0});
                }
            }
        }

        // Định kỳ 100ms (hoặc khi có dữ liệu mới) thực hiện cập nhật và đẩy đi
        if (now - lastPushTimeMs >= CYCLE_TIME_MS || gotData)
        {
            // Remove old objects (Không xuất hiện trong 300ms)
            trackedObjects.erase(
                std::remove_if(trackedObjects.begin(), trackedObjects.end(),
                               [now](const trackedObject_t &t)
                               { return (now - t.lastSeenMs) > 200; }),
                trackedObjects.end());

            // Phân phối Frame tổng hợp cho Output
            auto pAbsFrame = std::make_shared<frameAbsolute_t>();
            pAbsFrame->reserve(trackedObjects.size());
            for (const auto &track : trackedObjects)
            {
                // Chỉ gửi đi các điểm KHÔNG bị đóng băng quá 3 khung hình liên tiếp - ĐÃ COMMENT
                // if (track.frozenCount < NUMBER_FRAME_OUT - 1)
                // {
                    pAbsFrame->push_back(track.data);
                // }
            }

            g_queueLog.Push(pAbsFrame);
            g_queueGcs.Push(pAbsFrame);
            g_queueFc.Push(pAbsFrame);

            lastPushTimeMs = now;
        }

        if (!gotData)
        {
            // Ngủ ngắn nếu hàng đợi trống để tiết kiệm tài nguyên CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    if (isLogging)
    {
        rawRadarLogger.Close();
    }
    std::cout << "DataProcessingThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 3: Ghi Log
// ---------------------------------------------------------
void WriteLogThread(const std::string &logDir, bool forceLog)
{
    std::string logFilePath = logDir + "/radar_log.csv";
    CsvLogger csvLogger(logFilePath);

    bool isLogging = false;
    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    // Nếu chạy chế độ mock/forceLog, bật ghi log ngay từ đầu
    if (forceLog)
    {
        if (csvLogger.Open())
        {
            isLogging = true;
            std::cout << "Force Log mode enabled. Started Blackbox recording immediately.\n";
        }
    }

    auto next_wake_time = std::chrono::steady_clock::now();

    while (g_isAppRunning)
    {
        // Rút cạn queue để luôn lấy frame mới nhất
        while (g_queueLog.TryPop(pTempFrame))
        {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame)
        {
            float alt = g_droneState.GetAltitude();

            // Nếu không dùng forceLog, ghi log phụ thuộc vào cờ toàn cục g_isSystemActive
            if (!forceLog)
            {
                if (g_isSystemActive)
                {
                    if (!isLogging)
                    {
                        if (csvLogger.Open())
                        {
                            isLogging = true;
                            std::cout << "Drone reached target altitude. Started Blackbox recording.\n";
                        }
                    }
                }
                else
                {
                    if (isLogging)
                    {
                        csvLogger.Close();
                        isLogging = false;
                        std::cout << "Drone descended below threshold. Stopped Blackbox recording.\n";
                    }
                }
            }

            if (isLogging)
            {
                csvLogger.LogObstacles(GetCurrentTimestampUsec(), *pLatestFrame, -alt);
            }
        }

        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time)
            next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
    }

    if (isLogging)
        csvLogger.Close();
    std::cout << "WriteLogThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 4: Gửi GCS (OBSTACLE_DISTANCE_3D)
// ---------------------------------------------------------
void SendDataToGcsThread(const std::string &ip, int port)
{
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0)
        return;

    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    // Force MAVLink 2 for messages > 255
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    auto next_wake_time = std::chrono::steady_clock::now();

    while (g_isAppRunning)
    {
        // Rút cạn queue để đảm bảo luôn lấy được frame mới nhất
        while (g_queueGcs.TryPop(pTempFrame))
        {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame)
        {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;
            uint32_t time_boot_ms = GetTimeBootMs();

            for (const auto &obs : *pLatestFrame)
            {
                uint16_t tracking_id = obs.id;

                mavlink_msg_obstacle_distance_3d_pack(
                    1, 195, &msg,
                    time_boot_ms,
                    MAV_DISTANCE_SENSOR_RADAR,
                    MAV_FRAME_BODY_FRD,
                    tracking_id,
                    obs.x, obs.y, obs.z,
                    2.0f, 40.0f);
                int len = mavlink_msg_to_send_buffer(buffer, &msg);
                sendto(sock, buffer, len, 0, (struct sockaddr *)&addr, sizeof(addr));
            }
        }

        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time)
            next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
    }
    close(sock);
    std::cout << "SendDataToGcsThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 5: Gửi FC (OBSTACLE_DISTANCE)
// ---------------------------------------------------------
void SendDataToFcThread(const std::string &ip, int port, const std::string &logDir, bool forceLog)
{
    struct sockaddr_in addr;
    int sock = CreateUdpSocket(ip, port, addr);
    if (sock < 0)
        return;

    sharedFrameAbsolute_t pLatestFrame;
    sharedFrameAbsolute_t pTempFrame;

    // Ghi log đồng bộ CubeFC
    std::string logFilePath = logDir + "/cubefc_radar_log.csv";
    CsvLogger csvLogger(logFilePath, LoggerType::FC);
    bool isLogging = false;

    // Ép cấu hình MAVLink 2 trên kênh COMM_2
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_2);
    status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    auto next_wake_time = std::chrono::steady_clock::now();

    while (g_isAppRunning)
    {
        // Rút cạn queue để lấy được frame mới nhất
        while (g_queueFc.TryPop(pTempFrame))
        {
            pLatestFrame = pTempFrame;
        }

        if (pLatestFrame)
        {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            mavlink_message_t msg;

            // Khởi tạo mảng 72 phần tử đại diện cho 72 cung (mỗi cung 5 độ).
            // Giá trị mặc định là UINT16_MAX (65535) đại diện cho vùng mù (không phủ sóng cảm biến)
            uint16_t distances[72];
            for (uint8_t i = 0; i < 72; i++)
                distances[i] = 65535;

            bool isSendFCActive = g_isSystemActive; // Lấy cờ trạng thái hệ thống tại thời điểm này

            // Nếu drone đạt độ cao an toàn, bắt đầu phân tích điểm ảnh radar để chèn vào bản tin
            if (isSendFCActive)
            {
                // Chỉ thiết lập 4001 (Không có vật cản) cho các cung nằm trong FOV quét của 4 radar
                // Front FOV: ±22.5 độ quanh 0 độ
                // Right FOV: 90 ± 22.5 độ
                // Back FOV: 180 ± 22.5 độ
                // Left FOV: 270 ± 22.5 độ
                auto setFovClear = [&](int startSector, int endSector) {
                    for (int i = startSector; i <= endSector; ++i) {
                        distances[i % 72] = 4001; 
                    }
                };
                setFovClear(68, 76);  // Front (68..71 và 0..4)
                setFovClear(14, 22);  // Right (70 -> 110 độ)
                setFovClear(32, 40);  // Back (160 -> 200 độ)
                setFovClear(50, 58);  // Left (250 -> 290 độ)

                for (const auto &obs : *pLatestFrame)
                {
                    // Chỉ xử lý các vật thể có độ cao tương đối so với drone trong khoảng [-2m, 2m]
                    if (std::abs(obs.z) > 2.0f)
                    {
                        continue;
                    }

                    float dist_cm = obs.range * 100.0f; // MAVLink yêu cầu đơn vị cm

                    // Đổi góc từ radian sang độ
                    float angle_deg = obs.angle * (180.0f / M_PI);

                    // Chuẩn hoá góc về miền [0, 360) độ
                    // (Vì hàm atan2 có thể trả về góc âm từ -180 đến 0)
                    while (angle_deg < 0.0f)
                        angle_deg += 360.0f;
                    while (angle_deg >= 360.0f)
                        angle_deg -= 360.0f;

                    // Bản tin OBSTACLE_DISTANCE chia không gian 360 độ thành 72 cung (cách nhau 5 độ).
                    // Góc 0 là hướng thẳng mặt drone, tăng dần theo chiều kim đồng hồ.
                    // Công thức sau tính index của mảng dựa trên góc:
                    uint8_t idx = (uint16_t)(angle_deg / 5.0f + 0.5f) % 72;

                    // Nếu cung này chưa có điểm nào, hoặc điểm hiện tại gần hơn điểm trước đó:
                    // Cập nhật khoảng cách vật cản nhỏ nhất vào cung này.
                    if (dist_cm < distances[idx])
                    {
                        distances[idx] = (uint16_t)dist_cm;
                    }
                }
            }

            // Ghi log đồng bộ CubeFC
            if (forceLog)
            {
                if (!isLogging)
                {
                    if (csvLogger.Open())
                    {
                        isLogging = true;
                        std::cout << "Force Log mode enabled. Started cubefc_log recording.\n";
                    }
                }
            }
            else
            {
                if (isSendFCActive)
                {
                    if (!isLogging)
                    {
                        if (csvLogger.Open())
                        {
                            isLogging = true;
                            std::cout << "System active. Started cubefc_log recording.\n";
                        }
                    }
                }
                else
                {
                    if (isLogging)
                    {
                        csvLogger.Close();
                        isLogging = false;
                        std::cout << "System inactive. Stopped cubefc_log recording.\n";
                    }
                }
            }

            if (isLogging)
            {
                float droneAlt = g_droneState.GetAltitude();
                csvLogger.LogFcDistances(GetCurrentTimestampUsec(), distances, droneAlt);
            }

            // Đóng gói bản tin sử dụng MAVLINK_COMM_2, timestamp bằng 0
            mavlink_msg_obstacle_distance_pack_chan(
                g_fcSystemId, 195, MAVLINK_COMM_2, &msg,
                0, // time_usec = 0 để FC tự động đồng bộ thời gian thực nhận
                MAV_DISTANCE_SENSOR_RADAR,
                distances,
                5, // angular_width (5 độ mỗi sector)
                200, 4000,
                5.0f,
                0.0f,
                MAV_FRAME_BODY_FRD);

            int len = mavlink_msg_to_send_buffer(buffer, &msg);
            sendto(sock, buffer, len, 0, (struct sockaddr *)&addr, sizeof(addr));
        }

        // Cho thread ngử để tối ưu năng lượng
        next_wake_time += std::chrono::milliseconds(CYCLE_TIME_MS);
        auto current_time = std::chrono::steady_clock::now();
        if (next_wake_time < current_time)
            next_wake_time = current_time;
        std::this_thread::sleep_until(next_wake_time);
    }
    if (isLogging)
        csvLogger.Close();
    close(sock);
    std::cout << "SendDataToFcThread Exited.\n";
}

// ---------------------------------------------------------
// THREAD 6: Lắng nghe FC (Cập nhật Drone State)
// ---------------------------------------------------------
void FcListenerThread(int listenPort, int altMin, int altDis)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(listenPort);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buffer[2048];
    uint32_t lastDummySentMs = 0;
    uint32_t lastPacketReceivedMs = 0;

    while (g_isAppRunning)
    {
        uint32_t now = GetTimeBootMs();

        // Gửi gói tin dummy định kỳ nếu không nhận được dữ liệu (hoặc khi bắt đầu)
        // Nếu chưa từng nhận gói nào/quá 2 giây mất kết nối, và chưa từng gửi dummy/đã quá 1 giây từ lần gửi dummy trước đó:
        if (lastPacketReceivedMs == 0 || now - lastPacketReceivedMs > 2000)
        {
            if (lastDummySentMs == 0 || now - lastDummySentMs > 1000)
            {
                char dummy = 'X';
                sendto(sock, &dummy, 1, 0, (struct sockaddr *)&addr, sizeof(addr));
                lastDummySentMs = now;
            }
        }

        int n = recv(sock, buffer, sizeof(buffer), 0);
        if (n > 0)
        {
            lastPacketReceivedMs = now;
            mavlink_message_t msg;
            mavlink_status_t status;
            for (int i = 0; i < n; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status))
                {
                    g_fcSystemId = msg.sysid; // Cập nhật System ID thực tế của FC
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
                    {
                        mavlink_global_position_int_t gpi;
                        mavlink_msg_global_position_int_decode(&msg, &gpi);

                        // Altitude (Dương = Bay lên)
                        float altM = (gpi.relative_alt / 1000.0f);
                        if (gpi.hdg != 65535)
                        {
                            float yawRad = (gpi.hdg / 100.0f) * (M_PI / 180.0f);
                            float vx = gpi.vx / 100.0f;
                            float vy = gpi.vy / 100.0f;
                            float vz = gpi.vz / 100.0f;

                            float forward = vx * cosf(yawRad) + vy * sinf(yawRad);
                            float right = -vx * sinf(yawRad) + vy * cosf(yawRad);

                            g_droneState.Update(forward, right, vz, altM);
                        }
                        else
                        {
                            g_droneState.Update(0.0f, 0.0f, 0.0f, altM);
                        }

                        // Cập nhật cờ hoạt động hệ thống dựa trên độ cao (hysteresis)
                        float altCm = altM * 100.0f;
                        if (altCm >= altMin)
                        {
                            if (!g_isSystemActive)
                            {
                                g_isSystemActive = true;
                                std::cout << "System activated (Altitude " << altCm << " cm >= altMin " << altMin << " cm).\n";
                            }
                        }
                        else if (altCm < altDis)
                        {
                            if (g_isSystemActive)
                            {
                                g_isSystemActive = false;
                                std::cout << "System deactivated (Altitude " << altCm << " cm < altDis " << altDis << " cm).\n";
                            }
                        }
                    }
                }
            }
        }
    }
    close(sock);
    std::cout << "FcListenerThread Exited.\n";
}

// ---------------------------------------------------------
// LRF Helper: Đọc nội dung file sysfs, trả về string đã trim
// ---------------------------------------------------------
static std::string ReadSysfsFile(const std::string& path)
{
    std::string result;
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) return result;
    
    char buf[64];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    
    if (n > 0) {
        buf[n] = '\0';
        result = buf;
        // Trim trailing whitespace/newline
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r' || result.back() == ' '))
            result.pop_back();
    }
    return result;
}

// ---------------------------------------------------------
// LRF Helper: Quét /dev/ttyUSB*, trả về danh sách tất cả
//             cổng có VID:PID khớp CP2102 (10C4:EA60)
// ---------------------------------------------------------
static std::vector<std::string> AutoDetectCP2102Ports()
{
    std::vector<std::string> found_ports;
    
    for (int i = 0; i <= 9; i++)
    {
        std::string dev_name = "ttyUSB" + std::to_string(i);
        std::string dev_path = "/dev/" + dev_name;
        
        // Kiểm tra device node tồn tại
        if (access(dev_path.c_str(), F_OK) != 0)
            continue;
        
        // Đọc idVendor và idProduct từ sysfs
        // Path: /sys/class/tty/ttyUSBx/device/../../idVendor
        std::string sysfs_base = "/sys/class/tty/" + dev_name + "/device/../..";
        std::string vid = ReadSysfsFile(sysfs_base + "/idVendor");
        std::string pid = ReadSysfsFile(sysfs_base + "/idProduct");

        // Dự phòng nếu cấp bậc thư mục USB nông hơn
        if (vid.empty()) {
            sysfs_base = "/sys/class/tty/" + dev_name + "/device/..";
            vid = ReadSysfsFile(sysfs_base + "/idVendor");
            pid = ReadSysfsFile(sysfs_base + "/idProduct");
        }
        
        if (vid == "10c4" && pid == "ea60")
        {
            found_ports.push_back(dev_path);
        }
    }
    
    return found_ports;
}

// ---------------------------------------------------------
// LRF Helper: Mở và cấu hình Serial Port (115200, 8N1, Raw)
// Cấu hình y hệt dự án F405: 115200 baud, 8 data bits,
// No parity, 1 stop bit, No flow control
// VMIN=0, VTIME=5 → read() timeout 500ms (khớp F405 timeout)
// ---------------------------------------------------------
static int OpenSerialPort(const std::string& port)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "LRF: Failed to open " << port << ": " << strerror(errno) << "\n";
        return -1;
    }
    
    // Xóa flag O_NONBLOCK sau khi open thành công (chuyển về blocking + timeout)
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "LRF: tcgetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }
    
    // Cấu hình Raw mode
    cfmakeraw(&tty);
    
    // Baud rate 115200 (y hệt F405: huart4.Init.BaudRate = 115200)
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    
    // 8N1, No flow control (y hệt F405: 8B, NO_PARITY, 1 STOP, NO_HWCONTROL)
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CLOCAL | CREAD; // Enable receiver, ignore modem control
    
    // Timeout: VMIN=0, VTIME=5 → read() trả về sau 500ms nếu không có data
    // Khớp với F405: osThreadFlagsWait(..., 500) timeout 500ms
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;  // 5 * 100ms = 500ms
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "LRF: tcsetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }
    
    // Flush bộ đệm input/output
    tcflush(fd, TCIOFLUSH);
    
    return fd;
}

// ---------------------------------------------------------
// LRF Helper: Cấu hình Laser SF20 — COPY 1:1 từ dự án F405
// Tham chiếu: PhaseOne_Controller_F405/Component/Laser_Driver/Src/laser_task.c
// Hàm Setup_SF20() dòng 89-106
// ---------------------------------------------------------
static void SetupLaserSF20(Laser_Handle_t* handle)
{
    // Gửi Handshake — nhận diện cổng (Command 0: Product Name)
    Laser_SF20_Send_Handshake(handle);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Xuất kết hợp Last Return Filtered (Bit 7) và First Return Strength (Bit 4)
    Laser_SF20_Set_DistanceOutput(handle, SF20_OUT_LAST_FILTERED | SF20_OUT_FIRST_STRENGTH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Bật  Filter
    Laser_SF20_Set_Filter_Mode(handle, SF20_CMD_MEDIAN_ENABLE, 8, SF20_FILTER_ENABLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    Laser_SF20_Set_Filter_Mode(handle, SF20_CMD_SMOOTHING_ENABLE, 78, SF20_FILTER_ENABLE);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Kích hoạt Stream khoảng cách cm (Command 30, giá trị 5)
    Laser_SF20_Set_StreamMode(handle, SF20_STREAM_DISTANCE_CM);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

// ---------------------------------------------------------
// LRF Callback: Gửi dữ liệu xuống UART (thay thế HAL_UART_Transmit của F405)
// Được gọi bởi các hàm Laser_SF20_xxx() trong driver C
// ---------------------------------------------------------
static int8_t Laser_Linux_TxCallback(uint8_t* data, uint16_t size)
{
    if (g_laser_serial_fd < 0) return -1;
    ssize_t written = write(g_laser_serial_fd, data, size);
    return (written == (ssize_t)size) ? 0 : -1;
}

// ---------------------------------------------------------
// LRF Helper: Convert SF20 Strength (%) → MAVLink signal_quality (0-100)
// MAVLink spec: 0 = unknown, 1 = worst, 100 = best
// ---------------------------------------------------------
static uint8_t ConvertStrengthToQuality(int16_t strength_raw)
{
    if (strength_raw <= 0) return 0;
    if (strength_raw >= 100) return 100;
    return static_cast<uint8_t>(strength_raw);
}

// ---------------------------------------------------------
// THREAD 7: Xử lý cảm biến LRF (Laser Range Finder)
//           Hoạt động hoàn toàn độc lập với pipeline Radar
//           Giao tiếp qua USB Serial (CP2102) → MAVLink DISTANCE_SENSOR → FC
// ---------------------------------------------------------
void LaserRangeFinderThread(const std::string& fcIp, int fcPort)
{
    std::cout << "LRF: Thread started.\n";

    // ── Khởi tạo Laser Handle (y hệt F405: laser_task.c dòng 43-45) ──
    Laser_Handle_t laser_handle;
    memset(&laser_handle, 0, sizeof(laser_handle));
    laser_handle.config.transmit_cb = Laser_Linux_TxCallback;
    laser_handle.is_initialized = true;

    // ── Tạo UDP Socket để gửi DISTANCE_SENSOR tới FC (cùng ip:port với Radar) ──
    struct sockaddr_in fc_addr;
    int udp_sock = CreateUdpSocket(fcIp, fcPort, fc_addr);
    if (udp_sock < 0)
    {
        std::cerr << "LRF: Failed to create UDP socket.\n";
        return;
    }

    // ── Ép MAVLink 2 trên kênh COMM_3 (kênh riêng cho LRF) ──
    mavlink_status_t* mav_status = mavlink_get_channel_status(MAVLINK_COMM_3);
    mav_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;

    // ── Biến trạng thái ──
    LaserObject_t laser_data;
    uint8_t circular_buf[256]; // Bộ đệm vòng mô phỏng DMA
    int circular_head = 0;
    
    auto last_send_time = std::chrono::steady_clock::now();
    auto last_valid_parse_time = std::chrono::steady_clock::now();
    constexpr int SEND_INTERVAL_MS = 50;  // 20Hz — khuyến cáo ArduPilot cho rangefinder altitude

    // ══════════════════════════════════════════════════════════════
    // VÒNG LẶP NGOÀI: Auto-detect → Connect → Setup → Read loop
    // Nếu mất kết nối USB → quay lại đây để dò lại
    // ══════════════════════════════════════════════════════════════
    while (g_isAppRunning)
    {
        // ── PHASE 1: Auto-detect cổng CP2102 ──
        // Jetson có thể có nhiều CP2102 (debug, GPS, v.v.)
        // Thuật toán: Lấy danh sách tất cả CP2102 → thử từng cổng →
        //             gửi Setup → chờ parse thành công → xác nhận đúng Laser
        g_laser_serial_fd = -1;
        std::string connected_port;

        while (g_isAppRunning && g_laser_serial_fd < 0)
        {
            std::cout << "LRF: Scanning for CP2102 USB devices...\n";
            // std::vector<std::string> cp2102_ports = AutoDetectCP2102Ports();
            std::vector<std::string> cp2102_ports = {"/dev/ttyREAR"};

            if (cp2102_ports.empty())
            {
                std::cerr << "LRF: No CP2102 found. Retrying in 2s...\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            std::cout << "LRF: Found " << cp2102_ports.size() << " CP2102 port(s). Validating...\n";

            // Thử từng cổng CP2102: mở → cấu hình SF20 → chờ parse thành công
            for (const auto& port : cp2102_ports)
            {
                if (!g_isAppRunning) break;

                std::cout << "LRF: Trying " << port << "...\n";
                int fd = OpenSerialPort(port);
                if (fd < 0) continue;

                // Gán fd tạm để TxCallback có thể gửi dữ liệu
                g_laser_serial_fd = fd;

                // Gửi cấu hình SF20
                SetupLaserSF20(&laser_handle);

                // Chờ tối đa 2 giây để nhận và parse thành công ít nhất 1 gói
                bool validated = false;
                auto validate_start = std::chrono::steady_clock::now();
                
                memset(circular_buf, 0, sizeof(circular_buf));
                circular_head = 0;

                while (std::chrono::steady_clock::now() - validate_start < std::chrono::seconds(2))
                {
                    if (!g_isAppRunning) break;

                    uint8_t temp[64];
                    ssize_t n = read(fd, temp, sizeof(temp));
                    if (n > 0)
                    {
                        // Đưa vào circular buffer
                        for (ssize_t i = 0; i < n; i++) {
                            circular_buf[circular_head] = temp[i];
                            circular_head = (circular_head + 1) % sizeof(circular_buf);
                        }

                        // Trải phẳng buffer để parse (byte mới nhất nằm ở cuối)
                        uint8_t linear_buf[256];
                        for (int i = 0; i < 256; i++) {
                            linear_buf[i] = circular_buf[(circular_head + i) % 256];
                        }

                        memset(&laser_data, 0, sizeof(laser_data));
                        if (Laser_SF20_Parse_DMABuffer(&laser_handle, linear_buf, 256, &laser_data))
                        {
                            if (laser_data.is_valid)
                            {
                                validated = true;
                                break;
                            }
                        }
                    }
                }

                if (validated)
                {
                    connected_port = port;
                    std::cout << "LRF: ✓ Laser SF20 confirmed on " << port
                              << " (distance: " << laser_data.distance_cm << " cm)\n";
                    break;  // Thoát vòng for — đã tìm đúng cổng
                }
                else
                {
                    // Cổng này không phải Laser → đóng và thử cổng tiếp theo
                    std::cout << "LRF: ✗ " << port << " is not Laser SF20.\n";
                    close(fd);
                    g_laser_serial_fd = -1;
                }
            }

            // Nếu không cổng nào validate thành công → chờ rồi quét lại
            if (g_laser_serial_fd < 0)
            {
                std::cerr << "LRF: No valid Laser found on any CP2102 port. Retrying in 2s...\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        if (!g_isAppRunning) break;

        // ── PHASE 2: Đã kết nối thành công — Vào vòng đọc dữ liệu chính ──
        last_valid_parse_time = std::chrono::steady_clock::now();
        last_send_time = std::chrono::steady_clock::now();

        memset(circular_buf, 0, sizeof(circular_buf));
        circular_head = 0;

        std::cout << "LRF: Entering main read loop on " << connected_port << "\n";

        while (g_isAppRunning)
        {
            uint8_t temp[64];
            ssize_t n = read(g_laser_serial_fd, temp, sizeof(temp));

            if (n > 0)
            {
                // Đưa vào circular buffer
                for (ssize_t i = 0; i < n; i++) {
                    circular_buf[circular_head] = temp[i];
                    circular_head = (circular_head + 1) % sizeof(circular_buf);
                }

                // Trải phẳng buffer để parse (byte mới nhất nằm ở cuối)
                uint8_t linear_buf[256];
                for (int i = 0; i < 256; i++) {
                    linear_buf[i] = circular_buf[(circular_head + i) % 256];
                }

                // Parse dữ liệu nhận được
                memset(&laser_data, 0, sizeof(laser_data));
                if (Laser_SF20_Parse_DMABuffer(&laser_handle, linear_buf, 256, &laser_data))
                {
                    if (laser_data.is_valid)
                    {
                        last_valid_parse_time = std::chrono::steady_clock::now();

                        // Kiểm tra rate limit 20Hz (50ms) trước khi gửi MAVLink
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_send_time).count();

                        if (elapsed >= SEND_INTERVAL_MS)
                        {
                            last_send_time = now;

                            // Quy đổi mm → cm cho MAVLink DISTANCE_SENSOR
                            uint16_t distance_cm = (uint16_t)laser_data.distance_cm;
                            uint8_t signal_quality = ConvertStrengthToQuality(laser_data.signal_strength_raw);

                            // In log khoảng cách mỗi 500ms (2Hz) ra terminal để tiện theo dõi
                            static auto last_log_time = std::chrono::steady_clock::now();
                            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 50)
                            {
                                std::cout << "LRF: Distance = " << distance_cm << " cm, Quality = " << (int)signal_quality << "%\n";
                                last_log_time = now;
                            }

                            // Đóng gói MAVLink DISTANCE_SENSOR (Message ID #132)
                            // Cấu hình theo khuyến cáo ArduPilot cho downward rangefinder
                            mavlink_message_t msg;
                            float quaternion[4] = {0, 0, 0, 0};  // Không dùng (orientation != CUSTOM)

                            mavlink_msg_distance_sensor_pack_chan(
                                g_fcSystemId,       // sysid: dùng System ID thực tế của FC
                                195,                // compid: 195 = MAV_COMP_ID_OBSTACLE_AVOIDANCE (giống Radar)
                                MAVLINK_COMM_3,     // channel: kênh riêng cho LRF
                                &msg,
                                GetTimeBootMs(),    // time_boot_ms
                                20,                 // min_distance: 20 cm (SF20 min range ~0.2m)
                                10000,              // max_distance: 10000 cm (SF20 max range 100m)
                                distance_cm,        // current_distance: khoảng cách hiện tại (cm)
                                MAV_DISTANCE_SENSOR_LASER,  // type: 0 = LASER
                                1,                  // id: sensor ID = 1 (phân biệt với radar)
                                MAV_SENSOR_ROTATION_PITCH_270,  // orientation: 25 = hướng xuống (altitude)
                                255,                // covariance: UINT8_MAX = không biết
                                0.0f,               // horizontal_fov: 0 = không biết
                                0.0f,               // vertical_fov: 0 = không biết
                                quaternion,         // quaternion: không dùng
                                signal_quality      // signal_quality: convert từ SF20 First Return Strength (0-100)
                            );

                            uint8_t mav_buffer[MAVLINK_MAX_PACKET_LEN];
                            int len = mavlink_msg_to_send_buffer(mav_buffer, &msg);
                            sendto(udp_sock, mav_buffer, len, 0,
                                   (struct sockaddr*)&fc_addr, sizeof(fc_addr));
                        }
                    }
                }
            }
            else if (n == 0)
            {
                // Kiểm tra xem thiết bị vật lý (file device node) còn tồn tại không
                // Nếu CP2102 bị reset đột ngột hoặc rút ra, node /dev/ttyUSBx sẽ bị mất
                if (access(connected_port.c_str(), F_OK) != 0)
                {
                    std::cerr << "LRF: CP2102 disconnected (" << connected_port << " missing). Reconnecting...\n";
                    close(g_laser_serial_fd);
                    g_laser_serial_fd = -1;
                    break;
                }
            }
            else if (n < 0)
            {
                // Lỗi read() → có thể USB bị rút / disconnect
                if (errno == EINTR) continue;  // Signal interrupt → thử lại

                std::cerr << "LRF: Serial read error on " << connected_port
                          << ": " << strerror(errno) << ". Reconnecting...\n";
                close(g_laser_serial_fd);
                g_laser_serial_fd = -1;
                break;  // Thoát vòng while trong → quay lại PHASE 1 auto-detect
            }

            // Kiểm tra timeout: Quá 1000ms không nhận được bản tin hợp lệ
            // Bất kể n = 0 (mất kết nối vật lý laser) hay n > 0 (laser gửi data rác/sai định dạng)
            auto now = std::chrono::steady_clock::now();
            auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_valid_parse_time).count();

            if (silent_ms > 1000)
            {
                std::cout << "LRF: No valid data for " << silent_ms << "ms. Reconfiguring SF20...\n";
                SetupLaserSF20(&laser_handle);
                last_valid_parse_time = std::chrono::steady_clock::now();
                
                // Flush bộ đệm vòng để xóa rác (tránh kẹt parse dữ liệu cũ)
                memset(circular_buf, 0, sizeof(circular_buf));
                circular_head = 0;
            }
        }
    }

    // ── PHASE 3: Cleanup ──
    if (g_laser_serial_fd >= 0)
    {
        close(g_laser_serial_fd);
        g_laser_serial_fd = -1;
    }
    close(udp_sock);
    std::cout << "LaserRangeFinderThread Exited.\n";
}

void PrintUsage()
{
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

int main(int argc, char **argv)
{
    int opt;
    std::string gcsIp = "192.168.0.29";
    int gcsPort = 14550;
    std::string fcIp = "127.0.0.1";
    int fcPort = 14550;
    int localFcListenPort = 14551; // Port nghe telemetry từ FC
    std::string canInterface = "can0";
    std::string logDir = "/usr/local/etc/connect_radar_for_drone/blackbox"; // Thư mục lưu log mặc định
    int altMin = 1001; // cm
    int altDis = 500; // cm
    int parserRate = 50;
    int oldId = -1;
    int newId = -1;
    bool forceLog = false;

    while ((opt = getopt(argc, argv, "G:P:F:Q:L:d:a:b:r:I:O:fh")) != -1)
    {
        switch (opt)
        {
        case 'G':
            gcsIp = optarg;
            break;
        case 'P':
            gcsPort = std::atoi(optarg);
            break;
        case 'F':
            fcIp = optarg;
            break;
        case 'Q':
            fcPort = std::atoi(optarg);
            break;
        case 'L':
            localFcListenPort = std::atoi(optarg);
            break;
        case 'd':
            logDir = optarg;
            break;
        case 'a':
            altMin = std::atoi(optarg);
            break;
        case 'b':
            altDis = std::atoi(optarg);
            break;
        case 'r':
            parserRate = std::atoi(optarg);
            break;
        case 'I':
            newId = std::atoi(optarg);
            break;
        case 'O':
            oldId = std::atoi(optarg);
            break;
        case 'f':
            forceLog = true;
            break;
        case 'h':
            PrintUsage();
            return 0;
        default:
            PrintUsage();
            return -1;
        }
    }

    // Logic: Nếu đang ở chế độ đổi ID (có cờ -I và -O), đổi ID xong thì thoát.
    if (newId != -1 || oldId != -1)
    {
        if (newId < 0 || newId > 7 || oldId < 0 || oldId > 7)
        {
            std::cerr << "Radar ID must be between 0-7\n";
            return -1;
        }
        std::cout << "ID Mode: Changing Radar ID from " << oldId << " to " << newId << "...\n";
        CanBusManager canBus(canInterface);
        if (!canBus.Connect())
        {
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

        if (canBus.WriteCanFrame(frame))
        {
            std::cout << "ID change command sent successfully.\n";
        }
        else
        {
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
              << "Log Directory: " << logDir << "\n"
              << "LRF: Auto-detect mode (CP2102 USB)\n";

    // Khởi tạo 7 Threads
    std::thread t1(ReadDataFromRadarThread, canInterface);
    std::thread t2(DataProcessingThread, logDir, forceLog);
    std::thread t3(WriteLogThread, logDir, forceLog);
    std::thread t4(SendDataToGcsThread, gcsIp, gcsPort);
    std::thread t5(SendDataToFcThread, fcIp, fcPort, logDir, forceLog);
    std::thread t6(FcListenerThread, localFcListenPort, altMin, altDis);
    std::thread t7(LaserRangeFinderThread, fcIp, fcPort);

    // Join chờ ứng dụng kết thúc
    if (t1.joinable())
        t1.join();
    if (t2.joinable())
        t2.join();
    if (t3.joinable())
        t3.join();
    if (t4.joinable())
        t4.join();
    if (t5.joinable())
        t5.join();
    if (t6.joinable())
        t6.join();
    if (t7.joinable())
        t7.join();

    std::cout << "Application successfully terminated.\n";
    return 0;
}
