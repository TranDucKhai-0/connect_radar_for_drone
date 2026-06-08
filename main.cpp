#include "CanBusManager.hpp"
#include "MR72Radar.hpp"
#include "CsvLogger.hpp"
#include "GcsTelemetry.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <string>

std::atomic<bool> g_isAppRunning{true};

void SignalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down...\n";
    g_isAppRunning = false;
}

// Lấy thời gian hiện tại theo ms
long long GetCurrentTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void PrintUsage() {
    std::cout << "Usage: radar_app [options]\n"
              << "Options:\n"
              << "  -G <ip>      GCS IP\n"
              << "  -P <port>    GCS Port\n"
              << "  -a <alt>     Altitude Minimum\n"
              << "  -b <alt>     Altitude Distance\n"
              << "  -r <rate>    Parser rate\n"
              << "  -h           Show this help\n";
}

int main(int argc, char **argv) {
    int opt;
    std::string gcsIp = "127.0.0.1";
    std::string gcsPort = "14550";
    int altMin = 601;
    int altDis = 300;
    int parserRate = 50;

    while ((opt = getopt(argc, argv, "G:P:a:b:r:h")) != -1) {
        switch (opt) {
            case 'G': gcsIp = optarg; break; // Cấu hình IP GCS
            case 'P': gcsPort = optarg; break; // Cấu hình Port GCS
            case 'a': altMin = std::atoi(optarg); break; // Cấu hình Altitude Minimum
            case 'b': altDis = std::atoi(optarg); break; // Cấu hình Altitude Distance
            case 'r': parserRate = std::atoi(optarg); break; // Cấu hình Parser rate
            case 'h': PrintUsage(); return 0; // Hiển thị help
            default: PrintUsage(); return -1; // Lỗi khi nhập tham số không hợp lệ
        }
    }

    std::signal(SIGINT, SignalHandler);

    std::cout << "Starting Radar App...\n"
              << "GCS: " << gcsIp << ":" << gcsPort << "\n"
              << "Alt Min/Dis: " << altMin << "/" << altDis << "\n";

    // Khởi tạo interface kết nối. Sẽ cấu hình thông qua code hoặc factory sau này.
    std::string canInterface = "vcan0"; 
    CanBusManager canBus(canInterface);
    MR72Radar radarMR72(0);
    CsvLogger csvLogger("radar_log.csv");
    GcsTelemetry telemetry;

    radarMR72.Init(0.0f);

    if (!canBus.Connect()) {
        std::cerr << "Failed to connect to CAN bus. Exiting.\n";
        return -1;
    }

    if (!csvLogger.Open()) {
        std::cerr << "Failed to open CSV Logger. Exiting.\n";
        return -1;
    }

    telemetry.Init();

    std::cout << "Radar App Started. Press Ctrl+C to exit.\n";

    struct can_frame frame;

    while (g_isAppRunning) {
        if (canBus.ReadCanFrame(frame)) {
            float droneVForward = 0.0f;
            float droneVRight = 0.0f;

            bool isCycleComplete = radarMR72.ParseCanFrame(frame, droneVForward, droneVRight);
            
            if (isCycleComplete) {
                auto obstacles = radarMR72.GetObstacles();
                csvLogger.LogObstacles(GetCurrentTimestampMs(), obstacles);
                telemetry.SendObstacles(obstacles);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    csvLogger.Close();
    canBus.Disconnect();
    std::cout << "Application successfully terminated.\n";

    return 0;
}
