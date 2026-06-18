#ifndef SHARED_LATEST_DATA_HPP
#define SHARED_LATEST_DATA_HPP

#include <mutex>
#include <memory>
#include <chrono>

// ---------------------------------------------------------
// Hộp chứa dữ liệu mới nhất (Latest Data Box / Double Buffer)
// Thread-safe: Dùng để chia sẻ dữ liệu (chỉ giữ lại 1 Frame mới nhất)
// Tránh hiện tượng ứ đọng và trễ (latency) của Queue
// ---------------------------------------------------------
template<typename T>
class SharedLatestData {
private:
    std::shared_ptr<T> data_; // Con trỏ thông minh lưu dữ liệu mới nhất
    uint32_t timestampMs_ = 0; // Thời điểm cập nhật dữ liệu (ms)
    mutable std::mutex mutex_; // Mutex đảm bảo an toàn đa luồng

    // Hàm tiện ích lấy thời gian hiện tại
    uint32_t GetCurrentTimeMs() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    }

public:
    SharedLatestData() = default;
    
    // Khóa copy để tránh copy nhầm object Mutex
    SharedLatestData(const SharedLatestData&) = delete;
    SharedLatestData& operator=(const SharedLatestData&) = delete;

    // Luồng sản xuất (Producer) dùng hàm này để ghi đè dữ liệu mới
    void Set(std::shared_ptr<T> newData) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = std::move(newData);
        timestampMs_ = GetCurrentTimeMs(); // Đóng dấu thời gian cập nhật
    }

    // Luồng tiêu thụ (Consumer) dùng hàm này để lấy dữ liệu
    // Trả về true nếu có dữ liệu, kèm theo con trỏ và timestamp của dữ liệu đó
    bool Get(std::shared_ptr<T>& outData, uint32_t& outTimestampMs) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!data_) {
            return false; // Chưa có dữ liệu nào được set
        }
        outData = data_; // Shared pointer tăng ref count an toàn
        outTimestampMs = timestampMs_;
        return true;
    }
};

#endif // SHARED_LATEST_DATA_HPP
