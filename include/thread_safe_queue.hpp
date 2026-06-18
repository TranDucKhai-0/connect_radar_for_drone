#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <optional>

// ---------------------------------------------------------
// Hàng đợi an toàn đa luồng (Thread-safe Queue)
// Dùng để truyền dữ liệu (vd: các frame vật cản) giữa các luồng một cách an toàn
// Sử dụng Mutex và Condition Variable để đồng bộ
// ---------------------------------------------------------
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue_; // Cấu trúc dữ liệu hàng đợi gốc
    mutable std::mutex mutex_; // Đảm bảo an toàn khi truy cập
    std::condition_variable cond_; // Biến điều kiện dùng để block/wake luồng

public:
    ThreadSafeQueue() = default;
    
    // Khóa việc copy
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // Đẩy một phần tử vào hàng đợi và đánh thức 1 luồng đang đợi
    void Push(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(std::move(value));
        cond_.notify_one();
    }

    // Lấy phần tử ra khỏi Queue, block nếu rỗng
    void WaitAndPop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        value = std::move(queue_.front());
        queue_.pop();
    }

    // Thử lấy phần tử ra khỏi Queue, trả về false nếu rỗng (không block)
    bool TryPop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    // Lấy phần tử với std::optional
    std::optional<T> WaitAndPop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        T res = std::move(queue_.front());
        queue_.pop();
        return res;
    }

    // Kiểm tra hàng đợi có rỗng không (an toàn đa luồng)
    bool Empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    // Lấy số lượng phần tử hiện tại
    size_t Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
};

#endif // THREAD_SAFE_QUEUE_HPP
