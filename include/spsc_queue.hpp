#pragma once
#include <vector>
#include <atomic>

template <typename T>
class SPSCQueue {
public:
    explicit SPSCQueue(size_t capacity) : capacity_(capacity), buffer_(capacity + 1) {
        head_.store(0);
        tail_.store(0);
    }

    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) % (capacity_ + 1);
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false; // Queue is full
        }
        buffer_[head] = item;
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue is empty
        }
        item = buffer_[tail];
        tail_.store((tail + 1) % (capacity_ + 1), std::memory_order_release);
        return true;
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};