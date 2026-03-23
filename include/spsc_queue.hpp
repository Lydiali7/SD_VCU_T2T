#pragma once
#include <atomic>
#include <vector>
#include <cstddef>

/**
 * Single Producer Single Consumer (SPSC) Lock-less Queue.
 * Optimized for inter-core communication in real-time systems.
 */
template<typename T>
class SPSCQueue {
public:
    explicit SPSCQueue(size_t size) : size_(size), buffer_(size) {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    // Called by Producer (Core 3)
    bool push(const T& data) {
        size_t const head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) % size_;
        
        // Check if queue is full
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false; 
        }
        
        buffer_[head] = data;
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    // Called by Consumer (Core 2)
    bool pop(T& data) {
        size_t const tail = tail_.load(std::memory_order_relaxed);
        
        // Check if queue is empty
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; 
        }
        
        data = buffer_[tail];
        tail_.store((tail + 1) % size_, std::memory_order_release);
        return true;
    }

private:
    const size_t size_;
    std::vector<T> buffer_;

    // Use alignas(64) to prevent "False Sharing" between CPU cores
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};