/**
 * @file ringbuffer.cpp
 * @brief Minimal stub implementations for RingBuffer
 */

#include "unified_mavlink_handler.hpp"

namespace Communication {

template<size_t SIZE>
bool RingBuffer<SIZE>::push(uint8_t byte) {
    size_t next = (head_ + 1) % SIZE;
    if (next == tail_) return false;
    buffer_[head_] = byte;
    head_ = next;
    return true;
}

template<size_t SIZE>
Config::Result<uint8_t> RingBuffer<SIZE>::pop() {
    if (head_ == tail_) {
        return Config::Result<uint8_t>(Config::ErrorCode::OUT_OF_RANGE);
    }
    uint8_t byte = buffer_[tail_];
    tail_ = (tail_ + 1) % SIZE;
    return Config::Result<uint8_t>(byte);
}

template<size_t SIZE>
bool RingBuffer<SIZE>::empty() const {
    return head_ == tail_;
}

// Explicit instantiation for the sizes used
template class RingBuffer<256>;
template class RingBuffer<512>;

} // namespace Communication
