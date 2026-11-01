#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ferrox::logic {

/// Single-producer / single-consumer byte ring buffer (ISR producer, main consumer).
/// Capacity is N - 1 bytes, matching the one-slot-open convention of the Rust driver.
/// Single-core Cortex-M0 (ARMv6-M) has no LDREX/DMB, so std::atomic would lower to libatomic calls;
/// volatile indices plus compiler fences give the required ordering on this core.
template <std::size_t N>
    requires(N >= 2 && std::has_single_bit(N))
class SpscRing {
public:
    /// Returns false (and drops the byte) when full.
    bool push(const uint8_t byte) {
        const std::size_t head = head_;
        const std::size_t next = (head + 1) & (N - 1);
        if (next == tail_) {
            return false;
        }
        buf_[head] = byte;
        std::atomic_signal_fence(std::memory_order_seq_cst);
        head_ = next;
        return true;
    }

    [[nodiscard]] std::optional<uint8_t> pop() {
        const std::size_t tail = tail_;
        if (head_ == tail) {
            return std::nullopt;
        }
        std::atomic_signal_fence(std::memory_order_seq_cst);
        const uint8_t byte = buf_[tail];
        std::atomic_signal_fence(std::memory_order_seq_cst);
        tail_ = (tail + 1) & (N - 1);
        return byte;
    }

    void clear() {
        head_ = 0;
        tail_ = 0;
    }

private:
    std::array<uint8_t, N> buf_{};
    volatile std::size_t head_{0};
    volatile std::size_t tail_{0};
};

}  // namespace ferrox::logic
