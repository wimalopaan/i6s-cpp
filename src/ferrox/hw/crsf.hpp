#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ferrox/hw/uart.hpp"
#include "ferrox/logic/crsf.hpp"

namespace ferrox::hw::crsf {

// Main-loop state only (no ISR access), hence no volatile.
inline constinit bool g_enabled = false;
inline constinit uint8_t g_baud_idx = 0;
inline constinit bool g_half_duplex = false;
inline constinit uint32_t g_last_tx_ms = 0;
inline constinit logic::crsf::Framer g_framer{};
inline constinit logic::crsf::Telemetry g_telemetry{};

inline constexpr uint32_t kTxPeriodMs = 10;
inline constexpr uint32_t kTelemetryTimeoutMs = 1000;
inline constexpr std::size_t kMaxRxBytesPerPoll = 128;

/// Enables or disables CRSF on USART2 and the module power switch.
inline void set_enabled(bool enabled, uint8_t baud_idx, bool half_duplex) {
    if (enabled == g_enabled &&
        (!enabled || (baud_idx == g_baud_idx && half_duplex == g_half_duplex))) {
        return;
    }
    g_enabled = enabled;
    g_baud_idx = baud_idx;
    g_half_duplex = half_duplex;

    uart::set_uart_enabled(enabled, baud_idx, half_duplex);
    uart::set_module_power(enabled);

    if (!enabled) {
        g_telemetry = logic::crsf::Telemetry{};
        g_framer.reset();
        g_framer.last_byte_ms = 0;
    }
}

inline bool is_enabled() {
    return g_enabled;
}

/// Sends an RC channels frame every 10 ms (100 Hz).
template <std::size_t N>
inline void update_channels(uint32_t now_ms, const std::array<uint16_t, N>& channels,
                            bool arm_mode_switch, bool armed) {
    if (!g_enabled || static_cast<uint32_t>(now_ms - g_last_tx_ms) < kTxPeriodMs) {
        return;
    }
    g_last_tx_ms = now_ms;

    std::array<uint8_t, logic::crsf::kRcExtFrameSize> frame{};
    const std::size_t len = logic::crsf::build_rc_frame(channels, arm_mode_switch, armed, frame);
    uart::write_bytes(std::span<const uint8_t>(frame.data(), len));
}

/// Drains USART2 RX and updates telemetry. Call regularly from the main loop.
inline void poll_telemetry(uint32_t now_ms) {
    if (!g_enabled) {
        return;
    }

    g_framer.check_stall(now_ms);

    std::size_t count = 0;
    while (const auto byte = uart::read_byte()) {
        const auto frame = g_framer.push(*byte, now_ms);
        if (!frame.empty()) {
            logic::crsf::parse_telemetry(frame[2], frame.subspan(3, frame.size() - 4), g_telemetry,
                                         now_ms);
        }
        if (++count >= kMaxRxBytesPerPoll) {
            break;
        }
    }

    if (g_telemetry.connected &&
        static_cast<uint32_t>(now_ms - g_telemetry.last_telemetry_ms) > kTelemetryTimeoutMs) {
        g_telemetry.connected = false;
    }
}

inline const logic::crsf::Telemetry& telemetry() {
    return g_telemetry;
}

}  // namespace ferrox::hw::crsf
