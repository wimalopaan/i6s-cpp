#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ferrox/hw/uart.hpp"
#include "ferrox/logic/crsf.hpp"
#include "ferrox/logic/crsf_menu.hpp"

namespace ferrox::hw::crsf {

// Main-loop state only (no ISR access), hence no volatile.
inline constinit bool g_enabled = false;
inline constinit uint8_t g_baud_idx = 0;
inline constinit bool g_half_duplex = false;
inline constinit uint32_t g_last_tx_ms = 0;
inline constinit uint32_t g_last_ping_ms = 0;
inline constinit logic::crsf::Framer g_framer{};
inline constinit logic::crsf::Telemetry g_telemetry{};
inline constinit logic::crsf::menu::Loader g_menu{};
inline constinit logic::crsf::menu::Command g_command{};

inline constexpr uint32_t kTxPeriodMs = 10;
inline constexpr uint32_t kTelemetryTimeoutMs = 1000;
inline constexpr uint32_t kPingPeriodMs = 1000;
inline constexpr std::size_t kMaxRxBytesPerPoll = 128;

/// Enables or disables CRSF on USART2 and the module power switch.
inline void set_enabled(bool enabled, uint8_t baud_idx, uart::Duplex duplex) {
    const bool half_duplex = duplex == uart::Duplex::Half;
    if (enabled == g_enabled &&
        (!enabled || (baud_idx == g_baud_idx && half_duplex == g_half_duplex))) {
        return;
    }
    g_enabled = enabled;
    g_baud_idx = baud_idx;
    g_half_duplex = half_duplex;

    uart::set_uart_enabled(enabled, baud_idx, duplex);
    uart::set_module_power(enabled);

    if (!enabled) {
        g_telemetry = logic::crsf::Telemetry{};
        g_framer.reset();
        g_framer.last_byte_ms = 0;
        g_menu.stop();
        g_command = logic::crsf::menu::Command{};
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

namespace detail {

using MenuAction = logic::crsf::menu::Action;

/// Sends the frame(s) requested by the menu engine.
inline void perform(const MenuAction& a) {
    if (a.kind == MenuAction::Kind::Read) {
        std::array<uint8_t, 8> frame{};
        const std::size_t n = logic::crsf::build_param_ext(
            g_menu.device, logic::crsf::kTypeParamRead, a.param, a.arg, frame);
        uart::write_bytes(std::span<const uint8_t>(frame.data(), n));
    } else if (a.kind == MenuAction::Kind::Write) {
        const std::array<uint8_t, 1> payload{a.arg};
        std::array<uint8_t, 16> frame{};
        const std::size_t n =
            logic::crsf::build_param_write_multi(g_menu.device, a.param, payload, frame);
        uart::write_bytes(std::span<const uint8_t>(frame.data(), n));
    }
}

/// Device info (0x29): payload is [dest, origin, name\0, serial(4), hw(4), fw(4), param_count, ...].
inline void on_device_info(std::span<const uint8_t> p, uint32_t now_ms) {
    if (p.size() < 3 || g_menu.state != logic::crsf::menu::Loader::State::Idle) {
        return;
    }
    const std::size_t count_idx = logic::crsf::menu::find_nul(p, 2) + 1 + 12;
    if (count_idx >= p.size()) {
        return;
    }
    perform(g_menu.start(p[1], p[count_idx], now_ms));
}

/// Parameter entry (0x2B): payload is [dest, origin, param, chunks_remain, chunk...].
inline void on_param_entry(std::span<const uint8_t> p, uint32_t now_ms) {
    if (p.size() < 4) {
        return;
    }
    perform(g_menu.on_entry(p[1], p[2], p[3], p.subspan(4), now_ms));

    if (!g_menu.completed) {
        return;
    }
    const uint8_t id = *g_menu.completed;
    if (id < logic::crsf::menu::kMaxParams && g_menu.entries[id] &&
        g_menu.entries[id]->type == logic::crsf::menu::Type::Command) {
        const auto& e = *g_menu.entries[id];
        g_command.on_status(id, static_cast<uint8_t>(e.value), e.cmd_timeout, now_ms);
    }
}

}  // namespace detail

/// Drains USART2 RX, updates telemetry and the parameter menu. Call regularly from the main loop.
inline void poll_telemetry(uint32_t now_ms) {
    if (!g_enabled) {
        return;
    }

    g_framer.check_stall(now_ms);

    std::size_t count = 0;
    while (const auto byte = uart::read_byte()) {
        const auto frame = g_framer.push(*byte, now_ms);
        if (!frame.empty()) {
            const uint8_t type = frame[2];
            const auto payload = frame.subspan(3, frame.size() - 4);
            switch (type) {
                case logic::crsf::kTypeDeviceInfo:
                    detail::on_device_info(payload, now_ms);
                    break;
                case logic::crsf::kTypeParamEntry:
                    detail::on_param_entry(payload, now_ms);
                    break;
                default:
                    logic::crsf::parse_telemetry(type, payload, g_telemetry, now_ms);
                    break;
            }
        }
        if (++count >= kMaxRxBytesPerPoll) {
            break;
        }
    }

    if (g_telemetry.connected &&
        static_cast<uint32_t>(now_ms - g_telemetry.last_telemetry_ms) > kTelemetryTimeoutMs) {
        g_telemetry.connected = false;
    }

    // Parameter discovery: ping while no device is loaded.
    if (g_menu.state == logic::crsf::menu::Loader::State::Idle &&
        static_cast<uint32_t>(now_ms - g_last_ping_ms) >= kPingPeriodMs) {
        g_last_ping_ms = now_ms;
        std::array<uint8_t, 6> ping{};
        const std::size_t n = logic::crsf::build_ping(ping);
        uart::write_bytes(std::span<const uint8_t>(ping.data(), n));
    }

    detail::perform(g_menu.tick(now_ms));
    detail::perform(g_command.tick(now_ms));
}

/// Parameter menu API (no UI). Entries are filled after the device has been loaded.
inline const std::array<std::optional<logic::crsf::menu::Entry>,
                        logic::crsf::menu::kMaxParams>& menu_entries() {
    return g_menu.entries;
}

inline bool menu_ready() {
    return g_menu.state == logic::crsf::menu::Loader::State::Ready;
}

/// Writes a value (clamped to the parameter range) to the loaded device.
inline bool menu_set_value(uint8_t param, int32_t value) {
    if (!menu_ready() || param >= logic::crsf::menu::kMaxParams || !g_menu.entries[param]) {
        return false;
    }
    const auto& e = *g_menu.entries[param];
    std::array<uint8_t, 2> bytes{};
    const std::size_t n = logic::crsf::menu::encode_value(e, value, bytes);
    if (n == 0) {
        return false;
    }
    std::array<uint8_t, 16> frame{};
    const std::size_t len = logic::crsf::build_param_write_multi(
        g_menu.device, param, std::span<const uint8_t>(bytes.data(), n), frame);
    uart::write_bytes(std::span<const uint8_t>(frame.data(), len));
    return true;
}

/// Cycles a select parameter to its next option.
inline bool menu_cycle_value(uint8_t param) {
    if (!menu_ready() || param >= logic::crsf::menu::kMaxParams || !g_menu.entries[param]) {
        return false;
    }
    return menu_set_value(param, logic::crsf::menu::cycled_value(*g_menu.entries[param]));
}

inline void menu_enter_folder(uint8_t folder, uint32_t now_ms) {
    detail::perform(g_menu.enter_folder(folder, now_ms));
}

inline void menu_start_command(uint8_t param, uint32_t now_ms) {
    if (menu_ready()) {
        detail::perform(g_command.start(param, now_ms));
    }
}

inline void menu_confirm_command(uint32_t now_ms) {
    detail::perform(g_command.confirm(now_ms));
}

inline void menu_cancel_command() {
    detail::perform(g_command.cancel());
}

inline const logic::crsf::Telemetry& telemetry() {
    return g_telemetry;
}

}  // namespace ferrox::hw::crsf
