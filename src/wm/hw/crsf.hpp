#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "wm/hw/uart.hpp"
#include "wm/logic/crsf.hpp"
#include "wm/logic/crsf_config.hpp"
#include "wm/logic/crsf_menu.hpp"
#if WM_PCB_I6S && WM_USE_MULTISWITCH
#include "wm/logic/multiswitch.hpp"
#endif

namespace wm::hw::crsf {

// Main-loop state only (no ISR access), hence no volatile.
inline constinit bool g_enabled = false;
inline constinit uint8_t g_baud_idx = 0;
inline constinit bool g_half_duplex = false;
inline constinit uint32_t g_last_tx_ms = 0;
inline constinit uint32_t g_last_ping_ms = 0;
// Set by the first menu_restart (configurator opened). No discovery pings before that.
inline constinit bool g_discovery = false;
inline constinit logic::crsf::Framer g_framer{};
inline constinit logic::crsf::Telemetry g_telemetry{};
inline constinit logic::crsf::menu::Loader g_menu{};
inline constinit logic::crsf::menu::Command g_command{};
inline constinit logic::crsf::config::DeviceList g_devices{};
// Ring position where the RX line last went idle and that the reader has not reached yet.
inline constinit std::optional<std::size_t> g_rx_idle_at{};

inline constexpr uint32_t kTxPeriodMs = 10;
inline constexpr uint32_t kTelemetryTimeoutMs = 1000;
inline constexpr uint32_t kPingPeriodMs = 1000;
inline constexpr std::size_t kMaxRxBytesPerPoll = 128;

/// Enables or disables CRSF on the CRSF USART (USART2 / USART1 on the FS-i6S) and the module power switch.
inline void set_enabled(const bool enabled, const uint8_t baud_idx, const uart::Duplex duplex) {
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
        g_rx_idle_at.reset();
        g_menu.stop();
        g_command = logic::crsf::menu::Command{};
    }
}

inline bool is_enabled() {
    return g_enabled;
}

/// Sends an RC channels frame every 10 ms (100 Hz).
template <std::size_t N>
inline void update_channels(const uint32_t now_ms, const std::array<uint16_t, N>& channels,
                            const logic::crsf::ArmModeSwitch arm_mode_switch, const logic::crsf::ArmedState armed) {
    if (!g_enabled || static_cast<uint32_t>(now_ms - g_last_tx_ms) < kTxPeriodMs) {
        return;
    }
    g_last_tx_ms = now_ms;

    std::array<uint8_t, logic::crsf::kRcExtFrameSize> frame{};
    const std::size_t len = logic::crsf::build_rc_frame(channels, arm_mode_switch, armed, frame);
    uart::write_bytes(std::span<const uint8_t>(frame.data(), len));
}

#if WM_PCB_I6S && WM_USE_MULTISWITCH
inline void update_multiswitch(const uint32_t now_ms, const uint8_t address, logic::multiswitch::State& state) {
    // RC frames have priority; retry on the next loop instead of waiting for their DMA transfer.
    if (!g_enabled || uart::g_tx_busy || !state.due(now_ms)) {
        return;
    }
    const auto frame = logic::multiswitch::build_set4(address, state.switches);
    if (uart::write_bytes(frame) == frame.size()) {
        state.sent(now_ms);
    }
}
#endif

namespace detail {

using MenuAction = logic::crsf::menu::Action;

/// Sends the frame(s) requested by the menu engine.
inline void perform(const MenuAction& a) {
    if (a.kind == MenuAction::Kind::Read) {
        std::array<uint8_t, 8> frame{};
        const std::size_t n = logic::crsf::build_param_ext(
            logic::crsf::DeviceAddr{g_menu.device}, logic::crsf::FrameType{logic::crsf::kTypeParamRead},
            logic::crsf::ParamId{a.param}, logic::crsf::ParamArg{a.arg}, frame);
        uart::write_bytes(std::span<const uint8_t>(frame.data(), n));
    } else if (a.kind == MenuAction::Kind::Write) {
        const std::array<uint8_t, 1> payload{a.arg};
        std::array<uint8_t, 16> frame{};
        const std::size_t n = logic::crsf::build_param_write_multi(
            logic::crsf::DeviceAddr{g_menu.device}, logic::crsf::ParamId{a.param}, payload, frame);
        uart::write_bytes(std::span<const uint8_t>(frame.data(), n));
    }
}

/// Device info (0x29): payload is [dest, origin, name\0, serial(4), hw(4), fw(4), param_count, ...].
/// Answers to the discovery ping are collected in the device list; loading starts when one is selected.
inline void on_device_info(std::span<const uint8_t> p, const uint32_t now_ms) {
    if (p.size() < 3 || g_menu.state != logic::crsf::menu::Loader::State::Idle) {
        return;
    }
    const std::size_t name_end = logic::crsf::menu::find_nul(p, 2);
    const std::size_t count_idx = name_end + 1 + 12;
    const uint8_t param_count = count_idx < p.size() ? p[count_idx] : static_cast<uint8_t>(logic::crsf::menu::kMaxParams);
    g_devices.observe(p[1], {reinterpret_cast<const char*>(p.data()) + 2, name_end - 2}, param_count, now_ms);
}

/// Parameter entry (0x2B): payload is [dest, origin, param, chunks_remain, chunk...].
inline void on_param_entry(std::span<const uint8_t> p, const uint32_t now_ms) {
    if (p.size() < 4) {
        return;
    }
    perform(g_menu.on_entry(logic::crsf::DeviceAddr{p[1]}, logic::crsf::ParamId{p[2]},
                            logic::crsf::menu::ChunksRemain{p[3]}, p.subspan(4), now_ms));

    if (!g_menu.completed) {
        return;
    }
    const uint8_t id = *g_menu.completed;
    if (const auto* e = g_menu.find(id); e != nullptr && e->type == logic::crsf::menu::Type::Command) {
        g_command.on_status(logic::crsf::ParamId{id},
                            static_cast<logic::crsf::menu::CmdStatus>(static_cast<uint8_t>(e->value)),
                            logic::crsf::menu::TimeoutByte{e->cmd_timeout}, now_ms, e->text.len > 0);
    }
}

}  // namespace detail

/// Drains the CRSF USART RX, updates telemetry and the parameter menu. Call regularly from the main loop.
inline void poll_telemetry(const uint32_t now_ms) {
    if (!g_enabled) {
        return;
    }

    // The IDLE interrupt marks the end of a burst at a ring position. The main loop often runs while a frame is
    // still arriving, so bytes before that position may be unread: the framer is only reset once the reader has
    // caught up with it (a truncated frame), never before draining.
    if (const auto idle_at = uart::take_rx_idle()) {
        g_rx_idle_at = idle_at;
    }

    std::size_t count = 0;
    bool drained = false;
    while (true) {
        if (g_rx_idle_at && uart::rx_tail() == *g_rx_idle_at) {
            g_framer.reset();
            g_rx_idle_at.reset();
        }
        const auto byte = uart::read_byte();
        if (!byte) {
            drained = true;
            break;
        }
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
    // Stall check only after draining: last_byte_ms is the read time, so bytes waiting in the ring while the
    // loop was busy (LCD flush, EEPROM write) must not count as a stall.
    if (drained) {
        g_framer.check_stall(now_ms);
    }

    if (g_telemetry.connected &&
        static_cast<uint32_t>(now_ms - g_telemetry.last_telemetry_ms) > kTelemetryTimeoutMs) {
        g_telemetry.connected = false;
    }

    // Parameter discovery: ping while no device is loaded, once the configurator has been opened
    // (Rust `start_config`).
    if (g_discovery && g_menu.state == logic::crsf::menu::Loader::State::Idle &&
        static_cast<uint32_t>(now_ms - g_last_ping_ms) >= kPingPeriodMs) {
        g_last_ping_ms = now_ms;
        g_devices.prune(now_ms);
        std::array<uint8_t, 6> ping{};
        const std::size_t n = logic::crsf::build_ping(ping);
        uart::write_bytes(std::span<const uint8_t>(ping.data(), n));
    }

    detail::perform(g_menu.tick(now_ms));
    detail::perform(g_command.tick(now_ms));
}

inline bool menu_ready() {
    return g_menu.state == logic::crsf::menu::Loader::State::Ready;
}

/// Writes a value (clamped to the parameter range) to the loaded device.
inline bool menu_set_value(const uint8_t param, const int32_t value) {
    auto* found = menu_ready() ? g_menu.find(param) : nullptr;
    if (found == nullptr) {
        return false;
    }
    auto& e = *found;
    std::array<uint8_t, 2> bytes{};
    const std::size_t n = logic::crsf::menu::encode_value(e, value, bytes);
    if (n == 0) {
        return false;
    }
    e.value = logic::crsf::menu::clamp_value(e, value);
    std::array<uint8_t, 16> frame{};
    const std::size_t len = logic::crsf::build_param_write_multi(
        logic::crsf::DeviceAddr{g_menu.device}, logic::crsf::ParamId{param},
        std::span<const uint8_t>(bytes.data(), n), frame);
    uart::write_bytes(std::span<const uint8_t>(frame.data(), len));
    return true;
}

/// Cycles a select parameter to its next option.
inline bool menu_cycle_value(const uint8_t param) {
    const auto* e = menu_ready() ? g_menu.find(param) : nullptr;
    if (e == nullptr || e->type != logic::crsf::menu::Type::Select || e->max <= 0) {
        return false;
    }
    return menu_set_value(param, logic::crsf::menu::cycled_value(*e));
}

inline void menu_enter_folder(const uint8_t folder, const uint32_t now_ms) {
    detail::perform(g_menu.enter_folder(folder, now_ms));
}

inline void menu_start_command(const uint8_t param, const uint32_t now_ms) {
    if (menu_ready()) {
        detail::perform(g_command.start(param, now_ms));
    }
}

inline void menu_confirm_command(const uint32_t now_ms) {
    detail::perform(g_command.confirm(now_ms));
}

inline void menu_cancel_command() {
    detail::perform(g_command.cancel());
}

/// Drops the loaded device and the device list so that discovery (1 Hz ping) starts over.
/// Sent when the configurator opens; discovery stays on from then (Rust `start_config`).
inline void menu_restart(const uint32_t now_ms) {
    g_discovery = true;
    g_menu.stop();
    g_command = logic::crsf::menu::Command{};
    g_devices.clear();
    g_last_ping_ms = now_ms - kPingPeriodMs;
}

/// Starts loading the parameters of device `index` of the device list.
inline void menu_select_device(const uint8_t index, const uint32_t now_ms) {
    if (index >= g_devices.count) {
        return;
    }
    const auto& d = g_devices.items[index];
    g_command = logic::crsf::menu::Command{};
    detail::perform(g_menu.start(logic::crsf::DeviceAddr{d.address}, logic::crsf::menu::ParamCount{d.param_count}, now_ms));
}

/// Leaves the loaded device but keeps the device list.
inline void menu_back_to_list(const uint32_t now_ms) {
    g_menu.stop();
    g_command = logic::crsf::menu::Command{};
    g_devices.touch(now_ms);
    g_last_ping_ms = now_ms - kPingPeriodMs;
}

inline const logic::crsf::menu::Loader& menu_loader() {
    return g_menu;
}

inline const logic::crsf::menu::Command& menu_command() {
    return g_command;
}

inline const logic::crsf::config::DeviceList& menu_devices() {
    return g_devices;
}

/// Performs a request of the Configure Module screen.
inline void menu_apply(const logic::crsf::config::Request& r, const uint32_t now_ms) {
    using Kind = logic::crsf::config::Request::Kind;
    switch (r.kind) {
        case Kind::Restart: menu_restart(now_ms); break;
        case Kind::SelectDevice: menu_select_device(r.param, now_ms); break;
        case Kind::BackToList: menu_back_to_list(now_ms); break;
        case Kind::Write: menu_set_value(r.param, r.value); break;
        case Kind::EnterFolder: menu_enter_folder(r.param, now_ms); break;
        case Kind::StartCommand: menu_start_command(r.param, now_ms); break;
        case Kind::ConfirmCommand: menu_confirm_command(now_ms); break;
        case Kind::CancelCommand: menu_cancel_command(); break;
        case Kind::DismissCommand: g_command.dismiss(); break;
        case Kind::None: break;
    }
}

inline const logic::crsf::Telemetry& telemetry() {
    return g_telemetry;
}

}  // namespace wm::hw::crsf
