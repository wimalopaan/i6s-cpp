#pragma once

// Pure AFHDS 2A radio state machine. Each entry point returns a plan that the
// hardware layer (hw/rf.hpp) executes on the A7105. No SPI, GPIO or timers
// are touched here, so the logic runs on the host.

#include <array>
#include <cstdint>
#include <optional>

#include "ferrox/logic/a7105.hpp"
#include "ferrox/logic/afhds2a.hpp"

namespace ferrox::logic::afhds2a {

enum class SubState : uint8_t { Idle, WaitingTxDone, ListeningRx };

enum class FrontEnd : uint8_t { Off, RxEn, TxEn };

struct TxPlan {
    uint8_t channel = 0;
    Packet packet{};
    bool toggle_antenna = false;
    std::optional<uint8_t> power_reg{};
};

enum class GioStep : uint8_t { None, StartRx, ReceiveDone };

struct GioPlan {
    GioStep step = GioStep::None;
    FrontEnd front = FrontEnd::Off;
};

// Staleness gate: the 3.85 ms TIM16 tick stops transmitting after 30 ticks
// without a main-loop heartbeat, so the receiver failsafe can engage.
struct Heartbeat {
    static constexpr uint8_t kLimit = 30;

    uint16_t last = 0;
    uint8_t stale = 0;

    constexpr bool allow_tx(uint16_t hb) {
        if (hb == last) {
            if (stale < 0xFF) {
                ++stale;
            }
            return stale < kLimit;
        }
        last = hb;
        stale = 0;
        return true;
    }
};

struct Radio {
    static constexpr uint16_t kLossLimit = 52;
    static constexpr uint32_t kFailsafePeriod = 1569;
    static constexpr uint8_t kBindConfirmCount = 4;

    Link link;
    HoppingTable hop{};
    SubState sub = SubState::Idle;
    uint8_t hopping_idx = 0;
    uint8_t bind_confirm = 0;
    uint16_t loss = 0;
    uint32_t packets_sent = 0;
    std::array<uint16_t, kNumChannels> channels{};

    constexpr explicit Radio(uint32_t tx_id) : link(tx_id), hop(hopping_table(tx_id)) {}

    constexpr bool binding() const { return link.binding; }

    // TIM16 tick: build the next packet and the channel to send it on.
    constexpr TxPlan on_timer_tick() {
        ++packets_sent;
        if (loss < 0xFFFF) {
            ++loss;
        }
        if (loss > kLossLimit) {
            link.telemetry.connected = false;
            link.telemetry.rssi = 0;
        }

        TxPlan plan{};
        if (link.binding) {
            plan.channel = (packets_sent & 1u) != 0 ? 0x0D : 0x8C;
            plan.packet = build_bind(link.tx_id, link.rx_id, link.bind_step, hop);
            if (link.bind_step == BindStep::Confirm) {
                ++bind_confirm;
                if (bind_confirm >= kBindConfirmCount) {
                    link.binding = false;
                    link.bind_step = BindStep::Phase1;
                    bind_confirm = 0;
                    link.bind_done = true;
                    link.rx_id_needs_save = true;
                    hopping_idx = 1;
                    link.next = PacketKind::Sticks;
                    plan.power_reg = a7105::power_from_index(link.rf_power_idx);
                }
            }
        } else {
            plan.channel = hop[hopping_idx & 0x0F];
            hopping_idx = static_cast<uint8_t>((hopping_idx + 1) & 0x0F);
            plan.toggle_antenna = hopping_idx != 0;

            if (link.next == PacketKind::Sticks && packets_sent % kFailsafePeriod == 0) {
                link.next = PacketKind::Failsafe;
            }
            switch (link.next) {
                case PacketKind::Settings:
                    plan.packet = build_settings(link.tx_id, link.rx_id, link.settings);
                    link.next = PacketKind::Sticks;
                    break;
                case PacketKind::Failsafe:
                    plan.packet = build_failsafe(link.tx_id, link.rx_id);
                    link.next = PacketKind::Sticks;
                    break;
                case PacketKind::Sticks:
                    plan.packet = build_sticks(link.tx_id, link.rx_id, channels);
                    break;
            }
        }

        sub = SubState::WaitingTxDone;
        return plan;
    }

    // GIO2 falling edge: TX finished (WaitingTxDone) or RX packet ready (ListeningRx).
    constexpr GioPlan on_gio2_event() {
        GioPlan plan{};
        switch (sub) {
            case SubState::WaitingTxDone:
                sub = SubState::ListeningRx;
                plan.step = GioStep::StartRx;
                if (link.binding) {
                    plan.front = FrontEnd::Off;
                    if (link.bind_step != BindStep::Confirm) {
                        const auto p = static_cast<uint8_t>(link.bind_step);
                        link.bind_step = static_cast<BindStep>(p % 3 + 1);
                    }
                } else {
                    plan.front = FrontEnd::RxEn;
                }
                break;
            case SubState::ListeningRx:
                sub = SubState::Idle;
                plan.step = GioStep::ReceiveDone;
                break;
            case SubState::Idle:
                break;
        }
        return plan;
    }

    // Called by the hardware after a CRC-OK RX frame has been read from the FIFO.
    constexpr void on_rx_frame(const Frame& rx, uint8_t raw_rssi, uint32_t now_ms) {
        if (link.binding) {
            if (link.on_bind_reply(rx)) {
                bind_confirm = 0;
            }
            return;
        }
        if (link.on_downlink(rx, raw_rssi, now_ms) == RxEvent::Data) {
            loss = 0;
        }
    }

    // Returns the TX power register to apply now, or nullopt while binding.
    constexpr std::optional<uint8_t> set_bind_mode(bool on) {
        if (on) {
            link.binding = true;
            link.bind_step = BindStep::Phase1;
            bind_confirm = 0;
            link.bind_done = false;
            return a7105::kBindPower;
        }
        link.binding = false;
        link.bind_done = true;
        link.next = PacketKind::Sticks;
        if (link.rx_id == 0 || link.rx_id == 0xFFFF'FFFFu) {
            link.rx_id = 0x0001'0000u | (link.tx_id & 0xFFFFu);
        }
        link.rx_id_needs_save = true;
        return a7105::power_from_index(link.rf_power_idx);
    }

    constexpr std::optional<uint8_t> set_rf_power(uint8_t idx) {
        link.set_rf_power(idx);
        if (link.binding) {
            return std::nullopt;
        }
        return a7105::power_from_index(link.rf_power_idx);
    }

    constexpr void set_rx_id(uint32_t id) {
        link.rx_id = id == 0 ? 0xFFFF'FFFFu : id;
        link.bind_done = id != 0 && id != 0xFFFF'FFFFu;
        loss = 0;
        link.telemetry.connected = false;
    }

    constexpr void set_rx_settings(uint16_t rate_hz, bool ppm_out, bool sbus) {
        link.set_rx_settings(rate_hz, ppm_out, sbus);
    }

    constexpr void set_channels(const std::array<uint16_t, kNumChannels>& ch) {
        channels = ch;
    }

    // Returns the RX ID once after it was captured or changed, so the caller can persist it.
    constexpr std::optional<uint32_t> take_rx_save() {
        if (!link.rx_id_needs_save) {
            return std::nullopt;
        }
        link.rx_id_needs_save = false;
        return link.rx_id;
    }
};

}  // namespace ferrox::logic::afhds2a
