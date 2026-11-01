#pragma once

// Pure AFHDS 2A protocol logic (frame builders, hopping table, bind and
// telemetry handling). The A7105 radio driver (SPI, GIO2/TIM16 ISRs) lives in
// the hardware layer and is not part of this header.

#include <array>
#include <cstddef>
#include <cstdint>

namespace ferrox::logic::afhds2a {

inline constexpr std::size_t kNumChannels = 18;
inline constexpr std::size_t kNumFreq = 16;
inline constexpr std::size_t kTxPacketSize = 38;
inline constexpr std::size_t kRxPacketSize = 37;

inline constexpr uint16_t kChannelMinUs = 988;
inline constexpr uint16_t kChannelCenterUs = 1500;
inline constexpr uint16_t kChannelMaxUs = 2012;
inline constexpr uint16_t kFailsafeThrottleUs = kChannelMinUs;

inline constexpr uint8_t kPacketSticks = 0x58;
inline constexpr uint8_t kPacketFailsafe = 0x56;
inline constexpr uint8_t kPacketSettings = 0xAA;
inline constexpr uint8_t kPacketBind1 = 0xBB;
inline constexpr uint8_t kPacketBind2 = 0xBC;

inline constexpr uint16_t kRateMinHz = 50;
inline constexpr uint16_t kRateMaxHz = 400;

using Packet = std::array<uint8_t, kTxPacketSize>;
using Frame = std::array<uint8_t, kRxPacketSize>;
using HoppingTable = std::array<uint8_t, kNumFreq>;

enum class PacketKind : uint8_t { Sticks, Settings, Failsafe };

enum class BindStep : uint8_t { Phase1 = 1, Phase2 = 2, Phase3 = 3, Confirm = 4 };

enum class RxEvent : uint8_t { Ignored, RequestSettings, RequestFailsafe, Data };

struct RxSettings {
    uint16_t rate_hz = kRateMinHz;
    bool ppm_out = false;
    bool sbus = false;
};

struct Telemetry {
    bool connected = false;
    uint8_t rssi = 0;
    uint16_t rx_voltage_mv = 0;
    int16_t alt_m = 0;
    int16_t alt_dm = 0;
    int16_t vspeed_cms = 0;
    bool has_vspeed = false;
    uint32_t rpm = 0;
    int16_t temp_c_x10 = 0;
    uint32_t last_rx_voltage_ms = 0;
    uint32_t last_alt_ms = 0;
    uint32_t last_vspd_ms = 0;
    uint32_t last_rpm_ms = 0;
    uint32_t last_temp_ms = 0;
    uint32_t packets_received = 0;
};

// Hopping table derived from the TX UID (LCG + band/separation rules).
constexpr HoppingTable hopping_table(uint32_t tx_id) {
    HoppingTable table{};
    std::size_t idx = 0;
    uint32_t rnd = tx_id;
    const uint32_t tx_byte3 = (tx_id >> 24) & 0xFFu;
    uint32_t attempts = 0;
    while (idx < kNumFreq) {
        ++attempts;
        const uint32_t band_no = (((idx << 1) | ((idx >> 1) & 0x01u)) + tx_byte3) & 0x03u;
        rnd = rnd * 0x0019660Du + 0x3C6EF35Fu;
        const auto next_ch = static_cast<uint8_t>(band_no * 41u + 1u + ((rnd >> idx) % 41u));
        const uint32_t min_sep = attempts > 500 ? 1u : (attempts > 200 ? 3u : 5u);
        bool valid = true;
        for (std::size_t i = 0; i < idx; ++i) {
            const uint32_t diff = next_ch > table[i] ? next_ch - table[i] : table[i] - next_ch;
            if (diff < min_sep) {
                valid = false;
                break;
            }
        }
        if (valid) {
            table[idx] = next_ch;
            ++idx;
        }
    }
    return table;
}

constexpr uint16_t clamp_channel_us(uint16_t us) {
    return us < kChannelMinUs ? kChannelMinUs : (us > kChannelMaxUs ? kChannelMaxUs : us);
}

constexpr RxSettings make_rx_settings(uint16_t rate_hz, bool ppm_out, bool sbus) {
    const uint16_t r = rate_hz < kRateMinHz ? kRateMinHz : (rate_hz > kRateMaxHz ? kRateMaxHz : rate_hz);
    return RxSettings{r, ppm_out, sbus};
}

// RF power index is cyclic in the UI; out-of-range values fall back to 0.
constexpr uint8_t clamp_rf_power(uint8_t idx) { return idx > 3 ? 0 : idx; }

// A7105 raw RSSI register value -> percent (0..100).
constexpr uint8_t rssi_from_raw(uint8_t raw) {
    const int32_t v = 256 - (static_cast<int32_t>(raw) * 8) / 5;
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 100 ? 100 : v));
}

namespace detail {

constexpr void put_u32_le(Packet& p, std::size_t off, uint32_t v) {
    for (std::size_t i = 0; i < 4; ++i) {
        p[off + i] = static_cast<uint8_t>(v >> (8 * i));
    }
}

constexpr void put_header(Packet& p, uint8_t cmd, uint32_t tx_id, uint32_t rx_id) {
    p[0] = cmd;
    put_u32_le(p, 1, tx_id);
    put_u32_le(p, 5, rx_id);
}

}  // namespace detail

// 14 base channels as 12-bit LE values in slots 0..13, channels 15..18 split
// across the upper nibbles of slots 0..11 (iBUS-style extension).
constexpr Packet build_sticks(uint32_t tx_id, uint32_t rx_id,
                              const std::array<uint16_t, kNumChannels>& ch) {
    Packet p{};
    detail::put_header(p, kPacketSticks, tx_id, rx_id);
    for (std::size_t i = 0; i < 14; ++i) {
        const uint16_t v = clamp_channel_us(ch[i]);
        p[9 + i * 2] = static_cast<uint8_t>(v & 0xFF);
        p[10 + i * 2] = static_cast<uint8_t>((v >> 8) & 0x0F);
    }
    for (std::size_t ext = 0; ext < 4; ++ext) {
        const uint16_t v = clamp_channel_us(ch[14 + ext]);
        const std::size_t base = ext * 3;
        p[10 + base * 2] |= static_cast<uint8_t>((v & 0x000F) << 4);
        p[10 + (base + 1) * 2] |= static_cast<uint8_t>(v & 0x00F0);
        p[10 + (base + 2) * 2] |= static_cast<uint8_t>((v >> 4) & 0x00F0);
    }
    return p;
}

constexpr Packet build_settings(uint32_t tx_id, uint32_t rx_id, const RxSettings& s) {
    Packet p{};
    detail::put_header(p, kPacketSettings, tx_id, rx_id);
    p[9] = 0xFD;
    p[10] = 0xFF;
    p[11] = static_cast<uint8_t>(s.rate_hz & 0xFF);
    p[12] = static_cast<uint8_t>(s.rate_hz >> 8);
    p[13] = s.ppm_out ? 0x01 : 0x00;
    p[14] = 0x00;
    for (std::size_t i = 15; i < 37; ++i) {
        p[i] = 0xFF;
    }
    p[18] = static_cast<uint8_t>(kChannelCenterUs >> 8);
    p[19] = static_cast<uint8_t>(kChannelCenterUs & 0xFF);
    p[20] = 0x05;
    p[21] = s.sbus ? 0xDD : 0xDE;
    return p;
}

constexpr Packet build_failsafe(uint32_t tx_id, uint32_t rx_id) {
    Packet p{};
    detail::put_header(p, kPacketFailsafe, tx_id, rx_id);
    for (std::size_t ch = 0; ch < 14; ++ch) {
        const uint16_t v = ch == 2 ? kFailsafeThrottleUs : 0xFFFF;
        p[9 + ch * 2] = static_cast<uint8_t>(v & 0xFF);
        p[10 + ch * 2] = static_cast<uint8_t>(v >> 8);
    }
    return p;
}

constexpr Packet build_bind(uint32_t tx_id, uint32_t rx_id, BindStep step, const HoppingTable& hop) {
    Packet p{};
    detail::put_u32_le(p, 1, tx_id);
    switch (step) {
        case BindStep::Phase1:
            p[0] = kPacketBind1;
            for (std::size_t i = 5; i < 9; ++i) p[i] = 0xFF;
            p[9] = 0x01;
            for (std::size_t i = 0; i < kNumFreq; ++i) p[11 + i] = hop[i];
            for (std::size_t i = 27; i < 37; ++i) p[i] = 0xFF;
            break;
        case BindStep::Phase2:
        case BindStep::Phase3:
            p[0] = kPacketBind2;
            for (std::size_t i = 5; i < 9; ++i) p[i] = 0xFF;
            p[9] = step == BindStep::Phase2 ? 0x00 : 0x01;
            for (std::size_t i = 0; i < kNumFreq; ++i) p[11 + i] = hop[i];
            p[27] = 0x01;
            p[28] = 0x80;
            for (std::size_t i = 29; i < 37; ++i) p[i] = 0xFF;
            break;
        case BindStep::Confirm:
            p[0] = kPacketBind2;
            detail::put_u32_le(p, 5, rx_id);
            p[9] = 0x02;
            for (std::size_t i = 11; i < 27; ++i) p[i] = 0xFF;
            p[27] = 0x01;
            p[28] = 0x80;
            for (std::size_t i = 29; i < 37; ++i) p[i] = 0xFF;
            break;
    }
    return p;
}

constexpr uint32_t read_u32_le(const Frame& f, std::size_t off) {
    return static_cast<uint32_t>(f[off]) | (static_cast<uint32_t>(f[off + 1]) << 8) |
           (static_cast<uint32_t>(f[off + 2]) << 16) | (static_cast<uint32_t>(f[off + 3]) << 24);
}

// Link state for one bound TX/RX pair. Pure: the caller supplies the raw
// A7105 RSSI register and the current time.
struct Link {
    uint32_t tx_id = 0;
    uint32_t rx_id = 0;
    bool binding = false;
    BindStep bind_step = BindStep::Phase1;
    bool bind_done = false;
    bool rx_id_needs_save = false;
    PacketKind next = PacketKind::Sticks;
    RxSettings settings{};
    uint8_t rf_power_idx = 0;
    Telemetry telemetry{};

    constexpr explicit Link(uint32_t uid) : tx_id(uid) {}

    constexpr void set_rx_settings(uint16_t rate_hz, bool ppm_out, bool sbus) {
        settings = make_rx_settings(rate_hz, ppm_out, sbus);
        next = PacketKind::Settings;
    }

    constexpr void set_rf_power(uint8_t idx) { rf_power_idx = clamp_rf_power(idx); }

    // Receiver answer to a bind request: rx[9] == 0x01 carries the RX ID.
    constexpr bool on_bind_reply(const Frame& rx) {
        if (!binding || rx[0] != kPacketBind2 || rx[9] != 0x01) {
            return false;
        }
        rx_id = read_u32_le(rx, 5);
        bind_step = BindStep::Confirm;
        return true;
    }

    constexpr RxEvent on_downlink(const Frame& rx, uint8_t raw_rssi, uint32_t now_ms) {
        if (rx[0] != 0xAA && rx[0] != 0xAC) {
            return RxEvent::Ignored;
        }
        if (read_u32_le(rx, 1) != tx_id) {
            return RxEvent::Ignored;
        }
        const uint32_t rid = read_u32_le(rx, 5);
        const bool placeholder = rx_id == 0xFFFF'FFFFu || rx_id == 0 || (rx_id & 0xFFFF'0000u) == 0x0001'0000u;
        if (placeholder && rid != 0xFFFF'FFFFu && rid != 0) {
            rx_id = rid;
            rx_id_needs_save = true;
            bind_done = true;
        } else if (rid != rx_id && !placeholder) {
            return RxEvent::Ignored;
        } else {
            bind_done = true;
        }

        if (rx[0] == 0xAA && rx[9] == 0xFC) {
            next = PacketKind::Settings;
            return RxEvent::RequestSettings;
        }
        if (rx[0] == 0xAA && rx[9] == 0xFD) {
            next = PacketKind::Failsafe;
            return RxEvent::RequestFailsafe;
        }

        telemetry.connected = true;
        ++telemetry.packets_received;
        telemetry.rssi = rssi_from_raw(raw_rssi);
        parse_sensors(rx, now_ms);
        return RxEvent::Data;
    }

private:
    constexpr void parse_sensors(const Frame& rx, uint32_t now_ms) {
        for (std::size_t off = 10; off + 4 <= kRxPacketSize; off += 4) {
            const uint8_t id = rx[off];
            const auto val = static_cast<uint16_t>(rx[off + 2] | (rx[off + 3] << 8));
            switch (id) {
                case 0x00:
                case 0x03:
                    telemetry.rx_voltage_mv = static_cast<uint16_t>(val * 10);
                    telemetry.last_rx_voltage_ms = now_ms;
                    break;
                case 0x01:
                    telemetry.temp_c_x10 = static_cast<int16_t>(static_cast<int32_t>(val) - 400);
                    telemetry.last_temp_ms = now_ms;
                    break;
                case 0x02:
                    telemetry.rpm = val;
                    telemetry.last_rpm_ms = now_ms;
                    break;
                case 0x09:
                    telemetry.vspeed_cms = static_cast<int16_t>(static_cast<int16_t>(val) * 10);
                    telemetry.has_vspeed = true;
                    telemetry.last_vspd_ms = now_ms;
                    break;
                case 0x41:
                case 0x83:
                case 0xF9: {
                    const auto alt = static_cast<int16_t>(val);
                    telemetry.alt_m = alt;
                    telemetry.alt_dm = static_cast<int16_t>(alt * 10);
                    telemetry.last_alt_ms = now_ms;
                    break;
                }
                case 0xFE:
                    if (val <= 100) {
                        telemetry.rssi = static_cast<uint8_t>(100 - val);
                    }
                    break;
                default:
                    break;
            }
        }
    }
};

}  // namespace ferrox::logic::afhds2a
