#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace ferrox::logic::crsf {

inline constexpr uint8_t kSyncByte = 0xC8;
inline constexpr uint8_t kAddrRadioTransmitter = 0xEA;
inline constexpr uint8_t kAddrCrsfReceiver = 0xEC;
inline constexpr uint8_t kAddrCrsfTransmitter = 0xEE;

inline constexpr uint8_t kTypeGps = 0x02;
inline constexpr uint8_t kTypeVario = 0x07;
inline constexpr uint8_t kTypeBattery = 0x08;
inline constexpr uint8_t kTypeBaro = 0x09;
inline constexpr uint8_t kTypeRpm = 0x0C;
inline constexpr uint8_t kTypeTemp = 0x0D;
inline constexpr uint8_t kTypeCells = 0x0E;
inline constexpr uint8_t kTypeLinkStatistics = 0x14;
inline constexpr uint8_t kTypeRcChannelsPacked = 0x16;
inline constexpr uint8_t kTypeAttitude = 0x1E;
inline constexpr uint8_t kTypeFlightMode = 0x21;
inline constexpr uint8_t kTypeElrsStatus = 0x2E;

// Mixer output range in microseconds (see mixer.rs CHANNEL_*_US).
inline constexpr uint16_t kChannelMinUs = 988;
inline constexpr uint16_t kChannelMaxUs = 2012;
inline constexpr uint32_t kChannelSpanUs = kChannelMaxUs - kChannelMinUs;

inline constexpr uint16_t kCrsfMin = 172;
inline constexpr uint16_t kCrsfMax = 1811;
inline constexpr uint16_t kCrsfCenter = 992;
inline constexpr uint32_t kCrsfSpan = kCrsfMax - kCrsfMin;
inline constexpr uint16_t kCrsfValueMax = 2047;

inline constexpr std::size_t kMaxFrameSize = 64;
inline constexpr std::size_t kRcFrameSize = 26;
inline constexpr std::size_t kRcExtFrameSize = 27;
inline constexpr uint8_t kChannelsStatusArmed = 0x01;

/// CRC-8/DVB-S2 (polynomial 0xD5) lookup table, generated at compile time.
consteval std::array<uint8_t, 256> make_crc8_table() {
    std::array<uint8_t, 256> table{};
    for (std::size_t i = 0; i < 256; ++i) {
        auto crc = static_cast<uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0xD5u)
                                     : static_cast<uint8_t>(crc << 1);
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr auto kCrc8Table = make_crc8_table();

constexpr uint8_t crc8(std::span<const uint8_t> data) {
    uint8_t crc = 0;
    for (const uint8_t b : data) {
        crc = kCrc8Table[static_cast<uint8_t>(crc ^ b)];
    }
    return crc;
}

/// Maps a pulse width in µs onto the 11-bit CRSF channel range (clamped).
constexpr uint16_t us_to_crsf(const uint16_t us) {
    int32_t clamped = us;
    if (clamped < kChannelMinUs) {
        clamped = kChannelMinUs;
    } else if (clamped > kChannelMaxUs) {
        clamped = kChannelMaxUs;
    }
    const int32_t val = ((clamped - kChannelMinUs) * static_cast<int32_t>(kCrsfSpan) +
                         static_cast<int32_t>(kChannelSpanUs / 2)) /
                            static_cast<int32_t>(kChannelSpanUs) +
                        kCrsfMin;
    if (val < 0) {
        return 0;
    }
    if (val > kCrsfValueMax) {
        return kCrsfValueMax;
    }
    return static_cast<uint16_t>(val);
}

struct ArmModeSwitch {
    bool value = false;
    friend constexpr bool operator==(const ArmModeSwitch&, const ArmModeSwitch&) = default;
};

struct ArmedState {
    bool value = false;
    friend constexpr bool operator==(const ArmedState&, const ArmedState&) = default;
};

/// Builds an RC channels frame (type 0x16) from the first 16 channels.
/// With `arm_mode_switch` the 27-byte ExpressLRS variant with arm flag is produced.
/// Returns the number of valid bytes in `out`.
template <std::size_t N>
constexpr std::size_t build_rc_frame(const std::array<uint16_t, N>& channels, const ArmModeSwitch arm_mode_switch,
                                     const ArmedState armed, std::array<uint8_t, kRcExtFrameSize>& out) {
    static_assert(N >= 16, "CRSF RC frame needs 16 channels");
    out = {};
    out[0] = kSyncByte;
    out[2] = kTypeRcChannelsPacked;

    // 16 channels x 11 bit, LSB-first bit stream into bytes 3..24.
    std::size_t bit = 0;
    for (std::size_t ch = 0; ch < 16; ++ch) {
        const uint16_t value = us_to_crsf(channels[ch]);
        for (std::size_t b = 0; b < 11; ++b, ++bit) {
            if (((value >> b) & 1u) != 0) {
                out[3 + bit / 8] = static_cast<uint8_t>(out[3 + bit / 8] | (1u << (bit % 8)));
            }
        }
    }

    if (arm_mode_switch.value) {
        out[1] = 25;
        out[25] = armed.value ? kChannelsStatusArmed : 0;
        out[26] = crc8(std::span<const uint8_t>(out.data() + 2, 24));
        return kRcExtFrameSize;
    }
    out[1] = 24;
    out[25] = crc8(std::span<const uint8_t>(out.data() + 2, 23));
    return kRcFrameSize;
}

struct Telemetry {
    bool connected = false;
    int8_t uplink_rssi_1 = 0;
    int8_t uplink_rssi_2 = 0;
    uint8_t uplink_link_quality = 0;
    int8_t uplink_snr = 0;
    uint8_t active_antenna = 0;
    uint8_t rf_mode = 0;
    uint16_t tx_power_mw = 0;
    uint16_t rx_battery_mv = 0;
    uint16_t rx_current_ma = 0;
    uint32_t rx_capacity_mah = 0;
    uint8_t rx_battery_pct = 0;
    int16_t vspeed_cms = 0;
    bool has_vspeed = false;
    int16_t baro_alt_m = 0;
    int16_t baro_alt_dm = 0;
    uint8_t cell_count = 0;
    uint16_t cell_min_mv = 0;
    uint32_t rpm = 0;
    int16_t temp_c_x10 = 0;
    uint16_t gps_groundspeed_kmh_x10 = 0;
    uint16_t gps_heading_cdeg = 0;
    uint8_t gps_satellites = 0;
    int16_t pitch_cdeg = 0;
    int16_t roll_cdeg = 0;
    int16_t yaw_cdeg = 0;
    std::array<uint8_t, 8> flight_mode{};
    uint32_t last_telemetry_ms = 0;
    uint32_t last_vbat_ms = 0;
    uint32_t last_cells_ms = 0;
    uint32_t last_alt_ms = 0;
    uint32_t last_vspd_ms = 0;
    uint32_t last_rpm_ms = 0;
    uint32_t last_temp_ms = 0;
    uint32_t last_gps_ms = 0;
    uint32_t last_att_ms = 0;
    uint32_t last_link_ms = 0;
};

constexpr uint16_t be16(std::span<const uint8_t> p, const std::size_t i) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[i]) << 8) | p[i + 1]);
}

constexpr int16_t be16s(std::span<const uint8_t> p, const std::size_t i) {
    return static_cast<int16_t>(be16(p, i));
}

/// Decodes the 1-byte logarithmic vertical speed used in the 3-byte baro frame.
constexpr int16_t decode_packed_vspeed(const int8_t raw) {
    if (raw == 0) {
        return 0;
    }
    const int64_t abs_v = raw < 0 ? -static_cast<int64_t>(raw) : static_cast<int64_t>(raw);
    const int64_t z = abs_v * 2600;
    const int64_t term1 = z / 40;
    const int64_t term2 = (z * z) / 8'000'000;
    const int64_t term3 = ((z * z) / 1'000'000 * z) / 2'400'000'000;
    int64_t cms = term1 + term2 + term3;
    if (cms > 32767) {
        cms = 32767;
    }
    return static_cast<int16_t>(raw < 0 ? -cms : cms);
}

constexpr void mark_alive(Telemetry& t, const uint32_t now_ms) {
    t.connected = true;
    t.last_telemetry_ms = now_ms;
}

/// Decodes a validated telemetry frame. Returns true if the type was handled.
constexpr bool parse_telemetry(const uint8_t type, std::span<const uint8_t> p, Telemetry& t,
                               const uint32_t now_ms) {
    switch (type) {
        case kTypeElrsStatus:
            mark_alive(t, now_ms);
            return true;

        case kTypeLinkStatistics: {
            if (p.size() < 10) {
                return false;
            }
            t.uplink_rssi_1 = static_cast<int8_t>(-static_cast<int>(static_cast<int8_t>(p[0])));
            t.uplink_rssi_2 = static_cast<int8_t>(-static_cast<int>(static_cast<int8_t>(p[1])));
            t.uplink_link_quality = p[2];
            t.uplink_snr = static_cast<int8_t>(p[3]);
            t.active_antenna = p[4];
            t.rf_mode = p[5];
            constexpr std::array<uint16_t, 9> kPowerMw{0, 10, 25, 100, 500, 1000, 2000, 250, 50};
            t.tx_power_mw = p[6] < kPowerMw.size() ? kPowerMw[p[6]] : 0;
            t.last_link_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeBattery: {
            if (p.size() < 8) {
                return false;
            }
            // Voltage and current are in 0.1 V / 0.1 A units.
            t.rx_battery_mv = static_cast<uint16_t>(be16(p, 0) * 100u);
            t.rx_current_ma = static_cast<uint16_t>(be16(p, 2) * 100u);
            t.rx_capacity_mah = (static_cast<uint32_t>(p[4]) << 16) |
                                (static_cast<uint32_t>(p[5]) << 8) | p[6];
            t.rx_battery_pct = p[7];
            t.last_vbat_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeVario: {
            if (p.size() < 2) {
                return false;
            }
            t.vspeed_cms = be16s(p, 0);
            t.has_vspeed = true;
            t.last_vspd_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeBaro: {
            if (p.size() < 2) {
                return false;
            }
            const uint16_t raw = be16(p, 0);
            if ((raw & 0x8000u) == 0) {
                // Decimeters with a 10000 dm (1000.0 m) offset.
                const auto alt_dm = static_cast<int32_t>(raw) - 10'000;
                t.baro_alt_dm = static_cast<int16_t>(alt_dm);
                t.baro_alt_m = static_cast<int16_t>(alt_dm / 10);
            } else {
                // Whole meters without offset.
                const auto alt_m = static_cast<int16_t>(raw & 0x7FFFu);
                t.baro_alt_m = alt_m;
                t.baro_alt_dm = static_cast<int16_t>(alt_m * 10);
            }
            t.last_alt_ms = now_ms;

            if (p.size() >= 4) {
                t.vspeed_cms = be16s(p, 2);
                t.has_vspeed = true;
                t.last_vspd_ms = now_ms;
            } else if (p.size() == 3) {
                t.vspeed_cms = decode_packed_vspeed(static_cast<int8_t>(p[2]));
                t.has_vspeed = true;
                t.last_vspd_ms = now_ms;
            }
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeCells: {
            if (p.size() < 2) {
                return false;
            }
            uint32_t total_mv = 0;
            uint16_t min_cell = 0xFFFF;
            uint8_t count = 0;
            std::size_t offset = (p.size() % 2 == 1) ? 1 : 0;
            for (; offset + 2 <= p.size(); offset += 2) {
                const uint16_t cell_mv = be16(p, offset);
                if (cell_mv > 0 && cell_mv < 10'000) {
                    total_mv += cell_mv;
                    if (cell_mv < min_cell) {
                        min_cell = cell_mv;
                    }
                    ++count;
                }
            }
            if (total_mv == 0) {
                return false;
            }
            t.rx_battery_mv = total_mv > 0xFFFF ? uint16_t{0xFFFF} : static_cast<uint16_t>(total_mv);
            t.cell_count = count;
            t.cell_min_mv = min_cell != 0xFFFF ? min_cell : 0;
            t.last_cells_ms = now_ms;
            t.last_vbat_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeGps: {
            if (p.size() < 15) {
                return false;
            }
            const auto alt_m = static_cast<int32_t>(be16(p, 12)) - 1000;
            if (t.baro_alt_m == 0 && t.baro_alt_dm == 0) {
                t.baro_alt_m = static_cast<int16_t>(alt_m);
                t.baro_alt_dm = static_cast<int16_t>(alt_m * 10);
                t.last_alt_ms = now_ms;
            }
            t.gps_groundspeed_kmh_x10 = be16(p, 8);
            t.gps_heading_cdeg = be16(p, 10);
            t.gps_satellites = p[14];
            t.last_gps_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeRpm: {
            if (p.size() < 3) {
                return false;
            }
            const std::size_t off = p.size() >= 4 ? 1 : 0;
            t.rpm = (static_cast<uint32_t>(p[off]) << 16) | (static_cast<uint32_t>(p[off + 1]) << 8) |
                    p[off + 2];
            t.last_rpm_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeTemp: {
            if (p.size() < 2) {
                return false;
            }
            const std::size_t off = p.size() >= 3 ? 1 : 0;
            t.temp_c_x10 = be16s(p, off);
            t.last_temp_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeAttitude: {
            if (p.size() < 6) {
                return false;
            }
            t.pitch_cdeg = be16s(p, 0);
            t.roll_cdeg = be16s(p, 2);
            t.yaw_cdeg = be16s(p, 4);
            t.last_att_ms = now_ms;
            mark_alive(t, now_ms);
            return true;
        }

        case kTypeFlightMode: {
            if (p.empty()) {
                return false;
            }
            t.flight_mode = {};
            const std::size_t len = p.size() < 8 ? p.size() : 8;
            for (std::size_t i = 0; i < len; ++i) {
                t.flight_mode[i] = p[i];
            }
            mark_alive(t, now_ms);
            return true;
        }

        default:
            return false;
    }
}

inline constexpr uint8_t kAddrBroadcast = 0x00;
inline constexpr uint8_t kTypeDevicePing = 0x28;
inline constexpr uint8_t kTypeDeviceInfo = 0x29;
inline constexpr uint8_t kTypeParamEntry = 0x2B;
inline constexpr uint8_t kTypeParamRead = 0x2C;
inline constexpr uint8_t kTypeParamWrite = 0x2D;

/// Device ping (0x28) to all devices. Returns 6 bytes.
constexpr std::size_t build_ping(std::array<uint8_t, 6>& out) {
    out = {kSyncByte, 4, kTypeDevicePing, kAddrBroadcast, kAddrRadioTransmitter, 0};
    out[5] = crc8(std::span<const uint8_t>(out.data() + 2, 3));
    return out.size();
}

struct DeviceAddr {
    uint8_t value = 0;
    friend constexpr bool operator==(const DeviceAddr&, const DeviceAddr&) = default;
};

struct FrameType {
    uint8_t value = 0;
    friend constexpr bool operator==(const FrameType&, const FrameType&) = default;
};

struct ParamId {
    uint8_t value = 0;
    friend constexpr bool operator==(const ParamId&, const ParamId&) = default;
};

struct ParamArg {
    uint8_t value = 0;
    friend constexpr bool operator==(const ParamArg&, const ParamArg&) = default;
};

/// Extended parameter read/write frame with a single value or chunk byte. Returns 8 bytes.
constexpr std::size_t build_param_ext(const DeviceAddr target, const FrameType type, const ParamId param,
                                      const ParamArg value, std::array<uint8_t, 8>& out) {
    out = {kSyncByte, 6, type.value, target.value, kAddrRadioTransmitter, param.value, value.value, 0};
    out[7] = crc8(std::span<const uint8_t>(out.data() + 2, 5));
    return out.size();
}

/// Parameter write with an arbitrary payload. Returns the frame length.
constexpr std::size_t build_param_write_multi(const DeviceAddr target, const ParamId param,
                                              std::span<const uint8_t> payload,
                                              std::array<uint8_t, 16>& out) {
    out = {};
    out[0] = kSyncByte;
    out[1] = static_cast<uint8_t>(5 + payload.size());
    out[2] = kTypeParamWrite;
    out[3] = target.value;
    out[4] = kAddrRadioTransmitter;
    out[5] = param.value;
    for (std::size_t i = 0; i < payload.size(); ++i) {
        out[6 + i] = payload[i];
    }
    const std::size_t end = 6 + payload.size();
    out[end] = crc8(std::span<const uint8_t>(out.data() + 2, end - 2));
    return end + 1;
}

/// Byte-wise CRSF frame assembler. Accepts frames with sync bytes 0xC8/0xEA/0xEC/0xEE
/// and drops frames with a bad CRC. The returned span points into the internal buffer
/// and stays valid until the next call to push().
struct Framer {
    std::array<uint8_t, kMaxFrameSize> buf{};
    std::size_t len = 0;
    uint32_t last_byte_ms = 0;

    constexpr void reset() {
        len = 0;
    }

    /// A partially received frame that stalls for >= 3 ms is discarded.
    constexpr void check_stall(const uint32_t now_ms) {
        if (len > 0 && static_cast<uint32_t>(now_ms - last_byte_ms) >= 3) {
            len = 0;
        }
    }

    constexpr std::span<const uint8_t> push(const uint8_t b, const uint32_t now_ms) {
        last_byte_ms = now_ms;
        if (len == 0) {
            if (b == kSyncByte || b == kAddrRadioTransmitter || b == kAddrCrsfTransmitter ||
                b == kAddrCrsfReceiver) {
                buf[0] = b;
                len = 1;
            }
            return {};
        }
        if (len == 1) {
            const std::size_t frame_len = b;
            if (frame_len >= 2 && frame_len + 2 <= kMaxFrameSize) {
                buf[1] = b;
                len = 2;
            } else {
                len = 0;
            }
            return {};
        }
        const std::size_t total = static_cast<std::size_t>(buf[1]) + 2;
        buf[len++] = b;
        if (len < total) {
            return {};
        }
        len = 0;
        const std::span<const uint8_t> frame(buf.data(), total);
        if (frame[total - 1] != crc8(frame.subspan(2, total - 3))) {
            return {};
        }
        return frame;
    }
};

}  // namespace ferrox::logic::crsf
