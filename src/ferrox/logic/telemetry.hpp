#pragma once

// Pure telemetry display logic (Rust `ui/dashboard/pages/telemetry.rs`): sensor freshness,
// RF rate names, and the text and stale state of the 22 telemetry sensor tiles for both the
// AFHDS2A and CRSF sources. Drawing and slot storage are in the hardware layer.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <string_view>

#include "afhds2a.hpp"
#include "crsf.hpp"
#include "format.hpp"

namespace ferrox::logic::telemetry {

// Strong types so timestamps, sources and counters cannot be mixed up at a call site.
struct LastSeenMs {
    uint32_t value = 0;  // 0 means the sensor was never received
    constexpr bool operator==(const LastSeenMs&) const = default;
};

struct NowMs {
    uint32_t value = 0;
    constexpr bool operator==(const NowMs&) const = default;
};

struct SensorStatus {
    bool has_data = false;
    bool stale = false;
    constexpr bool operator==(const SensorStatus&) const = default;
};

struct TxBatteryMv {
    uint16_t value = 0;
    constexpr bool operator==(const TxBatteryMv&) const = default;
};

struct PacketCount {
    uint32_t value = 0;
    constexpr bool operator==(const PacketCount&) const = default;
};

struct SlotId {
    uint8_t value = 0;
    constexpr bool operator==(const SlotId&) const = default;
};

enum class Source : uint8_t { Afhds2a, Crsf };

constexpr SensorStatus check_sensor_status(LastSeenMs last, NowMs now, bool connected) {
    if (last.value == 0) {
        return {};
    }
    const uint32_t elapsed = now.value - last.value;  // unsigned wrap-around is intended
    return {.has_data = true, .stale = !connected || elapsed >= 2500u};
}

constexpr std::string_view rf_mode_name(uint8_t rf_mode) {
    constexpr std::string_view names[] = {
        "4Hz",  "25Hz", "50Hz", "100Hz", "100F", "150Hz", "200Hz",
        "250Hz", "333Hz", "500Hz", "D250", "D500", "F500", "F1000",
    };
    return rf_mode < std::size(names) ? names[rf_mode] : std::string_view{"---"};
}

// Source data for the tiles. `timer` is empty when no timer is shown ("00:00" is drawn).
struct Sources {
    Source source;
    const crsf::Telemetry& crsf;
    const afhds2a::Telemetry& afhds;
    TxBatteryMv tx_battery;
    PacketCount packets_sent;
    std::string_view timer;
};

struct Tile {
    std::string_view label;
    format::Text<12> value{};
    bool stale = false;  // label and value carry a '*' when true
    bool slot_off = false;  // only the "SLOT OFF" text is drawn
};

namespace detail {

constexpr format::Text<12> to_text(std::string_view s) {
    format::Text<12> t;
    for (const char c : s) {
        t.push(c);
    }
    return t;
}

template <std::size_t N>
constexpr format::Text<12> to_text(const format::Text<N>& src) {
    return to_text(src.view());
}

// Same as Rust `append_stale`: keep at most 11 characters, then append '*'.
constexpr format::Text<12> mark_stale(format::Text<12> v) {
    if (v.len > 11) {
        v.len = 11;
    }
    v.push('*');
    return v;
}

constexpr format::Text<12> plain(int32_t value) {
    return to_text(format::format_fixed({.value = value}));
}

}  // namespace detail

constexpr Tile sensor_tile(SlotId slot, NowMs now, const Sources& s) {
    using detail::mark_stale;
    using detail::plain;
    using detail::to_text;
    const bool crsf = s.source == Source::Crsf;
    const crsf::Telemetry& ct = s.crsf;
    const afhds2a::Telemetry& at = s.afhds;

    // Builds a tile whose value is shown with '*' when the status is stale.
    const auto tile = [](std::string_view label, format::Text<12> value, SensorStatus st) {
        return Tile{.label = label, .value = st.stale ? mark_stale(value) : value, .stale = st.stale};
    };
    const auto fixed = [](format::FixedFormat f) { return to_text(format::format_fixed(f)); };
    constexpr auto force = format::SignMode::Force;

    switch (slot.value) {
        case 0: {
            const SensorStatus st = crsf ? check_sensor_status(LastSeenMs{ct.last_vbat_ms}, now, ct.connected)
                                         : check_sensor_status(LastSeenMs{at.last_rx_voltage_ms}, now, at.connected);
            const uint16_t mv = crsf ? ct.rx_battery_mv : at.rx_voltage_mv;
            const auto v = st.has_data && mv > 0 ? to_text(format::format_vbat(mv)) : to_text(std::string_view{"----"});
            return tile("RX BAT", v, st);
        }
        case 1: {
            if (crsf) {
                const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
                const auto v = st.has_data ? fixed({.value = std::min<int>(ct.uplink_link_quality, 100), .pad_digits = 3, .suffix = "%"})
                                           : to_text(std::string_view{"---%"});
                return tile("LINK Q", v, st);
            }
            const SensorStatus st = check_sensor_status(LastSeenMs{at.last_rx_voltage_ms}, now, at.connected);
            const auto v = st.has_data ? fixed({.value = std::min<int>(at.rssi, 100), .pad_digits = 3, .suffix = "%"})
                                       : to_text(std::string_view{"---%"});
            return tile("RSSI", v, st);
        }
        case 2:
            return Tile{.label = "TX BAT", .value = to_text(format::format_vbat(s.tx_battery.value))};
        case 3: {
            if (crsf) {
                const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_vbat_ms}, now, ct.connected);
                const auto v = st.has_data && ct.rx_current_ma > 0
                                   ? fixed({.value = ct.rx_current_ma / 10, .decimals = 1, .suffix = "A"})
                                   : to_text(std::string_view{"0.0A"});
                return tile("CURRENT", v, st);
            }
            return Tile{.label = "LINK", .value = to_text(std::string_view{at.connected ? "OK" : "DISC"})};
        }
        case 4: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_vbat_ms}, now, ct.connected);
            const auto v = crsf && st.has_data && ct.rx_capacity_mah > 0
                               ? fixed({.value = static_cast<int32_t>(std::min<uint32_t>(ct.rx_capacity_mah, 99999u))})
                               : to_text(std::string_view{"0"});
            return tile("CAP mAh", v, st);
        }
        case 5: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
            const auto v = crsf && st.has_data && ct.tx_power_mw > 0
                               ? fixed({.value = ct.tx_power_mw})
                               : to_text(std::string_view{"---"});
            return tile("PWR mW", v, st);
        }
        case 6: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? fixed({.value = ct.uplink_snr, .pad_digits = 2, .pad_zero = true, .sign = force, .suffix = "dB"})
                                               : to_text(std::string_view{"--dB"});
            return tile("SNR dB", v, st);
        }
        case 7: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? to_text(rf_mode_name(ct.rf_mode)) : to_text(std::string_view{"---"});
            return tile("RATE", v, st);
        }
        case 8: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? to_text(std::string_view{ct.active_antenna == 0 ? "ANT 1" : "ANT 2"})
                                               : to_text(std::string_view{"ANT -"});
            return tile("ANTENNA", v, st);
        }
        case 9:
            return Tile{.label = "PKTS TX", .value = plain(static_cast<int32_t>(s.packets_sent.value))};
        case 10:
            return Tile{.label = "PKTS RX", .value = plain(static_cast<int32_t>(at.packets_received))};
        case 11:
            return Tile{.label = "TIMER", .value = to_text(s.timer.empty() ? std::string_view{"00:00"} : s.timer)};
        // Deliberate deviation from Rust: under CRSF these tiles use CRSF data (Rust reads AFHDS fields for both).
        case 12: {
            const SensorStatus st = crsf ? check_sensor_status(LastSeenMs{ct.last_vspd_ms}, now, ct.connected)
                                         : check_sensor_status(LastSeenMs{at.last_vspd_ms}, now, at.connected);
            const int16_t cms = crsf ? ct.vspeed_cms : at.vspeed_cms;
            const auto v = st.has_data ? fixed({.value = cms / 10, .decimals = 1, .sign = force, .suffix = "m/s"})
                                       : to_text(std::string_view{"---m/s"});
            return tile("VSPD", v, st);
        }
        case 13: {
            const SensorStatus st = crsf ? check_sensor_status(LastSeenMs{ct.last_alt_ms}, now, ct.connected)
                                         : check_sensor_status(LastSeenMs{at.last_alt_ms}, now, at.connected);
            const int16_t m = crsf ? ct.baro_alt_m : at.alt_m;
            const auto v = st.has_data ? fixed({.value = m, .suffix = "m"}) : to_text(std::string_view{"---m"});
            return tile("ALT", v, st);
        }
        case 14: {
            if (crsf) {
                const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_cells_ms}, now, ct.connected);
                if (st.has_data && ct.cell_count > 0 && ct.cell_min_mv > 0) {
                    format::Text<12> v;
                    if (ct.cell_count > 1 && ct.cell_count < 10) {
                        v.push(static_cast<char>('0' + ct.cell_count));
                        v.push('S');
                        v.push(' ');
                    }
                    const auto cell = format::format_fixed({.value = ct.cell_min_mv / 10,
                                                            .decimals = 2,
                                                            .suffix = "V"});
                    for (const char c : cell.view()) {
                        v.push(c);
                    }
                    return tile("CELLS", v, st);
                }
                if (st.has_data && ct.rx_battery_mv > 0) {
                    return tile("CELLS", to_text(format::format_vbat(ct.rx_battery_mv)), st);
                }
                return Tile{.label = "CELLS", .value = to_text(std::string_view{"---V"})};
            }
            const SensorStatus st = check_sensor_status(LastSeenMs{at.last_rx_voltage_ms}, now, at.connected);
            if (st.has_data && at.rx_voltage_mv > 0) {
                return tile("CELLS", to_text(format::format_vbat(at.rx_voltage_mv)), st);
            }
            return Tile{.label = "CELLS", .value = to_text(std::string_view{"---V"})};
        }
        case 15: {
            const SensorStatus st = crsf ? check_sensor_status(LastSeenMs{ct.last_rpm_ms}, now, ct.connected)
                                         : check_sensor_status(LastSeenMs{at.last_rpm_ms}, now, at.connected);
            const auto rpm = static_cast<uint32_t>(crsf ? ct.rpm : at.rpm);
            const auto v = !st.has_data ? to_text(std::string_view{"----"})
                           : rpm >= 100000u ? fixed({.value = static_cast<int32_t>(rpm / 1000u), .suffix = "k"})
                                            : fixed({.value = static_cast<int32_t>(rpm)});
            return tile("RPM", v, st);
        }
        case 16: {
            const SensorStatus st = crsf ? check_sensor_status(LastSeenMs{ct.last_temp_ms}, now, ct.connected)
                                         : check_sensor_status(LastSeenMs{at.last_temp_ms}, now, at.connected);
            const int16_t t = crsf ? ct.temp_c_x10 : at.temp_c_x10;
            const auto v = st.has_data ? fixed({.value = t, .decimals = 1, .suffix = "C"}) : to_text(std::string_view{"---C"});
            return tile("TEMP", v, st);
        }
        case 17: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_gps_ms}, now, ct.connected);
            const auto v = crsf && st.has_data
                               ? fixed({.value = ct.gps_groundspeed_kmh_x10 / 10, .suffix = "kmh"})
                               : to_text(std::string_view{"---km"});
            return tile("GSPD", v, st);
        }
        case 18: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_gps_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? fixed({.value = ct.gps_satellites})
                                               : to_text(std::string_view{"--"});
            return tile("SATS", v, st);
        }
        case 19: {
            const uint32_t hdg_last = ct.last_gps_ms > 0 ? ct.last_gps_ms : ct.last_att_ms;
            const SensorStatus st = check_sensor_status(LastSeenMs{hdg_last}, now, ct.connected);
            format::Text<12> v = to_text(std::string_view{"---"});
            if (crsf && st.has_data) {
                const int32_t deg = ct.last_gps_ms > 0
                                        ? static_cast<int32_t>(ct.gps_heading_cdeg / 100)
                                        : ((ct.yaw_cdeg / 10) % 360 + 360) % 360;
                v = fixed({.value = deg, .suffix = "D"});
            }
            return tile("HDG", v, st);
        }
        case 20: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_att_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? fixed({.value = ct.pitch_cdeg / 10, .sign = force, .suffix = "D"})
                                               : to_text(std::string_view{"---"});
            return tile("PITCH", v, st);
        }
        case 21: {
            const SensorStatus st = check_sensor_status(LastSeenMs{ct.last_att_ms}, now, ct.connected);
            const auto v = crsf && st.has_data ? fixed({.value = ct.roll_cdeg / 10, .sign = force, .suffix = "D"})
                                               : to_text(std::string_view{"---"});
            return tile("ROLL", v, st);
        }
        default:
            return Tile{.label = "SLOT OFF", .slot_off = true};
    }
}

// Text of the CRSF link diagnostics page (Rust `draw_telemetry_page`, CRSF branch). Each stat is the
// full drawn text, including its label prefix and '*' stale mark. Placeholders are used without data.
struct CrsfDiag {
    format::Text<12> lq{};
    format::Text<12> rssi{};
    format::Text<12> snr{};
    format::Text<12> ant{};
    format::Text<12> pwr{};
    std::string_view rate{};
    format::Text<4> bat_label{};
    format::Text<12> bat{};
    format::Text<12> cap{};
};

namespace detail {

constexpr format::Text<12> with_prefix(std::string_view prefix, const format::Text<12>& body) {
    format::Text<12> t = to_text(prefix);
    for (const char c : body.view()) {
        t.push(c);
    }
    return t;
}

constexpr format::Text<12> diag_stat(bool has_data, bool stale, const format::Text<12>& value,
                                     std::string_view placeholder) {
    if (!has_data) {
        return to_text(placeholder);
    }
    return stale ? mark_stale(value) : value;
}

}  // namespace detail

constexpr CrsfDiag crsf_diag(const crsf::Telemetry& ct, NowMs now) {
    using detail::diag_stat;
    using detail::to_text;
    using detail::with_prefix;
    constexpr auto force = format::SignMode::Force;
    const auto fixed = [](format::FixedFormat f) { return to_text(format::format_fixed(f)); };

    const SensorStatus link = check_sensor_status(LastSeenMs{ct.last_link_ms}, now, ct.connected);
    const SensorStatus vbat = check_sensor_status(LastSeenMs{ct.last_vbat_ms}, now, ct.connected);

    CrsfDiag d;
    d.lq = diag_stat(link.has_data, link.stale,
                     with_prefix("LQ:", fixed({.value = std::min<int32_t>(ct.uplink_link_quality, 100),
                                               .pad_digits = 3,
                                               .suffix = "%"})),
                     "LQ: ---%");
    d.rssi = diag_stat(link.has_data, link.stale,
                       with_prefix("RS: -", fixed({.value = std::abs(static_cast<int32_t>(ct.uplink_rssi_1)),
                                                   .pad_digits = 3,
                                                   .pad_zero = true,
                                                   .suffix = "dB"})),
                       "RS: ---dB");
    d.snr = diag_stat(link.has_data, link.stale,
                      with_prefix("SNR:", fixed({.value = ct.uplink_snr,
                                                 .pad_digits = 2,
                                                 .pad_zero = true,
                                                 .sign = force,
                                                 .suffix = "dB"})),
                      "SNR: --dB");
    // The antenna label is shown without the stale mark, as in Rust.
    d.ant = to_text(std::string_view{!ct.connected ? "ANT: -" : ct.active_antenna == 0 ? "ANT: 1" : "ANT: 2"});
    d.pwr = diag_stat(link.has_data && ct.tx_power_mw > 0, link.stale,
                      with_prefix("PWR:", fixed({.value = ct.tx_power_mw, .pad_digits = 4, .suffix = "mW"})),
                      "PWR: ---");
    d.rate = link.has_data ? rf_mode_name(ct.rf_mode) : std::string_view{"---"};

    if (vbat.has_data && ct.cell_count > 1 && ct.cell_count < 10) {
        d.bat_label.push(static_cast<char>('0' + ct.cell_count));
        d.bat_label.push('S');
        d.bat_label.push(':');
        d.bat_label.push(' ');
    } else {
        for (const char c : std::string_view{"BAT:"}) {
            d.bat_label.push(c);
        }
    }
    d.bat = diag_stat(vbat.has_data && ct.rx_battery_mv > 0, vbat.stale,
                      to_text(format::format_vbat(ct.rx_battery_mv)), "----");
    d.cap = diag_stat(vbat.has_data && ct.rx_capacity_mah > 0, vbat.stale,
                      fixed({.value = static_cast<int32_t>(std::min<uint32_t>(ct.rx_capacity_mah, 99999u)),
                             .pad_digits = 5}),
                      "----");
    return d;
}

}  // namespace ferrox::logic::telemetry
