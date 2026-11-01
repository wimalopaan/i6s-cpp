#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "ferrox/logic/afhds2a.hpp"
#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/flight.hpp"
#include "ferrox/logic/flight_timer.hpp"
#include "ferrox/logic/format.hpp"
#include "ferrox/logic/labels.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/glyphs.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/telemetry.hpp"
#include "ferrox/logic/trim.hpp"
#include "ferrox/logic/widgets.hpp"

namespace ferrox::logic::dashboard {

using display::Framebuffer;
using display::kWidth;
using display::Point;
using display::Rect;

inline constexpr uint8_t kPageCount = 5;
inline constexpr uint16_t kKeyPageDown = 1u << 8;
inline constexpr uint16_t kKeyPageUp = 1u << 9;

struct PageState {
    uint8_t page{0};
    uint8_t blink_phase{0};
};

constexpr void next_page(PageState& s) { s.page = static_cast<uint8_t>((s.page + 1) % kPageCount); }

constexpr void prev_page(PageState& s) {
    s.page = s.page == 0 ? static_cast<uint8_t>(kPageCount - 1) : static_cast<uint8_t>(s.page - 1);
}

// Takes newly pressed keys (edge-detected). Up has priority over Down, as in the Rust firmware.
constexpr void handle_keys(PageState& s, const uint16_t newly_pressed) {
    if ((newly_pressed & kKeyPageUp) != 0) {
        prev_page(s);
    } else if ((newly_pressed & kKeyPageDown) != 0) {
        next_page(s);
    }
}

// Session minimums for the telemetry page. Updated from connected samples, never reset (as in Rust).
struct SessionStats {
    bool seen = false;
    uint8_t min_rssi = 100;
    uint16_t min_rx_v_mv = 0xFFFF;
};

constexpr void track_session(SessionStats& s, const afhds2a::Telemetry& t) {
    if (!t.connected) {
        return;
    }
    if (!s.seen) {
        s.seen = true;
        s.min_rssi = t.rssi;
        if (t.rx_voltage_mv > 0) {
            s.min_rx_v_mv = t.rx_voltage_mv;
        }
        return;
    }
    s.min_rssi = std::min(s.min_rssi, t.rssi);
    if (t.rx_voltage_mv > 0 && t.rx_voltage_mv < s.min_rx_v_mv) {
        s.min_rx_v_mv = t.rx_voltage_mv;
    }
}

struct View {
    const input::InputState& in;
    const storage::RadioStorage& storage;
    const trim::Controller& trims;
    const std::array<uint16_t, kNumChannels>& rf_chs;
    bool crsf;
    // AFHDS 2A link state, only meaningful when !crsf.
    afhds2a::Telemetry telem{};
    // Radio is binding, already gated by bind::binding_shown (AFHDS 2A only). Drives the BIND status and footer.
    bool binding = false;
    bool rf_ok = false;
    bool bound = false;
    uint32_t now_ms = 0;
    uint32_t packets_sent = 0;
    SessionStats stats{};
    // Flight timer footer and reset overlay; empty when the timer is off or no overlay is due.
    std::optional<flight_timer::Display> timer{};
    std::optional<flight_timer::Modal> timer_modal{};
    // CRSF link state for page P5, only meaningful when crsf.
    ferrox::logic::crsf::Telemetry crsf_link{};
};

using labels::model_name;
using labels::model_type_label;

/// Which footer the P1 page shows. Same precedence as Rust `gimbals::render_footer`.
enum class FooterKind : uint8_t { Bind, Trim, Timer, Default };

constexpr FooterKind footer_kind(const View& v) {
    if (v.binding) {
        return FooterKind::Bind;
    }
    if (v.trims.last_active) {
        return FooterKind::Trim;
    }
    if (v.timer) {
        return FooterKind::Timer;
    }
    return FooterKind::Default;
}

// Fixed status text; empty when the RSSI value is shown instead. Same order as Rust `status_bar.rs`.
constexpr std::string_view rf_status_fixed(const View& v) {
    if (!v.crsf && !v.rf_ok) {
        return "NO RF";
    }
    if (v.binding) {
        return "BIND";
    }
    if (!v.crsf && v.telem.connected) {
        return {};
    }
    if (v.crsf) {
        return "CRSF";
    }
    return v.bound ? "RF:OK" : "NO RF";
}

constexpr void draw_rf_status(Framebuffer& fb, const View& v) {
    const auto fixed = rf_status_fixed(v);
    if (!fixed.empty()) {
        fb.draw_str_4x6({64, 3}, fixed, false);
        return;
    }
    const auto r = std::min<unsigned>(v.telem.rssi, 100u);
    if (r >= 100u) {
        fb.draw_str_4x6({64, 3}, "R:100", false);
        return;
    }
    const auto digits = format::format_dec2(r);
    format::Text<5> t;
    t.push('R');
    t.push(':');
    for (const char c : digits.view()) {
        t.push(c);
    }
    fb.draw_str_4x6({64, 3}, t.view(), false);
}


constexpr std::string_view afhds_link_line(const View& v) {
    if (v.binding) {
        return "AFHDS2A: BINDING";
    }
    return v.telem.connected ? "AFHDS2A: CONNECTED" : "AFHDS2A: NO LINK";
}

// Telemetry older than 2.5 s counts as stale, as in Rust `check_sensor_status`.
constexpr bool sensor_fresh(const View& v, const uint32_t last_ms) {
    const auto st = telemetry::check_sensor_status({.value = last_ms}, {.value = v.now_ms}, v.telem.connected);
    return st.has_data && !st.stale;
}

constexpr void draw_status_bar(Framebuffer& fb, const storage::RadioStorage& st, const uint16_t battery_mv, const View& v,
                               const uint8_t blink_phase) {
    const auto name = model_name(st.active_model);
    const auto trimmed = format::trim_spaces(name.view());
    if (trimmed.empty()) {
        const auto digits = format::format_dec2(static_cast<unsigned>(st.radio.active_model) + 1u);
        format::Text<3> fallback;
        fallback.push('M');
        for (const char c : digits.view()) {
            fallback.push(c);
        }
        fb.draw_str_6x10({2, 2}, fallback.view(), false);
    } else {
        fb.draw_str_6x10({2, 2}, trimmed, false);
    }

    draw_rf_status(fb, v);

    const auto vbat = format::format_vbat(battery_mv);
    const auto warn_mv = static_cast<uint16_t>(st.radio.vbat_warn_deci * 100u);
    if (battery_mv < warn_mv) {
        if ((blink_phase & 0x10u) != 0) {
            fb.fill_rect({{92, 1}, {22, 8}}, true);
            fb.draw_str_4x6({93, 3}, vbat.view(), true);
        } else {
            glyphs::draw_battery_gauge(fb, {116, 2}, battery_mv, {warn_mv, std::max<uint16_t>(warn_mv, 6000)});
            fb.draw_str_4x6({93, 3}, vbat.view(), false);
        }
    } else {
        const auto max_mv = static_cast<uint16_t>(std::max<unsigned>(6000u, warn_mv + 800u));
        glyphs::draw_battery_gauge(fb, {116, 2}, battery_mv, {warn_mv, max_mv});
        fb.draw_str_4x6({93, 3}, vbat.view(), false);
    }

    fb.draw_hline({0, 11}, kWidth, true);
}

constexpr void draw_stick_row(Framebuffer& fb, const int y, std::string_view label, const Rect gauge,
                              const widgets::Deflection val, const widgets::Trim trim) {
    fb.draw_str_6x10({2, y}, label, false);
    widgets::draw_channel_gauge(fb, gauge, val, trim);
    const auto pct = format::format_percent(static_cast<int16_t>(val.value));
    fb.draw_str_6x10({92, y}, pct.view(), false);
}

constexpr void draw_primary_sticks(Framebuffer& fb, const View& v) {
    const auto& g = v.in.gimbals;
    const auto& t = v.trims.values;
    const auto& radio = v.storage.radio;

    const bool general = static_cast<mixer::ModelType>(v.storage.active_model.model_type) == mixer::ModelType::General;
    if (general) {
        draw_stick_row(fb, 12, "RH", {{16, 13}, {72, 7}}, {g.rh}, {t.roll});
        draw_stick_row(fb, 20, "RV", {{16, 21}, {72, 7}}, {g.rv}, {t.pitch});
        draw_stick_row(fb, 28, "LV", {{16, 29}, {72, 7}}, {g.lv}, {t.throttle});
        draw_stick_row(fb, 36, "LH", {{16, 37}, {72, 7}}, {g.lh}, {t.yaw});
        return;
    }

    const auto controls = flight::flight_controls(g, safety::stick_mode_from(radio.stick_mode));
    draw_stick_row(fb, 12, "A", {{12, 13}, {76, 7}}, {controls.aileron}, {t.roll});
    draw_stick_row(fb, 20, "E", {{12, 21}, {76, 7}}, {controls.elevator}, {t.pitch});

    fb.draw_str_6x10({2, 28}, "T", false);
    const int thr_trim = radio.throttle_trim != 0 ? t.throttle : 0;
    widgets::draw_progress_bar(fb, {{12, 29}, {76, 7}}, {controls.throttle}, {thr_trim});
    const auto thr_pct = format::format_throttle_percent(controls.throttle);
    fb.draw_str_6x10({92, 28}, thr_pct.view(), false);

    draw_stick_row(fb, 36, "R", {{12, 37}, {76, 7}}, {controls.rudder}, {t.yaw});
}

constexpr void draw_fixed_switch_slot(Framebuffer& fb, const Point label_pos, const Point val_pos,
                                      std::string_view label, const SwitchPos pos) {
    fb.draw_str_4x6(label_pos, label, false);
    glyphs::draw_switch_arrow(fb, val_pos, pos);
}

constexpr void draw_default_layout(Framebuffer& fb, const input::InputState& in, const storage::RadioStorage& st) {
    const auto& s = in.switches;
    if (st.radio.ext_switches != 0) {
        draw_fixed_switch_slot(fb, {2, 47}, {7, 46}, "A", s.sa);
        draw_fixed_switch_slot(fb, {13, 47}, {18, 46}, "B", s.sb);
        draw_fixed_switch_slot(fb, {24, 47}, {29, 46}, "C", s.sc);
        draw_fixed_switch_slot(fb, {35, 47}, {40, 46}, "D", s.sd);
        draw_fixed_switch_slot(fb, {46, 47}, {51, 46}, "E", s.se);
        draw_fixed_switch_slot(fb, {57, 47}, {62, 46}, "F", s.sf);
    } else {
        draw_fixed_switch_slot(fb, {2, 47}, {9, 46}, "A", s.sa);
        draw_fixed_switch_slot(fb, {20, 47}, {27, 46}, "B", s.sb);
        draw_fixed_switch_slot(fb, {38, 47}, {45, 46}, "C", s.sc);
        draw_fixed_switch_slot(fb, {56, 47}, {63, 46}, "D", s.sd);
    }
    fb.draw_str_4x6({74, 47}, "VR", false);
    widgets::draw_split_pot_bar(fb, {84, 46}, 42, {in.pots.vr1}, {in.pots.vr2});
}

struct SwitchSlot {
    std::string_view name{};
    SwitchPos pos{SwitchPos::Up};
    bool six_pos{false};
    uint8_t mode_num{0};
};

struct AdaptiveLayout {
    std::array<SwitchSlot, 16> switches{};
    std::size_t switch_count{0};
    std::array<int16_t, 10> pots{};
    std::size_t pot_count{0};
};

constexpr std::array<std::string_view, 10> kAdaptiveNames{"A", "B", "C", "D", "1", "2", "3", "4", "5", "6"};

constexpr void push_switch(AdaptiveLayout& l, const SwitchSlot& s) {
    if (l.switch_count < l.switches.size()) {
        l.switches[l.switch_count++] = s;
    }
}

constexpr void push_pot(AdaptiveLayout& l, const int16_t v) {
    if (l.pot_count < l.pots.size()) {
        l.pots[l.pot_count++] = v;
    }
}

// Switch position of a two/three-position channel. Channels 4 and 5 are analog pots read as three-position.
constexpr SwitchPos switch_for_channel(const input::InputState& in, const std::size_t ch) {
    const auto& s = in.switches;
    switch (ch) {
        case 0: return s.sa;
        case 1: return s.sb;
        case 2: return s.sc;
        case 3: return s.sd;
        case 6: return s.swi;
        case 7: return s.swj;
        case 8: return s.swk;
        case 9: return s.swl;
        default: {
            const int16_t p = in.aux_pots[ch];
            if (p < -333) {
                return SwitchPos::Up;
            }
            return p < 333 ? SwitchPos::Mid : SwitchPos::Down;
        }
    }
}

constexpr AdaptiveLayout build_adaptive_layout(const input::InputState& in, const storage::RadioStorage& st) {
    AdaptiveLayout out{};
    const std::size_t max_aux = st.radio.ext_adc != 0 ? 10 : 6;
    for (std::size_t ch = 0; ch < max_aux; ++ch) {
        const auto mode = input::resolve_mode(ch, st.radio.adc_modes[ch]);
        switch (mode) {
            case input::AdcMode::TwoPos:
            case input::AdcMode::ThreePos:
            case input::AdcMode::InstantTrim:
                push_switch(out, {kAdaptiveNames[ch], switch_for_channel(in, ch), false, 0});
                break;
            case input::AdcMode::SixPos: {
                const uint16_t raw = in.raw[input::aux_index_to_raw(ch)];
                push_switch(out, {kAdaptiveNames[ch], SwitchPos::Mid, true, input::decode_switch_6pos(raw)});
                break;
            }
            case input::AdcMode::Pot:
            case input::AdcMode::PotDetent:
                push_pot(out, in.aux_pots[ch]);
                break;
            case input::AdcMode::Default:
                break;
        }
    }
    if (st.radio.ext_switches != 0) {
        push_switch(out, {"E", in.switches.se, false, 0});
        push_switch(out, {"F", in.switches.sf, false, 0});
    }
    return out;
}

struct RowMetrics {
    int spacing{0};
    bool compact{false};
    int pot_start_x{0};
    int pot_w{0};
};

struct SwitchCount {
    std::size_t value;
    friend constexpr bool operator==(const SwitchCount&, const SwitchCount&) = default;
};

struct PotCount {
    std::size_t value;
    friend constexpr bool operator==(const PotCount&, const PotCount&) = default;
};

constexpr RowMetrics adaptive_metrics(const SwitchCount sw_count_arg, const PotCount pot_count_arg) {
    const std::size_t sw_count = sw_count_arg.value;
    const std::size_t pot_count = pot_count_arg.value;
    const int sw = static_cast<int>(sw_count);
    if (pot_count == 0) {
        const int spacing = sw <= 1 ? 124 : std::min(124 / sw, 14);
        return {spacing, spacing < 10, 126, 0};
    }
    if (sw == 0) {
        return {0, false, 2, 124};
    }

    const int min_pot_w = pot_count == 1 ? 24 : pot_count == 2 ? (sw >= 12 ? 24 : 28) : 36;
    const bool compact = sw * 10 + min_pot_w + 3 > 124;
    const int slot_w = compact ? 8 : 10;
    const int max_sw_w = 124 - min_pot_w - 3;
    const int natural_sw_w = sw * (compact ? 8 : 11);
    const int actual_sw_w = std::max(std::min(natural_sw_w, max_sw_w), sw * (slot_w - 1));
    const int spacing = sw <= 1 ? actual_sw_w : actual_sw_w / sw;

    const int last_sw_end = 2 + (sw - 1) * spacing + (compact ? 7 : 10);
    const int pot_start_x = std::max(last_sw_end + 3, 2 + actual_sw_w + 3);
    const int pot_w = std::max(126 - pot_start_x, 0);
    return {spacing, compact, pot_start_x, pot_w};
}

constexpr void draw_adaptive_slot(Framebuffer& fb, const int x, const bool compact, const SwitchSlot& s) {
    fb.draw_str_4x6({x + 1, 48}, s.name, false);
    const int glyph_x = x + 5;
    if (s.six_pos) {
        const auto digit = static_cast<char>('0' + std::clamp<int>(s.mode_num, 1, 6));
        fb.draw_str_4x6({glyph_x, 47}, std::string_view(&digit, 1), false);
    } else if (compact) {
        glyphs::draw_switch_arrow_compact(fb, {glyph_x, 46}, s.pos);
    } else {
        glyphs::draw_switch_arrow(fb, {glyph_x, 46}, s.pos);
    }
}

constexpr void draw_adaptive_layout(Framebuffer& fb, const input::InputState& in, const storage::RadioStorage& st) {
    const auto layout = build_adaptive_layout(in, st);
    const auto m = adaptive_metrics({layout.switch_count}, {layout.pot_count});

    for (std::size_t i = 0; i < layout.switch_count; ++i) {
        const int x = 2 + static_cast<int>(i) * m.spacing;
        draw_adaptive_slot(fb, x, m.compact, layout.switches[i]);
    }

    if (layout.pot_count == 0 || m.pot_w <= 0) {
        return;
    }
    if (layout.switch_count > 0) {
        const int div_x = m.pot_start_x - 2;
        if (div_x > 2 && div_x < kWidth) {
            fb.draw_vline({div_x, 46}, 8, true);
        }
    }

    if (layout.pot_count == 1) {
        widgets::draw_single_pot_bar(fb, {m.pot_start_x, 46}, {m.pot_w}, {layout.pots[0]});
    } else if (layout.pot_count == 2) {
        widgets::draw_split_pot_bar(fb, {m.pot_start_x, 46}, m.pot_w, {layout.pots[0]}, {layout.pots[1]});
    } else {
        widgets::draw_multi_pot_bar(fb, {m.pot_start_x, 45}, m.pot_w,
                                    std::span<const int16_t>(layout.pots.data(), layout.pot_count));
    }
}

// Shown instead of the page footer while binding. Same text as Rust `widgets::draw_footer` calls in each page.
constexpr void draw_bind_footer(Framebuffer& fb) { widgets::draw_footer(fb, "[ESC] Finish Bind"); }

// Footer of the pages without trim or timer overlays: bind text while binding, else the page label.
constexpr void draw_page_footer(Framebuffer& fb, const View& v, std::string_view page, std::string_view title) {
    if (v.binding) {
        draw_bind_footer(fb);
    } else {
        widgets::draw_footer_split(fb, page, title);
    }
}

constexpr void draw_gimbal_page(Framebuffer& fb, const View& v, const uint8_t blink_phase) {
    draw_primary_sticks(fb, v);

    const auto& st = v.storage;
    const bool has_custom_modes = std::any_of(st.radio.adc_modes.begin(), st.radio.adc_modes.end(),
                                              [](uint8_t m) { return m != 0; });
    if (!has_custom_modes && st.radio.ext_adc == 0) {
        draw_default_layout(fb, v.in, st);
    } else {
        draw_adaptive_layout(fb, v.in, st);
    }

    switch (footer_kind(v)) {
        case FooterKind::Bind: draw_bind_footer(fb); break;
        case FooterKind::Trim: {
            const auto axis = *v.trims.last_active;
            const auto trim_text = format::format_trim(trim::axis_letter(axis), trim::value_of(v.trims.values, axis));
            widgets::draw_footer_split(fb, trim_text.view(), "Hold OK:Menu");
            break;
        }
        case FooterKind::Timer: {
            const auto timer_text = format::format_timer(v.timer->value.value, v.timer->expired);
            const bool blink_on = (blink_phase & 0x08u) != 0;
            widgets::draw_footer_three(fb, "P1/5", timer_text.view(), "Hold OK:Menu", v.timer->expired && blink_on);
            break;
        }
        case FooterKind::Default: widgets::draw_footer_split(fb, "P1/5", "Hold OK:Menu"); break;
    }
}

constexpr void draw_channel_page(Framebuffer& fb, const View& v, const bool second_part) {
    const auto& chs = v.rf_chs;
    const int base_ch = second_part ? 10 : 0;
    const int rows_per_col = second_part ? 4 : 5;

    for (int col = 0; col < 2; ++col) {
        const int col_x = col == 0 ? 2 : 66;
        const int col_start = base_ch + col * rows_per_col;
        for (int row = 0; row < rows_per_col; ++row) {
            const int ch = col_start + row;
            if (ch >= static_cast<int>(kNumChannels)) {
                break;
            }
            const int y = 13 + row * 8;

            const int ch_num = ch + 1;
            std::array<char, 3> label{' ', ' ', ':'};
            if (ch_num >= 10) {
                label[0] = static_cast<char>('0' + ch_num / 10);
                label[1] = static_cast<char>('0' + ch_num % 10);
            } else {
                label[1] = static_cast<char>('0' + ch_num);
            }
            fb.draw_str_4x6({col_x, y}, std::string_view(label.data(), label.size()), false);

            const auto us = std::clamp<uint16_t>(chs[static_cast<std::size_t>(ch)], kChannelMinUs, kChannelMaxUs);
            const int fill = std::min((us - kChannelMinUs) * 24 / kChannelSpanUs, 24);
            widgets::draw_bar_gauge(fb, {{col_x + 14, y + 1}, {26, 5}}, fill);

            const auto digits = widgets::format_u16_4(us);
            fb.draw_str_4x6({col_x + 42, y}, std::string_view(digits.data(), digits.size()), false);
        }
    }

    fb.draw_vline({64, 12}, second_part ? 33 : 41, true);
    if (second_part) {
        draw_page_footer(fb, v, "P3/5", "CH 11-18 MONITOR");
    } else {
        draw_page_footer(fb, v, "P2/5", "CH 1-10 MONITOR");
    }
}

constexpr void draw_model_page(Framebuffer& fb, const View& v) {
    const auto& st = v.storage;
    const auto& m = st.active_model;
    const auto name = model_name(m);
    fb.draw_str_6x10({2, 14}, format::trim_spaces(name.view()), false);
    fb.draw_str_6x10({74, 14}, model_type_label(m.model_type), false);

    fb.draw_str_6x10({2, 24}, "RxID:", false);
    const auto rx_id = format::format_hex32(m.rx_id);
    fb.draw_str_6x10({36, 24}, rx_id.view(), false);

    fb.draw_str_6x10({2, 34}, "TCrv:", false);
    fb.draw_str_6x10({36, 34}, m.thr_curve_pts == 9 ? std::string_view("9-PT") : std::string_view("5-PT"), false);
    fb.draw_str_6x10({74, 34}, m.thr_curve_smooth != 0 ? std::string_view("SMOOTH") : std::string_view("LINEAR"),
                     false);

    // CRSF telemetry is not ported yet, so its line always shows DISCONNECTED.
    fb.draw_str_6x10({2, 44}, v.crsf ? std::string_view("CRSF: DISCONNECTED") : afhds_link_line(v), false);

    draw_page_footer(fb, v, "P4/5", "MODEL DASHBOARD");
}

// AFHDS 2A sensor page, same layout as Rust `telemetry.rs` (AFHDS branch). CRSF is not ported yet.
constexpr void draw_crsf_telemetry_page(Framebuffer& fb, const View& v) {
    const auto d = telemetry::crsf_diag(v.crsf_link, {.value = v.now_ms});
    fb.draw_vline({64, 12}, 42, true);

    fb.draw_str_6x10({2, 14}, d.lq.view(), false);
    fb.draw_str_6x10({2, 24}, d.rssi.view(), false);
    fb.draw_str_6x10({2, 34}, d.snr.view(), false);
    fb.draw_str_6x10({2, 44}, d.ant.view(), false);

    fb.draw_str_6x10({66, 14}, d.pwr.view(), false);
    fb.draw_str_6x10({66, 24}, "RATE:", false);
    fb.draw_str_6x10({98, 24}, d.rate, false);
    fb.draw_str_6x10({66, 34}, d.bat_label.view(), false);
    fb.draw_str_6x10({92, 34}, d.bat.view(), false);
    fb.draw_str_6x10({66, 44}, "CAP:", false);
    fb.draw_str_6x10({92, 44}, d.cap.view(), false);

    widgets::draw_footer_split(fb, "P5/5", "CRSF LINK DIAG");
}

// Sensor tile of the custom page, same layout as Rust `render_tile`: label, '*' when stale, value below.
constexpr void draw_sensor_tile(Framebuffer& fb, const telemetry::TileOrigin o, const telemetry::Tile& t) {
    if (t.slot_off) {
        fb.draw_str_4x6({o.x, o.y}, "SLOT OFF", false);
        return;
    }
    fb.draw_str_4x6({o.x, o.y}, t.label, false);
    if (t.stale) {
        fb.draw_str_4x6({o.x + 50, o.y}, "*", false);
    }
    fb.draw_str_6x10({o.x, o.y + 7}, t.value.view(), false);
}

constexpr telemetry::Sources telem_sources(const View& v, std::string_view timer) {
    return {.source = v.crsf ? telemetry::Source::Crsf : telemetry::Source::Afhds2a,
            .crsf = v.crsf_link,
            .afhds = v.telem,
            .tx_battery = {.value = v.in.battery_mv},
            .packets_sent = {.value = v.packets_sent},
            .timer = timer};
}

// Custom 2x2 sensor page, drawn when the model's slots are not the default ones.
constexpr void draw_custom_telemetry_page(Framebuffer& fb, const View& v) {
    const auto timer_text =
        v.timer ? format::format_timer(v.timer->value.value, v.timer->expired) : format::Text<6>{};
    const auto sources = telem_sources(v, timer_text.view());
    const telemetry::NowMs now{v.now_ms};
    const auto& slots = v.storage.active_model.telem_slots;
    for (std::size_t i = 0; i < slots.size(); ++i) {
        draw_sensor_tile(fb, telemetry::tile_origin(i), telemetry::sensor_tile({slots[i]}, now, sources));
    }

    fb.draw_vline({64, 12}, 42, true);
    fb.draw_hline({2, 31}, 124, true);
    draw_page_footer(fb, v, "P5/5", "CUSTOM TELEM");
}

constexpr void draw_telemetry_page(Framebuffer& fb, const View& v) {
    if (telemetry::is_custom_telem_layout(v.storage.active_model.telem_slots)) {
        draw_custom_telemetry_page(fb, v);
        return;
    }
    if (v.crsf) {
        draw_crsf_telemetry_page(fb, v);
        return;
    }

    fb.draw_vline({64, 12}, 42, true);

    const bool rssi_ok = sensor_fresh(v, v.telem.last_rx_voltage_ms);
    fb.draw_str_6x10({2, 14}, "RSSI:", false);
    if (rssi_ok) {
        const auto r = std::min<unsigned>(v.telem.rssi, 100u);
        const auto digits = format::format_dec2(r);
        format::Text<4> t;
        for (const char c : digits.view()) {
            t.push(c);
        }
        t.push('%');
        fb.draw_str_6x10({44, 14}, t.view(), false);
    } else {
        fb.draw_str_6x10({44, 14}, "--%", false);
    }

    fb.draw_str_6x10({2, 24}, v.telem.connected ? "LINK: OK" : "LINK: DISC", false);

    format::Text<5> tx_count;
    format::detail::push_uint(tx_count, v.packets_sent);
    fb.draw_str_6x10({2, 34}, "TX:", false);
    fb.draw_str_6x10({24, 34}, tx_count.view(), false);

    format::Text<5> rx_count;
    format::detail::push_uint(rx_count, v.telem.packets_received);
    fb.draw_str_6x10({2, 44}, "RX:", false);
    fb.draw_str_6x10({24, 44}, rx_count.view(), false);

    const bool rx_fresh = sensor_fresh(v, v.telem.last_rx_voltage_ms) && v.telem.rx_voltage_mv > 0;
    const auto rx_text = format::format_vbat(v.telem.rx_voltage_mv);
    fb.draw_str_6x10({66, 14}, "RX:", false);
    fb.draw_str_6x10({88, 14}, rx_fresh ? rx_text.view() : std::string_view("----"), false);

    const auto tx_text = format::format_vbat(v.in.battery_mv);
    fb.draw_str_6x10({66, 24}, "TX:", false);
    fb.draw_str_6x10({88, 24}, tx_text.view(), false);

    fb.draw_str_6x10({66, 34}, "mRSS:", false);
    if (v.stats.seen) {
        const auto digits = format::format_dec2(std::min<unsigned>(v.stats.min_rssi, 100u));
        format::Text<4> t;
        for (const char c : digits.view()) {
            t.push(c);
        }
        t.push('%');
        fb.draw_str_6x10({98, 34}, t.view(), false);
    } else {
        fb.draw_str_6x10({98, 34}, " --%", false);
    }

    fb.draw_str_6x10({66, 44}, "mRX:", false);
    const bool min_rx_ok = v.stats.seen && v.stats.min_rx_v_mv != 0xFFFF && v.stats.min_rx_v_mv > 0;
    const auto min_rx_text = format::format_vbat(v.stats.min_rx_v_mv);
    fb.draw_str_6x10({92, 44}, min_rx_ok ? min_rx_text.view() : std::string_view("----"), false);

    draw_page_footer(fb, v, "P5/5", "TELEMETRY SENSORS");
}

// Centered box shown after a timer reset: a hold progress bar, then a confirmation toast.
constexpr void draw_timer_modal(Framebuffer& fb, const flight_timer::Modal& m) {
    const Rect box{{18, 19}, {92, 26}};
    fb.fill_rect(box, false);
    fb.draw_rect(box, true);

    if (m.reset_done) {
        fb.draw_str_6x10({28, 28}, "TIMER RESET!", false);
        return;
    }
    fb.draw_str_6x10({31, 22}, "RESET TIMER", false);
    fb.draw_rect({{27, 33}, {74, 6}}, true);
    const int fill_w = std::min(m.progress_pct * 70 / 100, 70);
    if (fill_w > 0) {
        fb.fill_rect({{29, 35}, {fill_w, 2}}, true);
    }
}

// Renders the status bar and the current page, then advances the blink phase.
constexpr void render(Framebuffer& fb, PageState& s, const View& v) {
    draw_status_bar(fb, v.storage, v.in.battery_mv, v, s.blink_phase);

    switch (s.page) {
        case 0: draw_gimbal_page(fb, v, s.blink_phase); break;
        case 1: draw_channel_page(fb, v, false); break;
        case 2: draw_channel_page(fb, v, true); break;
        case 3: draw_model_page(fb, v); break;
        default: draw_telemetry_page(fb, v); break;
    }

    if (v.timer_modal) {
        draw_timer_modal(fb, *v.timer_modal);
    }

    s.blink_phase = static_cast<uint8_t>(s.blink_phase + 1);
}

constexpr void draw_splash(Framebuffer& fb) {
    fb.clear();
    glyphs::draw_ferrox_logo(fb, {50, 3});
    fb.draw_str_6x10({37, 34}, "FERROX-RC", false);
    fb.draw_str_4x6({38, 45}, "flysky-i6x-rs", false);
}

}  // namespace ferrox::logic::dashboard
