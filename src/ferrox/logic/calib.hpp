#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/widgets.hpp"

namespace ferrox::logic::calib {

using display::Framebuffer;

inline constexpr uint16_t kKeyOk = 1u << 10;
inline constexpr uint16_t kKeyCancel = 1u << 11;
inline constexpr uint16_t kStickMinSpanHall = 200;
inline constexpr uint16_t kStickMinSpanPot = 250;
inline constexpr uint16_t kPotMovedSpan = 400;
inline constexpr uint32_t kPotTargetSpan = 900;
inline constexpr uint16_t kCompleteMs = 1200;
inline constexpr uint8_t kModelTypeGeneral = 4;
inline constexpr std::size_t kSticks = 4;
inline constexpr std::size_t kAuxSlots = input::kAuxCount;

inline constexpr std::array<std::string_view, kAuxSlots> kPotNames{"A", "B", "C", "D", "V1", "V2", "V3", "V4", "V5", "V6"};

enum class Step : uint8_t { Inactive, Center, Limits, Complete };

// Sticks and pots have separate arrays, so a pot never overwrites a stick's endpoints.
struct Wizard {
    Step step{Step::Inactive};
    bool waiting_release{false};
    std::array<uint16_t, kSticks> stick_center{};
    std::array<uint16_t, kSticks> stick_min{};
    std::array<uint16_t, kSticks> stick_max{};
    std::array<uint16_t, kAuxSlots> pot_center{};
    std::array<uint16_t, kAuxSlots> pot_min{};
    std::array<uint16_t, kAuxSlots> pot_max{};
    uint16_t timer_ms{0};
};

struct Input {
    uint16_t pressed;
    uint16_t held;
    input::Raw raw;
    uint16_t dt_ms;
};

struct Pots {
    std::array<uint8_t, kAuxSlots> channel{};
    std::size_t count{0};
};

constexpr bool active(const Wizard& w) { return w.step != Step::Inactive; }

constexpr uint16_t sat_sub(uint16_t a, uint16_t b) { return a > b ? static_cast<uint16_t>(a - b) : uint16_t{0}; }

constexpr uint16_t aux_raw(const input::Raw& raw, std::size_t ch) { return raw[input::aux_index_to_raw(ch)]; }

// Aux channels configured as potentiometers (continuous or detented). External ADC adds 4 more.
constexpr Pots configured_pots(const storage::RadioConfig& radio) {
    Pots p{};
    const std::size_t max_ch = radio.ext_adc != 0 ? kAuxSlots : 6;
    for (std::size_t ch = 0; ch < max_ch; ++ch) {
        const auto mode = input::resolve_mode(ch, radio.adc_modes[ch]);
        if (mode == input::AdcMode::Pot || mode == input::AdcMode::PotDetent) {
            p.channel[p.count++] = static_cast<uint8_t>(ch);
        }
    }
    return p;
}

constexpr uint16_t stick_min_span(const storage::RadioConfig& radio) {
    return radio.gimbal_type == 1 ? kStickMinSpanHall : kStickMinSpanPot;
}

constexpr std::array<bool, kSticks> stick_ready(const Wizard& w, const storage::RadioConfig& radio) {
    std::array<bool, kSticks> ready{};
    const uint16_t min_span = stick_min_span(radio);
    for (std::size_t i = 0; i < kSticks; ++i) {
        ready[i] = sat_sub(w.stick_center[i], w.stick_min[i]) >= min_span &&
                   sat_sub(w.stick_max[i], w.stick_center[i]) >= min_span;
    }
    return ready;
}

constexpr void start(Wizard& w) {
    w.step = Step::Center;
    w.stick_center.fill(storage::kAdcCenter);
    w.stick_min.fill(storage::kAdcMax);
    w.stick_max.fill(storage::kAdcMin);
    w.pot_center.fill(storage::kAdcCenter);
    w.pot_min.fill(storage::kAdcMax);
    w.pot_max.fill(storage::kAdcMin);
    w.waiting_release = true;
    w.timer_ms = 0;
}

// 63/64 margin on each side of the travel, as in the OpenTX calibration.
constexpr uint16_t with_margin(uint16_t span) { return static_cast<uint16_t>((uint32_t{span} * 63) / 64); }

constexpr void save_endpoints(storage::RadioConfig& radio, const Wizard& w, const Pots& pots) {
    for (std::size_t i = 0; i < kSticks; ++i) {
        const uint16_t c = w.stick_center[i];
        const uint16_t neg = with_margin(sat_sub(c, w.stick_min[i]));
        const uint16_t pos = with_margin(sat_sub(w.stick_max[i], c));
        radio.sticks[i] = storage::ChannelCalib{sat_sub(c, neg), c, static_cast<uint16_t>(c + pos), 0};
    }
    for (std::size_t i = 0; i < pots.count; ++i) {
        const std::size_t ch = pots.channel[i];
        if (sat_sub(w.pot_max[ch], w.pot_min[ch]) < kPotMovedSpan) {
            continue;
        }
        const uint16_t c = w.pot_center[ch];
        const uint16_t neg = with_margin(sat_sub(c, w.pot_min[ch]));
        const uint16_t pos = with_margin(sat_sub(w.pot_max[ch], c));
        radio.aux_pots[ch] = storage::PotCalib{sat_sub(c, neg), c, static_cast<uint16_t>(c + pos)};
    }
}

// Returns true when the calibration was saved. The caller applies it and writes EEPROM.
constexpr bool update(Wizard& w, const Input& in, storage::RadioConfig& radio) {
    if (w.waiting_release && (in.held & kKeyOk) == 0) {
        w.waiting_release = false;
    }
    const uint16_t newly = w.waiting_release ? uint16_t{0} : in.pressed;
    const bool ok = (newly & kKeyOk) != 0;
    const bool cancel = (newly & kKeyCancel) != 0;
    const Pots pots = configured_pots(radio);

    switch (w.step) {
        case Step::Inactive: break;

        case Step::Center:
            if (cancel) {
                w.step = Step::Inactive;
                break;
            }
            if (ok) {
                for (std::size_t i = 0; i < kSticks; ++i) {
                    w.stick_center[i] = in.raw[i];
                    w.stick_min[i] = in.raw[i];
                    w.stick_max[i] = in.raw[i];
                }
                for (std::size_t i = 0; i < pots.count; ++i) {
                    const std::size_t ch = pots.channel[i];
                    const uint16_t v = aux_raw(in.raw, ch);
                    w.pot_center[ch] = v;
                    w.pot_min[ch] = v;
                    w.pot_max[ch] = v;
                }
                w.step = Step::Limits;
                w.waiting_release = true;
            }
            break;

        case Step::Limits: {
            if (cancel) {
                w.step = Step::Inactive;
                break;
            }
            for (std::size_t i = 0; i < kSticks; ++i) {
                w.stick_min[i] = std::min(w.stick_min[i], in.raw[i]);
                w.stick_max[i] = std::max(w.stick_max[i], in.raw[i]);
            }
            for (std::size_t i = 0; i < pots.count; ++i) {
                const std::size_t ch = pots.channel[i];
                const uint16_t v = aux_raw(in.raw, ch);
                w.pot_min[ch] = std::min(w.pot_min[ch], v);
                w.pot_max[ch] = std::max(w.pot_max[ch], v);
            }
            const auto ready = stick_ready(w, radio);
            const bool all_ready = std::all_of(ready.begin(), ready.end(), [](bool r) { return r; });
            if (ok && all_ready) {
                save_endpoints(radio, w, pots);
                w.step = Step::Complete;
                w.timer_ms = kCompleteMs;
                return true;
            }
            break;
        }

        case Step::Complete:
            if (w.timer_ms > in.dt_ms) {
                w.timer_ms = static_cast<uint16_t>(w.timer_ms - in.dt_ms);
            } else {
                w.step = Step::Inactive;
            }
            break;
    }
    return false;
}

// Mirrors the Rust gauge: a centre bar, travel fill on each side, and a live cursor.
constexpr void draw_gauge(Framebuffer& fb, int x, int y, int w, int h, uint16_t center, uint16_t lo, uint16_t hi,
                          uint16_t cur, uint32_t target) {
    if (w < 6 || h < 3) {
        return;
    }
    fb.draw_rect(x, y, w, h, true);
    const int cx = x + w / 2;
    fb.draw_vline(cx, y, h, true);

    const int half = w / 2 - 1;
    if (half <= 0 || target == 0) {
        return;
    }
    const int mid = y + h / 2;
    const auto scaled = [&](uint32_t span) {
        return static_cast<int>(std::min<uint32_t>((span * static_cast<uint32_t>(half)) / target,
                                                   static_cast<uint32_t>(half)));
    };

    const int left = scaled(sat_sub(center, lo));
    if (left > 0) {
        fb.draw_hline(cx - left, mid, left + 1, true);
    }
    const int right = scaled(sat_sub(hi, center));
    if (right > 0) {
        fb.draw_hline(cx, mid, right + 1, true);
    }

    const int delta = static_cast<int>(cur) - static_cast<int>(center);
    const int dist = scaled(static_cast<uint32_t>(delta < 0 ? -delta : delta));
    const int cur_x = delta < 0 ? cx - dist : cx + dist;
    const int marker_h = h - 2;
    if (marker_h > 0) {
        fb.draw_vline(cur_x, y + 1, marker_h, true);
    }
}

constexpr void draw_sticks(Framebuffer& fb, const Wizard& w, const input::Raw& raw, const storage::RadioConfig& radio,
                           bool general) {
    const std::array<std::string_view, kSticks> labels = general ? std::array<std::string_view, kSticks>{"1", "2", "3", "4"}
                                                                 : std::array<std::string_view, kSticks>{"A", "E", "T", "R"};
    const auto ready = stick_ready(w, radio);
    for (std::size_t i = 0; i < kSticks; ++i) {
        const int y = 13 + static_cast<int>(i) * 8;
        fb.draw_str_6x10(2, y - 1, labels[i], false);
        // Horizontal axes (aileron, rudder) travel ~1350 counts, vertical ones ~1250.
        const uint32_t target = (i == 0 || i == 3) ? 1350u : 1250u;
        draw_gauge(fb, 10, y, 63, 7, w.stick_center[i], w.stick_min[i], w.stick_max[i], raw[i], target);
        fb.draw_str_6x10(75, y - 1, ready[i] ? "OK" : "--", false);
    }
}

constexpr void draw_pots(Framebuffer& fb, const Wizard& w, const Pots& pots, const input::Raw& raw) {
    const std::size_t count = pots.count;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t ch = pots.channel[i];
        const auto name = kPotNames[ch];
        const uint16_t center = w.pot_center[ch];
        const uint16_t lo = w.pot_min[ch];
        const uint16_t hi = w.pot_max[ch];
        const uint16_t cur = aux_raw(raw, ch);
        const bool moved = sat_sub(hi, lo) >= kPotMovedSpan;
        const std::string_view status = moved ? "OK" : "--";

        if (count <= 2) {
            const int text_y = count == 1 ? 18 : (i == 0 ? 12 : 28);
            const int box_y = count == 1 ? 27 : (i == 0 ? 21 : 37);
            fb.draw_str_6x10(92, text_y, name, false);
            fb.draw_str_6x10(110, text_y, status, false);
            draw_gauge(fb, 92, box_y, 35, 7, center, lo, hi, cur, kPotTargetSpan);
        } else if (count <= 4) {
            const int y = 13 + static_cast<int>(i) * 8;
            fb.draw_str_4x6(89, y + 1, name, false);
            fb.draw_str_4x6(99, y + 1, status, false);
            draw_gauge(fb, 110, y, 17, 7, center, lo, hi, cur, kPotTargetSpan);
        } else {
            constexpr int kColWidth = 18;
            const std::size_t rows_per_col = (count + 1) / 2;
            const int x = i / rows_per_col == 0 ? 89 : 108;
            const int y = 13 + static_cast<int>(i % rows_per_col) * 7;
            fb.draw_str_4x6(x, y, name, false);
            if (moved) {
                fb.draw_rect(x + 9, y, kColWidth - 9, 6, true);
            }
            draw_gauge(fb, x + 9, y, kColWidth - 9, 6, center, lo, hi, cur, kPotTargetSpan);
        }
    }
}

constexpr void render(Framebuffer& fb, const Wizard& w, const input::Raw& raw, const storage::RadioConfig& radio,
                      uint8_t model_type) {
    switch (w.step) {
        case Step::Inactive: break;

        case Step::Center:
            widgets::draw_header(fb, "CALIBRATION (1/2)");
            fb.draw_str_6x10(2, 15, "1. Center all sticks", false);
            fb.draw_str_6x10(2, 25, "   & rotary pots.", false);
            fb.draw_str_6x10(2, 36, "2. Move THR to middle!", false);
            if (w.waiting_release) {
                widgets::draw_footer(fb, "Release [OK] key...");
            } else {
                widgets::draw_footer_split(fb, "[OK] Next", "[ESC] Exit");
            }
            break;

        case Step::Limits: {
            widgets::draw_header(fb, "CALIBRATION (2/2)");
            const auto pots = configured_pots(radio);
            draw_sticks(fb, w, raw, radio, model_type == kModelTypeGeneral);
            draw_pots(fb, w, pots, raw);
            const auto ready = stick_ready(w, radio);
            const bool all_ready = std::all_of(ready.begin(), ready.end(), [](bool r) { return r; });
            if (w.waiting_release) {
                widgets::draw_footer(fb, "Release [OK] key...");
            } else if (all_ready) {
                widgets::draw_footer_split(fb, "[OK] Save", "[ESC] Exit");
            } else {
                widgets::draw_footer_split(fb, "Stir sticks & pots", "[ESC] Exit");
            }
            break;
        }

        case Step::Complete:
            widgets::draw_header(fb, "CALIBRATION (OK)");
            fb.draw_str_6x10(10, 19, "CALIBRATION SAVED!", false);
            fb.draw_str_6x10(14, 33, "EEPROM updated OK", false);
            widgets::draw_footer(fb, "Ready");
            break;
    }
}

}  // namespace ferrox::logic::calib
