#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::glyphs {

/// 5x7 switch position arrow.
constexpr void draw_switch_arrow(display::Framebuffer& fb, int x, int y, SwitchPos pos) {
    switch (pos) {
        case SwitchPos::Up:
            fb.set_pixel(x + 2, y, true);
            fb.draw_hline(x + 1, y + 1, 3, true);
            fb.draw_hline(x, y + 2, 5, true);
            fb.draw_vline(x + 2, y + 3, 4, true);
            break;
        case SwitchPos::Mid:
            fb.draw_hline(x, y + 3, 5, true);
            break;
        case SwitchPos::Down:
            fb.draw_vline(x + 2, y, 4, true);
            fb.draw_hline(x, y + 4, 5, true);
            fb.draw_hline(x + 1, y + 5, 3, true);
            fb.set_pixel(x + 2, y + 6, true);
            break;
    }
}

/// Compact 3x7 switch position arrow for dense rows.
constexpr void draw_switch_arrow_compact(display::Framebuffer& fb, int x, int y, SwitchPos pos) {
    switch (pos) {
        case SwitchPos::Up:
            fb.set_pixel(x + 1, y, true);
            fb.draw_hline(x, y + 1, 3, true);
            fb.draw_vline(x + 1, y + 2, 5, true);
            break;
        case SwitchPos::Mid:
            fb.draw_hline(x, y + 3, 3, true);
            break;
        case SwitchPos::Down:
            fb.draw_vline(x + 1, y, 5, true);
            fb.draw_hline(x, y + 5, 3, true);
            fb.set_pixel(x + 1, y + 6, true);
            break;
    }
}

/// 11x7 battery gauge. The fill scales `mv` between `min_mv` and `max_mv` across 8 pixels.
constexpr void draw_battery_gauge(display::Framebuffer& fb, int x, int y, uint16_t mv, uint16_t min_mv,
                                  uint16_t max_mv) {
    fb.draw_vline(x, y + 2, 3, true);
    fb.draw_rect(x + 1, y, 10, 7, true);

    const uint32_t span = std::max<uint32_t>(max_mv > min_mv ? max_mv - min_mv : 1u, 1u);
    const uint32_t delta = mv > min_mv ? mv - min_mv : 0u;
    const auto fill_w = static_cast<int>(std::min<uint32_t>((delta * 8u) / span, 8u));
    if (fill_w > 0) {
        fb.fill_rect(x + 10 - fill_w, y + 1, fill_w, 5, true);
    }
}

inline constexpr int kLogoWidth = 28;
inline constexpr int kLogoHeight = 27;

/// 28x27 Ferrox-RC emblem. Bit index is row * 28 + col, LSB first.
inline constexpr std::array<uint8_t, 95> kLogo28x27{
    0,   96,  0,   0,   0,   6,   0,   0,   240, 0,   0,   0,   9,   0,   0,   152, 1,   0,   192, 48,
    0,   0,   12,  3,   0,   64,  32,  0,   0,   6,   6,   0,   32,  64,  0,   0,   99,  12,  0,   144,
    144, 0,   128, 9,   25,  0,   8,   6,   1,   64,  96,  32,  0,   134, 16,  6,   32,  0,   64,  0,
    1,   0,   8,   48,  0,   192, 128, 1,   0,   24,  8,   0,   0,   129, 6,   0,   22,  60,  0,   192,
    227, 3,   0,   124, 30,  240, 128, 247, 1,   6,   248, 7,   96,  0,   14,
};

constexpr void draw_ferrox_logo(display::Framebuffer& fb, int x, int y) {
    for (int row = 0; row < kLogoHeight; ++row) {
        for (int col = 0; col < kLogoWidth; ++col) {
            const int bit = row * kLogoWidth + col;
            if ((kLogo28x27[static_cast<std::size_t>(bit / 8)] >> (bit % 8)) & 1u) {
                fb.set_pixel(x + col, y + row, true);
            }
        }
    }
}

// 12x12 MDI icons, 1 bit per pixel, row-major, 18 bytes each.
using Glyph12 = std::array<uint8_t, 18>;

inline constexpr std::array<Glyph12, 12> kMenuGlyphs{{
    {0, 0, 0, 0, 194, 59, 240, 1, 30, 224, 193, 27, 24, 1, 17, 0, 0, 0},           // airplane
    {0, 0, 16, 142, 193, 15, 240, 0, 15, 216, 192, 32, 136, 15, 80, 128, 15, 32},  // airplane cog
    {0, 32, 0, 2, 36, 96, 50, 34, 23, 154, 161, 0, 6, 224, 127, 254, 7, 0},        // chart line
    {0, 32, 0, 226, 32, 10, 18, 33, 17, 18, 161, 32, 14, 38, 0, 254, 7, 0},        // bell curve
    {0, 0, 0, 0, 0, 48, 224, 3, 48, 8, 192, 7, 8, 0, 0, 0, 0, 0},                  // swap horizontal
    {0, 0, 0, 0, 128, 31, 252, 227, 71, 126, 196, 63, 248, 1, 0, 0, 0, 0},         // toggle switch
    {0, 0, 0, 48, 128, 3, 16, 0, 9, 144, 0, 8, 192, 1, 24, 0, 0, 0},               // swap vertical
    {0, 0, 6, 240, 192, 63, 158, 199, 48, 12, 227, 121, 252, 3, 15, 96, 0, 0},     // cog
    {0, 0, 15, 12, 67, 44, 66, 36, 70, 98, 36, 64, 244, 194, 63, 240, 0, 0},       // gauge
    {0, 0, 6, 248, 193, 48, 100, 98, 111, 246, 70, 38, 12, 131, 31, 96, 0, 0},     // crosshairs gps
    {0, 0, 0, 0, 0, 1, 16, 128, 11, 252, 67, 34, 32, 0, 0, 0, 0, 0},               // pulse
    {0, 0, 15, 12, 67, 32, 2, 36, 64, 98, 36, 70, 4, 194, 48, 240, 0, 0},          // information outline
}};

/// Draws a 12x12 icon at (x, y); only set bits are drawn, in colour `on`.
constexpr void draw_glyph_12x12(display::Framebuffer& fb, int x, int y, const Glyph12& data, bool on) {
    for (int row = 0; row < 12; ++row) {
        for (int col = 0; col < 12; ++col) {
            const int bit = row * 12 + col;
            if ((data[static_cast<std::size_t>(bit / 8)] >> (bit % 8)) & 1u) {
                fb.set_pixel(x + col, y + row, on);
            }
        }
    }
}

}  // namespace ferrox::logic::glyphs
