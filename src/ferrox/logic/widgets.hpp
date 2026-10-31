#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/framebuffer.hpp"

namespace ferrox::logic::widgets {

using display::Framebuffer;
using display::kWidth;

// Header: centred 6x10 title at y = 2, divider at y = 11.
constexpr void draw_header(Framebuffer& fb, std::string_view title) {
    const int x = std::max((kWidth - static_cast<int>(title.size()) * 6) / 2, 2);
    fb.draw_str_6x10(x, 2, title, false);
    fb.draw_hline(0, 11, kWidth, true);
}

// Footer: divider at y = 55, 4x6 text at y = 57.
constexpr void draw_footer(Framebuffer& fb, std::string_view text) {
    fb.draw_hline(0, 55, kWidth, true);
    fb.draw_str_4x6(2, 57, text, false);
}

constexpr void draw_footer_split(Framebuffer& fb, std::string_view left, std::string_view right) {
    fb.draw_hline(0, 55, kWidth, true);
    fb.draw_str_4x6(2, 57, left, false);
    const int right_x = std::max(126 - static_cast<int>(right.size()) * 4, 2);
    fb.draw_str_4x6(right_x, 57, right, false);
}

// With `center_inverted`, the centre text sits on a solid bar.
constexpr void draw_footer_three(Framebuffer& fb, std::string_view left, std::string_view center,
                                 std::string_view right, bool center_inverted) {
    fb.draw_hline(0, 55, kWidth, true);
    fb.draw_str_4x6(2, 57, left, false);
    if (!center.empty()) {
        const int len = static_cast<int>(center.size());
        const int center_x = std::max((kWidth - len * 4) / 2, 2);
        if (center_inverted) {
            fb.fill_rect(center_x - 2, 56, len * 4 + 3, 7, true);
            fb.draw_str_4x6(center_x, 57, center, true);
        } else {
            fb.draw_str_4x6(center_x, 57, center, false);
        }
    }
    const int right_x = std::max(126 - static_cast<int>(right.size()) * 4, 2);
    fb.draw_str_4x6(right_x, 57, right, false);
}

// Framed bar for a value in -1000..+1000, with a trim tick at the trim position (-250..+250 maps to the bar).
constexpr void draw_progress_bar(Framebuffer& fb, int x, int y, int width, int height, int val, int trim) {
    fb.draw_rect(x, y, width, height, true);

    const int max_fill = width - 2;
    const int normalized = std::clamp(val + 1000, 0, 2000);
    const int fill_len = normalized * max_fill / 2000;

    if (fill_len > 0) {
        fb.fill_rect(x + 1, y + 1, fill_len, std::max(height - 2, 0), true);
    }

    if (trim != 0) {
        const int trim_offset = trim * max_fill / 250;
        const int trim_x = std::min(x + 1 + std::max(trim_offset, 0), x + max_fill);
        const bool tick_on = (trim_x - (x + 1)) >= fill_len;
        fb.draw_vline(trim_x, y + 1, std::max(height - 2, 0), tick_on);
    }
}

constexpr void draw_bar_gauge(Framebuffer& fb, int x, int y, int width, int height, int fill_width) {
    fb.draw_rect(x, y, width, height, true);
    if (fill_width > 0) {
        fb.fill_rect(x + 1, y + 1, fill_width, std::max(height - 2, 0), true);
    }
}

// Zero-padded 4-digit decimal, clamped to 9999 (the display field is four characters wide).
constexpr std::array<char, 4> format_u16_4(uint16_t value) {
    unsigned v = std::min<unsigned>(value, 9999u);
    std::array<char, 4> out{'0', '0', '0', '0'};
    for (int i = 3; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = static_cast<char>('0' + v % 10);
        v /= 10;
    }
    return out;
}

// Label at (2, y), framed bar at (44, y + 1) 40x5, value at (90, y).
constexpr void draw_gauge_row(Framebuffer& fb, int y, std::string_view label, int fill_width, uint16_t val) {
    fb.draw_str_4x6(2, y, label, false);
    draw_bar_gauge(fb, 44, y + 1, 40, 5, fill_width);
    const auto digits = format_u16_4(val);
    fb.draw_str_4x6(90, y, std::string_view(digits.data(), digits.size()), false);
}

// List row in a 4-slot view: highlighted 124x9 bar at y = 14 + slot * 9 when selected.
// An empty value draws no value; value_x <= 0 right-aligns the value to x = 124.
constexpr void draw_list_row(Framebuffer& fb, int slot, bool selected, std::string_view label,
                             std::string_view value, int value_x) {
    const int y = 14 + slot * 9;
    if (selected) {
        fb.fill_rect(2, y, 124, 9, true);
    }

    fb.draw_str_6x10(4, y, label, selected);
    if (!value.empty()) {
        const int vx = value_x <= 0 ? 124 - static_cast<int>(value.size()) * 6 : value_x;
        fb.draw_str_6x10(vx, y, value, selected);
    }
}

// Vertical scrollbar at x = 125..127 with the thumb proportional to the visible share.
constexpr void draw_scrollbar(Framebuffer& fb, int selected, int count, int top_y, int height) {
    if (count <= 1) {
        return;
    }
    fb.draw_vline(126, top_y, height, true);

    const int thumb_h = std::clamp(height * 3 / count, 6, std::max(height, 6));
    const int travel = std::max(height - thumb_h, 0);
    const int thumb_y = top_y + selected * travel / (count - 1);

    fb.fill_rect(125, thumb_y, 3, thumb_h, true);
}

}  // namespace ferrox::logic::widgets
