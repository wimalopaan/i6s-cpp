#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "ferrox/logic/framebuffer.hpp"

namespace ferrox::logic::widgets {

using display::Framebuffer;
using display::kWidth;
using display::Point;
using display::Rect;

struct Deflection {
    int value;
    friend constexpr bool operator==(const Deflection&, const Deflection&) = default;
};

struct Trim {
    int value;
    friend constexpr bool operator==(const Trim&, const Trim&) = default;
};

struct PotPosition {
    int value;
    friend constexpr bool operator==(const PotPosition&, const PotPosition&) = default;
};

struct Index {
    int value;
    friend constexpr bool operator==(const Index&, const Index&) = default;
};

struct Count {
    int value;
    friend constexpr bool operator==(const Count&, const Count&) = default;
};

struct Span {
    int top;
    int height;
    friend constexpr bool operator==(const Span&, const Span&) = default;
};

struct Range {
    uint16_t min;
    uint16_t max;
    friend constexpr bool operator==(const Range&, const Range&) = default;
};

struct RowY {
    int value;
    friend constexpr bool operator==(const RowY&, const RowY&) = default;
};

struct Width {
    int value;
    friend constexpr bool operator==(const Width&, const Width&) = default;
};

struct Slot {
    int value;
    friend constexpr bool operator==(const Slot&, const Slot&) = default;
};

struct ValueX {
    int value;
    friend constexpr bool operator==(const ValueX&, const ValueX&) = default;
};

struct Track {
    int min_pos;
    int travel;
    friend constexpr bool operator==(const Track&, const Track&) = default;
};

// Header: centred 6x10 title at y = 2, divider at y = 11.
constexpr void draw_header(Framebuffer& fb, std::string_view title) {
    const int x = std::max((kWidth - static_cast<int>(title.size()) * 6) / 2, 2);
    fb.draw_str_6x10({x, 2}, title, false);
    fb.draw_hline({0, 11}, kWidth, true);
}

// Footer: divider at y = 55, 4x6 text at y = 57.
constexpr void draw_footer(Framebuffer& fb, std::string_view text) {
    fb.draw_hline({0, 55}, kWidth, true);
    fb.draw_str_4x6({2, 57}, text, false);
}

constexpr void draw_footer_split(Framebuffer& fb, std::string_view left, std::string_view right) {
    fb.draw_hline({0, 55}, kWidth, true);
    fb.draw_str_4x6({2, 57}, left, false);
    const int right_x = std::max(126 - static_cast<int>(right.size()) * 4, 2);
    fb.draw_str_4x6({right_x, 57}, right, false);
}

// With `center_inverted`, the centre text sits on a solid bar.
constexpr void draw_footer_three(Framebuffer& fb, std::string_view left, std::string_view center,
                                 std::string_view right, bool center_inverted) {
    fb.draw_hline({0, 55}, kWidth, true);
    fb.draw_str_4x6({2, 57}, left, false);
    if (!center.empty()) {
        const int len = static_cast<int>(center.size());
        const int center_x = std::max((kWidth - len * 4) / 2, 2);
        if (center_inverted) {
            fb.fill_rect({{center_x - 2, 56}, {len * 4 + 3, 7}}, true);
            fb.draw_str_4x6({center_x, 57}, center, true);
        } else {
            fb.draw_str_4x6({center_x, 57}, center, false);
        }
    }
    const int right_x = std::max(126 - static_cast<int>(right.size()) * 4, 2);
    fb.draw_str_4x6({right_x, 57}, right, false);
}

// Framed bar for a value in -1000..+1000, with a trim tick at the trim position (-250..+250 maps to the bar).
constexpr void draw_progress_bar(Framebuffer& fb, const Rect r, const Deflection deflection, const Trim trim_setting) {
    const auto [x, y] = r.pos;
    const auto [width, height] = r.size;
    fb.draw_rect(r, true);
    const int val = deflection.value;
    const int trim = trim_setting.value;

    const int max_fill = width - 2;
    const int normalized = std::clamp(val + 1000, 0, 2000);
    const int fill_len = normalized * max_fill / 2000;

    if (fill_len > 0) {
        fb.fill_rect({{x + 1, y + 1}, {fill_len, std::max(height - 2, 0)}}, true);
    }

    if (trim != 0) {
        const int trim_offset = trim * max_fill / 250;
        const int trim_x = std::min(x + 1 + std::max(trim_offset, 0), x + max_fill);
        const bool tick_on = (trim_x - (x + 1)) >= fill_len;
        fb.draw_vline({trim_x, y + 1}, std::max(height - 2, 0), tick_on);
    }
}

constexpr void draw_bar_gauge(Framebuffer& fb, const Rect r, const int fill_width) {
    const auto [x, y] = r.pos;
    fb.draw_rect(r, true);
    if (fill_width > 0) {
        fb.fill_rect({{x + 1, y + 1}, {fill_width, std::max(r.size.h - 2, 0)}}, true);
    }
}

// Zero-padded 4-digit decimal, clamped to 9999 (the display field is four characters wide).
constexpr std::array<char, 4> format_u16_4(const uint16_t value) {
    unsigned v = std::min<unsigned>(value, 9999u);
    std::array<char, 4> out{'0', '0', '0', '0'};
    for (int i = 3; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = static_cast<char>('0' + v % 10);
        v /= 10;
    }
    return out;
}

// Label at (2, y), framed bar at (44, y + 1) 40x5, value at (90, y).
constexpr void draw_gauge_row(Framebuffer& fb, const RowY row, std::string_view label, const Width fill_width, const uint16_t val) {
    const int y = row.value;
    fb.draw_str_4x6({2, y}, label, false);
    draw_bar_gauge(fb, {{44, y + 1}, {40, 5}}, fill_width.value);
    const auto digits = format_u16_4(val);
    fb.draw_str_4x6({90, y}, std::string_view(digits.data(), digits.size()), false);
}

// List row in a 4-slot view: highlighted 124x9 bar at y = 14 + slot * 9 when selected.
// An empty value draws no value; value_x <= 0 right-aligns the value to x = 124.
constexpr void draw_list_row(Framebuffer& fb, const Slot slot, const bool selected, std::string_view label,
                             std::string_view value, const ValueX value_x) {
    const int y = 14 + slot.value * 9;
    if (selected) {
        fb.fill_rect({{2, y}, {124, 9}}, true);
    }

    fb.draw_str_6x10({4, y}, label, selected);
    if (!value.empty()) {
        const int vx = value_x.value <= 0 ? 124 - static_cast<int>(value.size()) * 6 : value_x.value;
        fb.draw_str_6x10({vx, y}, value, selected);
    }
}

// Vertical scrollbar at x = 125..127 with the thumb proportional to the visible share.
constexpr void draw_scrollbar(Framebuffer& fb, const Index selected, const Count count, const Span span) {
    const auto [top_y, height] = span;
    if (count.value <= 1) {
        return;
    }
    fb.draw_vline({126, top_y}, height, true);

    const int thumb_h = std::clamp(height * 3 / count.value, 6, std::max(height, 6));
    const int travel = std::max(height - thumb_h, 0);
    const int thumb_y = top_y + selected.value * travel / (count.value - 1);

    fb.fill_rect({{125, thumb_y}, {3, thumb_h}}, true);
}

// Maps -1000..+1000 onto a cursor position in [min_pos, max_pos].
constexpr int cursor_position(const int val, const Track track) {
    return track.min_pos + ((val + 1000) * track.travel + 1000) / 2000;
}

// Channel monitor gauge with centre ticks, trim tick and cursor.
constexpr void draw_channel_gauge(Framebuffer& fb, const Rect r, const Deflection deflection, const Trim trim_setting) {
    const auto [x, y] = r.pos;
    const auto [width, height] = r.size;
    fb.draw_rect(r, true);
    const int val = deflection.value;
    const int trim = trim_setting.value;

    const int center_x = x + width / 2;
    fb.draw_vline({center_x, y}, 2, true);
    fb.draw_vline({center_x, y + height - 2}, 2, true);

    const int min_pos = x + 2;
    const int max_pos = x + width - 3;
    const int travel = max_pos - min_pos;
    const int cursor = cursor_position(val, {min_pos, travel});

    if (trim != 0) {
        const int trim_x = center_x + (trim * (travel / 2)) / 25;
        fb.draw_vline({trim_x, y + 2}, std::max(height - 4, 0), true);
    }

    fb.fill_rect({{cursor - 1, y + 1}, {3, std::max(height - 2, 0)}}, true);
}

// Two-lane pot bar (width x 7) with separate cursors for VR1 and VR2.
constexpr void draw_split_pot_bar(Framebuffer& fb, const Point p, const int width, const PotPosition vr1, const PotPosition vr2) {
    const auto [x, y] = p;
    if (width < 8) {
        return;
    }
    fb.draw_hline({x, y}, width, true);
    fb.draw_hline({x, y + 3}, width, true);
    fb.draw_hline({x, y + 6}, width, true);

    fb.draw_vline({x, y}, 7, true);
    fb.draw_vline({x + width - 1, y}, 7, true);

    const int center_x = x + width / 2;
    fb.set_pixel({center_x, y + 1}, true);
    fb.set_pixel({center_x, y + 5}, true);

    const int min_pos = x + 2;
    const int travel = std::max(width - 5, 1);

    const int c1 = cursor_position(vr1.value, {min_pos, travel});
    fb.fill_rect({{c1 - 1, y + 1}, {3, 2}}, true);

    const int c2 = cursor_position(vr2.value, {min_pos, travel});
    fb.fill_rect({{c2 - 1, y + 4}, {3, 2}}, true);
}

// Single pot bar (width x 7) with centre tick and cursor.
constexpr void draw_single_pot_bar(Framebuffer& fb, const Point p, const Width width, const PotPosition val) {
    const auto [x, y] = p;
    const int w = width.value;
    if (w < 8) {
        return;
    }
    fb.draw_rect({p, {w, 7}}, true);
    fb.draw_vline({x + w / 2, y + 1}, 5, true);

    const int min_pos = x + 2;
    const int travel = std::max(w - 5, 1);
    const int c = cursor_position(val.value, {min_pos, travel});
    fb.fill_rect({{c - 1, y + 1}, {3, 5}}, true);
}

// Borderless pot lane (3 px high) with a baseline and a 2 px cursor.
constexpr void draw_pot_track(Framebuffer& fb, const Point p, const Width width, const PotPosition val) {
    const auto [x, y] = p;
    const int w = width.value;
    if (w < 6) {
        return;
    }
    fb.draw_hline({x, y + 1}, w, true);

    const int min_pos = x + 1;
    const int travel = std::max(w - 3, 1);
    const int c = cursor_position(val.value, {min_pos, travel});
    fb.fill_rect({{c - 1, y}, {2, 3}}, true);
}

// Cluster of pot tracks: 3 pots in one column, 4-6 in two columns, 7+ in three columns.
constexpr void draw_multi_pot_bar(Framebuffer& fb, const Point p, const int width, std::span<const int16_t> pots) {
    const auto [x, y] = p;
    const int count = static_cast<int>(pots.size());
    if (count == 0 || width < 12) {
        return;
    }

    if (count == 3) {
        draw_pot_track(fb, {x, y}, {width}, {pots[0]});
        draw_pot_track(fb, {x, y + 3}, {width}, {pots[1]});
        draw_pot_track(fb, {x, y + 6}, {width}, {pots[2]});
    } else if (count <= 6) {
        constexpr int kColGap = 2;
        const int col_w = std::max((width - kColGap) / 2, 6);
        const int right_x = x + col_w + kColGap;

        const int left_count = count == 4 ? 2 : 3;
        const int right_count = count - left_count;

        for (int row = 0; row < left_count; ++row) {
            const int ry = left_count == 2 ? y + 1 + row * 4 : y + row * 3;
            draw_pot_track(fb, {x, ry}, {col_w}, {pots[static_cast<std::size_t>(row)]});
        }
        for (int row = 0; row < right_count; ++row) {
            const int ry = right_count == 2 ? y + 1 + row * 4 : y + row * 3;
            draw_pot_track(fb, {right_x, ry}, {col_w}, {pots[static_cast<std::size_t>(left_count + row)]});
        }
    } else {
        constexpr int kColGap = 2;
        const int col_w = std::max((width - kColGap * 2) / 3, 6);
        const int rows_per_col = (count + 2) / 3;

        for (int i = 0; i < count; ++i) {
            const int col = i / rows_per_col;
            const int row = i % rows_per_col;
            const int cx = x + col * (col_w + kColGap);
            draw_pot_track(fb, {cx, y + row * 3}, {col_w}, {pots[static_cast<std::size_t>(i)]});
        }
    }
}

}  // namespace ferrox::logic::widgets
