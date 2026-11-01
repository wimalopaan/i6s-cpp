#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/fonts.hpp"

namespace ferrox::logic::display {

inline constexpr int kWidth = 128;
inline constexpr int kHeight = 64;
inline constexpr std::size_t kBufferSize = kWidth * (kHeight / 8);

struct Point {
    int x;
    int y;
    friend constexpr bool operator==(const Point&, const Point&) = default;
};

struct Size {
    int w;
    int h;
};

struct Rect {
    Point pos;
    Size size;
};

// Page-organised 1bpp framebuffer: byte index = page * kWidth + x, bit = y % 8.
class Framebuffer {
public:
    constexpr void clear() { px_.fill(0); }

    constexpr const std::array<uint8_t, kBufferSize>& data() const { return px_; }

    constexpr void set_pixel(Point p, bool on) {
        if (p.x < 0 || p.x >= kWidth || p.y < 0 || p.y >= kHeight) {
            return;
        }
        const uint8_t mask = bit_mask(p.y);
        uint8_t& b = px_[index(p)];
        b = on ? static_cast<uint8_t>(b | mask) : static_cast<uint8_t>(b & ~mask);
    }

    constexpr void fill_rect(Rect r, bool on) {
        if (r.size.w <= 0 || r.size.h <= 0) {
            return;
        }
        const int x0 = std::max(r.pos.x, 0);
        const int x1 = std::min(r.pos.x + r.size.w, kWidth);
        const int y0 = std::max(r.pos.y, 0);
        const int y1 = std::min(r.pos.y + r.size.h, kHeight);
        if (x0 >= x1 || y0 >= y1) {
            return;
        }
        for (int py = y0; py < y1; ++py) {
            const uint8_t mask = bit_mask(py);
            for (int px = x0; px < x1; ++px) {
                uint8_t& b = px_[index({px, py})];
                b = on ? static_cast<uint8_t>(b | mask) : static_cast<uint8_t>(b & ~mask);
            }
        }
    }

    constexpr void draw_hline(Point p, int len, bool on) { fill_rect({p, {len, 1}}, on); }

    constexpr void draw_vline(Point p, int len, bool on) { fill_rect({p, {1, len}}, on); }

    // Outline rectangle; the frame is drawn with w x h outer dimensions.
    constexpr void draw_rect(Rect r, bool on) {
        const auto [x, y] = r.pos;
        const auto [w, h] = r.size;
        if (w <= 0 || h <= 0) {
            return;
        }
        draw_hline({x, y}, w, on);
        if (h > 1) {
            draw_hline({x, y + h - 1}, w, on);
            if (h > 2) {
                draw_vline({x, y + 1}, h - 2, on);
                if (w > 1) {
                    draw_vline({x + w - 1, y + 1}, h - 2, on);
                }
            }
        }
    }

    // Glyph columns are drawn as lit pixels; with `invert`, lit pixels are cleared instead.
    template <std::size_t N>
    constexpr void draw_glyph(Point p, const std::array<uint16_t, N>& cols, int height, bool invert) {
        for (std::size_t col = 0; col < N; ++col) {
            for (int row = 0; row < height; ++row) {
                if ((cols[col] & (1u << row)) != 0) {
                    set_pixel({p.x + static_cast<int>(col), p.y + row}, !invert);
                }
            }
        }
    }

    constexpr void draw_char_4x6(Point p, char c, bool invert) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E) {
            return;
        }
        const auto& g = fonts::kFont4x6[u - 0x20];
        draw_glyph<4>(p, {g[0], g[1], g[2], g[3]}, 6, invert);
    }

    constexpr void draw_str_4x6(Point p, std::string_view text, bool invert) {
        for (char c : text) {
            draw_char_4x6(p, c, invert);
            p.x += 4;
        }
    }

    constexpr void draw_char_6x10(Point p, char c, bool invert) {
        const auto u = static_cast<unsigned char>(c);
        if (u < 0x20 || u > 0x7E) {
            return;
        }
        draw_glyph<6>(p, fonts::kFont6x10[u - 0x20], 10, invert);
    }

    constexpr void draw_str_6x10(Point p, std::string_view text, bool invert) {
        for (char c : text) {
            draw_char_6x10(p, c, invert);
            p.x += 6;
        }
    }

private:
    static constexpr std::size_t index(Point p) {
        return static_cast<std::size_t>((p.y / 8) * kWidth + p.x);
    }

    static constexpr uint8_t bit_mask(int y) { return static_cast<uint8_t>(1u << (y % 8)); }

    std::array<uint8_t, kBufferSize> px_{};
};

}  // namespace ferrox::logic::display
