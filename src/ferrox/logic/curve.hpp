#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace ferrox::logic::curve {

/// Curve points in percent (0..100). Only the first 5, 7 or 9 entries are used.
using Points = std::array<uint8_t, 17>;

/// Number of points for a stored mode (5, 7 or 9). Any other value is treated as 5.
constexpr std::size_t point_count(uint8_t pts) {
    return pts == 9 ? 9 : (pts == 7 ? 7 : 5);
}

struct Knots {
    int32_t p0 = 0;
    int32_t p1 = 0;
    int32_t p2 = 0;
    int32_t p3 = 0;
    friend constexpr bool operator==(const Knots&, const Knots&) = default;
};

struct Fraction {
    int32_t value = 0;
    friend constexpr bool operator==(const Fraction&, const Fraction&) = default;
};

/// Catmull-Rom cubic between p1 and p2 at t (0..256 fixed point). Result clamped to 0..1000.
constexpr int32_t catmull_rom(Knots k, Fraction frac) {
    const auto [p0, p1, p2, p3] = k;
    const int32_t t = frac.value;
    const int32_t t2 = (t * t) / 256;
    const int32_t t3 = (t2 * t) / 256;
    const int32_t h00 = 2 * t3 - 3 * t2 + 256;
    const int32_t h10 = t3 - 2 * t2 + t;
    const int32_t h01 = -2 * t3 + 3 * t2;
    const int32_t h11 = t3 - t2;
    const int32_t m1 = (p2 - p0) / 2;
    const int32_t m2 = (p3 - p1) / 2;
    return std::clamp((h00 * p1 + h01 * p2 + h10 * m1 + h11 * m2) / 256, 0, 1000);
}

template <std::size_t N>
constexpr uint16_t evaluate_n(int32_t x, bool smooth, const Points& c) {
    std::array<int32_t, N> pts{};
    for (std::size_t i = 0; i < N; ++i) {
        pts[i] = c[i] * 10;
    }
    if (x >= 1000) {
        return static_cast<uint16_t>(std::clamp<int32_t>(pts[N - 1], 0, 1000));
    }
    if (x <= 0) {
        return static_cast<uint16_t>(std::clamp<int32_t>(pts[0], 0, 1000));
    }

    constexpr int32_t kSegments = static_cast<int32_t>(N) - 1;
    const auto i = static_cast<std::size_t>(std::min<int32_t>(x * kSegments / 1000, kSegments - 1));
    const int32_t x0 = static_cast<int32_t>(i) * 1000 / kSegments;
    const int32_t x1 = (static_cast<int32_t>(i) + 1) * 1000 / kSegments;
    const int32_t dx = x - x0;
    const int32_t span = std::max<int32_t>(x1 - x0, 1);

    if (!smooth) {
        const int32_t y = pts[i] + (pts[i + 1] - pts[i]) * dx / span;
        return static_cast<uint16_t>(std::clamp<int32_t>(y, 0, 1000));
    }

    // Boundary segments extrapolate the missing neighbour linearly.
    const int32_t p0 = i > 0 ? pts[i - 1] : 2 * pts[0] - pts[1];
    const int32_t p3 = i + 2 < N ? pts[i + 2] : 2 * pts[N - 1] - pts[N - 2];
    return static_cast<uint16_t>(catmull_rom(Knots{p0, pts[i], pts[i + 1], p3}, Fraction{dx * 256 / span}));
}

/// Maps the input 0..1000 through the curve. Output 0..1000.
constexpr uint16_t evaluate(uint16_t input, uint8_t pts, bool smooth, const Points& c) {
    const auto x = static_cast<int32_t>(std::min<uint16_t>(input, 1000));
    switch (point_count(pts)) {
        case 9: return evaluate_n<9>(x, smooth, c);
        case 7: return evaluate_n<7>(x, smooth, c);
        default: return evaluate_n<5>(x, smooth, c);
    }
}

/// Curve output (0..1000) rounded to a percent value (0..100).
constexpr uint8_t to_percent(uint16_t out) {
    return static_cast<uint8_t>(std::min<uint16_t>((out + 5) / 10, 100));
}

/// Steps the mode 5 -> 7 -> 9 -> 5. Growing modes resample the old curve at evenly spaced inputs.
/// Shrinking to 5 keeps every second point.
constexpr void advance_mode(Points& c, uint8_t& pts) {
    const Points old = c;
    const std::size_t count = point_count(pts);
    c = Points{};
    if (count == 5) {
        for (std::size_t i = 0; i < 7; ++i) {
            c[i] = to_percent(evaluate(static_cast<uint16_t>(i * 1000 / 6), 5, false, old));
        }
        pts = 7;
    } else if (count == 7) {
        for (std::size_t i = 0; i < 9; ++i) {
            c[i] = to_percent(evaluate(static_cast<uint16_t>(i * 1000 / 8), 7, false, old));
        }
        pts = 9;
    } else {
        for (std::size_t k = 0; k < 5; ++k) {
            c[k] = old[2 * k];
        }
        pts = 5;
    }
}

}  // namespace ferrox::logic::curve
