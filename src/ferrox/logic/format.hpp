#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string_view>

namespace ferrox::logic::format {

/// Fixed-capacity text buffer. Characters beyond N are dropped.
template <std::size_t N>
struct Text {
    std::array<char, N> buf{};
    std::size_t len{0};

    constexpr void push(char c) {
        if (len < N) {
            buf[len++] = c;
        }
    }
    constexpr std::string_view view() const { return {buf.data(), len}; }
    constexpr bool empty() const { return len == 0; }
};

namespace detail {

template <std::size_t N>
constexpr void push_spaces(Text<N>& t, int count) {
    for (int i = 0; i < count; ++i) {
        t.push(' ');
    }
}

/// Appends the decimal digits of `v` without padding.
template <std::size_t N>
constexpr void push_uint(Text<N>& t, unsigned v) {
    std::array<char, 10> tmp{};
    std::size_t n = 0;
    do {
        tmp[n++] = static_cast<char>('0' + v % 10);
        v /= 10;
    } while (v > 0);
    while (n > 0) {
        t.push(tmp[--n]);
    }
}

constexpr char hex_digit(unsigned v) { return "0123456789ABCDEF"[v & 0x0Fu]; }

}  // namespace detail

/// Signed integer without padding, e.g. "-100", "35", "0".
constexpr Text<4> format_int(int val) {
    Text<4> t;
    if (val < 0) {
        t.push('-');
    }
    detail::push_uint(t, static_cast<unsigned>(std::abs(val)));
    return t;
}

/// Percent of a stick value (-1000..+1000) as "+100%", "+ 50%", "   0%", "- 10%" (always 5 characters).
constexpr Text<5> format_percent(int16_t val) {
    const int pct = val / 10;
    Text<5> t;
    if (pct < 0) {
        t.push('-');
    } else if (pct > 0) {
        t.push('+');
    } else {
        t.push(' ');
    }
    const auto mag = static_cast<unsigned>(std::abs(pct));
    const int digits = mag >= 100 ? 3 : mag >= 10 ? 2 : 1;
    detail::push_spaces(t, 3 - digits);
    detail::push_uint(t, mag);
    t.push('%');
    return t;
}

/// Unipolar throttle percent 0..100 as "   0%" .. " 100%" (always 5 characters).
constexpr Text<5> format_throttle_percent(int16_t val) {
    const int pct = std::clamp((static_cast<int>(val) + 1000) * 100 / 2000, 0, 100);
    Text<5> t;
    t.push(' ');
    const int digits = pct >= 100 ? 3 : pct >= 10 ? 2 : 1;
    detail::push_spaces(t, 3 - digits);
    detail::push_uint(t, static_cast<unsigned>(pct));
    t.push('%');
    return t;
}

/// Active trim as "TRM A:+05". `axis` is the one-letter axis name (A, E, T, R).
constexpr Text<9> format_trim(char axis, int8_t val) {
    Text<9> t;
    for (char c : std::string_view("TRM  :")) {
        t.push(c);
    }
    t.buf[4] = axis;
    t.push(val < 0 ? '-' : val > 0 ? '+' : ' ');
    const auto mag = static_cast<unsigned>(std::abs(static_cast<int>(val)));
    t.push(static_cast<char>('0' + (mag / 10) % 10));
    t.push(static_cast<char>('0' + mag % 10));
    return t;
}

/// Battery voltage from millivolts as "4.20V" or "12.60V".
constexpr Text<6> format_vbat(uint16_t mv) {
    const unsigned centi = mv / 10;
    Text<6> t;
    const unsigned integer = centi / 100;
    detail::push_uint(t, integer);
    t.push('.');
    t.push(static_cast<char>('0' + (centi / 10) % 10));
    t.push(static_cast<char>('0' + centi % 10));
    t.push('V');
    return t;
}

/// Two-digit zero-padded decimal, clamped to 99.
constexpr Text<2> format_dec2(unsigned val) {
    const unsigned v = std::min(val, 99u);
    Text<2> t;
    t.push(static_cast<char>('0' + v / 10));
    t.push(static_cast<char>('0' + v % 10));
    return t;
}

/// Eight-digit upper-case hex.
constexpr Text<8> format_hex32(uint32_t val) {
    Text<8> t;
    for (int shift = 28; shift >= 0; shift -= 4) {
        t.push(detail::hex_digit(val >> shift));
    }
    return t;
}

/// Trims ASCII spaces on both ends.
constexpr std::string_view trim_spaces(std::string_view s) {
    while (!s.empty() && s.front() == ' ') {
        s.remove_prefix(1);
    }
    while (!s.empty() && s.back() == ' ') {
        s.remove_suffix(1);
    }
    return s;
}

}  // namespace ferrox::logic::format
