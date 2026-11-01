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

    constexpr void push(const char c) {
        if (len < N) {
            buf[len++] = c;
        }
    }
    constexpr std::string_view view() const { return {buf.data(), len}; }
    constexpr bool empty() const { return len == 0; }
};

namespace detail {

template <std::size_t N>
constexpr void push_spaces(Text<N>& t, const int count) {
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

constexpr char hex_digit(const unsigned v) { return "0123456789ABCDEF"[v & 0x0Fu]; }

}  // namespace detail

/// Signed integer without padding, e.g. "-100", "35", "0".
constexpr Text<4> format_int(const int val) {
    Text<4> t;
    if (val < 0) {
        t.push('-');
    }
    detail::push_uint(t, static_cast<unsigned>(std::abs(val)));
    return t;
}

/// Percent of a stick value (-1000..+1000) as "+100%", "+ 50%", "   0%", "- 10%" (always 5 characters).
constexpr Text<5> format_percent(const int16_t val) {
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
// Vario deadband in tenths of m/s (0..99) as "+1.2m/s" (climb) or "-0.5m/s" (sink).
constexpr Text<7> format_deadband_ms(const uint8_t tenths, const bool sink) {
    Text<7> t;
    t.push(sink ? '-' : '+');
    t.push(static_cast<char>('0' + (tenths / 10) % 10));
    t.push('.');
    t.push(static_cast<char>('0' + tenths % 10));
    t.push('m');
    t.push('/');
    t.push('s');
    return t;
}

constexpr Text<5> format_throttle_percent(const int16_t val) {
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
constexpr Text<9> format_trim(const char axis, const int8_t val) {
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

/// Flight timer as "MM:SS", prefixed with '+' once expired. Minutes cap at 99.
constexpr Text<6> format_timer(const uint16_t secs, const bool expired) {
    Text<6> t;
    if (expired) {
        t.push('+');
    }
    const unsigned mins = std::min(static_cast<unsigned>(secs) / 60u, 99u);
    const unsigned sec = static_cast<unsigned>(secs) % 60u;
    t.push(static_cast<char>('0' + mins / 10));
    t.push(static_cast<char>('0' + mins % 10));
    t.push(':');
    t.push(static_cast<char>('0' + sec / 10));
    t.push(static_cast<char>('0' + sec % 10));
    return t;
}

/// Battery voltage from millivolts as "4.20V" or "12.60V".
constexpr Text<6> format_vbat(const uint16_t mv) {
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
constexpr Text<2> format_dec2(const unsigned val) {
    const unsigned v = std::min(val, 99u);
    Text<2> t;
    t.push(static_cast<char>('0' + v / 10));
    t.push(static_cast<char>('0' + v % 10));
    return t;
}

/// Eight-digit upper-case hex.
constexpr Text<8> format_hex32(const uint32_t val) {
    Text<8> t;
    for (int shift = 28; shift >= 0; shift -= 4) {
        t.push(detail::hex_digit(val >> shift));
    }
    return t;
}

/// `None` prints only a minus sign; `Force` prints '+' for non-negative values.
enum class SignMode : uint8_t { None, Force };

/// Fixed-point number with padded integer part and unit suffix, e.g. "+05dB" or "4.20V".
struct FixedFormat {
    int32_t value = 0;
    uint8_t decimals = 0;
    uint8_t pad_digits = 0;  // minimum integer digits, padded with zeros or spaces
    bool pad_zero = false;
    SignMode sign = SignMode::None;
    std::string_view suffix{};
};

/// Same output as Rust `ui/format.rs::format_fixed`; text beyond 12 characters is dropped.
constexpr Text<12> format_fixed(const FixedFormat& f) {
    Text<12> t;
    const bool negative = f.value < 0;
    auto mag = static_cast<uint32_t>(negative ? -static_cast<int64_t>(f.value) : static_cast<int64_t>(f.value));
    if (negative) {
        t.push('-');
    } else if (f.sign == SignMode::Force) {
        t.push('+');
    }

    std::array<uint8_t, 10> digits{};
    std::size_t count = 0;
    do {
        digits[count++] = static_cast<uint8_t>(mag % 10);
        mag /= 10;
    } while (mag > 0);
    while (count < digits.size() && count < static_cast<std::size_t>(f.decimals) + 1u) {
        digits[count++] = 0;
    }

    const std::size_t int_digits = count - f.decimals;
    for (std::size_t i = int_digits; i < f.pad_digits; ++i) {
        t.push(f.pad_zero ? '0' : ' ');
    }
    for (std::size_t i = count; i-- > f.decimals;) {
        t.push(static_cast<char>('0' + digits[i]));
    }
    if (f.decimals > 0) {
        t.push('.');
        for (std::size_t i = f.decimals; i-- > 0;) {
            t.push(static_cast<char>('0' + digits[i]));
        }
    }
    for (const char c : f.suffix) {
        t.push(c);
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
