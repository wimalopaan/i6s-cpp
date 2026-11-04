#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>

#include "wm/logic/crsf.hpp"
#include "wm/logic/framebuffer.hpp"

namespace wm::logic::multiswitch {

inline constexpr uint32_t kRepeatMs = 100;
inline constexpr uint8_t kSwitchCount = 8;
inline constexpr uint8_t kMaxPages = 4;
using Addresses = std::array<int16_t, kMaxPages>;

constexpr uint8_t enabled_count(const Addresses& addresses) {
    uint8_t count = 0;
    for (const auto address : addresses) {
        if (address >= 0 && address <= 255) {
            ++count;
        }
    }
    return count;
}

constexpr std::optional<uint8_t> slot_at(const Addresses& addresses, uint8_t ordinal) {
    for (uint8_t slot = 0; slot < kMaxPages; ++slot) {
        if (addresses[slot] >= 0 && addresses[slot] <= 255) {
            if (ordinal == 0) {
                return slot;
            }
            --ordinal;
        }
    }
    return std::nullopt;
}

constexpr display::Rect switch_rect(const uint8_t index) {
    return {{1 + (index % 4) * 32, 14 + (index / 4) * 18}, {30, 17}};
}

constexpr std::optional<uint8_t> switch_at(const uint8_t x, const uint8_t y) {
    for (uint8_t i = 0; i < kSwitchCount; ++i) {
        const auto r = switch_rect(i);
        if (x >= r.pos.x && x < r.pos.x + r.size.w && y >= r.pos.y && y < r.pos.y + r.size.h) {
            return i;
        }
    }
    return std::nullopt;
}

constexpr std::array<uint8_t, 12> build_set4(const uint8_t address, const uint8_t switches) {
    uint16_t state = 0;
    for (uint8_t i = 0; i < kSwitchCount; ++i) {
        if ((switches & (1u << i)) != 0) {
            state |= static_cast<uint16_t>(1u << (2 * i));
        }
    }
    std::array<uint8_t, 12> frame{crsf::kSyncByte, 10, 0x32, 0, crsf::kAddrRadioTransmitter,
                                 0xA1, 0x07, address, static_cast<uint8_t>(state >> 8),
                                 static_cast<uint8_t>(state), 0, 0};
    // Command CRC uses polynomial 0xBA, independently of the outer CRSF CRC (0xD5).
    uint8_t crc = 0;
    for (const uint8_t b : std::span<const uint8_t>(frame).subspan(2, 8)) {
        crc ^= b;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0xBAu)
                                     : static_cast<uint8_t>(crc << 1);
        }
    }
    frame[10] = crc;
    frame[11] = crsf::crc8(std::span<const uint8_t>(frame).subspan(2, 9));
    return frame;
}

struct State {
    uint8_t switches = 0;
    uint32_t last_tx_ms = 0;
    bool pending = true;

    constexpr void toggle(const uint8_t index) {
        switches ^= static_cast<uint8_t>(1u << index);
        pending = true;
    }

    constexpr bool due(const uint32_t now_ms) const {
        return pending || now_ms - last_tx_ms >= kRepeatMs;
    }

    constexpr void sent(const uint32_t now_ms) {
        last_tx_ms = now_ms;
        pending = false;
    }
};

struct Pages {
    Addresses addresses{-1, -1, -1, -1};
    std::array<State, kMaxPages> states{};
    uint8_t next_slot = 0;

    constexpr void configure(const Addresses& configured) {
        for (uint8_t slot = 0; slot < kMaxPages; ++slot) {
            if (addresses[slot] != configured[slot]) {
                states[slot] = {};
                addresses[slot] = configured[slot];
            }
        }
    }

    constexpr std::optional<uint8_t> due(const uint32_t now_ms) const {
        for (uint8_t i = 0; i < kMaxPages; ++i) {
            const auto slot = static_cast<uint8_t>((next_slot + i) % kMaxPages);
            if (addresses[slot] >= 0 && addresses[slot] <= 255 && states[slot].due(now_ms)) {
                return slot;
            }
        }
        return std::nullopt;
    }

    constexpr void sent(const uint8_t slot, const uint32_t now_ms) {
        states[slot].sent(now_ms);
        next_slot = static_cast<uint8_t>((slot + 1) % kMaxPages);
    }

    constexpr std::array<uint8_t, kMaxPages> switches() const {
        std::array<uint8_t, kMaxPages> result{};
        for (uint8_t slot = 0; slot < kMaxPages; ++slot) {
            result[slot] = states[slot].switches;
        }
        return result;
    }
};

}  // namespace wm::logic::multiswitch
