#pragma once

#include <cstdint>

#include "wm/board.hpp"

namespace wm::logic {

/// Key bits as produced by hw::keys::scan_keys().
inline constexpr uint16_t kKeyRollLeft = 1u << 1;   // Roll L (inward trim)
inline constexpr uint16_t kKeyYawRight = 1u << 6;   // Yaw R (inward trim)
inline constexpr uint16_t kKeyOk = 1u << 10;
inline constexpr uint16_t kKeyCancel = 1u << 11;
inline constexpr uint16_t kKeyBind = 1u << 12;

/// DFU request at power-on. I6X: both horizontal trims pushed inward (Roll Left + Yaw Right).
/// I6S: both rear buttons held (they scan as OK and Cancel).
constexpr bool is_dfu_requested(const uint16_t keys) {
    if constexpr (board::kIsI6S) {
        return (keys & kKeyOk) != 0 && (keys & kKeyCancel) != 0;
    } else {
        return (keys & kKeyRollLeft) != 0 && (keys & kKeyYawRight) != 0;
    }
}

/// I6X: OpenI6X calibrated formula (1/2 divider and 0.20 V series diode).
/// I6S: 10k/5.1k divider, Vbat = raw * 3300 / 4095 * 15100 / 5100.
constexpr uint16_t battery_mv(const uint16_t raw) {
    if constexpr (board::kIsI6S) {
        return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 977'064) / 409'500);
    } else {
        const uint32_t vbat_10mv = ((static_cast<uint32_t>(raw) * 100) / 421) + 20;
        return static_cast<uint16_t>(vbat_10mv * 10);
    }
}

}  // namespace wm::logic
