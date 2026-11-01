#pragma once

#include <cstdint>

namespace ferrox::logic {

/// Key bits as produced by hw::keys::scan_keys().
inline constexpr uint16_t kKeyRollLeft = 1u << 1;   // Roll L (inward trim)
inline constexpr uint16_t kKeyYawRight = 1u << 6;   // Yaw R (inward trim)
inline constexpr uint16_t kKeyOk = 1u << 10;
inline constexpr uint16_t kKeyBind = 1u << 12;

/// Both horizontal trims pushed inward (Roll Left + Yaw Right) requests the DFU bootloader.
constexpr bool is_dfu_requested(uint16_t keys) {
    return (keys & kKeyRollLeft) != 0 && (keys & kKeyYawRight) != 0;
}

/// OpenI6X calibrated battery formula: 1/2 divider and 0.20 V series diode.
constexpr uint16_t battery_mv(uint16_t raw) {
    const uint32_t vbat_10mv = ((static_cast<uint32_t>(raw) * 100) / 421) + 20;
    return static_cast<uint16_t>(vbat_10mv * 10);
}

}  // namespace ferrox::logic
