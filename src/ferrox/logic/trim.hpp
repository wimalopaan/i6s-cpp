#pragma once

#include <algorithm>
#include <cstdint>

#include "ferrox/logic/channels.hpp"

namespace ferrox::logic::trim {

inline constexpr int8_t kTrimMin = -25;
inline constexpr int8_t kTrimMax = 25;
inline constexpr int16_t kTrimStepUs = 4;
inline constexpr int16_t kTrimMaxOffsetUs = kTrimMax * kTrimStepUs;

/// Trim values of the four primary flight channels.
struct TrimValues {
    int8_t roll{0};      // CH1 (AIL)
    int8_t pitch{0};     // CH2 (ELE)
    int8_t throttle{0};  // CH3 (THR)
    int8_t yaw{0};       // CH4 (RUD)
};

/// Adds the linear trim offset to a pulse width and clamps it to the channel range.
constexpr uint16_t apply(uint16_t base_us, int8_t trim) {
    const int32_t offset = static_cast<int32_t>(trim) * kTrimStepUs;
    return static_cast<uint16_t>(std::clamp<int32_t>(base_us + offset, kChannelMinUs, kChannelMaxUs));
}

/// Throttle trim: mode 0 = locked, 1 = idle trim (tapers to 0 at full stick), 2 = linear.
constexpr uint16_t apply_throttle(uint16_t base_us, int8_t trim, uint8_t mode) {
    switch (mode) {
        case 1: {
            const int32_t max_offset = static_cast<int32_t>(trim) * kTrimStepUs;
            const int32_t travel = std::clamp<int32_t>(base_us - kChannelMinUs, 0, kChannelSpanUs);
            const int32_t effective = (max_offset * (kChannelSpanUs - travel)) / kChannelSpanUs;
            const int32_t lo = kChannelMinUs - kTrimMaxOffsetUs;
            const int32_t hi = kChannelMaxUs + kTrimMaxOffsetUs;
            return static_cast<uint16_t>(std::clamp<int32_t>(base_us + effective, lo, hi));
        }
        case 2:
            return apply(base_us, trim);
        default:
            return static_cast<uint16_t>(std::clamp<uint16_t>(base_us, kChannelMinUs, kChannelMaxUs));
    }
}

}  // namespace ferrox::logic::trim
