#pragma once

#include <cstddef>
#include <cstdint>

namespace ferrox::logic {

inline constexpr std::size_t kNumChannels = 18;

// Normalized mixer range.
inline constexpr int16_t kMixerMin = -1000;
inline constexpr int16_t kMixerCenter = 0;
inline constexpr int16_t kMixerMax = 1000;

// AFHDS 2A / i-BUS pulse range in microseconds.
inline constexpr uint16_t kChannelMinUs = 988;
inline constexpr uint16_t kChannelCenterUs = 1500;
inline constexpr uint16_t kChannelMaxUs = 2012;
inline constexpr uint16_t kChannelSpanUs = kChannelMaxUs - kChannelMinUs;
inline constexpr int32_t kChannelHalfSpanUs = kChannelSpanUs / 2;
inline constexpr uint16_t kChannelReverseSum = kChannelMinUs + kChannelMaxUs;

}  // namespace ferrox::logic
