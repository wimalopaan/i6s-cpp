#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::safety {

enum class StickMode : uint8_t { Mode1 = 0, Mode2 = 1, Mode3 = 2, Mode4 = 3 };

/// Unknown values fall back to Mode 2, as in the Rust firmware.
constexpr StickMode stick_mode_from(const uint8_t val) {
    return val <= 3 ? static_cast<StickMode>(val) : StickMode::Mode2;
}

/// Raw ADC index of the throttle stick: RightVertical (1) in Mode 1/3, LeftVertical (2) in Mode 2/4.
constexpr std::size_t throttle_raw_index(const StickMode m) {
    return (m == StickMode::Mode1 || m == StickMode::Mode3) ? 1 : 2;
}

/// Key bit of the cancel/ESC key as produced by hw::keys::scan_keys().
inline constexpr uint16_t kKeyCancel = 1u << 11;

/// Calibration limits for one 12-bit gimbal channel.
struct ChannelCalibration {
    uint16_t min = 800;
    uint16_t center = 2048;
    uint16_t max = 3300;

    /// Uses stored limits only when they look valid; otherwise nominal defaults.
    static constexpr ChannelCalibration from(const storage::ChannelCalib& c) {
        if (c.min > 200 && c.max > c.center && c.center > c.min) {
            return {c.min, c.center, c.max};
        }
        return {};
    }

    /// Signed permille (-1000..1000) around the center.
    constexpr int16_t normalize_bipolar(const uint16_t raw) const {
        const int32_t r = std::clamp<int32_t>(raw, min, max);
        if (r >= center) {
            const int32_t span = std::max<int32_t>(max - center, 1);
            return static_cast<int16_t>(std::clamp<int32_t>(((r - center) * 1000) / span, 0, 1000));
        }
        const int32_t span = std::max<int32_t>(center - min, 1);
        return static_cast<int16_t>(std::clamp<int32_t>(-(((center - r) * 1000) / span), -1000, 0));
    }

    /// Unipolar permille (0..1000) from min to max.
    constexpr uint16_t normalize_unipolar(const uint16_t raw) const {
        const int32_t r = std::clamp<int32_t>(raw, min, max);
        const int32_t span = std::max<int32_t>(max - min, 1);
        return static_cast<uint16_t>(std::min<int32_t>(((r - min) * 1000) / span, 1000));
    }
};

enum class Preflight : uint8_t { Ok, ThrottleHigh, ThrottleNotCentered, ThrottleHoldDisengaged };

/// Throttle rule at startup. Heli needs the throttle hold (SA Down) instead of a position check.
constexpr Preflight check_preflight_throttle(const std::array<uint16_t, 4>& raw,
                                            const std::array<ChannelCalibration, 4>& cal, const StickMode sm,
                                            const mixer::ModelType mt, const bool throttle_hold) {
    const std::size_t idx = throttle_raw_index(sm);
    switch (mt) {
        case mixer::ModelType::Airplane:
        case mixer::ModelType::Glider:
        case mixer::ModelType::Quad:
            if (cal[idx].normalize_unipolar(raw[idx]) > 20) {
                return Preflight::ThrottleHigh;
            }
            break;
        case mixer::ModelType::General:
            if (std::abs(cal[idx].normalize_bipolar(raw[idx])) > 20) {
                return Preflight::ThrottleNotCentered;
            }
            break;
        case mixer::ModelType::Heli:
            if (!throttle_hold) {
                return Preflight::ThrottleHoldDisengaged;
            }
            break;
    }
    return Preflight::Ok;
}

/// Startup switch rule: all switches up, except SA which is Down for heli (throttle hold).
constexpr bool switches_safe(const Switches& sw, const mixer::ModelType mt) {
    const bool sa_ok = mt == mixer::ModelType::Heli ? sw.sa == SwitchPos::Down : sw.sa == SwitchPos::Up;
    return sa_ok && sw.sb == SwitchPos::Up && sw.sc == SwitchPos::Up && sw.sd == SwitchPos::Up;
}

/// Failsafe pulses during the preflight warning: centered sticks, throttle at minimum, switches safe.
constexpr std::array<uint16_t, kNumChannels> make_safe_idle() {
    std::array<uint16_t, kNumChannels> ch{};
    for (auto& v : ch) {
        v = kChannelCenterUs;
    }
    ch[2] = kChannelMinUs;
    ch[4] = kChannelMinUs;
    ch[5] = kChannelMinUs;
    ch[8] = kChannelMinUs;
    ch[9] = kChannelMinUs;
    return ch;
}

inline constexpr std::array<uint16_t, kNumChannels> kSafeIdleChannels = make_safe_idle();

}  // namespace ferrox::logic::safety
