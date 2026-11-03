#pragma once

// Pure trainer logic (Rust `trainer/ppm_in.rs`, `trainer/ppm_out.rs`, `trainer/handover.rs`):
// the PPM decoder for the student input, the 8-channel PPM frame periods for the
// trainer output, and the safety handover between instructor sticks and student.
// Timer, DMA and pin access live in `hw/trainer.hpp`.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <optional>

namespace wm::logic::trainer {

// Strong types so pulse widths and elapsed time cannot be mixed up at a call site.
struct PulseUs {
    uint16_t value = 0;
    constexpr bool operator==(const PulseUs&) const = default;
};

struct DeltaMs {
    uint16_t value = 0;
    constexpr bool operator==(const DeltaMs&) const = default;
};

inline constexpr std::size_t kPpmInChannels = 8;
inline constexpr std::size_t kPpmOutChannels = 8;
inline constexpr uint16_t kCenterUs = 1500;
inline constexpr uint16_t kDecodeMinUs = 988;
inline constexpr uint16_t kDecodeMaxUs = 2012;
inline constexpr uint16_t kChannelMinUs = 850;
inline constexpr uint16_t kChannelMaxUs = 2150;
inline constexpr uint16_t kSyncMinUs = 3000;
inline constexpr uint16_t kSyncMaxUs = 30000;
inline constexpr uint16_t kSignalTimeoutMs = 100;
inline constexpr std::size_t kMinChannelsForFrame = 4;

inline constexpr uint16_t kOutMinUs = 900;
inline constexpr uint16_t kOutMaxUs = 2100;
inline constexpr uint16_t kOutSyncUs = 400;
inline constexpr uint16_t kOutMinHighUs = 500;
inline constexpr uint16_t kOutMinGapUs = 3000;
inline constexpr uint32_t kFrameTotalUs = 22500;

inline constexpr uint16_t kTakeoverThresholdUs = 50;
inline constexpr uint16_t kThrottleTakeoverMinUs = 1050;

using InChannels = std::array<uint16_t, kPpmInChannels>;
using OutChannels = std::array<uint16_t, kPpmOutChannels>;

class PpmDecoder {
public:
    constexpr PpmDecoder() {
        channels_.fill(kCenterUs);
        temp_.fill(kCenterUs);
    }

    constexpr void process_pulse(const PulseUs pulse) {
        if (pulse.value >= kSyncMinUs && pulse.value < kSyncMaxUs) {
            if (in_sync_ && current_ >= kMinChannelsForFrame) {
                std::copy_n(temp_.begin(), current_, channels_.begin());
                valid_ = true;
                timer_ms_ = kSignalTimeoutMs;
                ++frame_count_;
            }
            in_sync_ = true;
            current_ = 0;
        } else if (in_sync_) {
            if (pulse.value >= kChannelMinUs && pulse.value <= kChannelMaxUs) {
                if (current_ < kPpmInChannels) {
                    temp_[current_] = std::clamp(pulse.value, kDecodeMinUs, kDecodeMaxUs);
                    ++current_;
                }
            } else {
                in_sync_ = false;
                current_ = 0;
            }
        }
    }

    constexpr void tick(const DeltaMs dt) {
        if (timer_ms_ > 0) {
            timer_ms_ = dt.value >= timer_ms_ ? 0 : static_cast<uint16_t>(timer_ms_ - dt.value);
            if (timer_ms_ == 0) {
                valid_ = false;
            }
        }
    }

    constexpr bool is_signal_valid() const { return valid_ && timer_ms_ > 0; }

    constexpr std::optional<InChannels> channels() const {
        if (is_signal_valid()) {
            return channels_;
        }
        return std::nullopt;
    }

    constexpr uint32_t frame_count() const { return frame_count_; }

    constexpr void reset() {
        current_ = 0;
        in_sync_ = false;
        timer_ms_ = 0;
        valid_ = false;
    }

private:
    InChannels channels_{};
    InChannels temp_{};
    std::size_t current_ = 0;
    bool in_sync_ = false;
    uint16_t timer_ms_ = 0;
    bool valid_ = false;
    uint32_t frame_count_ = 0;
};

// Timer periods (in us) of one PPM output frame. Every period starts with a kOutSyncUs low pulse and
// ends high, so a hardware PWM with a fixed compare value and a per-period ARR produces the frame.
// Periods 0..7 carry the channels (clamped to 900..2100 us); the last one is the sync gap that pads
// the frame to kFrameTotalUs (at least kOutMinGapUs high). Missing channels are centred.
inline constexpr std::size_t kPpmOutPeriods = kPpmOutChannels + 1;
using OutPeriods = std::array<uint16_t, kPpmOutPeriods>;

template <std::size_t N>
constexpr OutPeriods ppm_out_periods(const std::array<uint16_t, N>& chs) {
    constexpr uint16_t kMinPeriodUs = kOutSyncUs + kOutMinHighUs;
    OutPeriods periods{};
    uint32_t used = kOutSyncUs;
    for (std::size_t i = 0; i < kPpmOutChannels; ++i) {
        const uint16_t ch = i < N ? std::clamp(chs[i], kOutMinUs, kOutMaxUs) : kCenterUs;
        periods[i] = std::max(ch, kMinPeriodUs);
        used += periods[i];
    }
    const uint32_t gap = used < kFrameTotalUs ? kFrameTotalUs - used : 0;
    periods[kPpmOutChannels] = static_cast<uint16_t>(kOutSyncUs + std::max<uint32_t>(gap, kOutMinGapUs));
    return periods;
}

enum class State : uint8_t { Disabled, StudentActive, InstructorOverride, SignalLost };

// Instructor sticks count as deflected when roll, pitch or yaw is more than 50 us off centre,
// or the throttle is more than 50 us above idle.
constexpr bool is_instructor_deflected(const std::array<uint16_t, 4>& instructor) {
    const auto off = [](uint16_t v) { return std::abs(static_cast<int>(v) - kCenterUs) > kTakeoverThresholdUs; };
    return off(instructor[0]) || off(instructor[1]) || instructor[2] > kThrottleTakeoverMinUs || off(instructor[3]);
}

class Handover {
public:
    // Output is the instructor frame, with the first 4 channels replaced by the student's when the student is active.
    template <std::size_t N>
    constexpr std::array<uint16_t, N> process(const bool trainer_switch_active,
                                              const std::optional<InChannels>& student,
                                              const std::array<uint16_t, N>& instructor) {
        auto output = instructor;
        if (!trainer_switch_active) {
            state_ = State::Disabled;
            return output;
        }
        if (!student) {
            state_ = State::SignalLost;
            return output;
        }
        const std::array<uint16_t, 4> sticks{instructor[0], instructor[1], instructor[2], instructor[3]};
        if (is_instructor_deflected(sticks)) {
            state_ = State::InstructorOverride;
            return output;
        }
        state_ = State::StudentActive;
        std::copy_n(student->begin(), std::min<std::size_t>(4, N), output.begin());
        return output;
    }

    constexpr State state() const { return state_; }

private:
    State state_ = State::Disabled;
};

}  // namespace wm::logic::trainer
