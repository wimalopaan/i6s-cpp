#pragma once

// Pure trainer logic (Rust `trainer/ppm_in.rs`, `trainer/ppm_out.rs`, `trainer/handover.rs`):
// the PPM decoder for the student input, the 8-channel PPM generator for the
// trainer output, and the safety handover between instructor sticks and student.
// Timer capture, compare and pin access live in `hw/trainer.hpp`.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <optional>

namespace ferrox::logic::trainer {

// Strong types so pulse widths and elapsed time cannot be mixed up at a call site.
struct PulseUs {
    uint16_t value = 0;
    constexpr bool operator==(const PulseUs&) const = default;
};

struct DeltaMs {
    uint16_t value = 0;
    constexpr bool operator==(const DeltaMs&) const = default;
};

struct Interval {
    PulseUs duration{};
    bool high = false;
    constexpr bool operator==(const Interval&) const = default;
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

    constexpr void process_pulse(PulseUs pulse) {
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

    constexpr void tick(DeltaMs dt) {
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

class PpmGenerator {
public:
    constexpr PpmGenerator() { channels_.fill(kCenterUs); }

    // Copies up to 8 channels, clamped to 900..2100 us.
    template <std::size_t N>
    constexpr void update_channels(const std::array<uint16_t, N>& chs) {
        constexpr std::size_t kCount = std::min(N, kPpmOutChannels);
        for (std::size_t i = 0; i < kCount; ++i) {
            channels_[i] = std::clamp(chs[i], kOutMinUs, kOutMaxUs);
        }
    }

    constexpr void reset_frame() {
        current_ = 0;
        in_sync_pulse_ = true;
        elapsed_us_ = 0;
    }

    // Returns the next low or high interval. The 18 intervals of a frame sum to kFrameTotalUs.
    constexpr Interval next_interval() {
        if (current_ < kPpmOutChannels) {
            if (in_sync_pulse_) {
                in_sync_pulse_ = false;
                elapsed_us_ += kOutSyncUs;
                return {PulseUs{kOutSyncUs}, false};
            }
            const uint16_t high = static_cast<uint16_t>(std::max<int>(channels_[current_] - kOutSyncUs, kOutMinHighUs));
            in_sync_pulse_ = true;
            ++current_;
            elapsed_us_ += high;
            return {PulseUs{high}, true};
        }
        if (in_sync_pulse_) {
            in_sync_pulse_ = false;
            elapsed_us_ += kOutSyncUs;
            return {PulseUs{kOutSyncUs}, false};
        }
        const uint32_t gap = elapsed_us_ < kFrameTotalUs ? kFrameTotalUs - elapsed_us_ : 0;
        current_ = 0;
        in_sync_pulse_ = true;
        elapsed_us_ = 0;
        return {PulseUs{static_cast<uint16_t>(std::max<uint32_t>(gap, kOutMinGapUs))}, true};
    }

private:
    OutChannels channels_{};
    std::size_t current_ = 0;
    bool in_sync_pulse_ = true;
    uint32_t elapsed_us_ = 0;
};

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
    constexpr std::array<uint16_t, N> process(bool trainer_switch_active,
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

}  // namespace ferrox::logic::trainer
