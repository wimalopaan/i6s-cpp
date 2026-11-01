#pragma once

#include <cstddef>
#include <string_view>

#include "ferrox/logic/format.hpp"
#include "ferrox/logic/storage.hpp"

namespace ferrox::logic::model_copy {

/// Source model slot. Distinct from DstSlot so the two cannot be swapped in a call.
struct SrcSlot {
    std::size_t value{0};
    friend constexpr bool operator==(const SrcSlot&, const SrcSlot&) = default;
};

/// Destination model slot (0..kNumModels-1).
struct DstSlot {
    std::size_t value{0};
    friend constexpr bool operator==(const DstSlot&, const DstSlot&) = default;
};

/// Slot of the active model. Out-of-range values fall back to slot 0, as in `storage::save_active_model`.
constexpr SrcSlot active_slot(const storage::RadioStorage& s) {
    return SrcSlot{s.radio.active_model < storage::kNumModels ? s.radio.active_model : 0u};
}

/// Destination being chosen on the Copy row (Rust `ctrl.sub_idx`).
struct Picker {
    DstSlot dst{};
    friend constexpr bool operator==(const Picker&, const Picker&) = default;
};

/// Keys seen by the picker in one tick.
struct Keys {
    bool up{false};
    bool down{false};
    bool ok{false};
    bool cancel{false};
};

enum class Outcome : uint8_t { Choosing, Cancelled, Copy };

constexpr DstSlot next_target(const DstSlot dst) { return DstSlot{(dst.value + 1) % storage::kNumModels}; }

constexpr DstSlot prev_target(const DstSlot dst) {
    return DstSlot{(dst.value + storage::kNumModels - 1) % storage::kNumModels};
}

/// Opening the picker selects the slot after the source (Rust `(active_idx + 1) % NUM_MODELS`).
constexpr Picker open(const SrcSlot src) { return Picker{DstSlot{(src.value + 1) % storage::kNumModels}}; }

/// One tick of destination selection. Up and down wrap over all slots; OK confirms, ESC cancels.
constexpr Outcome update(Picker& p, const Keys k) {
    if (k.up) {
        p.dst = next_target(p.dst);
    } else if (k.down) {
        p.dst = prev_target(p.dst);
    }
    if (k.cancel) {
        return Outcome::Cancelled;
    }
    if (k.ok) {
        return Outcome::Copy;
    }
    return Outcome::Choosing;
}

/// Writes the in-memory active model to slot `dst` (Rust `save_model_to_slot(dst, &storage.active_model)`).
/// Only the active slot can be the source, since that is the only model held in memory.
/// Returns false when `src` is not the active slot or the EEPROM write fails.
template <storage::EepromDevice Dev>
bool copy_model(const storage::RadioStorage& s, const SrcSlot src, const DstSlot dst) {
    if (src != active_slot(s)) {
        return false;
    }
    return storage::save_model_to_slot<Dev>(dst.value, s.active_model);
}

/// Row text while the destination is chosen, e.g. "Copy -> M01" (the slot is shown 1-based).
constexpr format::Text<11> target_label(const DstSlot dst) {
    format::Text<11> t;
    for (const char c : std::string_view{"Copy -> M"}) {
        t.push(c);
    }
    const auto digits = format::format_dec2(static_cast<unsigned>(dst.value + 1));
    for (const char c : digits.view()) {
        t.push(c);
    }
    return t;
}

}  // namespace ferrox::logic::model_copy
