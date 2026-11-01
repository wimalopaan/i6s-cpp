#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/storage.hpp"

namespace ferrox::logic::protocol {

/// RF protocol of a model, stored as `ModelConfig::rf_protocol`.
enum class Protocol : uint8_t { Afhds2a = 0, Crsf = 1 };

/// Stored value to protocol. Values above 1 fall back to AFHDS2A, as `storage::sanitize` does.
constexpr Protocol from_raw(const uint8_t raw) {
    return raw == 1 ? Protocol::Crsf : Protocol::Afhds2a;
}

constexpr uint8_t to_raw(const Protocol p) {
    return static_cast<uint8_t>(p);
}

constexpr Protocol toggle(const Protocol p) {
    return p == Protocol::Afhds2a ? Protocol::Crsf : Protocol::Afhds2a;
}

constexpr std::string_view label(const Protocol p) {
    return p == Protocol::Crsf ? std::string_view{"CRSF"} : std::string_view{"AFHDS2A"};
}

/// Hardware state that follows the protocol.
struct Link {
    bool crsf_uart{false};   // CRSF on USART2 and its module power
    bool rf_silenced{false}; // A7105 muted, so it does not transmit AFHDS2A frames
    friend constexpr bool operator==(const Link&, const Link&) = default;
};

constexpr Link link_for(const Protocol p) {
    return p == Protocol::Crsf ? Link{true, true} : Link{false, false};
}

/// Model slot index, 0..kNumModels-1.
struct ModelSlot {
    std::size_t value{0};
    friend constexpr bool operator==(const ModelSlot&, const ModelSlot&) = default;
};

/// Factory defaults for a model slot (Rust `ModelConfig::default_for_index`). Clears the bound RX ID.
constexpr storage::ModelConfig reset_model(const ModelSlot slot) {
    return storage::default_model_config(slot.value);
}

}  // namespace ferrox::logic::protocol
