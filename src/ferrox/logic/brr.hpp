#pragma once

#include <array>
#include <cstdint>

namespace ferrox::logic {

inline constexpr uint32_t kSysclkHz = 48'000'000;

/// USART baud-rate register value for a 48 MHz PCLK (rounded to nearest).
consteval uint32_t usart_brr(const uint32_t baud) {
    return (kSysclkHz + baud / 2) / baud;
}

/// Baud rates selectable from the Radio Setup (index 0..4).
inline constexpr std::array<uint32_t, 5> kBaudRates{115'200, 416'666, 420'000, 921'600, 1'875'000};

/// Default ELRS rate (420 kBd) is used for out-of-range indices.
inline constexpr uint8_t kDefaultBaudIndex = 2;

constexpr uint32_t brr_for_baud_index(const uint8_t idx) {
    return idx < kBaudRates.size() ? (kSysclkHz + kBaudRates[idx] / 2) / kBaudRates[idx]
                                   : (kSysclkHz + kBaudRates[kDefaultBaudIndex] / 2) / kBaudRates[kDefaultBaudIndex];
}

}  // namespace ferrox::logic
