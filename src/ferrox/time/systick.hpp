#pragma once

#include <cstdint>

#include "ferrox/arch/regs.hpp"

namespace ferrox::time {

// volatile instead of std::atomic: single-core ARMv6-M, no libatomic available.
inline constinit volatile uint32_t g_system_millis = 0;

/// 1 kHz SysTick on the 48 MHz core clock. SysTick_Config sets the lowest priority (0xC0),
/// so it never delays the RF-critical IRQs.
inline void init() {
    SysTick_Config(48'000);
}

/// Called from SysTick_Handler.
inline void on_tick() {
    g_system_millis = g_system_millis + 1;
}

inline uint32_t millis() {
    return g_system_millis;
}

}  // namespace ferrox::time
