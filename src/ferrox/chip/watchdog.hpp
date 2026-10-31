#pragma once

#include <cstdint>

#include "ferrox/arch/regs.hpp"

namespace ferrox::chip::watchdog {

/// Starts the IWDG (LSI 40 kHz, prescaler /64, reload 1250 = 2.0 s). Irreversible until reset.
/// DBGMCU freezes the IWDG while the core is halted by a debugger.
inline void start() {
    // DBGMCU is only accessible with its peripheral clock enabled (APB2ENR bit 22), otherwise HardFault.
    RCC->APB2ENR |= (1u << 22);
    DBGMCU->APB1FZ |= (1u << 12);  // DBG_IWDG_STOP

    IWDG->KR = 0x5555;  // unlock PR and RLR
    IWDG->PR = 4;
    IWDG->RLR = 1250;

    for (uint32_t t = 10'000; t > 0 && (IWDG->SR & 0x3u) != 0; --t) {
    }

    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
}

inline void feed() {
    IWDG->KR = 0xAAAA;
}

}  // namespace ferrox::chip::watchdog
