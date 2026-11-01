#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ferrox/arch/regs.hpp"

namespace ferrox::chip {

enum class McuType : uint8_t { Stm32f072, Apm32f072 };

struct McuProfile {
    McuType type;
    const char* name;
    uintptr_t uid_addr;
    uintptr_t dfu_bootloader_addr;
};

#if defined(FERROX_MCU_APM32)
inline constexpr McuProfile kProfile{McuType::Apm32f072, "APM32F072VB", 0x1FFFF7E8, 0x1FFFF000};
#else
inline constexpr McuProfile kProfile{McuType::Stm32f072, "STM32F072VB", 0x1FFFF7AC, 0x1FFFC800};
#endif

/// Bounded spin count for oscillator / PLL ready flags (never an unbounded wait).
inline constexpr uint32_t kReadyTimeout = 100'000;

inline std::array<uint8_t, 12> read_uid(const McuProfile& profile) {
    std::array<uint8_t, 12> uid{};
    for (std::size_t i = 0; i < uid.size(); ++i) {
        uid[i] = *reinterpret_cast<volatile const uint8_t*>(profile.uid_addr + i);
    }
    return uid;
}

/// 48 MHz SYSCLK from the 8 MHz HSE crystal via PLL x6 (matches OpenI6X SetSysClock).
/// Also enables LSI (40 kHz) for the watchdog.
inline void init_system_clock() {
    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t t = kReadyTimeout; t > 0 && (RCC->CR & RCC_CR_HSERDY) == 0; --t) {
    }

    if ((RCC->CR & RCC_CR_HSERDY) != 0) {
        // Flash: 1 wait state (required at 48 MHz) + prefetch buffer enable
        FLASH->ACR = 1u | (1u << 4);

        // PLLSRC = HSE/PREDIV (bit 16), PLLMUL = x6 (value 0b0100 at bit 18)
        const uint32_t cfgr = RCC->CFGR & ~(0x003F'0000u | 0x3u);
        RCC->CFGR = cfgr | (1u << 16) | 0x0010'0000u;

        RCC->CR |= RCC_CR_PLLON;
        for (uint32_t t = kReadyTimeout; t > 0 && (RCC->CR & RCC_CR_PLLRDY) == 0; --t) {
        }

        // SW = PLL
        RCC->CFGR = (RCC->CFGR & ~0x3u) | 0x2u;
        for (uint32_t t = kReadyTimeout; t > 0 && (RCC->CFGR & 0x0Cu) != 0x08u; --t) {
        }

        // USBSW = PLLCLK (48 MHz USB clock)
        RCC->CFGR3 |= (1u << 7);
    }

    RCC->CSR |= RCC_CSR_LSION;
    for (uint32_t t = kReadyTimeout; t > 0 && (RCC->CSR & RCC_CSR_LSIRDY) == 0; --t) {
    }
}

/// Returns true for a warm reset (IWDG, WWDG or software) and clears the RCC reset flags.
inline bool check_and_clear_reset_flags() {
    const uint32_t csr = RCC->CSR;
    const bool warm = (csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF)) != 0;
    RCC->CSR |= RCC_CSR_RMVF;
    return warm;
}

/// Restores the power-on RCC state (HSI, no PLL, peripheral clocks off except SYSCFG/FLITF/SRAM).
inline void rcc_deinit() {
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
    }

    RCC->CFGR = 0;
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
    RCC->CIR = 0;

    RCC->APB2RSTR = 0xFFFF'FFFFu;
    RCC->APB2RSTR = 0;
    RCC->APB1RSTR = 0xFFFF'FFFFu;
    RCC->APB1RSTR = 0;

    RCC->AHBENR = (1u << 4) | 1u;  // SRAMEN | FLITFEN
    RCC->APB2ENR = 1u;             // SYSCFGEN
    RCC->APB1ENR = 0;
}

/// Jumps to the factory system-memory DFU bootloader (ST ROM or APM32 equivalent).
[[noreturn]] inline void enter_dfu_bootloader(const McuProfile& profile) {
    rcc_deinit();

    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    SysTick->VAL = 0;

    NVIC->ICER[0] = 0xFFFF'FFFFu;
    NVIC->ICPR[0] = 0xFFFF'FFFFu;

    __disable_irq();
    __DSB();

    // SYSCFG MEM_MODE = 0b01: system memory mapped at address 0
    SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~0x3u) | 0x1u;
    __DSB();
    __ISB();

    const uint32_t initial_msp = *reinterpret_cast<volatile const uint32_t*>(profile.dfu_bootloader_addr);
    const uint32_t reset_handler =
        *reinterpret_cast<volatile const uint32_t*>(profile.dfu_bootloader_addr + 4);

    // The ROM DFU stack requires interrupts to be enabled.
    __enable_irq();

    __asm volatile("msr msp, %0\n\t"
                   "bx %1"
                   :
                   : "r"(initial_msp), "r"(reset_handler)
                   : "memory");
    __builtin_unreachable();
}

}  // namespace ferrox::chip
