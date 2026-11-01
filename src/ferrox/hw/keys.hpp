#pragma once

#include <array>
#include <cstdint>

#include "ferrox/arch/regs.hpp"
#include "ferrox/chip/mcu.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/power.hpp"

namespace ferrox::hw::keys {

// Column outputs PC6..PC8, row inputs PD12..PD15, external switches PC12/PC15, Bind PF2.
inline constexpr uint32_t kColumnMask = (1u << 6) | (1u << 7) | (1u << 8);
inline constexpr uint32_t kRowMask = 0xF000u;   // PD12..PD15
inline constexpr uint32_t kSwitchMask = (1u << 12) | (1u << 15);
inline constexpr uint32_t kBindPin = 2;         // PF2, active low

// Magic word in the top 16 bytes of 16 KB SRAM: survives a soft reset, requests DFU.
inline constexpr uint32_t kDfuMagic = 0xDEAD'BEEF;
inline constexpr uintptr_t kDfuFlagAddr = 0x2000'3FF0;

inline void delay_cycles(const uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        __asm volatile("nop");
    }
}

inline void init_keys() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOFEN;

    set_mode_mask<Port::C>(kColumnMask, Mode::Output);
    set_mode_mask<Port::C>(kSwitchMask, Mode::Input);
    set_pull_mask<Port::C>(kSwitchMask, Pull::Up);
    set_pins<Port::C>(kColumnMask);

    set_mode_mask<Port::D>(kRowMask, Mode::Input);
    set_pull_mask<Port::D>(kRowMask, Pull::Up);

    set_mode_mask<Port::F>(1u << kBindPin, Mode::Input);
    set_pull_mask<Port::F>(1u << kBindPin, Pull::Up);
}

/// Scans the 3x4 matrix plus the Bind key. Bit layout as in the Rust driver:
/// bits 0..11 = column * 4 + row, bit 12 = Bind (PF2).
inline uint16_t scan_keys() {
    constexpr std::array<uint32_t, 3> columns{6, 7, 8};
    uint16_t result = 0;

    for (uint32_t idx = 0; idx < columns.size(); ++idx) {
        const uint32_t col = columns[idx];
        clear_pins<Port::C>(1u << col);
        delay_cycles(150);

        const uint32_t lines = (read_pins<Port::D>() >> 12) & 0xFu;
        const uint32_t pressed = (~lines) & 0xFu;
        result = static_cast<uint16_t>(result | (pressed << (idx * 4)));

        set_pins<Port::C>(1u << col);
        delay_cycles(100);
    }

    if ((read_pins<Port::F>() & (1u << kBindPin)) == 0) {
        result = static_cast<uint16_t>(result | (1u << 12));
    }
    return result;
}

/// Reads and clears the SRAM DFU magic word.
inline bool take_sram_dfu_flag() {
    auto* flag = reinterpret_cast<volatile uint32_t*>(kDfuFlagAddr);
    if (*flag == kDfuMagic) {
        *flag = 0;
        return true;
    }
    return false;
}

/// Power-on DFU check: flag from a previous request, or Roll L + Yaw R held (3 of 5 samples).
inline void check_dfu_entry(const chip::McuProfile& profile) {
    init_keys();

    if (take_sram_dfu_flag()) {
        chip::enter_dfu_bootloader(profile);
    }

    // ~20 ms for power rails and switch contacts to settle
    delay_cycles(40'000);

    uint32_t matches = 0;
    for (uint32_t i = 0; i < 5; ++i) {
        if (logic::is_dfu_requested(scan_keys())) {
            ++matches;
        }
        delay_cycles(2'000);
    }

    if (matches >= 3) {
        chip::enter_dfu_bootloader(profile);
    }
}

}  // namespace ferrox::hw::keys
