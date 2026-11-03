#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "wm/arch/regs.hpp"
#include "wm/chip/watchdog.hpp"
#include "wm/hw/gpio.hpp"
#if WM_PCB_I6S
#include "wm/hw/ft6236.hpp"
#endif
#include "wm/logic/storage.hpp"
#include "wm/time/systick.hpp"

namespace wm::hw {

/// External FT24C128A EEPROM on I2C2 (PB10 = SCL, PB11 = SDA, AF1), 400 kHz.
/// FS-i6S: falls back to I2C1 (PB8/PB9) when nothing answers on I2C2.
/// Static backend: satisfies logic::storage::EepromDevice without any virtual dispatch.
struct I2c2Eeprom {
    using Status = logic::storage::Status;

    static constexpr uint8_t kBaseAddr = 0x50;
    static constexpr uint8_t kMaxAddr = 0x57;
    static constexpr uint32_t kSpinLimit = 100000;
    static constexpr uint32_t kStopSpinLimit = 2000;
    static constexpr uint32_t kAckPolls = 50;
    static constexpr std::size_t kPageSize = logic::storage::kPageSize;
    static constexpr std::size_t kMaxChunk = 255;

    // PRESC 0, SCLDEL 3, SDADEL 0, SCLH 9, SCLL 25 (same as the Rust driver).
    static constexpr uint32_t kTimingr =
        (0u << I2C_TIMINGR_PRESC_Pos) | (3u << I2C_TIMINGR_SCLDEL_Pos) |
        (0u << I2C_TIMINGR_SDADEL_Pos) | (9u << I2C_TIMINGR_SCLH_Pos) |
        (25u << I2C_TIMINGR_SCLL_Pos);

    static constexpr uint32_t kErrorMask = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;

    /// Enables the bus and probes the 0x50..0x57 address range. Returns false if no chip answers;
    /// accesses then go to 0x50 (Rust `EEPROM_I2C_ADDR`).
    static bool init() {
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
        // SYSCFG needs its clock for the Fm+ bit below, otherwise the write is lost.
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

        set_mode_mask<Port::B>((1u << 10) | (1u << 11), Mode::Alternate);
        set_alt_function<Port::B>({10}, {1});
        set_alt_function<Port::B>({11}, {1});
        set_open_drain_mask<Port::B>((1u << 10) | (1u << 11), true);
        set_high_speed_mask<Port::B>((1u << 10) | (1u << 11));
        set_pull_mask<Port::B>((1u << 10) | (1u << 11), Pull::Up);

        SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C_FMP_I2C2;
        g_bus = 2;
        configure_peripheral();
        if (probe_bus()) {
            return true;
        }
#if WM_PCB_I6S
        // FS-i6S fallback: I2C1 on PB8/PB9 (shared with the FT6236 touch controller).
        ft6236::init_i2c1();
        g_bus = 1;
        if (probe_bus()) {
            return true;
        }
        g_bus = 2;
        configure_peripheral();
#endif
        g_addr = kBaseAddr;
        return false;
    }

    static Status read(const uint16_t addr, std::span<uint8_t> out) {
        if (out.empty()) {
            return Status::Ok;
        }
        return read_raw(addr, out);
    }

    static Status write(const uint16_t addr, std::span<const uint8_t> data) {
        std::size_t offset = 0;
        while (offset < data.size()) {
            const uint16_t cur = static_cast<uint16_t>(addr + offset);
            const std::size_t page_left = kPageSize - (cur % kPageSize);
            const std::size_t chunk = (data.size() - offset) < page_left ? (data.size() - offset) : page_left;
            if (const Status s = write_page(cur, data.subspan(offset, chunk)); s != Status::Ok) {
                g_write_err = true;
                return s;
            }
            offset += chunk;
        }
        g_write_err = false;
        return Status::Ok;
    }

    /// I2C bus of the device found by the last `init()` probe (1 = I2C1 PB8/PB9, 2 = I2C2 PB10/PB11).
    static uint8_t probed_bus() { return g_bus; }
    /// 7-bit address of the device found by the last `init()` probe.
    static uint8_t probed_addr() { return g_addr; }
    /// True when the most recent `write()` failed.
    static bool write_failed() { return g_write_err; }

private:
    inline static uint8_t g_addr = kBaseAddr;
    inline static uint8_t g_bus = 2;
    inline static bool g_write_err = false;

    static I2C_TypeDef* bus() {
#if WM_PCB_I6S
        if (g_bus == 1) {
            return I2C1;
        }
#endif
        return I2C2;
    }

    static bool probe_bus() {
        for (uint8_t addr = kBaseAddr; addr <= kMaxAddr; ++addr) {
            g_addr = addr;
            std::array<uint8_t, 1> probe{};
            if (read_raw(0, probe) == Status::Ok) {
                return true;
            }
            chip::watchdog::feed();
        }
        return false;
    }

    static void configure_peripheral() {
        bus()->CR1 &= ~I2C_CR1_PE;
#if WM_PCB_I6S
        bus()->TIMINGR = g_bus == 1 ? ft6236::kTimingr : kTimingr;
#else
        bus()->TIMINGR = kTimingr;
#endif
        bus()->CR1 |= I2C_CR1_PE;
    }

    /// Clears a hung bus: requests STOP and waits briefly for it, resets the peripheral, and restores the timing.
    static void recover() {
        if ((bus()->CR1 & I2C_CR1_PE) != 0) {
            bus()->CR2 |= I2C_CR2_STOP;
            for (uint32_t spin = 0; spin < kStopSpinLimit && (bus()->ISR & I2C_ISR_STOPF) == 0; ++spin) {
            }
        }
        bus()->CR1 &= ~I2C_CR1_PE;
        const uint32_t rst = g_bus == 1 ? RCC_APB1RSTR_I2C1RST : RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR |= rst;
        RCC->APB1RSTR &= ~rst;
        bus()->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
        configure_peripheral();
    }

    /// Waits until `ready` is set in ISR. Returns false on error flags or timeout.
    static bool wait_isr(const uint32_t ready) {
        for (uint32_t spin = 0; spin < kSpinLimit; ++spin) {
            const uint32_t isr = bus()->ISR;
            if ((isr & kErrorMask) != 0) {
                return false;
            }
            if ((isr & ready) != 0) {
                return true;
            }
        }
        return false;
    }

    static bool wait_idle() {
        for (uint32_t spin = 0; spin < kSpinLimit; ++spin) {
            if ((bus()->ISR & I2C_ISR_BUSY) == 0) {
                return true;
            }
        }
        return false;
    }

    static bool fail(Status& out) {
        recover();
        out = Status::I2cError;
        return false;
    }

    static Status read_raw(const uint16_t addr, std::span<uint8_t> out) {
        Status status = Status::Ok;
        if (!wait_idle()) {
            fail(status);
            return status;
        }
        bus()->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

        // Phase 1: write the 16-bit memory address without AUTOEND (repeated START follows).
        bus()->CR2 = (static_cast<uint32_t>(g_addr) << 1) | (2u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        bus()->TXDR = (addr >> 8) & 0xFF;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        bus()->TXDR = addr & 0xFF;
        if (!wait_isr(I2C_ISR_TC)) {
            fail(status);
            return status;
        }

        // Phase 2: chunked reads; the last chunk ends with STOP (AUTOEND).
        std::size_t offset = 0;
        while (offset < out.size()) {
            const std::size_t left = out.size() - offset;
            const std::size_t chunk = left < kMaxChunk ? left : kMaxChunk;
            const bool last = (offset + chunk) == out.size();
            uint32_t cr2 = (static_cast<uint32_t>(g_addr) << 1) | I2C_CR2_RD_WRN |
                           (static_cast<uint32_t>(chunk) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
            if (last) {
                cr2 |= I2C_CR2_AUTOEND;
            }
            bus()->CR2 = cr2;
            for (std::size_t i = 0; i < chunk; ++i) {
                if (!wait_isr(I2C_ISR_RXNE)) {
                    fail(status);
                    return status;
                }
                out[offset + i] = static_cast<uint8_t>(bus()->RXDR & I2C_RXDR_RXDATA);
            }
            if (last) {
                if (!wait_isr(I2C_ISR_STOPF)) {
                    fail(status);
                    return status;
                }
                bus()->ICR = I2C_ICR_STOPCF;
            } else if (!wait_isr(I2C_ISR_TC)) {
                fail(status);
                return status;
            }
            offset += chunk;
        }
        return Status::Ok;
    }

    /// Writes one chunk that does not cross a page boundary, then polls for the internal write cycle.
    static Status write_page(const uint16_t addr, std::span<const uint8_t> data) {
        Status status = Status::Ok;
        if (!wait_idle()) {
            fail(status);
            return status;
        }
        bus()->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

        const uint32_t nbytes = static_cast<uint32_t>(data.size() + 2);
        bus()->CR2 = (static_cast<uint32_t>(g_addr) << 1) | (nbytes << I2C_CR2_NBYTES_Pos) |
                    I2C_CR2_AUTOEND | I2C_CR2_START;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        bus()->TXDR = (addr >> 8) & 0xFF;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        bus()->TXDR = addr & 0xFF;
        for (const uint8_t b : data) {
            if (!wait_isr(I2C_ISR_TXIS)) {
                fail(status);
                return status;
            }
            bus()->TXDR = b;
        }
        if (!wait_isr(I2C_ISR_STOPF)) {
            fail(status);
            return status;
        }
        bus()->ICR = I2C_ICR_STOPCF;
        return ack_poll();
    }

    /// Waits for the EEPROM to finish its internal write by probing with a 0-byte write.
    static Status ack_poll() {
        for (uint32_t poll = 0; poll < kAckPolls; ++poll) {
            if (wait_idle()) {
                bus()->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
                bus()->CR2 = (static_cast<uint32_t>(g_addr) << 1) | I2C_CR2_AUTOEND | I2C_CR2_START;
                for (uint32_t spin = 0; spin < kSpinLimit; ++spin) {
                    const uint32_t isr = bus()->ISR;
                    if ((isr & I2C_ISR_NACKF) != 0) {
                        break;
                    }
                    if ((isr & I2C_ISR_STOPF) != 0) {
                        bus()->ICR = I2C_ICR_STOPCF;
                        return Status::Ok;
                    }
                }
            }
            bus()->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            const uint32_t start = time::millis();
            while (time::millis() - start < 1) {
                chip::watchdog::feed();
            }
        }
        recover();
        return Status::I2cError;
    }
};

}  // namespace wm::hw
