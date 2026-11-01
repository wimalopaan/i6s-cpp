#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ferrox/arch/regs.hpp"
#include "ferrox/chip/watchdog.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/time/systick.hpp"

namespace ferrox::hw {

/// External FT24C128A EEPROM on I2C2 (PB10 = SCL, PB11 = SDA, AF1), 400 kHz.
/// Static backend: satisfies logic::storage::EepromDevice without any virtual dispatch.
struct I2c2Eeprom {
    using Status = logic::storage::Status;

    static constexpr uint8_t kBaseAddr = 0x50;
    static constexpr uint8_t kMaxAddr = 0x57;
    static constexpr uint32_t kSpinLimit = 100000;
    static constexpr uint32_t kAckPolls = 50;
    static constexpr std::size_t kPageSize = logic::storage::kPageSize;
    static constexpr std::size_t kMaxChunk = 255;

    // PRESC 0, SCLDEL 3, SDADEL 0, SCLH 9, SCLL 25 (same as the Rust driver).
    static constexpr uint32_t kTimingr =
        (0u << I2C_TIMINGR_PRESC_Pos) | (3u << I2C_TIMINGR_SCLDEL_Pos) |
        (0u << I2C_TIMINGR_SDADEL_Pos) | (9u << I2C_TIMINGR_SCLH_Pos) |
        (25u << I2C_TIMINGR_SCLL_Pos);

    static constexpr uint32_t kErrorMask = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;

    /// Enables the bus and probes the 0x50..0x57 address range. Returns false if no chip answers.
    static bool init() {
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

        set_mode_mask<Port::B>((1u << 10) | (1u << 11), Mode::Alternate);
        set_alt_function<Port::B>({10}, {1});
        set_alt_function<Port::B>({11}, {1});
        set_open_drain_mask<Port::B>((1u << 10) | (1u << 11), true);
        set_high_speed_mask<Port::B>((1u << 10) | (1u << 11));
        set_pull_mask<Port::B>((1u << 10) | (1u << 11), Pull::Up);

        SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C_FMP_I2C2;
        configure_peripheral();

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

    /// 7-bit address of the device found by the last `init()` probe.
    static uint8_t probed_addr() { return g_addr; }
    /// True when the most recent `write()` failed.
    static bool write_failed() { return g_write_err; }

private:
    inline static uint8_t g_addr = kBaseAddr;
    inline static bool g_write_err = false;

    static void configure_peripheral() {
        I2C2->CR1 &= ~I2C_CR1_PE;
        I2C2->TIMINGR = kTimingr;
        I2C2->CR1 |= I2C_CR1_PE;
    }

    /// Clears a hung bus: requests STOP, resets the peripheral, and restores the timing.
    static void recover() {
        if ((I2C2->CR1 & I2C_CR1_PE) != 0) {
            I2C2->CR2 |= I2C_CR2_STOP;
        }
        I2C2->CR1 &= ~I2C_CR1_PE;
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
        I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
        configure_peripheral();
    }

    /// Waits until `ready` is set in ISR. Returns false on error flags or timeout.
    static bool wait_isr(const uint32_t ready) {
        for (uint32_t spin = 0; spin < kSpinLimit; ++spin) {
            const uint32_t isr = I2C2->ISR;
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
            if ((I2C2->ISR & I2C_ISR_BUSY) == 0) {
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
        I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

        // Phase 1: write the 16-bit memory address without AUTOEND (repeated START follows).
        I2C2->CR2 = (static_cast<uint32_t>(g_addr) << 1) | (2u << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        I2C2->TXDR = (addr >> 8) & 0xFF;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        I2C2->TXDR = addr & 0xFF;
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
            I2C2->CR2 = cr2;
            for (std::size_t i = 0; i < chunk; ++i) {
                if (!wait_isr(I2C_ISR_RXNE)) {
                    fail(status);
                    return status;
                }
                out[offset + i] = static_cast<uint8_t>(I2C2->RXDR & I2C_RXDR_RXDATA);
            }
            if (last) {
                if (!wait_isr(I2C_ISR_STOPF)) {
                    fail(status);
                    return status;
                }
                I2C2->ICR = I2C_ICR_STOPCF;
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
        I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

        const uint32_t nbytes = static_cast<uint32_t>(data.size() + 2);
        I2C2->CR2 = (static_cast<uint32_t>(g_addr) << 1) | (nbytes << I2C_CR2_NBYTES_Pos) |
                    I2C_CR2_AUTOEND | I2C_CR2_START;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        I2C2->TXDR = (addr >> 8) & 0xFF;
        if (!wait_isr(I2C_ISR_TXIS)) {
            fail(status);
            return status;
        }
        I2C2->TXDR = addr & 0xFF;
        for (const uint8_t b : data) {
            if (!wait_isr(I2C_ISR_TXIS)) {
                fail(status);
                return status;
            }
            I2C2->TXDR = b;
        }
        if (!wait_isr(I2C_ISR_STOPF)) {
            fail(status);
            return status;
        }
        I2C2->ICR = I2C_ICR_STOPCF;
        return ack_poll();
    }

    /// Waits for the EEPROM to finish its internal write by probing with a 0-byte write.
    static Status ack_poll() {
        for (uint32_t poll = 0; poll < kAckPolls; ++poll) {
            if (wait_idle()) {
                I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
                I2C2->CR2 = (static_cast<uint32_t>(g_addr) << 1) | I2C_CR2_AUTOEND | I2C_CR2_START;
                for (uint32_t spin = 0; spin < kSpinLimit; ++spin) {
                    const uint32_t isr = I2C2->ISR;
                    if ((isr & I2C_ISR_NACKF) != 0) {
                        break;
                    }
                    if ((isr & I2C_ISR_STOPF) != 0) {
                        I2C2->ICR = I2C_ICR_STOPCF;
                        return Status::Ok;
                    }
                }
            }
            I2C2->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            const uint32_t start = time::millis();
            while (time::millis() - start < 1) {
                chip::watchdog::feed();
            }
        }
        recover();
        return Status::I2cError;
    }
};

}  // namespace ferrox::hw
