#pragma once

// A7105 chip access over SPI1 plus the front-end and antenna switch GPIOs.
// Every call here is polled and runs with the caller's interrupt state; the
// ISRs in hw/rf.hpp call it from TIM16 and EXTI2_3 context.

#include <cstddef>
#include <cstdint>
#include <span>

#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/logic/a7105.hpp"
#include "ferrox/logic/afhds2a.hpp"
#include "ferrox/logic/afhds2a_radio.hpp"

namespace ferrox::hw::a7105 {

// PE12: CSN (active low). PE13/PE14/PE15: SCK/MISO/MOSI on SPI1 (AF1).
// PE8: RX_EN, PE9: TX_EN, PE10/PE11: antenna switch. PB2: GIO2 (EXTI2).
inline constexpr uint32_t kCsnPin = 12;
inline constexpr uint32_t kSpiPinMask = (1u << 13) | (1u << 14) | (1u << 15);
inline constexpr uint32_t kRxEnPin = 8;
inline constexpr uint32_t kTxEnPin = 9;
inline constexpr uint32_t kAntennaA = 10;
inline constexpr uint32_t kAntennaB = 11;
inline constexpr uint32_t kGio2Pin = 2;
inline constexpr uint32_t kSpiAf = 1;
inline constexpr uint32_t kSpiTimeout = 10000;

inline void csn_low() { clear_pins<Port::E>(1u << kCsnPin); }
inline void csn_high() { set_pins<Port::E>(1u << kCsnPin); }

inline bool wait_flag(uint32_t mask, bool want_set) {
    for (uint32_t i = 0; i < kSpiTimeout; ++i) {
        const bool set = (SPI1->SR & mask) != 0;
        if (set == want_set) {
            return true;
        }
    }
    return false;
}

// 8-bit transfer. The DR byte view avoids a 16-bit access that would
// pack two frames into one transfer.
inline uint8_t transfer(uint8_t out) {
    volatile uint8_t* dr = reinterpret_cast<volatile uint8_t*>(&SPI1->DR);
    wait_flag(SPI_SR_BSY, false);
    wait_flag(SPI_SR_TXE, true);
    *dr = out;
    wait_flag(SPI_SR_RXNE, true);
    return *dr;
}

/// Configures PE12..PE15 and PB2, enables SPI1 (8-bit, master, software NSS).
inline void init_bus() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    set_mode_mask<Port::E>((1u << kCsnPin) | (1u << kRxEnPin) | (1u << kTxEnPin) |
                               (1u << kAntennaA) | (1u << kAntennaB),
                           Mode::Output);
    csn_high();
    set_mode_mask<Port::E>(kSpiPinMask, Mode::Alternate);
    set_high_speed_mask<Port::E>(kSpiPinMask | (1u << kCsnPin));
    for (uint32_t pin : {13u, 14u, 15u}) {
        set_alt_function<Port::E>(pin, kSpiAf);
    }

    SPI1->CR1 = 0;
    SPI1->CR2 = (0x7u << 8) | SPI_CR2_FRXTH;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_SSM | SPI_CR1_SSI;
    for (int i = 0; i < 16 && (SPI1->SR & SPI_SR_RXNE) != 0; ++i) {
        volatile uint8_t* dr = reinterpret_cast<volatile uint8_t*>(&SPI1->DR);
        (void)*dr;
    }
    SPI1->CR1 |= SPI_CR1_SPE;

    set_mode_mask<Port::B>(1u << kGio2Pin, Mode::Input);
    set_pull_mask<Port::B>(1u << kGio2Pin, Pull::Up);
}

inline void strobe(uint8_t cmd) {
    csn_low();
    transfer(cmd);
    csn_high();
}

inline uint8_t read_reg(uint8_t addr) {
    csn_low();
    transfer(logic::a7105::cmd_read(addr));
    const uint8_t v = transfer(0x00);
    csn_high();
    return v;
}

inline void write_reg(uint8_t addr, uint8_t val) {
    csn_low();
    transfer(logic::a7105::cmd_write(addr));
    transfer(val);
    csn_high();
}

inline void write_id(uint32_t id) {
    csn_low();
    transfer(logic::a7105::cmd_write(logic::a7105::kRegIdData));
    transfer(static_cast<uint8_t>(id >> 24));
    transfer(static_cast<uint8_t>(id >> 16));
    transfer(static_cast<uint8_t>(id >> 8));
    transfer(static_cast<uint8_t>(id));
    csn_high();
}

inline void write_fifo(std::span<const uint8_t> data, uint8_t channel) {
    strobe(logic::a7105::kStrobeStandby);
    strobe(logic::a7105::kStrobeRstWrPtr);
    csn_low();
    transfer(logic::a7105::cmd_write(logic::a7105::kRegFifoData));
    for (uint8_t b : data) {
        transfer(b);
    }
    csn_high();
    write_reg(logic::a7105::kRegPllI, channel);
    strobe(logic::a7105::kStrobeTx);
}

inline void read_fifo(std::span<uint8_t> out) {
    strobe(logic::a7105::kStrobeRstRdPtr);
    csn_low();
    transfer(logic::a7105::cmd_read(logic::a7105::kRegFifoData));
    for (uint8_t& b : out) {
        b = transfer(0x00);
    }
    csn_high();
}

inline bool is_crc_ok() {
    return logic::a7105::crc_ok(read_reg(logic::a7105::kRegMode));
}

inline uint8_t read_raw_rssi() {
    return read_reg(logic::a7105::kRegRssi);
}

inline void set_power(uint8_t tx_test) {
    write_reg(logic::a7105::kRegTxTest, tx_test);
}

/// Front-end switch. Off puts both RX_EN and TX_EN in the disabled state.
inline void set_front_end(logic::afhds2a::FrontEnd mode) {
    using logic::afhds2a::FrontEnd;
    switch (mode) {
        case FrontEnd::RxEn:
            set_pins<Port::E>(1u << kTxEnPin);
            clear_pins<Port::E>(1u << kRxEnPin);
            break;
        case FrontEnd::TxEn:
            set_pins<Port::E>(1u << kRxEnPin);
            clear_pins<Port::E>(1u << kTxEnPin);
            break;
        case FrontEnd::Off:
            set_pins<Port::E>((1u << kRxEnPin) | (1u << kTxEnPin));
            break;
    }
}

/// Returns true when the chip answers with the AFHDS 2A signature.
inline bool reset() {
    using namespace logic::a7105;
    for (int attempt = 0; attempt < 5; ++attempt) {
        strobe(kStrobeStandby);
        for (int i = 0; i < 2000; ++i) {
            __NOP();
        }
        write_reg(kRegMode, 0x00);
        for (int i = 0; i < 16000; ++i) {
            __NOP();
        }
        write_reg(kRegGpio1, kGpio1SdoOutput);
        write_reg(kRegGpio2, kGpio2Wtr);
        set_front_end(logic::afhds2a::FrontEnd::Off);
        if (read_reg(kRegPllII) == kChipSignature) {
            strobe(kStrobeStandby);
            return true;
        }
    }
    return false;
}

inline bool wait_calc_done() {
    for (uint32_t i = 0; i < 10000; ++i) {
        if ((read_reg(logic::a7105::kRegCalc) & 1u) == 0) {
            return true;
        }
    }
    return false;
}

/// Writes the sync ID, the AFHDS 2A register table and runs the calibrations.
inline void init_regs() {
    using namespace logic::a7105;
    write_id(kSyncId);
    for (std::size_t addr = 0; addr < kAfhds2aInit.size(); ++addr) {
        if (kAfhds2aInit[addr] != kSkipEntry) {
            write_reg(static_cast<uint8_t>(addr), kAfhds2aInit[addr]);
        }
    }
    strobe(kStrobeStandby);

    write_reg(kRegCalc, kCalcIfBank);
    wait_calc_done();
    write_reg(kRegVcoCurcal, kVcoCurcal);
    write_reg(kRegVcoSbcalII, kVcoSbcalII);
    write_reg(kRegPllI, 0x00);
    write_reg(kRegCalc, kCalcVcoBank);
    wait_calc_done();
    write_reg(kRegPllI, 0xA0);
    write_reg(kRegCalc, kCalcVcoBank);
    wait_calc_done();
    write_reg(kRegVcoSbcalI, kVcoSbcalIDefault);

    set_front_end(logic::afhds2a::FrontEnd::TxEn);
    set_power(kPowerHigh);
    strobe(kStrobeStandby);
}

inline void switch_antenna(bool state) {
    if (state) {
        clear_pins<Port::E>(1u << kAntennaA);
        set_pins<Port::E>(1u << kAntennaB);
    } else {
        set_pins<Port::E>(1u << kAntennaA);
        clear_pins<Port::E>(1u << kAntennaB);
    }
}

}  // namespace ferrox::hw::a7105
