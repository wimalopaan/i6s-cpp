#pragma once

// Pure A7105 register map, command bytes and AFHDS 2A init table. The SPI and
// GPIO access lives in the hardware layer (hw/a7105.hpp).

#include <array>
#include <cstdint>

namespace ferrox::logic::a7105 {

inline constexpr uint8_t kStrobeSleep = 0x80;
inline constexpr uint8_t kStrobeIdle = 0x90;
inline constexpr uint8_t kStrobeStandby = 0xA0;
inline constexpr uint8_t kStrobePll = 0xB0;
inline constexpr uint8_t kStrobeRx = 0xC0;
inline constexpr uint8_t kStrobeTx = 0xD0;
inline constexpr uint8_t kStrobeRstWrPtr = 0xE0;
inline constexpr uint8_t kStrobeRstRdPtr = 0xF0;

inline constexpr uint8_t kRegMode = 0x00;
inline constexpr uint8_t kRegCalc = 0x02;
inline constexpr uint8_t kRegFifoData = 0x05;
inline constexpr uint8_t kRegIdData = 0x06;
inline constexpr uint8_t kRegGpio1 = 0x0B;
inline constexpr uint8_t kRegGpio2 = 0x0C;
inline constexpr uint8_t kRegPllI = 0x0F;  // RF channel
inline constexpr uint8_t kRegPllII = 0x10;  // Chip signature readback
inline constexpr uint8_t kRegRssi = 0x1D;
inline constexpr uint8_t kRegVcoCurcal = 0x24;
inline constexpr uint8_t kRegVcoSbcalI = 0x25;
inline constexpr uint8_t kRegVcoSbcalII = 0x26;
inline constexpr uint8_t kRegTxTest = 0x28;

inline constexpr uint8_t kCalcIfBank = 0x01;
inline constexpr uint8_t kCalcVcoBank = 0x02;

inline constexpr uint8_t kGpio1SdoOutput = 0x19;
inline constexpr uint8_t kGpio2Wtr = 0x01;
inline constexpr uint8_t kVcoCurcal = 0x13;
inline constexpr uint8_t kVcoSbcalII = 0x3B;
inline constexpr uint8_t kVcoSbcalIDefault = 0x0A;

inline constexpr uint32_t kSyncId = 0x5475'C52Au;
inline constexpr uint8_t kChipSignature = 0x9E;
inline constexpr uint8_t kSkipEntry = 0xFF;

inline constexpr uint8_t kPowerHigh = 0x1F;
inline constexpr uint8_t kPowerMid = 0x0D;
inline constexpr uint8_t kPowerLow = 0x02;
inline constexpr uint8_t kPowerTest = 0x00;
inline constexpr uint8_t kBindPower = kPowerTest;

/// Maps the user power index (0 HIGH, 1 MID, 2 LOW, 3 TEST) to the TX_TEST register value.
/// Unknown indices fall back to HIGH.
constexpr uint8_t power_from_index(uint8_t idx) {
    switch (idx) {
        case 1: return kPowerMid;
        case 2: return kPowerLow;
        case 3: return kPowerTest;
        default: return kPowerHigh;
    }
}

constexpr uint8_t cmd_write(uint8_t addr) { return static_cast<uint8_t>(addr & 0x3F); }
constexpr uint8_t cmd_read(uint8_t addr) { return static_cast<uint8_t>((addr & 0x3F) | 0x40); }

/// MODE bit 5 (CRCF) is 0 when the last received packet passed CRC.
constexpr bool crc_ok(uint8_t mode_reg) { return (mode_reg & (1u << 5)) == 0; }

// Register init for AFHDS 2A at 500 kbps GFSK, indexed by register address.
inline constexpr std::array<uint8_t, 50> kAfhds2aInit = {
    kSkipEntry,
    0xE2,  // 01: MODE CONTROL (FIFO mode, FCRC)
    0x00,  // 02: CALC
    0x25,  // 03: FIFO I (FIFO margin)
    0x00,  // 04: FIFO II
    kSkipEntry,  // 05: FIFO DATA
    kSkipEntry,  // 06: ID DATA
    0x00,  // 07: RC OSC I
    0x00,  // 08: RC OSC II
    0x00,  // 09: RC OSC III
    0x00,  // 0A: CKO PIN
    kGpio1SdoOutput,  // 0B: GPIO1
    kGpio2Wtr,  // 0C: GPIO2
    0x05,  // 0D: CLOCK
    0x00,  // 0E: DATA RATE (500 kbps)
    0x50,  // 0F: PLL I (initial channel 80)
    0x9E,  // 10: PLL II
    0x4B,  // 11: PLL III (2400 MHz base)
    0x00,  // 12: PLL IV
    0x02,  // 13: PLL V
    0x16,  // 14: TX I
    0x2B,  // 15: TX II
    0x12,  // 16: DELAY I
    0x4F,  // 17: DELAY II
    0x62,  // 18: RX
    0x80,  // 19: RX GAIN I
    kSkipEntry,  // 1A: RX GAIN II
    kSkipEntry,  // 1B: RX GAIN III
    0x2A,  // 1C: RX GAIN IV
    0x32,  // 1D: RSSI THRESHOLD
    0xC3,  // 1E: ADC
    0x1E,  // 1F: CODE I
    0x1E,  // 20: CODE II
    kSkipEntry,  // 21: CODE III
    0x00,  // 22: IF CALIB I
    kSkipEntry,  // 23: IF CALIB II
    0x00,  // 24: VCO CURCAL
    0x00,  // 25: VCO SBCAL I
    0x3B,  // 26: VCO SBCAL II
    0x00,  // 27: BATTERY DET
    0x17,  // 28: TX TEST (power)
    0x47,  // 29: RX DEM TEST I
    0x80,  // 2A: RX DEM TEST II
    0x03,  // 2B: CPC
    0x01,  // 2C: XTAL TEST
    0x45,  // 2D: PLL TEST
    0x18,  // 2E: VCO TEST I
    0x00,  // 2F: VCO TEST II
    0x01,  // 30: IFAT
    0x0F,  // 31: RSCALE
};

}  // namespace ferrox::logic::a7105
