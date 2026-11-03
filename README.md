# i6s-cpp

C++26 port of the FlySky FS-i6X firmware (originally `ferrox-rc/flysky-i6x-rs`, Rust), plus the FS-i6S variant
(`ferrox-rc/flysky-i6s-rs`), selected with the CMake option `PCB`.

Constraints: CMSIS only (no HAL), `constexpr`/`consteval` where possible, static polymorphism only (no virtual), no heap, no exceptions/RTTI.

## Ported

- Chip: clock (48 MHz), reset flags, DFU entry, UID, watchdog, SysTick
- Peripherals: GPIO, ADC (DMA), USART (DMA, CRSF/half-duplex), keys, LCD (ST7567), buzzer (TIM1), trainer PPM (TIM15 + TIM2, DMA)
- CRSF: framing, telemetry parser, RC frames, parameter menu engine
- AFHDS 2A: protocol logic, A7105 driver, radio state machine, ISRs
- Storage: I2C EEPROM, A/B radio slots, model slots, legacy v1–v4 migration
- USB: device stack (HID gamepad, CDC serial with CLI and telemetry), mode switched from Radio Setup
- USB sim mode (trainer): joystick mode with a host connected mutes the RF link, stops CRSF and PPM out
- Mixer, input, RC output, safety/preflight, calibration, trim, flight timer, vario, bind
- Dashboard pages P1–P5 (P5: AFHDS 2A or CRSF diagnostics, or the custom 2×2 telemetry slot page)
- FS-i6S with `USE_MULTISWITCH=ON`: up to four MultiSwitch dashboards P6–P9, each with eight independent touch
  functions, CRSF SET4 (realm `0xa1`, command `0x07`). Model Setup → MultiSwitch lists MS1–MS4; OK opens each entry's
  settings: Address (Off or 0–255) and eight functions. OK on a function opens Text (up to six characters:
  space, A–Z, 0–9, `-`, `_`) and Type: Toggle (latching, default) or Momentary (ON only while held).
  Up/Down or the jog wheel changes a character, OK advances the cursor, ESC finishes editing; changes are saved
  together when leaving the entry's settings, not the function sub-dialog. Buttons show centered captions,
  with inversion indicating ON; momentary buttons have a double frame, visible in both states.
  Momentary buttons activate on press and release on lift, leaving the button, missing touch reports
  (40 ms timeout), opening a menu or leaving the page. A captured touch cannot swipe pages or activate other
  buttons; sliding back in requires a fresh press. Type changes reset the affected output to OFF.
  Empty captions fall back to 1–8. Only enabled entries get pages, numbered consecutively from P6.
  Defaults: MS1 address 0, MS2–MS4 Off. Addresses, captions and types are saved per model and preserved with
  MultiSwitch compiled out. Captions stay with their MS entry across address/Off changes and page renumbering.
  Existing models retain addresses/captions and default all functions to Toggle; models without captions gain
  empty captions. Older single-address models keep that address in MS1 with MS2–MS4 Off. States start OFF at boot,
  model change or a change to that page's address and are not saved. SET4 is sent on changes and repeated for
  every enabled address every 100 ms while CRSF is active, including on other pages and in menus; USB simulator
  mode suppresses transmission. MultiSwitch header taps/swipes change pages;
  hold the footer to open the menu. Touch fields do not change trims or open the menu.
- FS-i6S (`PCB=I6S`): power latch and shutdown (PB15/PB14, LEDs PD10/PD11), FT6236 touch navigation (long centre tap
  opens the menu), rear pots VRA/VRB as menu jog wheels, rear buttons PA9/PA10 (SE/SF, timer reset, instant trim or
  trim modifier), J4 buttons PA13/PA14 as SG/SH, ext ADC PC1..PC5 (VRC..VRG / SWI..SWM), CRSF on USART1 PB6/PB7 with
  module power on PF6, EEPROM fallback to I2C1 (PB8/PB9), I6S EEPROM layout
- Menus: main menu, model setup (incl. copy), model select, dual rate/expo, throttle curve, wing/mixer, aux channels, diagnostics, system info, radio setup, vario setup, telemetry slot setup, protocol setup (CRSF baud, duplex, arm mode, module parameter browser) and reset

## Not yet ported

- None

## Known deviations from Rust

- CRSF: telemetry tiles 12 (VSPD) and 13 (ALT) use CRSF data instead of AFHDS 2A data.
- Trainer: no interrupts (Rust drives PF10 and reads PF9 from the TIM15 ISR).
  - PPM out: PF10 is TIM15_CH2 (AF0) in PWM mode; the TIM15 update DMA (DMA1 channel 5) loads each period into ARR.
  - PPM in: each PF9 capture on TIM15 triggers (TRGO, ITR1) a TRC capture of the free-running TIM2; the TIM2 CC4
    DMA (DMA1 channel 4) stores the timestamps in a ring that the main loop decodes. TIM15 has only one DMA channel.
- FS-i6S:
  - Sanitize keeps the ADC mode InstantTrim (Rust clamps at > 6) and aux sources 41..44 (VRG, SWK..SWM).
  - SE/SF availability follows the rear-button function (`rear_*_func == 0`); Rust checks `ext_switches`, which has
    no setup row on the i6S.
  - Timer arm and mix switches accept all conditions up to SWM (28); Rust stops at 14 / 22.
  - Not ported: the Rust UI drift (Protocol/Vario row order, Vario "Audio Test" row, pot-centre hysteresis) and the
    unused GPIO extension switches PB6/PB7/PF10.

## Build

Firmware (clang + arm-none-eabi-gcc for Cortex-M0, MinSizeRel = -Oz):

    cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-clang.cmake -DCMAKE_BUILD_TYPE=MinSizeRel
    cmake --build build-fw

`USE_USB` (default ON) sets `WM_USE_USB`, so USB code can be left out to save flash: `-DUSE_USB=OFF`. Without USB the "USB Mode" row is also removed from Radio Setup.

`USE_MULTISWITCH` (default OFF) enables P6–P9, the model's four "MS Addr" settings and CRSF SET4 transmission only with
`PCB=I6S`: add `-DPCB=I6S -DUSE_MULTISWITCH=ON` to the firmware or host-test CMake configuration.

`PCB` selects the board: `-DPCB=I6X` (default, also when unset) or `-DPCB=I6S`. It sets `WM_PCB_I6S` for firmware and
host tests; board constants live in `src/wm/board.hpp`. The firmware target and its output files are named after the
board: `i6x_fw` (`i6x_fw.hex`, `i6x_fw.map`) or `i6s_fw` (`i6s_fw.hex`, `i6s_fw.map`).

Host tests (clang with C++26 support):

    cmake -S . -B build-host -DCMAKE_CXX_COMPILER=clang++ [-DPCB=I6S]
    cmake --build build-host --target host_logic_tests
    ./build-host/host_logic_tests

## Layout

- `src/wm/arch/`: CMSIS include, critical section
- `src/wm/chip/`: clock, reset, DFU, watchdog
- `src/wm/hw/`: drivers (GPIO, ADC, UART, LCD, RF, buzzer, trainer, EEPROM)
- `src/wm/time/`: SysTick millis
- `src/wm/logic/`, `src/wm/util/`: pure logic (host-testable)
- `tests/host/`: host logic tests
- `third_party/cmsis/`: ARM CMSIS_5 and ST device files (Apache-2.0)

## Notes

- Target is single-core Cortex-M0 (ARMv6-M). `std::atomic` read-modify-write lowers to libatomic calls there, so ISR-shared state uses `volatile` plus compiler fences.
- Not verified on hardware yet. Pin and timer assignments come from the Rust source and are not checked against the schematic.
