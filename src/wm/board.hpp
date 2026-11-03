#pragma once

#include <cstddef>
#include <cstdint>

// Selected by the CMake option PCB (I6X default, I6S).
#ifndef WM_PCB_I6S
#define WM_PCB_I6S 0
#endif

namespace wm::board {

enum class Pcb : uint8_t { I6X, I6S };

inline constexpr Pcb kPcb = WM_PCB_I6S ? Pcb::I6S : Pcb::I6X;
inline constexpr bool kIsI6S = kPcb == Pcb::I6S;

/// Auxiliary analog channels: SA..SD, VRA..VRB and the external ADC (I6X: VRC..VRF on P7, I6S: VRC..VRG on PC1..PC5).
inline constexpr std::size_t kAuxCount = kIsI6S ? 11 : 10;
/// ADC1 scan channels (raw indices 0..kRawCount-1).
inline constexpr std::size_t kRawCount = kIsI6S ? 16 : 15;
/// Highest switch condition (1..kMaxSwitchCond: SA^..SWJv, plus SWK^..SWMv on I6S).
inline constexpr uint8_t kMaxSwitchCond = kIsI6S ? 28 : 22;
/// Mixer sources 0..kSourceCount-1; the five global variables are the last ones.
inline constexpr uint8_t kSourceCount = kIsI6S ? 50 : 46;
inline constexpr uint8_t kSourceGv1 = kSourceCount - 5;
/// Highest stored ADC input mode (I6S inserts "Off" at 1).
inline constexpr uint8_t kMaxAdcMode = kIsI6S ? 7 : 6;
/// Menu navigation with the rear pots VRA/VRB (I6S has no menu buttons).
inline constexpr bool kHasJogWheels = kIsI6S;
inline constexpr bool kHasTouch = kIsI6S;

}  // namespace wm::board
