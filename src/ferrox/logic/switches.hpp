#pragma once

#include <cstdint>

namespace ferrox::logic {

enum class SwitchPos : uint8_t { Up, Mid, Down };

/// Physical switch positions. SA, SD, SE, SF, SG, SH, SWI and SWJ are shared by mixer and arm logic.
struct Switches {
    SwitchPos sa{SwitchPos::Up};   // 2-pos
    SwitchPos sb{SwitchPos::Up};   // 3-pos
    SwitchPos sc{SwitchPos::Up};   // 3-pos
    SwitchPos sd{SwitchPos::Up};   // 2-pos
    SwitchPos se{SwitchPos::Up};   // 2-pos
    SwitchPos sf{SwitchPos::Up};   // 2-pos
    SwitchPos sg{SwitchPos::Up};   // 2-pos
    SwitchPos sh{SwitchPos::Up};   // 2-pos
    SwitchPos swi{SwitchPos::Up};  // 2-pos or 3-pos (ext ADC 1)
    SwitchPos swj{SwitchPos::Up};  // 2-pos or 3-pos (ext ADC 2)
    SwitchPos swk{SwitchPos::Up};  // 2-pos or 3-pos (ext ADC 3)
    SwitchPos swl{SwitchPos::Up};  // 2-pos or 3-pos (ext ADC 4)

    constexpr bool operator==(const Switches&) const = default;
};

}  // namespace ferrox::logic
