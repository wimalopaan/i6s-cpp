#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/format.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/storage.hpp"

namespace ferrox::logic::labels {

constexpr std::string_view model_type_label(uint8_t type) {
    switch (static_cast<mixer::ModelType>(type)) {
        case mixer::ModelType::Airplane: return "AIRPLANE";
        case mixer::ModelType::Glider: return "GLIDER";
        case mixer::ModelType::Heli: return "HELICOPTER";
        case mixer::ModelType::Quad: return "MULTI / QUAD";
        case mixer::ModelType::General: return "GENERAL";
    }
    return "GENERAL";
}

// Index matches the switch condition codes used by mixer::is_switch_active (0 = always on).
inline constexpr std::array<std::string_view, 23> kSwitchCondNames{
    "ON",  "SA^", "SAv", "SB^", "SB-", "SBv", "SC^", "SC-", "SCv", "SD^", "SDv", "SE^",
    "SEv", "SF^", "SFv", "SG^", "SGv", "SH^", "SHv", "SWI^", "SWIv", "SWJ^", "SWJv",
};

constexpr std::string_view switch_cond_label(uint8_t cond) {
    return cond < kSwitchCondNames.size() ? kSwitchCondNames[cond] : kSwitchCondNames[0];
}

// Trimmed model name; stops at the first NUL byte.
constexpr format::Text<10> model_name(const storage::ModelConfig& m) {
    format::Text<10> t;
    for (const uint8_t c : m.name) {
        if (c == 0) {
            break;
        }
        t.push(static_cast<char>(c));
    }
    return t;
}

}  // namespace ferrox::logic::labels
