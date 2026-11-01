#pragma once

#include <array>
#include <cstddef>
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

inline constexpr std::array<std::string_view, 18> kChannelNames{
    "CH1 (AIL)", "CH2 (ELE)", "CH3 (THR)", "CH4 (RUD)", "CH5 (SA)",  "CH6 (SB)",
    "CH7 (VR1)", "CH8 (VR2)", "CH9 (SC)",  "CH10 (SD)", "CH11",     "CH12",
    "CH13",      "CH14",      "CH15",      "CH16",      "CH17",      "CH18",
};

// Dual rate switch codes: 0 = always high rate, 1..6 = SA..SF.
inline constexpr std::array<std::string_view, 7> kDrSwitchNames{"None", "SA", "SB", "SC", "SD", "SE", "SF"};

inline constexpr std::array<std::string_view, 3> kAxisNames{"Roll", "Pitch", "Yaw"};

inline constexpr std::array<std::string_view, 4> kTemplateNames{"NORMAL", "ELEVON/DELTA", "V-TAIL", "FLAPERON"};

inline constexpr std::array<std::string_view, 3> kMixModeNames{"ADD (+)", "MULT (*)", "REPL (:=)"};

// Index matches the weight source codes used by storage::MixLine::weight_src.
inline constexpr std::array<std::string_view, 42> kWeightSrcNames{
    "Fixed", "CH1",  "CH2",  "CH3",  "CH4",  "CH5",   "CH6",   "CH7",   "CH8",   "CH9",   "CH10",  "CH11",
    "CH12",  "CH13", "CH14", "CH15", "CH16", "CH17",  "CH18",  "CH1+",  "CH2+",  "CH3+",  "CH4+",  "CH5+",
    "CH6+",  "CH7+", "CH8+", "CH9+", "CH10+", "CH11+", "CH12+", "CH13+", "CH14+", "CH15+", "CH16+", "CH17+",
    "CH18+", "GV1",  "GV2",  "GV3",  "GV4",  "GV5",
};

// Source names indexed by the aux channel source code (0 = None).
inline constexpr std::array<std::string_view, 46> kSourceNames{
    "None", "Roll", "Pitch", "Thr", "Yaw", "VRA", "VRB", "SA", "SB", "SC", "SD", "MAX", "CH1",
    "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8", "CH9", "CH10", "CH11", "CH12", "CH13", "CH14",
    "CH15", "CH16", "CH17", "CH18", "Thr+", "SE", "SF", "SG", "SH", "SWI", "SWJ", "VRC", "VRD",
    "VRE", "VRF", "GV1", "GV2", "GV3", "GV4", "GV5",
};

// Channel function name. Wing/tail mix replaces the stick names on channels 1, 2, 4 and 6.
constexpr std::string_view channel_source_name(std::size_t ch, const storage::ModelConfig& m) {
    const uint8_t mix = m.wing_tail_mix;
    if (ch == 5 && mix == 3) {
        return "R.AIL";
    }
    switch (ch) {
        case 0: return mix == 1 ? "L.ELV" : mix == 3 ? "L.AIL" : "AIL";
        case 1: return mix == 1 ? "R.ELV" : mix == 2 ? "L.VTL" : "ELE";
        case 2: return "THR";
        case 3: return mix == 2 ? "R.VTL" : "RUD";
        default: break;
    }
    if (ch < 4 || ch >= 4 + m.aux_channels.size()) {
        return "CH";
    }
    const std::size_t src = m.aux_channels[ch - 4];
    return src < kSourceNames.size() ? kSourceNames[src] : "None";
}

// "N:NAME" for a 0-based channel index, e.g. "1:AIL", "18:None".
constexpr format::Text<10> channel_label(std::size_t ch, const storage::ModelConfig& m) {
    format::Text<10> t;
    format::detail::push_uint(t, static_cast<unsigned>(ch) + 1u);
    t.push(':');
    for (const char c : channel_source_name(ch, m)) {
        t.push(c);
    }
    return t;
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
