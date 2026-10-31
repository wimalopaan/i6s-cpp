#pragma once

#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>

namespace ferrox::logic::storage {

// Layout of the external I2C EEPROM (FT24C128A, 16 KB, 64-byte pages):
//   0x0000 radio config slot A, 0x0100 radio config slot B (A/B safe save),
//   0x0200 + n * 768 model slot n (n = 0..19), 0x3E00..0x4000 reserved.
inline constexpr uint32_t kFlashMagic = 0x4653'4B59;  // "FSKY"
inline constexpr uint32_t kConfigVersion = 5;
inline constexpr std::size_t kNumModels = 20;
inline constexpr std::size_t kNumMixLines = 32;

// Slot framing: 8-byte header followed by the payload.
inline constexpr uint16_t kSlotMagic = 0x4658;  // "FX" little-endian
inline constexpr uint8_t kSchemaVersion = 1;
inline constexpr uint8_t kFlagValid = 0x00;
inline constexpr uint8_t kFlagDeleted = 0xFF;
inline constexpr std::size_t kHeaderSize = 8;
inline constexpr std::size_t kPageSize = 64;

inline constexpr uint16_t kRadioSlotA = 0x0000;
inline constexpr uint16_t kRadioSlotB = 0x0100;
inline constexpr uint16_t kModelBase = 0x0200;
inline constexpr std::size_t kModelSlotSize = 768;

// Legacy payload sizes that are migrated on load (ModelConfig v1..v4).
inline constexpr std::size_t kLegacyModelV1Size = 128;
inline constexpr std::size_t kMaxLegacyModelSize = 364;

// Raw ADC calibration limits (same values as hw/adc.hpp).
inline constexpr uint16_t kAdcMin = 0;
inline constexpr uint16_t kAdcCenter = 2048;
inline constexpr uint16_t kAdcMax = 4095;

enum class Status : uint8_t {
    Ok,
    I2cError,
    EmptySlot,
    VersionMismatch,
    LengthMismatch,
    CrcMismatch,
};

/// CRC-16 with polynomial 0x8005, initial value 0, no reflection (CRC-16/BUYPASS).
constexpr uint16_t crc16(std::span<const uint8_t> data) {
    uint16_t crc = 0;
    for (const uint8_t b : data) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(b) << 8));
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x8005)
                                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

struct ChannelCalib {
    uint16_t min;
    uint16_t center;
    uint16_t max;
    uint16_t pad;
};
static_assert(sizeof(ChannelCalib) == 8);

struct PotCalib {
    uint16_t min;
    uint16_t center;
    uint16_t max;
};
static_assert(sizeof(PotCalib) == 6);

struct RadioConfig {
    uint32_t magic;
    uint32_t version;
    uint8_t active_model;
    uint8_t throttle_trim;
    uint8_t audio_enabled;
    uint8_t backlight_timeout;
    uint8_t backlight_brightness;
    uint8_t vbat_warn_deci;
    uint8_t lcd_contrast;
    uint8_t usb_mode;
    std::array<ChannelCalib, 4> sticks;
    std::array<PotCalib, 10> aux_pots;
    uint8_t ext_module_pwr;
    uint8_t tone_style;
    uint16_t servo_rate_hz;
    uint8_t rx_out_mode;
    uint8_t rx_serial_proto;
    uint8_t ext_switches;
    uint8_t ext_adc;
    std::array<uint8_t, 10> adc_modes;
    uint8_t stick_mode;
    std::array<uint8_t, 4> reserved_telem;
    uint8_t trainer_config;
    uint8_t rear_left_func;
    uint8_t rear_right_func;
    uint8_t touch_disabled;
    uint8_t j4_sg_sh_en;
    uint8_t switch_beep;
    uint8_t gimbal_type;
    uint8_t stick_invert_mask;
    uint8_t pad_align;
};
static_assert(sizeof(RadioConfig) == 140);
static_assert(offsetof(RadioConfig, sticks) == 16);
static_assert(offsetof(RadioConfig, aux_pots) == 48);
static_assert(offsetof(RadioConfig, servo_rate_hz) == 110);
static_assert(offsetof(RadioConfig, adc_modes) == 116);
static_assert(offsetof(RadioConfig, pad_align) == 139);

struct MixLine {
    uint8_t target_ch;
    uint8_t source;
    uint8_t weight_src;
    int8_t weight;
    int8_t offset;
    uint8_t switch_sel;
    uint8_t mode;
    uint8_t reserved;
};
static_assert(sizeof(MixLine) == 8);

struct ModelConfig {
    std::array<uint8_t, 10> name;
    uint8_t model_type;
    uint8_t pad0;
    uint32_t rx_id;
    std::array<int8_t, 4> trims;
    uint32_t channel_reverse;
    uint8_t dr_switch;
    uint8_t thr_curve_pts;
    uint8_t thr_curve_smooth;
    std::array<uint8_t, 17> thr_curve;
    uint8_t pad_crv;
    std::array<uint8_t, 3> dr_high;
    std::array<uint8_t, 3> dr_low;
    std::array<int8_t, 3> expo_high;
    std::array<int8_t, 3> expo_low;
    uint8_t pad1;
    uint16_t timer_secs;
    uint8_t timer_source;
    uint8_t rf_power;
    uint16_t failsafe_thr;
    std::array<uint8_t, 14> aux_channels;
    uint8_t wing_tail_mix;
    int8_t template_diff;
    std::array<MixLine, kNumMixLines> mixes;
    uint8_t failsafe_mode;
    uint8_t failsafe_timeout;
    uint8_t rf_protocol;
    uint8_t crsf_baud;
    uint8_t arm_switch;
    uint8_t rx_out_mode;
    uint16_t servo_rate_hz;
    uint8_t rx_serial_proto;
    uint8_t crsf_half_duplex;
    uint8_t crsf_arm_mode;
    uint8_t pad_end;
    std::array<int16_t, 5> gvars;
    uint8_t vario_switch;
    uint8_t vario_climb_deadband;
    uint8_t vario_sink_deadband;
    uint8_t vario_sink_mode;
    uint8_t vario_motor_mute;
    uint8_t pad_vario;
    std::array<uint8_t, 4> telem_slots;
};
static_assert(sizeof(ModelConfig) == 368);
static_assert(offsetof(ModelConfig, rx_id) == 12);
static_assert(offsetof(ModelConfig, timer_secs) == 58);
static_assert(offsetof(ModelConfig, mixes) == 80);
static_assert(offsetof(ModelConfig, gvars) == 348);
static_assert(offsetof(ModelConfig, telem_slots) == 364);

/// ModelConfig v1 (128 bytes) as written by earlier firmware releases.
struct LegacyMixLineV1 {
    uint8_t target_ch;
    uint8_t source;
    int8_t weight;
    int8_t offset;
    uint8_t switch_sel;
    uint8_t mode;
};
static_assert(sizeof(LegacyMixLineV1) == 6);

struct LegacyModelConfigV1 {
    std::array<uint8_t, 10> name;
    uint8_t model_type;
    uint8_t pad0;
    uint32_t rx_id;
    std::array<int8_t, 4> trims;
    uint32_t channel_reverse;
    uint8_t dr_switch;
    uint8_t thr_curve_pts;
    uint8_t thr_curve_smooth;
    std::array<uint8_t, 9> thr_curve;
    std::array<uint8_t, 3> dr_high;
    std::array<uint8_t, 3> dr_low;
    std::array<int8_t, 3> expo_high;
    std::array<int8_t, 3> expo_low;
    uint16_t timer_secs;
    uint8_t timer_source;
    uint8_t rf_power;
    uint16_t failsafe_thr;
    std::array<uint8_t, 14> aux_channels;
    uint8_t wing_tail_mix;
    int8_t template_diff;
    std::array<LegacyMixLineV1, 8> mixes;
    uint8_t failsafe_mode;
    uint8_t failsafe_timeout;
    uint8_t rf_protocol;
    uint8_t crsf_baud;
    uint8_t arm_switch;
    uint8_t rx_out_mode;
    uint16_t servo_rate_hz;
    uint8_t rx_serial_proto;
    uint8_t crsf_half_duplex;
};
static_assert(sizeof(LegacyModelConfigV1) == kLegacyModelV1Size);

struct RadioStorage {
    RadioConfig radio;
    ModelConfig active_model;
};
static_assert(sizeof(RadioStorage) == 508);

struct SlotHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t len;
    uint16_t crc;
};

static_assert(std::has_unique_object_representations_v<ChannelCalib>);
static_assert(std::has_unique_object_representations_v<PotCalib>);
static_assert(std::has_unique_object_representations_v<MixLine>);
static_assert(std::has_unique_object_representations_v<RadioConfig>);
static_assert(std::has_unique_object_representations_v<ModelConfig>);
static_assert(std::has_unique_object_representations_v<LegacyModelConfigV1>);

/// Byte image of a trivially laid-out config struct (little-endian on the target).
template <typename T>
constexpr std::array<uint8_t, sizeof(T)> to_bytes(const T& value) {
    return std::bit_cast<std::array<uint8_t, sizeof(T)>>(value);
}

template <typename T>
constexpr T from_bytes(const std::array<uint8_t, sizeof(T)>& bytes) {
    return std::bit_cast<T>(bytes);
}

constexpr std::array<uint8_t, kHeaderSize> header_to_bytes(const SlotHeader& h) {
    return {
        static_cast<uint8_t>(h.magic & 0xFF), static_cast<uint8_t>(h.magic >> 8),
        h.version,                            h.flags,
        static_cast<uint8_t>(h.len & 0xFF),   static_cast<uint8_t>(h.len >> 8),
        static_cast<uint8_t>(h.crc & 0xFF),   static_cast<uint8_t>(h.crc >> 8),
    };
}

constexpr SlotHeader header_from_bytes(const std::array<uint8_t, kHeaderSize>& b) {
    return SlotHeader{
        .magic = static_cast<uint16_t>(b[0] | (b[1] << 8)),
        .version = b[2],
        .flags = b[3],
        .len = static_cast<uint16_t>(b[4] | (b[5] << 8)),
        .crc = static_cast<uint16_t>(b[6] | (b[7] << 8)),
    };
}

/// Validates a header against the expected payload length (no CRC check).
constexpr Status validate_header(const SlotHeader& h, std::size_t expected_len) {
    if (h.magic != kSlotMagic) {
        return Status::EmptySlot;
    }
    if (h.version != kSchemaVersion) {
        return Status::VersionMismatch;
    }
    if (h.flags != kFlagValid) {
        return Status::EmptySlot;
    }
    if (h.len != expected_len) {
        return Status::LengthMismatch;
    }
    return Status::Ok;
}

constexpr std::optional<uint16_t> model_slot_addr(std::size_t index) {
    if (index >= kNumModels) {
        return std::nullopt;
    }
    return static_cast<uint16_t>(kModelBase + index * kModelSlotSize);
}

template <typename T, std::size_t N>
constexpr std::array<T, N> filled(const T& value) {
    std::array<T, N> out{};
    for (auto& x : out) {
        x = value;
    }
    return out;
}

constexpr MixLine disabled_mix() {
    return MixLine{
        .target_ch = 0,
        .source = 0,
        .weight_src = 0,
        .weight = 100,
        .offset = 0,
        .switch_sel = 0,
        .mode = 0,
        .reserved = 0,
    };
}

constexpr std::array<uint8_t, 10> default_name(std::size_t index) {
    const auto num = static_cast<uint8_t>(index + 1);
    return {'M', 'O', 'D', 'E', 'L', ' ', static_cast<uint8_t>('0' + num / 10),
            static_cast<uint8_t>('0' + num % 10), ' ', ' '};
}

constexpr RadioConfig default_radio_config() {
    return RadioConfig{
        .magic = kFlashMagic,
        .version = kConfigVersion,
        .active_model = 0,
        .throttle_trim = 0,
        .audio_enabled = 1,
        .backlight_timeout = 0,
        .backlight_brightness = 10,
        .vbat_warn_deci = 44,
        .lcd_contrast = 37,
        .usb_mode = 0,
        .sticks = filled<ChannelCalib, 4>(ChannelCalib{kAdcMin, kAdcCenter, kAdcMax, 0}),
        .aux_pots = filled<PotCalib, 10>(PotCalib{kAdcMin, kAdcCenter, kAdcMax}),
        .ext_module_pwr = 0,
        .tone_style = 1,
        .servo_rate_hz = 50,
        .rx_out_mode = 0,
        .rx_serial_proto = 0,
        .ext_switches = 0,
        .ext_adc = 0,
        .adc_modes = {},
        .stick_mode = 1,
        .reserved_telem = {0, 1, 2, 3},
        .trainer_config = 0,
        .rear_left_func = 0,
        .rear_right_func = 0,
        .touch_disabled = 0,
        .j4_sg_sh_en = 0,
        .switch_beep = 1,
        .gimbal_type = 0,
        .stick_invert_mask = 0x03,
        .pad_align = 0,
    };
}

inline constexpr std::array<uint8_t, 17> kDefaultThrCurve{0, 25, 50, 75, 100, 0, 0, 0, 0,
                                                          0, 0,  0,  0,  0,   0, 0, 0};

constexpr ModelConfig default_model_config(std::size_t index) {
    return ModelConfig{
        .name = default_name(index),
        .model_type = 0,
        .pad0 = 0,
        .rx_id = 0,
        .trims = {},
        .channel_reverse = 0,
        .dr_switch = 0,
        .thr_curve_pts = 5,
        .thr_curve_smooth = 0,
        .thr_curve = kDefaultThrCurve,
        .pad_crv = 0,
        .dr_high = {100, 100, 100},
        .dr_low = {70, 70, 70},
        .expo_high = {},
        .expo_low = {},
        .pad1 = 0,
        .timer_secs = 300,
        .timer_source = 1,
        .rf_power = 0,
        .failsafe_thr = 1000,
        .aux_channels = {7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0},
        .wing_tail_mix = 0,
        .template_diff = 0,
        .mixes = filled<MixLine, kNumMixLines>(disabled_mix()),
        .failsafe_mode = 0,
        .failsafe_timeout = 20,
        .rf_protocol = 0,
        .crsf_baud = 2,
        .arm_switch = 0,
        .rx_out_mode = 0,
        .servo_rate_hz = 50,
        .rx_serial_proto = 0,
        .crsf_half_duplex = 0,
        .crsf_arm_mode = 0,
        .pad_end = 0,
        .gvars = {},
        .vario_switch = 0,
        .vario_climb_deadband = 2,
        .vario_sink_deadband = 5,
        .vario_sink_mode = 1,
        .vario_motor_mute = 1,
        .pad_vario = 0,
        .telem_slots = {0, 1, 2, 3},
    };
}

constexpr RadioStorage default_storage() {
    return RadioStorage{default_radio_config(), default_model_config(0)};
}

/// Converts a legacy ModelConfig v1 into the current layout. Mixer lines 8..31 stay disabled.
constexpr ModelConfig unpack_legacy_v1(const LegacyModelConfigV1& v1) {
    ModelConfig v2 = default_model_config(0);
    v2.name = v1.name;
    v2.model_type = v1.model_type;
    v2.rx_id = v1.rx_id;
    v2.trims = v1.trims;
    v2.channel_reverse = v1.channel_reverse;
    v2.dr_switch = v1.dr_switch;
    v2.thr_curve_pts = v1.thr_curve_pts;
    v2.thr_curve_smooth = v1.thr_curve_smooth;
    v2.thr_curve = {};
    for (std::size_t i = 0; i < v1.thr_curve.size(); ++i) {
        v2.thr_curve[i] = v1.thr_curve[i];
    }
    v2.dr_high = v1.dr_high;
    v2.dr_low = v1.dr_low;
    v2.expo_high = v1.expo_high;
    v2.expo_low = v1.expo_low;
    v2.timer_secs = v1.timer_secs;
    v2.timer_source = v1.timer_source;
    v2.rf_power = v1.rf_power;
    v2.failsafe_thr = v1.failsafe_thr;
    v2.aux_channels = v1.aux_channels;
    v2.wing_tail_mix = v1.wing_tail_mix;
    v2.template_diff = v1.template_diff;
    for (std::size_t i = 0; i < v1.mixes.size(); ++i) {
        const auto& old = v1.mixes[i];
        v2.mixes[i] = MixLine{
            .target_ch = old.target_ch,
            .source = old.source,
            .weight_src = 0,
            .weight = old.weight,
            .offset = old.offset,
            .switch_sel = old.switch_sel,
            .mode = old.mode,
            .reserved = 0,
        };
    }
    v2.failsafe_mode = v1.failsafe_mode;
    v2.failsafe_timeout = v1.failsafe_timeout;
    v2.rf_protocol = v1.rf_protocol;
    v2.crsf_baud = v1.crsf_baud;
    v2.arm_switch = v1.arm_switch;
    v2.rx_out_mode = v1.rx_out_mode;
    v2.servo_rate_hz = v1.servo_rate_hz;
    v2.rx_serial_proto = v1.rx_serial_proto;
    v2.crsf_half_duplex = v1.crsf_half_duplex;
    return v2;
}

constexpr void sanitize_calib(uint16_t& min, uint16_t& center, uint16_t& max) {
    if (min >= center || center >= max || max > kAdcMax) {
        min = kAdcMin;
        center = kAdcCenter;
        max = kAdcMax;
    }
}

constexpr void sanitize_model(ModelConfig& m, uint8_t active_model) {
    if (m.model_type > 4) {
        m.model_type = 0;
    }
    if (m.rf_protocol > 1) {
        m.rf_protocol = 0;
    }
    if (m.crsf_baud > 4) {
        m.crsf_baud = 2;
    }
    if (m.crsf_half_duplex > 1) {
        m.crsf_half_duplex = 0;
    }
    if (m.crsf_arm_mode > 1) {
        m.crsf_arm_mode = 0;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (m.dr_high[axis] < 30 || m.dr_high[axis] > 100) {
            m.dr_high[axis] = 100;
        }
        if (m.dr_low[axis] < 30 || m.dr_low[axis] > 100) {
            m.dr_low[axis] = 70;
        }
        if (m.expo_high[axis] < -100 || m.expo_high[axis] > 100) {
            m.expo_high[axis] = 0;
        }
        if (m.expo_low[axis] < -100 || m.expo_low[axis] > 100) {
            m.expo_low[axis] = 0;
        }
    }
    if (m.dr_switch > 6) {
        m.dr_switch = 0;
    }
    if (m.wing_tail_mix > 3) {
        m.wing_tail_mix = 0;
    }
    if (m.template_diff < -100 || m.template_diff > 100) {
        m.template_diff = 0;
    }
    if (m.failsafe_thr < 900 || m.failsafe_thr > 2100) {
        m.failsafe_thr = 1000;
    }
    if (m.aux_channels[0] == 0 && m.aux_channels[1] == 0 && m.aux_channels[2] == 0 &&
        m.aux_channels[3] == 0 && m.aux_channels[4] == 0 && m.aux_channels[5] == 0) {
        m.aux_channels = {7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    for (auto& src : m.aux_channels) {
        if (src > 40) {
            src = 0;
        }
    }
    for (auto& mix : m.mixes) {
        if (mix.target_ch > 18 || mix.source > 45 || mix.weight_src > 41 || mix.mode > 2 ||
            mix.switch_sel > 22) {
            mix = disabled_mix();
        }
    }
    for (auto& gv : m.gvars) {
        if (gv < -1000) {
            gv = -1000;
        } else if (gv > 1000) {
            gv = 1000;
        }
    }
    if (m.thr_curve_pts != 5 && m.thr_curve_pts != 7 && m.thr_curve_pts != 9) {
        m.thr_curve_pts = 5;
        m.thr_curve = kDefaultThrCurve;
    }
    if (m.arm_switch > 22) {
        m.arm_switch = 0;
    }
    if (m.timer_source > 17) {
        m.timer_source = 0;
    }
    if (m.timer_secs > 3600) {
        m.timer_secs = 0;
    }
    if (m.rf_power > 3) {
        m.rf_power = 0;
    }
    if (m.servo_rate_hz < 50 || m.servo_rate_hz > 400) {
        m.servo_rate_hz = 50;
    }
    if (m.rx_out_mode > 1) {
        m.rx_out_mode = 0;
    }
    if (m.rx_serial_proto > 1) {
        m.rx_serial_proto = 0;
    }
    if (m.name[0] == 0 || m.name[0] == 0xFF) {
        m.name = default_name(active_model);
    }
    if (m.vario_switch > 23) {
        m.vario_switch = 0;
    }
    for (auto& slot : m.telem_slots) {
        if (slot > 22) {
            slot = 22;
        }
    }
    if (m.vario_climb_deadband < 1 || m.vario_climb_deadband > 10) {
        m.vario_climb_deadband = 2;
    }
    if (m.vario_sink_deadband < 1 || m.vario_sink_deadband > 20) {
        m.vario_sink_deadband = 5;
    }
    if (m.vario_sink_mode > 2) {
        m.vario_sink_mode = 1;
    }
    if (m.vario_motor_mute > 1) {
        m.vario_motor_mute = 1;
    }
}

/// Clamps every field to its valid range and restores defaults for invalid values.
constexpr void sanitize(RadioStorage& s) {
    RadioConfig& r = s.radio;
    r.magic = kFlashMagic;
    r.version = kConfigVersion;
    if (r.vbat_warn_deci < 35 || r.vbat_warn_deci > 60) {
        r.vbat_warn_deci = 44;
    }
    if (r.active_model >= kNumModels) {
        r.active_model = 0;
    }
    if (r.backlight_brightness == 0 || r.backlight_brightness > 10) {
        r.backlight_brightness = 10;
    }
    if (r.lcd_contrast < 15 || r.lcd_contrast > 55) {
        r.lcd_contrast = 37;
    }
    if (r.usb_mode > 3) {
        r.usb_mode = 0;
    }
    if (r.ext_module_pwr > 1) {
        r.ext_module_pwr = 0;
    }
    if (r.throttle_trim > 2) {
        r.throttle_trim = 0;
    }
    if (r.tone_style > 1) {
        r.tone_style = 1;
    }
    if (r.servo_rate_hz < 50 || r.servo_rate_hz > 400) {
        r.servo_rate_hz = 50;
    }
    if (r.rx_out_mode > 1) {
        r.rx_out_mode = 0;
    }
    if (r.rx_serial_proto > 1) {
        r.rx_serial_proto = 0;
    }
    if (r.ext_switches > 1) {
        r.ext_switches = 0;
    }
    if (r.rear_left_func > 2) {
        r.rear_left_func = 0;
    }
    if (r.rear_right_func > 2) {
        r.rear_right_func = 0;
    }
    if (r.touch_disabled > 1) {
        r.touch_disabled = 0;
    }
    if (r.j4_sg_sh_en > 1) {
        r.j4_sg_sh_en = 0;
    }
    if (r.switch_beep > 1) {
        r.switch_beep = 1;
    }
    if (r.gimbal_type > 1) {
        r.gimbal_type = 0;
    }
    if (r.stick_invert_mask > 0x0F) {
        r.stick_invert_mask = 0x03;
    }
    for (auto& stick : r.sticks) {
        sanitize_calib(stick.min, stick.center, stick.max);
    }
    if (r.ext_adc > 1) {
        r.ext_adc = 0;
    }
    for (auto& pot : r.aux_pots) {
        sanitize_calib(pot.min, pot.center, pot.max);
    }
    for (auto& mode : r.adc_modes) {
        if (mode > 6) {
            mode = 0;
        }
    }
    if (r.trainer_config > 3) {
        r.trainer_config = 0;
    }
    sanitize_model(s.active_model, r.active_model);
}

/// Storage backend contract (static polymorphism, no virtual calls):
///   static Status read(uint16_t addr, std::span<uint8_t> out);
///   static Status write(uint16_t addr, std::span<const uint8_t> data);
template <typename Dev>
concept EepromDevice = requires(uint16_t addr, std::span<uint8_t> out, std::span<const uint8_t> in) {
    { Dev::read(addr, out) } -> std::same_as<Status>;
    { Dev::write(addr, in) } -> std::same_as<Status>;
};

template <EepromDevice Dev, typename T>
Status save_slot(uint16_t addr, const T& src) {
    static_assert(std::is_same_v<T, RadioConfig> || std::is_same_v<T, ModelConfig>);
    const auto payload = to_bytes(src);
    const SlotHeader hdr{
        .magic = kSlotMagic,
        .version = kSchemaVersion,
        .flags = kFlagValid,
        .len = static_cast<uint16_t>(sizeof(T)),
        .crc = crc16(payload),
    };
    const auto hdr_bytes = header_to_bytes(hdr);
    if (Dev::write(addr, hdr_bytes) != Status::Ok) {
        return Status::I2cError;
    }
    return Dev::write(static_cast<uint16_t>(addr + kHeaderSize), payload);
}

/// Loads a Radio/Model slot. Legacy ModelConfig slots are converted and re-saved in place.
template <EepromDevice Dev, typename T>
Status load_slot(uint16_t addr, T& dst) {
    static_assert(std::is_same_v<T, RadioConfig> || std::is_same_v<T, ModelConfig>);
    std::array<uint8_t, kHeaderSize> hdr_bytes{};
    if (const Status s = Dev::read(addr, hdr_bytes); s != Status::Ok) {
        return s;
    }
    const SlotHeader hdr = header_from_bytes(hdr_bytes);
    const uint16_t payload_addr = static_cast<uint16_t>(addr + kHeaderSize);

    if constexpr (std::is_same_v<T, ModelConfig>) {
        if (hdr.magic == kSlotMagic && hdr.flags == kFlagValid && hdr.len == kLegacyModelV1Size) {
            std::array<uint8_t, kLegacyModelV1Size> legacy{};
            if (const Status s = Dev::read(payload_addr, legacy); s != Status::Ok) {
                return s;
            }
            if (crc16(legacy) != hdr.crc) {
                return Status::CrcMismatch;
            }
            const ModelConfig upgraded = unpack_legacy_v1(from_bytes<LegacyModelConfigV1>(legacy));
            dst = upgraded;
            save_slot<Dev>(addr, upgraded);
            return Status::Ok;
        }
        if (hdr.magic == kSlotMagic && hdr.flags == kFlagValid &&
            (hdr.len == 348 || hdr.len == 352 || hdr.len == 360 || hdr.len == kMaxLegacyModelSize)) {
            std::array<uint8_t, kMaxLegacyModelSize> prev{};
            const auto prefix = std::span<uint8_t>(prev).first(hdr.len);
            if (const Status s = Dev::read(payload_addr, prefix); s != Status::Ok) {
                return s;
            }
            if (crc16(prefix) != hdr.crc) {
                return Status::CrcMismatch;
            }
            // Newer fields that the old layout lacks keep their defaults.
            auto image = to_bytes(default_model_config(0));
            for (std::size_t i = 0; i < hdr.len; ++i) {
                image[i] = prev[i];
            }
            const auto upgraded = from_bytes<ModelConfig>(image);
            dst = upgraded;
            save_slot<Dev>(addr, upgraded);
            return Status::Ok;
        }
    }

    if (const Status s = validate_header(hdr, sizeof(T)); s != Status::Ok) {
        return s;
    }
    std::array<uint8_t, sizeof(T)> payload{};
    if (const Status s = Dev::read(payload_addr, payload); s != Status::Ok) {
        return s;
    }
    if (crc16(payload) != hdr.crc) {
        return Status::CrcMismatch;
    }
    dst = from_bytes<T>(payload);
    return Status::Ok;
}

/// Loads radio config (A, then backup B) and the active model. Writes defaults when the EEPROM is blank.
template <EepromDevice Dev>
void load_storage(RadioStorage& s) {
    bool radio_loaded = load_slot<Dev>(kRadioSlotA, s.radio) == Status::Ok;
    if (!radio_loaded && load_slot<Dev>(kRadioSlotB, s.radio) == Status::Ok) {
        radio_loaded = true;
        save_slot<Dev>(kRadioSlotA, s.radio);
    }
    if (!radio_loaded) {
        s = default_storage();
        save_slot<Dev>(kRadioSlotA, s.radio);
        save_slot<Dev>(kRadioSlotB, s.radio);
        save_slot<Dev>(model_slot_addr(0).value(), s.active_model);
        sanitize(s);
        return;
    }

    const std::size_t active_idx = s.radio.active_model < kNumModels ? s.radio.active_model : 0;
    const auto addr = model_slot_addr(active_idx);
    if (!addr || load_slot<Dev>(*addr, s.active_model) != Status::Ok) {
        s.active_model = default_model_config(active_idx);
    }
    sanitize(s);
}

/// Reloads the model selected by `s.radio.active_model`. Falls back to defaults on failure.
template <EepromDevice Dev>
bool load_active_model(RadioStorage& s) {
    const std::size_t active_idx = s.radio.active_model < kNumModels ? s.radio.active_model : 0;
    const auto addr = model_slot_addr(active_idx);
    const bool ok = addr && load_slot<Dev>(*addr, s.active_model) == Status::Ok;
    if (!ok) {
        s.active_model = default_model_config(active_idx);
    }
    sanitize(s);
    return ok;
}

template <EepromDevice Dev>
bool save_model_to_slot(std::size_t index, const ModelConfig& model) {
    const auto addr = model_slot_addr(index);
    return addr && save_slot<Dev>(*addr, model) == Status::Ok;
}

template <EepromDevice Dev>
bool save_active_model(const RadioStorage& s) {
    const std::size_t active_idx = s.radio.active_model < kNumModels ? s.radio.active_model : 0;
    return save_model_to_slot<Dev>(active_idx, s.active_model);
}

/// Writes backup B first, then primary A, so a power loss always leaves one valid copy.
template <EepromDevice Dev>
bool save_radio_config(const RadioStorage& s) {
    const bool ok_b = save_slot<Dev>(kRadioSlotB, s.radio) == Status::Ok;
    const bool ok_a = save_slot<Dev>(kRadioSlotA, s.radio) == Status::Ok;
    return ok_a || ok_b;
}

template <EepromDevice Dev>
bool save_storage(const RadioStorage& s) {
    return save_radio_config<Dev>(s) && save_active_model<Dev>(s);
}

}  // namespace ferrox::logic::storage
