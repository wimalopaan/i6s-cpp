#include <array>
#include <cstdint>
#include <cstdio>
#include <span>

#include "ferrox/logic/afhds2a.hpp"
#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/calib.hpp"
#include "ferrox/logic/crsf.hpp"
#include "ferrox/logic/crsf_menu.hpp"
#include "ferrox/logic/dashboard.hpp"
#include "ferrox/logic/flight.hpp"
#include "ferrox/logic/format.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/glyphs.hpp"
#include "ferrox/logic/input.hpp"
#include "ferrox/logic/labels.hpp"
#include "ferrox/logic/menu.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/trim.hpp"
#include "ferrox/logic/widgets.hpp"
#include "ferrox/util/ring.hpp"

using namespace ferrox;

namespace test_eeprom {

/// RAM-backed 16 KB EEPROM with the same static read/write contract as the hardware driver.
struct FakeEeprom {
    static inline std::array<uint8_t, 16384> mem{};
    static inline bool fail_writes = false;

    static logic::storage::Status read(uint16_t addr, std::span<uint8_t> out) {
        if (addr + out.size() > mem.size()) {
            return logic::storage::Status::I2cError;
        }
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = mem[addr + i];
        }
        return logic::storage::Status::Ok;
    }

    static logic::storage::Status write(uint16_t addr, std::span<const uint8_t> data) {
        if (fail_writes || addr + data.size() > mem.size()) {
            return logic::storage::Status::I2cError;
        }
        for (std::size_t i = 0; i < data.size(); ++i) {
            mem[addr + i] = data[i];
        }
        return logic::storage::Status::Ok;
    }
};

/// Writes a slot header followed by a raw payload, as earlier firmware versions did.
inline void write_slot_raw(uint16_t addr, uint8_t version, std::size_t len, std::span<const uint8_t> payload) {
    const logic::storage::SlotHeader hdr{
        .magic = logic::storage::kSlotMagic,
        .version = version,
        .flags = 0,
        .len = static_cast<uint16_t>(len),
        .crc = logic::storage::crc16(payload.first(len)),
    };
    (void)FakeEeprom::write(addr, logic::storage::header_to_bytes(hdr));
    (void)FakeEeprom::write(static_cast<uint16_t>(addr + logic::storage::kHeaderSize), payload.first(len));
}

inline logic::storage::SlotHeader read_header(uint16_t addr) {
    std::array<uint8_t, logic::storage::kHeaderSize> bytes{};
    (void)FakeEeprom::read(addr, bytes);
    return logic::storage::header_from_bytes(bytes);
}

}  // namespace test_eeprom

static_assert(logic::crsf::kCrc8Table[1] == 0xD5);
static_assert(logic::crsf::kCrc8Table[2] == 0x7F);
static_assert(logic::crsf::kCrc8Table[3] == 0xAA);
static_assert(logic::crsf::kCrc8Table[255] == 0xF9);
static_assert(logic::crsf::us_to_crsf(988) == 172);
static_assert(logic::crsf::us_to_crsf(1500) == 992);
static_assert(logic::crsf::us_to_crsf(2012) == 1811);
static_assert(logic::crsf::us_to_crsf(100) == 172);
static_assert(logic::crsf::us_to_crsf(5000) == 1811);
static_assert(logic::crsf::decode_packed_vspeed(0) == 0);

static_assert(logic::usart_brr(115200) == 417);
static_assert(logic::usart_brr(416666) == 115);
static_assert(logic::usart_brr(420000) == 114);
static_assert(logic::usart_brr(921600) == 52);
static_assert(logic::usart_brr(1875000) == 26);
static_assert(logic::brr_for_baud_index(99) == logic::usart_brr(420000));

static_assert(logic::battery_mv(2048) == 5060);

static_assert(logic::is_dfu_requested(logic::kKeyRollLeft | logic::kKeyYawRight));
static_assert(!logic::is_dfu_requested(logic::kKeyRollLeft));

namespace {

int failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        ++failures;
        std::fputs(what, stderr);
        std::fputc('\n', stderr);
    }
}

void ring_test() {
    logic::SpscRing<8> ring;
    check(!ring.pop().has_value(), "empty ring must be empty");
    for (uint8_t i = 0; i < 7; ++i) {
        check(ring.push(i), "ring of 8 must accept 7 bytes");
    }
    check(!ring.push(0xFF), "ring must be full at N-1");
    for (uint8_t i = 0; i < 7; ++i) {
        auto v = ring.pop();
        check(v.has_value() && *v == i, "ring must be FIFO");
    }
}

void crsf_framer_test() {
    namespace crsf = logic::crsf;
    std::array<uint16_t, 16> channels{};
    channels.fill(1500);
    std::array<uint8_t, crsf::kRcExtFrameSize> frame{};
    const std::size_t len = crsf::build_rc_frame(channels, false, false, frame);
    check(len == crsf::kRcFrameSize, "standard RC frame must be 26 bytes");

    crsf::Framer framer;
    std::span<const uint8_t> out;
    for (std::size_t i = 0; i < len; ++i) {
        out = framer.push(frame[i], 0);
    }
    check(out.size() == len, "framer must return complete RC frame");
    check(!out.empty() && out[2] == crsf::kTypeRcChannelsPacked, "framer must keep RC type");

    crsf::Framer bad;
    frame[10] ^= 0x01;
    for (std::size_t i = 0; i < len; ++i) {
        out = bad.push(frame[i], 0);
    }
    check(out.empty(), "framer must drop frame with bad CRC");

    crsf::Telemetry t{};
    const std::array<uint8_t, 8> battery{0x00, 0x7D, 0x00, 0x00, 0x00, 0x01, 0x00, 0x55};
    crsf::parse_telemetry(crsf::kTypeBattery, battery, t, 42);
    check(t.rx_battery_mv == 12500, "battery must be 12500 mV");
    check(t.rx_battery_pct == 0x55 && t.connected, "battery pct and connected flag");
}

void crsf_menu_test() {
    namespace menu = logic::crsf::menu;

    // Select: parent 0, type 0x09, name "Mode", options "A;B;C", value 1
    constexpr std::array<uint8_t, 14> select{0x00, 0x09, 'M', 'o', 'd', 'e', 0x00,
                                             'A',  ';',  'B', ';', 'C', 0x00, 0x01};
    constexpr auto sel = menu::parse_entry(select);
    static_assert(sel && sel->type == menu::Type::Select && sel->max == 2 && sel->value == 1);
    static_assert(sel->text.len == 5 && sel->name.len == 4);

    // Uint16 with hidden bit: value 0x0100, min 0, max 0x0200, default 0, unit "ms"
    constexpr std::array<uint8_t, 15> u16{0x05, 0x82, 'T', 0x00, 0x01, 0x00, 0x00, 0x00,
                                          0x02, 0x00, 0x00, 0x00, 'm', 's', 0x00};
    constexpr auto p16 = menu::parse_entry(u16);
    static_assert(p16 && p16->hidden && p16->parent == 5 && p16->value == 0x100 && p16->max == 0x200);
    static_assert(p16->text.len == 2);

    constexpr auto sel_e = *sel;
    static_assert(menu::clamp_value(sel_e, 7) == 2 && menu::clamp_value(sel_e, -3) == 0);
    static_assert(menu::cycled_value(sel_e) == 2);

    static_assert([] {
        std::array<uint8_t, 2> enc{};
        const auto e = *menu::parse_entry(std::array<uint8_t, 15>{0x05, 0x82, 'T', 0x00, 0x01, 0x00, 0x00, 0x00,
                                                                  0x02, 0x00, 0x00, 0x00, 'm', 's', 0x00});
        return menu::encode_value(e, 0x1234, enc) == 2 && enc[0] == 0x02 && enc[1] == 0x00;
    }());

    static_assert(menu::next_param_for_folder(0, 0, 3, [] {
        std::array<uint8_t, menu::kMaxParamMap> m{};
        m.fill(menu::kNoParent);
        m[1] = 0;
        m[2] = 7;
        m[3] = 0;
        return m;
    }(), true) == 1);

    menu::Loader loader;
    const auto a0 = loader.start(0xEE, 2, 0);
    check(a0.kind == menu::Action::Kind::Read && a0.param == 0, "loader must start with param 0");

    // Param 0 root folder with child 1, sent as a single frame.
    const std::array<uint8_t, 6> root{0xFF, 0x0B, 'R', 0x00, 0x01, 0xFF};
    auto step = loader.on_entry(0xEE, 0, 0, root, 10);
    check(step.kind == menu::Action::Kind::Read && step.param == 1, "root folder must read child param 1");

    // Multi-chunk param 1: first chunk requests the rest, second completes it.
    const std::array<uint8_t, 3> part1{0x00, 0x00, 'T'};
    step = loader.on_entry(0xEE, 1, 1, part1, 20);
    check(step.kind == menu::Action::Kind::Read && step.arg == 1, "first chunk must request chunk 1");
    const std::array<uint8_t, 2> part2{0x00, 0x00};
    step = loader.on_entry(0xEE, 1, 0, part2, 30);
    check(step.kind == menu::Action::Kind::Done && loader.state == menu::Loader::State::Ready,
          "last param in root must finish loading");
    check(loader.entries[1] && loader.entries[1]->name.len == 1, "param 1 must be stored");

    // Retry path on a fresh load with no reply: retries, then give up on param 1.
    menu::Loader retry;
    retry.start(0xEE, 1, 0);
    retry.on_entry(0xEE, 0, 0, root, 10);
    step = retry.tick(10 + 500);
    check(step.kind == menu::Action::Kind::Read && step.param == 1, "timeout must retry param 1");
    for (int i = 0; i < 10 && step.kind != menu::Action::Kind::Done; ++i) {
        step = retry.tick(retry.next_req_ms);
    }
    check(step.kind == menu::Action::Kind::Done && retry.state == menu::Loader::State::Ready,
          "retries exhausted must finish loading");

    menu::Command cmd;
    cmd.start(2, 0);
    cmd.on_status(2, static_cast<uint8_t>(menu::CmdStatus::ConfirmationNeeded), 0, 10);
    check(cmd.kind == menu::Command::Kind::WaitingConfirm, "confirmation-needed must wait for confirm");
    const auto confirm = cmd.confirm(20);
    check(confirm.kind == menu::Action::Kind::Write && confirm.arg == 4, "confirm must write 4");
    cmd.on_status(2, static_cast<uint8_t>(menu::CmdStatus::Progress), 5, 30);
    check(cmd.kind == menu::Command::Kind::Running && cmd.poll_interval_ms == 500, "progress runs with 500 ms poll");
    const auto poll = cmd.tick(30 + 500);
    check(poll.kind == menu::Action::Kind::Write && poll.arg == 6, "running command must poll");
    cmd.on_status(2, static_cast<uint8_t>(menu::CmdStatus::Ready), 0, 600);
    check(cmd.kind == menu::Command::Kind::Completed, "ready must complete command");
    cmd.tick(600 + 2500);
    check(cmd.kind == menu::Command::Kind::Idle, "completed command must go idle after display time");
}

void storage_test() {
    namespace st = logic::storage;
    static_assert(st::EepromDevice<test_eeprom::FakeEeprom>);
    static_assert(st::crc16(std::array<uint8_t, 9>{'1', '2', '3', '4', '5', '6', '7', '8', '9'}) == 0xFEE8);
    static_assert(st::model_slot_addr(0).value() == 0x0200);
    static_assert(st::model_slot_addr(19).value() == 0x3B00);
    static_assert(!st::model_slot_addr(20).has_value());
    static_assert(st::default_storage().active_model.name[7] == '1');

    using Dev = test_eeprom::FakeEeprom;
    auto& mem = Dev::mem;

    mem.fill(0xFF);
    st::RadioStorage fresh{};
    st::load_storage<Dev>(fresh);
    check(fresh.radio.lcd_contrast == 37 && fresh.active_model.timer_secs == 300,
          "blank EEPROM must load defaults");
    check(test_eeprom::read_header(st::kRadioSlotA).magic == st::kSlotMagic,
          "blank load must write a valid radio slot A");

    st::RadioStorage s = fresh;
    s.radio.lcd_contrast = 50;
    s.active_model.rx_id = 0xABCD1234;
    s.active_model.name = {'S', 'K', 'Y', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    check(st::save_storage<Dev>(s), "save_storage must succeed");
    st::RadioStorage loaded{};
    st::load_storage<Dev>(loaded);
    check(loaded.radio.lcd_contrast == 50 && loaded.active_model.rx_id == 0xABCD1234 &&
              loaded.active_model.name[0] == 'S',
          "round trip must restore radio and active model");

    mem[st::kRadioSlotA + st::kHeaderSize + 1] ^= 0xFF;
    st::RadioStorage backup{};
    st::load_storage<Dev>(backup);
    check(backup.radio.lcd_contrast == 50, "corrupt slot A must fall back to slot B");
    st::SlotHeader repaired = test_eeprom::read_header(st::kRadioSlotA);
    check(repaired.magic == st::kSlotMagic, "fallback must repair slot A");

    Dev::fail_writes = true;
    check(!st::save_storage<Dev>(s), "failing EEPROM must report save failure");
    Dev::fail_writes = false;

    st::RadioStorage bad = s;
    bad.radio.vbat_warn_deci = 200;
    bad.active_model.mixes[0].target_ch = 30;
    bad.active_model.name[0] = 0;
    st::sanitize(bad);
    check(bad.radio.vbat_warn_deci == 44, "vbat out of range must reset to 4.4 V");
    check(bad.active_model.mixes[0].weight == 100 && bad.active_model.mixes[0].target_ch == 0,
          "invalid mix line must be disabled");
    check(bad.active_model.name[7] == '1' && bad.active_model.name[8] == ' ',
          "empty name must become MODEL nn");

    mem.fill(0xFF);
    st::LegacyModelConfigV1 v1{};
    v1.name = {'L', 'E', 'G', 'A', 'C', 'Y', 0, 0, 0, 0};
    v1.rx_id = 0x1234;
    v1.thr_curve = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    v1.mixes[0] = st::LegacyMixLineV1{3, 4, -50, 5, 2, 1};
    const auto v1_bytes = st::to_bytes(v1);
    test_eeprom::write_slot_raw(st::kModelBase, 1, st::kLegacyModelV1Size, v1_bytes);
    st::ModelConfig migrated{};
    check(st::load_slot<Dev>(st::kModelBase, migrated) == st::Status::Ok, "legacy v1 slot must load");
    check(migrated.rx_id == 0x1234 && migrated.name[0] == 'L', "v1 identity fields must migrate");
    check(migrated.thr_curve[1] == 2 && migrated.thr_curve[8] == 9 && migrated.thr_curve[9] == 0 &&
              migrated.thr_curve[4] == 5,
          "v1 throttle curve must not keep default points");
    check(migrated.mixes[0].weight == -50 && migrated.mixes[0].weight_src == 0 &&
              migrated.mixes[8].weight == 100,
          "v1 mixer lines must migrate, extra lines stay disabled");
    st::SlotHeader upgraded = test_eeprom::read_header(st::kModelBase);
    check(upgraded.len == sizeof(st::ModelConfig), "legacy slot must be re-saved as v5");

    mem.fill(0xFF);
    st::ModelConfig v4 = st::default_model_config(3);
    v4.rx_id = 77;
    v4.name[0] = 'Q';
    const auto v4_bytes = st::to_bytes(v4);
    test_eeprom::write_slot_raw(st::model_slot_addr(3).value(), 1, 348,
                                std::span<const uint8_t>(v4_bytes).first(348));
    st::ModelConfig v4_loaded{};
    check(st::load_slot<Dev>(st::model_slot_addr(3).value(), v4_loaded) == st::Status::Ok &&
              v4_loaded.rx_id == 77 && v4_loaded.name[0] == 'Q',
          "legacy v2-v4 slot must migrate");
}

// Compile-time checks of the pure mixer math.
static_assert(logic::mixer::apply_dr_expo(1000, 100, 50) == 1000);
static_assert(logic::mixer::apply_dr_expo(-1000, 100, -50) == -1000);
static_assert(logic::trim::apply(1500, 10) == 1540);
static_assert(logic::trim::apply(990, -25) == logic::kChannelMinUs);

void mixer_test() {
    namespace mx = logic::mixer;
    using logic::Switches;
    using logic::SwitchPos;
    using logic::storage::default_model_config;
    using logic::storage::MixLine;
    using logic::storage::ModelConfig;

    const logic::trim::TrimValues no_trim{};
    const std::array<int16_t, 2> pots0{0, 0};
    const Switches sw_default{};

    auto run = [&](int16_t roll, int16_t pitch, uint16_t thr, int16_t yaw, std::span<const int16_t> pots,
                   const Switches& sw, const ModelConfig& m) {
        return mx::compute_channels(roll, pitch, thr, yaw, pots, sw, m, no_trim, 0);
    };

    // Dual rates and expo.
    check(mx::apply_dr_expo(0, 100, 0) == 0 && mx::apply_dr_expo(500, 100, 0) == 500, "linear identity");
    check(mx::apply_dr_expo(1000, 50, 0) == 500 && mx::apply_dr_expo(-1000, 50, 0) == -500, "50% rate");
    check(mx::apply_dr_expo(1000, 10, 0) == 300, "rate clamps to 30");
    check(mx::apply_dr_expo(1000, 120, 0) == 1000, "rate clamps to 100");
    check(mx::apply_dr_expo(500, 100, 50) < 500, "positive expo softens center");
    check(mx::apply_dr_expo(500, 100, -50) > 500, "negative expo sharpens center");

    // Switch conditions.
    Switches sw{};
    sw.sa = SwitchPos::Up;
    sw.sb = SwitchPos::Mid;
    sw.sc = SwitchPos::Down;
    sw.sd = SwitchPos::Up;
    check(mx::is_switch_active(0, sw) && mx::is_switch_active(1, sw) && !mx::is_switch_active(2, sw),
          "SA conditions");
    check(mx::is_switch_active(4, sw) && !mx::is_switch_active(5, sw), "SB mid/down");
    check(mx::is_switch_active(8, sw) && !mx::is_switch_active(7, sw), "SC down/mid");
    check(!mx::eval_arm_switch(0, sw) && !mx::eval_arm_switch(23, sw), "arm switch bounds");

    // Centered and full-deflection output.
    ModelConfig base = default_model_config(0);
    auto centered = run(0, 0, 500, 0, pots0, sw_default, base);
    check(centered[0] == 1500 && centered[1] == 1500 && centered[2] == 1500 && centered[3] == 1500,
          "neutral sticks must center CH1..CH4");
    auto full = run(1000, 0, 0, 0, pots0, sw_default, base);
    check(full[0] == logic::kChannelMaxUs && full[2] == logic::kChannelMinUs, "full deflection bounds");
    auto neg = run(-1000, 0, 1000, 0, pots0, sw_default, base);
    check(neg[0] == logic::kChannelMinUs && neg[2] == logic::kChannelMaxUs, "negative deflection bounds");

    // Wing/tail templates.
    ModelConfig elevon = base;
    elevon.wing_tail_mix = 1;
    auto e1 = run(0, 1000, 0, 0, pots0, sw_default, elevon);
    check(e1[0] == 1756 && e1[1] == 1756, "elevon pitch-up at 50%");
    auto e2 = run(1000, 1000, 0, 0, pots0, sw_default, elevon);
    check(e2[0] == 1500 && e2[1] == logic::kChannelMaxUs, "elevon corner without clipping");
    elevon.template_diff = 50;
    auto e3 = run(1000, 0, 0, 0, pots0, sw_default, elevon);
    check(e3[1] == 1756 && e3[0] == 1372, "elevon differential attenuates down side");

    ModelConfig vtail = base;
    vtail.wing_tail_mix = 2;
    auto v1 = run(0, 1000, 0, 0, pots0, sw_default, vtail);
    check(v1[1] == 1756 && v1[3] == 1756, "V-tail pitch-up at 50%");
    auto v2 = run(0, 1000, 0, 1000, pots0, sw_default, vtail);
    check(v2[1] == logic::kChannelMaxUs && v2[3] == 1500, "V-tail corner");

    // Unipolar throttle source used as compensation.
    ModelConfig uni = base;
    uni.mixes[0] = MixLine{.target_ch = 2, .source = 30, .weight_src = 0, .weight = -20, .offset = 0,
                           .switch_sel = 0, .mode = 0, .reserved = 0};
    check(run(0, 0, 0, 0, pots0, sw_default, uni)[1] == 1500, "elevator neutral at idle throttle");
    check(run(0, 0, 1000, 0, pots0, sw_default, uni)[1] == 1398, "elevator compensation at full throttle");

    // Aux channels 15..18 mapped to switches, CH18 reversed.
    ModelConfig aux = base;
    aux.aux_channels[10] = 7;
    aux.aux_channels[11] = 8;
    aux.aux_channels[12] = 9;
    aux.aux_channels[13] = 10;
    aux.channel_reverse = 1u << 17;
    Switches sw18{};
    sw18.sa = SwitchPos::Down;
    sw18.sb = SwitchPos::Mid;
    sw18.sc = SwitchPos::Up;
    sw18.sd = SwitchPos::Down;
    auto c18 = run(0, 0, 500, 0, pots0, sw18, aux);
    check(c18[14] == logic::kChannelMaxUs && c18[15] == 1500 && c18[16] == logic::kChannelMinUs &&
              c18[17] == logic::kChannelMinUs,
          "aux switch mapping with reversal on CH18");

    // Channel reversing.
    ModelConfig rev = base;
    rev.channel_reverse = 1;
    check(run(1000, 0, 0, 0, pots0, sw_default, rev)[0] == logic::kChannelMinUs, "CH1 reversed");

    // Matrix mixer ADD and REPLACE.
    ModelConfig matrix = base;
    matrix.aux_channels[0] = 0;
    matrix.mixes[0] = MixLine{.target_ch = 5, .source = 1, .weight_src = 0, .weight = 50, .offset = 0,
                              .switch_sel = 0, .mode = 0, .reserved = 0};
    check(run(1000, 0, 0, 0, pots0, sw_default, matrix)[4] == 1756, "matrix ADD 50%");
    matrix.mixes[0].mode = 2;
    matrix.mixes[0].weight = 100;
    check(run(1000, 0, 0, 0, pots0, sw_default, matrix)[4] == logic::kChannelMaxUs, "matrix REPLACE 100%");

    // Dynamic weight modulation from pot VRA (CH5).
    ModelConfig dyn = base;
    dyn.aux_channels[0] = 5;
    dyn.aux_channels[1] = 0;
    dyn.mixes[0] = MixLine{.target_ch = 6, .source = 1, .weight_src = 5, .weight = 100, .offset = 0,
                           .switch_sel = 0, .mode = 0, .reserved = 0};
    const std::array<int16_t, 2> pot_full{1000, 0};
    const std::array<int16_t, 2> pot_neg{-500, 0};
    check(run(1000, 0, 0, 0, pot_full, sw_default, dyn)[5] == logic::kChannelMaxUs, "VRA +100% weight");
    check(run(1000, 0, 0, 0, pots0, sw_default, dyn)[5] == 1500, "VRA 0% weight");
    check(run(1000, 0, 0, 0, pot_neg, sw_default, dyn)[5] == 1244, "VRA -50% weight inverts");

    // Gvars as source and as weight modulator.
    ModelConfig gv = base;
    gv.gvars[0] = 500;
    gv.gvars[1] = -800;
    gv.mixes[0] = MixLine{.target_ch = 6, .source = 41, .weight_src = 0, .weight = 100, .offset = 0,
                          .switch_sel = 0, .mode = 2, .reserved = 0};
    check(run(0, 0, 0, 0, pots0, sw_default, gv)[5] == 1756, "GV1 as source");
    gv.mixes[0] = MixLine{.target_ch = 6, .source = 1, .weight_src = 38, .weight = 100, .offset = 0,
                          .switch_sel = 0, .mode = 2, .reserved = 0};
    check(run(1000, 0, 0, 0, pots0, sw_default, gv)[5] == 1091, "GV2 modulates roll weight");

    // All 32 mix lines execute in order.
    ModelConfig cascade = base;
    for (auto& m : cascade.mixes) {
        m = MixLine{.target_ch = 18, .source = 11, .weight_src = 0, .weight = 1, .offset = 0,
                    .switch_sel = 0, .mode = 0, .reserved = 0};
    }
    check(run(0, 0, 0, 0, pots0, sw_default, cascade)[17] == 1663, "32 cascading mixes");

    // Switch-based source, 3-position mapping.
    ModelConfig sa_map = base;
    sa_map.aux_channels[0] = 7;
    Switches s3{};
    s3.sa = SwitchPos::Up;
    check(run(0, 0, 500, 0, pots0, s3, sa_map)[4] == logic::kChannelMinUs, "SA Up -> 988");
    s3.sa = SwitchPos::Mid;
    check(run(0, 0, 500, 0, pots0, s3, sa_map)[4] == 1500, "SA Mid -> 1500");
    s3.sa = SwitchPos::Down;
    check(run(0, 0, 500, 0, pots0, s3, sa_map)[4] == logic::kChannelMaxUs, "SA Down -> 2012");

    // Heli throttle hold and arm-switch lockout.
    ModelConfig heli = base;
    heli.model_type = static_cast<uint8_t>(mx::ModelType::Heli);
    Switches hold{};
    hold.sa = SwitchPos::Down;
    check(run(0, 0, 1000, 0, pots0, hold, heli)[2] == logic::kChannelMinUs, "heli throttle hold");
    Switches flight{};
    flight.sa = SwitchPos::Up;
    check(run(0, 0, 1000, 0, pots0, flight, heli)[2] == logic::kChannelMaxUs, "heli flight throttle");

    ModelConfig armed = base;
    armed.arm_switch = 1;
    check(run(0, 0, 1000, 0, pots0, hold, armed)[2] == logic::kChannelMinUs, "disarmed throttle lockout");
    check(run(0, 0, 1000, 0, pots0, flight, armed)[2] == logic::kChannelMaxUs, "armed throttle output");

    // Flight timer trigger modes.
    bool latched = false;
    check(!mx::is_timer_active(0, 500, latched, sw_default, true), "timer off");
    check(!mx::is_timer_active(1, -950, latched, sw_default, true) &&
              mx::is_timer_active(1, -850, latched, sw_default, true),
          "THs threshold");
    latched = false;
    check(!mx::is_timer_active(2, -1000, latched, sw_default, true), "THt idle not latched");
    check(mx::is_timer_active(2, -500, latched, sw_default, true), "THt latches on throttle");
    check(mx::is_timer_active(2, -1000, latched, sw_default, true), "THt keeps running at idle");
    check(mx::is_timer_active(3, -1000, latched, sw_default, true), "timer always on");
    check(!mx::is_timer_active(3, 0, latched, sw_default, false), "timer inhibited when disarmed");
}

void input_test() {
    namespace in = logic::input;
    namespace storage = logic::storage;
    using logic::SwitchPos;

    check(logic::battery_mv(0) <= 200, "battery empty");
    check(logic::battery_mv(2048) > 4500 && logic::battery_mv(2048) < 5500, "battery half scale");
    check(logic::battery_mv(4095) > 9500 && logic::battery_mv(4095) < 10500, "battery full scale");

    // Detent pots: solid zero in the window, full travel outside.
    in::AxisCalib detent{};
    detent.min = 1000;
    detent.center = 2048;
    detent.max = 3000;
    check(detent.normalize_detent(2048, in::kDetentDeadband) == 0, "detent center is zero");
    check(detent.normalize_detent(2048 - 15, in::kDetentDeadband) == 0, "detent low edge is zero");
    check(detent.normalize_detent(2048 + 15, in::kDetentDeadband) == 0, "detent high edge is zero");
    check(detent.normalize_detent(1000, in::kDetentDeadband) == -1000, "detent min is -1000");
    check(detent.normalize_detent(3000, in::kDetentDeadband) == 1000, "detent max is +1000");

    check(in::decode_switch_6pos(100) == 1 && in::decode_switch_6pos(1000) == 2 &&
              in::decode_switch_6pos(1700) == 3 && in::decode_switch_6pos(2400) == 4 &&
              in::decode_switch_6pos(3100) == 5 && in::decode_switch_6pos(3800) == 6,
          "6-pos steps");
    check(in::aux_index_to_raw(0) == 4 && in::aux_index_to_raw(2) == 8 && in::aux_index_to_raw(4) == 6 &&
              in::aux_index_to_raw(6) == 11 && in::aux_index_to_raw(9) == 14,
          "aux to raw ADC mapping");

    // Hall gimbals: deadband and inversion.
    in::AxisCalib hall{};
    hall.min = 1000;
    hall.center = 2048;
    hall.max = 3000;
    hall.deadband = 3;
    check(hall.normalize(2048) == 0 && hall.normalize(2049) == 0 && hall.normalize(2051) == 0 &&
              hall.normalize(2045) == 0,
          "hall deadband is zero");
    check(hall.normalize(2060) > 0 && hall.normalize(2030) < 0, "hall moves outside deadband");
    in::AxisCalib inv{};
    inv.min = 1000;
    inv.center = 2048;
    inv.max = 3000;
    inv.deadband = 3;
    inv.invert = true;
    check(inv.normalize(2060) < 0 && inv.normalize(2030) > 0, "inverted axis");

    // Calibration from radio config: gimbal inversion mask and hall deadband.
    auto cfg = storage::default_storage().radio;
    cfg.gimbal_type = 1;
    cfg.stick_invert_mask = 0b0000'1001;
    in::Calibration calib{};
    in::apply_calibration(cfg, calib);
    check(calib.rh.invert && !calib.rv.invert && !calib.lv.invert && calib.lh.invert, "stick inversion mask");
    check(calib.rh.deadband == in::kHallDeadband, "hall stick deadband");

    // Extended ADC: 2-pos, 3-pos, 6-pos and pot on VRC..VRF.
    auto ext = storage::default_storage().radio;
    ext.ext_adc = 1;
    ext.adc_modes[6] = 1;
    ext.adc_modes[7] = 2;
    ext.adc_modes[8] = 3;
    ext.adc_modes[9] = 4;
    in::Calibration ecal{};
    in::apply_calibration(ext, ecal);
    in::Raw adc_frame{};
    adc_frame.fill(2048);
    adc_frame[11] = 3500;
    adc_frame[12] = 2048;
    adc_frame[13] = 2500;
    adc_frame[14] = 4095;
    const auto st = in::process(adc_frame, in::BoardSwitches{}, ecal);
    check(st.switches.swi == SwitchPos::Down, "ext 2-pos switch down");
    check(st.switches.swj == SwitchPos::Mid, "ext 3-pos switch mid");
    check(st.aux_pots[8] == 200, "ext 6-pos step 4");
    check(st.aux_pots[9] == 1000, "ext pot at max");
    check(st.pots.vr3 == st.aux_pots[6], "pots mirror aux channels");

    // Board switches SE/SF on PC12/PC15 (low = Down) only when external switches are enabled.
    auto sw_cfg = storage::default_storage().radio;
    sw_cfg.ext_switches = 1;
    in::Calibration scal{};
    in::apply_calibration(sw_cfg, scal);
    const auto sw_st = in::process(in::Raw{}, in::BoardSwitches{.se_high = false, .sf_high = true}, scal);
    check(sw_st.switches.se == SwitchPos::Down && sw_st.switches.sf == SwitchPos::Up, "SE/SF pin decode");
    in::Calibration off{};
    in::apply_calibration(storage::default_storage().radio, off);
    const auto off_st = in::process(in::Raw{}, in::BoardSwitches{.se_high = false, .sf_high = false}, off);
    check(off_st.switches.se == SwitchPos::Up && off_st.switches.sf == SwitchPos::Up, "SE/SF disabled");
}

void safety_test() {
    namespace sf = logic::safety;
    namespace fl = logic::flight;
    using logic::Switches;
    using logic::SwitchPos;
    using logic::input::Gimbals;
    using logic::input::InputState;
    using logic::mixer::ModelType;
    using logic::storage::default_model_config;

    check(sf::stick_mode_from(0) == sf::StickMode::Mode1 && sf::stick_mode_from(9) == sf::StickMode::Mode2,
          "stick mode fallback to Mode2");
    check(sf::throttle_raw_index(sf::StickMode::Mode1) == 1 && sf::throttle_raw_index(sf::StickMode::Mode2) == 2,
          "throttle raw index per mode");

    const sf::ChannelCalibration cal{};
    check(cal.normalize_unipolar(800) == 0 && cal.normalize_unipolar(3300) == 1000, "unipolar endpoints");
    check(cal.normalize_bipolar(2048) == 0 && cal.normalize_bipolar(3300) == 1000 &&
              cal.normalize_bipolar(800) == -1000,
          "bipolar endpoints");
    logic::storage::ChannelCalib bad{};
    bad.min = 0;
    bad.center = 0;
    bad.max = 0;
    const auto fallback = sf::ChannelCalibration::from(bad);
    check(fallback.min == 800 && fallback.center == 2048 && fallback.max == 3300, "invalid calib falls back");

    const std::array<sf::ChannelCalibration, 4> cals{cal, cal, cal, cal};
    std::array<uint16_t, 4> raw{2048, 2048, 800, 2048};
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::Airplane, false) ==
              sf::Preflight::Ok,
          "airplane low throttle ok");
    raw[2] = 3300;
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::Airplane, false) ==
              sf::Preflight::ThrottleHigh,
          "airplane high throttle blocks");
    raw[2] = 2048;
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::General, false) ==
              sf::Preflight::Ok,
          "general centered throttle ok");
    raw[2] = 800;
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::General, false) ==
              sf::Preflight::ThrottleNotCentered,
          "general off-center throttle blocks");
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::Heli, false) ==
              sf::Preflight::ThrottleHoldDisengaged,
          "heli without hold blocks");
    check(sf::check_preflight_throttle(raw, cals, sf::StickMode::Mode2, ModelType::Heli, true) == sf::Preflight::Ok,
          "heli with hold ok");

    Switches sw{};
    check(sf::switches_safe(sw, ModelType::Airplane), "all up is safe for airplane");
    sw.sb = SwitchPos::Down;
    check(!sf::switches_safe(sw, ModelType::Airplane), "SB down blocks");
    sw = Switches{};
    sw.sa = SwitchPos::Down;
    check(!sf::switches_safe(sw, ModelType::Airplane), "SA down blocks airplane");
    check(sf::switches_safe(sw, ModelType::Heli), "SA down is safe for heli hold");

    const auto& idle = sf::kSafeIdleChannels;
    check(idle[0] == 1500 && idle[2] == 988 && idle[4] == 988 && idle[5] == 988 && idle[8] == 988 &&
              idle[9] == 988 && idle[1] == 1500,
          "safe idle channels");

    InputState in{};
    in.gimbals.rh = 500;
    in.gimbals.rv = 300;
    in.gimbals.lv = -200;
    in.gimbals.lh = 100;
    const auto m1 = fl::flight_controls(in.gimbals, sf::StickMode::Mode1);
    check(m1.aileron == 500 && m1.elevator == -200 && m1.throttle == 300 && m1.rudder == 100, "mode 1 mapping");
    const auto m2 = fl::flight_controls(in.gimbals, sf::StickMode::Mode2);
    check(m2.aileron == 500 && m2.elevator == 300 && m2.throttle == -200 && m2.rudder == 100, "mode 2 mapping");
    const auto m3 = fl::flight_controls(in.gimbals, sf::StickMode::Mode3);
    check(m3.aileron == 100 && m3.elevator == -200 && m3.throttle == 300 && m3.rudder == 500, "mode 3 mapping");
    const auto m4 = fl::flight_controls(in.gimbals, sf::StickMode::Mode4);
    check(m4.aileron == 100 && m4.elevator == 300 && m4.throttle == -200 && m4.rudder == 500, "mode 4 mapping");

    const auto model = default_model_config(0);
    const logic::trim::TrimValues no_trim{};
    in.gimbals = Gimbals{};
    const auto ch = fl::rf_channels(in, sf::StickMode::Mode2, model, no_trim, 0);
    check(ch[0] == 1500 && ch[1] == 1500, "centered sticks give centered channels");
    in.gimbals.lv = 1000;
    const auto thr_ch = fl::rf_channels(in, sf::StickMode::Mode2, model, no_trim, 0);
    check(thr_ch[2] > 1500 && thr_ch[2] <= 2012, "throttle stick up raises throttle");
}

int lit_pixels(const ferrox::logic::display::Framebuffer& fb) {
    int n = 0;
    for (uint8_t b : fb.data()) {
        for (int i = 0; i < 8; ++i) {
            n += (b >> i) & 1;
        }
    }
    return n;
}

void display_test() {
    using ferrox::logic::display::Framebuffer;
    using namespace ferrox::logic;

    Framebuffer fb;
    fb.set_pixel(5, 9, true);
    check(fb.data()[1 * 128 + 5] == 0x02 && lit_pixels(fb) == 1, "set_pixel page/bit placement");
    fb.set_pixel(-1, 0, true);
    fb.set_pixel(128, 0, true);
    fb.set_pixel(0, 64, true);
    check(lit_pixels(fb) == 1, "set_pixel ignores out-of-bounds");

    fb.clear();
    fb.fill_rect(-5, -5, 10, 10, true);
    check(lit_pixels(fb) == 25, "fill_rect clips to the screen");

    fb.clear();
    fb.draw_rect(0, 0, 10, 6, true);
    check(lit_pixels(fb) == 28, "draw_rect outline pixel count");

    fb.clear();
    fb.draw_char_4x6(0, 0, 'A', false);
    check(lit_pixels(fb) == 10 && (fb.data()[0] & 0x02) != 0, "4x6 'A' glyph");

    fb.fill_rect(0, 0, 4, 6, true);
    fb.draw_char_4x6(0, 0, 'A', true);
    check(lit_pixels(fb) == 14, "inverted glyph clears lit pixels only");

    fb.clear();
    fb.draw_char_4x6(0, 0, '\x7f', false);
    fb.draw_char_6x10(0, 0, '\t', false);
    check(lit_pixels(fb) == 0, "non-printable characters are skipped");

    fb.clear();
    fb.draw_char_6x10(0, 0, 'I', false);
    check(lit_pixels(fb) == 11, "6x10 'I' glyph");

    fb.clear();
    widgets::draw_header(fb, "AB");
    check((fb.data()[1 * 128 + 0] & 0x08) != 0 && (fb.data()[1 * 128 + 127] & 0x08) != 0,
          "header divider spans full width at y=11");

    fb.clear();
    widgets::draw_footer(fb, "X");
    check((fb.data()[6 * 128 + 0] & 0x80) != 0 && (fb.data()[6 * 128 + 127] & 0x80) != 0,
          "footer divider at y=55");

    fb.clear();
    widgets::draw_list_row(fb, 1, true, "A", "", 0);
    check((fb.data()[2 * 128 + 2] & 0x80) != 0 && (fb.data()[3 * 128 + 125] & 0x80) != 0 &&
              (fb.data()[2 * 128 + 126] & 0x80) == 0 && (fb.data()[4 * 128 + 2] & 0x01) == 0,
          "selected list row highlight 2..125 x 23..31");

    fb.clear();
    widgets::draw_scrollbar(fb, 0, 4, 14, 40);
    check((fb.data()[1 * 128 + 126] & 0x40) != 0 && (fb.data()[1 * 128 + 125] & 0x40) != 0 &&
              (fb.data()[5 * 128 + 125] & 0x10) == 0,
          "scrollbar track and thumb at top");

    fb.clear();
    widgets::draw_progress_bar(fb, 0, 0, 102, 8, 0, 0);
    check((fb.data()[0 * 128 + 50] & 0x10) != 0 && (fb.data()[0 * 128 + 51] & 0x10) == 0,
          "progress bar fill at mid-scale");

    const auto d = widgets::format_u16_4(42);
    const auto max = widgets::format_u16_4(65535);
    check(d[0] == '0' && d[1] == '0' && d[2] == '4' && d[3] == '2', "gauge value zero-padded");
    check(max[0] == '9' && max[3] == '9', "gauge value clamped to 9999");
}

void dashboard_test() {
    using logic::dashboard::PageState;
    namespace fmt = logic::format;

    check(fmt::format_percent(500).view() == "+ 50%", "percent positive");
    check(fmt::format_percent(-1000).view() == "-100%", "percent full negative");
    check(fmt::format_percent(0).view() == "   0%", "percent centre");
    check(fmt::format_throttle_percent(0).view() == "  50%", "throttle centre is 50%");
    check(fmt::format_throttle_percent(1000).view() == " 100%", "throttle full is 100%");
    check(fmt::format_trim('A', 5).view() == "TRM A:+05", "trim formatting");
    check(fmt::format_vbat(4200).view() == "4.20V", "vbat formatting");
    check(fmt::format_dec2(3).view() == "03", "dec2 zero padded");
    check(fmt::format_hex32(0x1A2B3C4Du).view() == "1A2B3C4D", "hex32 uppercase, 8 digits");
    check(fmt::trim_spaces("  AB ") == "AB", "trim spaces both sides");

    PageState ps{};
    logic::dashboard::handle_keys(ps, logic::dashboard::kKeyPageDown);
    check(ps.page == 1, "page down advances");
    logic::dashboard::handle_keys(ps, logic::dashboard::kKeyPageUp);
    check(ps.page == 0, "page up goes back");
    logic::dashboard::handle_keys(ps, logic::dashboard::kKeyPageUp);
    check(ps.page == 4, "page up wraps 0 to last page");
    logic::dashboard::handle_keys(ps, logic::dashboard::kKeyPageDown);
    check(ps.page == 0, "page down wraps last page to 0");

    // Layout invariant: pots never start on top of switch slots.
    bool layout_ok = true;
    for (std::size_t sw = 0; sw <= 16; ++sw) {
        for (std::size_t pots = 0; pots <= 10; ++pots) {
            const auto m = logic::dashboard::adaptive_metrics(sw, pots);
            if (m.pot_w < 0 || m.pot_start_x > 126 + 2) {
                layout_ok = false;
            }
            if (sw > 0 && pots > 0 && m.pot_w > 0) {
                const int last_end = 2 + static_cast<int>(sw - 1) * m.spacing + (m.compact ? 7 : 10);
                if (m.pot_start_x < last_end + 3) {
                    layout_ok = false;
                }
            }
        }
    }
    check(layout_ok, "adaptive layout never overlaps switches and pots");

    const auto ten_sw_two_pots = logic::dashboard::adaptive_metrics(10, 2);
    check(ten_sw_two_pots.compact && ten_sw_two_pots.spacing == 8 && ten_sw_two_pots.pot_start_x == 85,
          "10 switches + 2 pots layout matches Rust math");

    // Smoke test: every page renders into the framebuffer without touching out-of-range memory.
    logic::input::InputState in{};
    in.battery_mv = 7400;
    logic::storage::RadioStorage st{};
    logic::trim::TrimValues trims{};
    std::array<uint16_t, logic::kNumChannels> chs{};
    chs.fill(1500);
    logic::dashboard::View view{in, st, trims, chs, true};

    logic::display::Framebuffer fb{};
    PageState render_state{};
    for (uint8_t p = 0; p < logic::dashboard::kPageCount; ++p) {
        render_state.page = p;
        fb.clear();
        logic::dashboard::render(fb, render_state, view);
        bool any = false;
        for (const auto b : fb.data()) {
            any = any || b != 0;
        }
        check(any, "dashboard page draws something");
    }
    check(render_state.blink_phase == logic::dashboard::kPageCount, "blink phase advances once per render");

    fb.clear();
    logic::dashboard::draw_splash(fb);
    bool splash_any = false;
    for (const auto b : fb.data()) {
        splash_any = splash_any || b != 0;
    }
    check(splash_any, "splash draws logo and text");
}

}  // namespace

void menu_test() {
    namespace mn = logic::menu;
    namespace labels = logic::labels;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    auto st = logic::storage::default_storage();
    st.radio.active_model = 2;
    st.active_model.model_type = 0;
    st.active_model.arm_switch = 0;
    st.active_model.timer_secs = 0;

    mn::State s{};
    auto tick = [&](uint16_t pressed, uint16_t held, uint32_t now) {
        return mn::update(s, mn::Input{pressed, held, now}, st);
    };
    auto press = [&](uint16_t key, uint32_t now) {
        const Event e = tick(key, key, now);
        tick(0, 0, now + 1);
        return e;
    };

    auto press_in = [&](mn::State& state, uint16_t key, uint32_t now) {
        const Event e = mn::update(state, mn::Input{key, key, now}, st);
        mn::update(state, mn::Input{0, 0, now + 1}, st);
        return e;
    };

    // Keys held when the menu opens must be released before they count.
    mn::open(s);
    check(s.screen == Screen::Main && s.waiting_release, "open must enter main menu and wait for release");
    tick(mn::kKeyDown, mn::kKeyDown, 0);
    check(s.main.selected == 0, "held key must not navigate while waiting for release");
    tick(0, 0, 10);
    check(!s.waiting_release, "release must clear waiting state");

    press(mn::kKeyDown, 20);
    check(s.main.selected == 1, "down must move selection");
    press(mn::kKeyUp, 40);
    press(mn::kKeyUp, 60);
    check(s.main.selected == 11 && s.main.scroll == 9, "up from top must wrap to last item with scroll window");
    press(mn::kKeyDown, 80);
    check(s.main.selected == 0 && s.main.scroll == 0, "down from last item must wrap to first");

    mn::ListPos p{};
    for (int i = 0; i < 3; ++i) {
        mn::step_list(p, 12, mn::kMainVisible, true);
    }
    check(p.selected == 3 && p.scroll == 1, "scroll must follow selection past the visible window");

    // Key repeat: immediate on press, then after 300 ms every 70 ms while held.
    mn::Repeat r{};
    check(mn::repeat_step(r, true, true, 0), "press must step immediately");
    check(!mn::repeat_step(r, false, true, 299), "no repeat before 300 ms");
    check(mn::repeat_step(r, false, true, 300), "first repeat at 300 ms");
    check(!mn::repeat_step(r, false, true, 369), "no repeat before 70 ms interval");
    check(mn::repeat_step(r, false, true, 370), "repeat at 70 ms interval");
    check(!mn::repeat_step(r, false, false, 400) && !r.active, "release must stop repeat");

    // Model select: open from item 0, navigate, load a model.
    press(mn::kKeyUp, 100);    // wraps to item 11
    press(mn::kKeyDown, 120);  // back to item 0
    const Event enter = press(mn::kKeyOk, 140);
    check(enter.action == Action::EnterModelSelect && s.screen == Screen::ModelSelect,
          "OK on Model Select must open the model list");
    check(s.models.selected == 2, "model list must start on the active model");
    press(mn::kKeyDown, 160);
    const Event sw = press(mn::kKeyOk, 180);
    check(sw.action == Action::SwitchModel && sw.index == 3, "OK must switch to the selected model");
    check(s.screen == Screen::Main && s.main.selected == 0, "switch must return to main menu");

    // Model setup: type cycling, name editing, arm switch and timer.
    press(mn::kKeyDown, 200);
    press(mn::kKeyOk, 220);
    check(s.screen == Screen::ModelSetup, "OK on Model Setup must open setup");

    press(mn::kKeyDown, 230);  // row 0 (Name) -> row 1 (Type)
    const Event type_ev = press(mn::kKeyOk, 240);
    check(type_ev.action == Action::SaveActiveModel && st.active_model.model_type == 1,
          "OK on Type must cycle to next type and request save");
    for (int i = 0; i < 4; ++i) {
        press(mn::kKeyOk, 260 + i * 10);
    }
    check(st.active_model.model_type == 0, "type must wrap back to first type");

    press(mn::kKeyDown, 300);  // -> Arm Sw
    press(mn::kKeyOk, 320);    // edit arm switch
    press(mn::kKeyDown, 330);
    check(st.active_model.arm_switch == 22, "arm switch down from 0 must wrap to last condition");
    press(mn::kKeyUp, 340);
    press(mn::kKeyUp, 350);
    check(st.active_model.arm_switch == 1, "arm switch up must step through conditions");
    const Event arm_done = press(mn::kKeyOk, 370);
    check(arm_done.action == Action::SaveActiveModel && !s.editing, "OK must finish arm switch edit");

    press(mn::kKeyDown, 380);  // -> Timer
    press(mn::kKeyOk, 390);    // edit timer
    press(mn::kKeyUp, 400);
    press(mn::kKeyUp, 410);
    press(mn::kKeyDown, 420);
    check(st.active_model.timer_secs == 30, "timer must step by 30 s");
    press(mn::kKeyCancel, 430);
    check(!s.editing && s.screen == Screen::ModelSetup, "cancel must finish timer edit only");

    press(mn::kKeyUp, 440);    // -> Arm Sw
    press(mn::kKeyUp, 450);    // -> Type
    press(mn::kKeyUp, 460);    // -> Name
    press(mn::kKeyOk, 470);    // edit name
    check(s.editing && s.name_cursor == 0, "OK on Name must start editing at first character");
    st.active_model.name = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    press(mn::kKeyUp, 480);
    check(st.active_model.name[0] == 'A', "up on space must give 'A'");
    press(mn::kKeyUp, 490);
    press(mn::kKeyDown, 500);
    check(st.active_model.name[0] == 'A', "up then down must return to same character");
    press(mn::kKeyOk, 510);
    check(s.name_cursor == 1 && s.editing, "OK must advance the name cursor");
    const Event name_done = press(mn::kKeyCancel, 520);
    check(name_done.action == Action::SaveActiveModel && !s.editing && s.name_cursor == 0,
          "cancel must end name edit and request save");

    press(mn::kKeyCancel, 530);
    check(s.screen == Screen::Main, "cancel in setup must return to main menu");
    check(s.main.selected == 1, "main menu position must be kept");

    press(mn::kKeyCancel, 540);
    check(s.screen == Screen::Closed, "cancel in main menu must close the menu");

    // Every screen must render without faulting.
    logic::display::Framebuffer fb{};
    const mn::ModelNames names{};
    mn::open(s);
    for (const Screen screen : {Screen::Main, Screen::ModelSelect, Screen::ModelSetup, Screen::ChannelReverse,
                                Screen::ChannelMonitor, Screen::DiagAnalog, Screen::SystemInfo}) {
        s.screen = screen;
        fb.clear();
        mn::render(fb, s, names, st, mn::Live{});
    }

    check(logic::glyphs::kMenuGlyphs[0][4] == 194 && logic::glyphs::kMenuGlyphs[11][13] == 194,
          "menu glyph table must match the Rust bitmaps");
    {
        logic::display::Framebuffer icons{};
        logic::glyphs::draw_glyph_12x12(icons, 0, 0, logic::glyphs::kMenuGlyphs[0], true);
        const int lit = lit_pixels(icons);
        icons.clear();
        logic::glyphs::draw_glyph_12x12(icons, 0, 0, logic::glyphs::kMenuGlyphs[0], false);
        check(lit > 0 && lit_pixels(icons) == 0, "glyph with on=false must not light pixels on a clear buffer");
    }

    check(logic::labels::switch_cond_label(22) == "SWJv", "arm switch label 22");
    check(logic::labels::switch_cond_label(99) == "ON", "out-of-range switch label falls back to ON");
    check(mn::next_ascii('Z') == '0' && mn::next_ascii('9') == '-' && mn::next_ascii('_') == ' ',
          "next_ascii wrap points");
    check(mn::prev_ascii('0') == 'Z' && mn::prev_ascii('A') == ' ', "prev_ascii wrap points");
    check(mn::prev_ascii(mn::next_ascii('M')) == 'M', "prev must invert next");
    check(mn::timer_text(3570).view() == std::string_view{"59:30"}, "timer text must format mm:ss");
    check(mn::timer_text(30).view() == std::string_view{"00:30"}, "timer text must pad seconds");

    const mn::ModelName named{'A', 'B', 'C', 0, 0, 0, 0, 0, 0, 0};
    check(mn::model_row_text(0, named, true).view() == std::string_view{"M01:ABC*"}, "active model row label");
    check(mn::model_row_text(9, named, false).view() == std::string_view{"M10:ABC"}, "two-digit model row label");

    // Channel labels follow the wing/tail mix and the aux source table.
    auto model = logic::storage::default_storage().active_model;
    check(labels::channel_label(0, model).view() == std::string_view{"1:AIL"}, "channel 1 label");
    check(labels::channel_label(4, model).view() == std::string_view{"5:SA"}, "channel 5 default aux source");
    check(labels::channel_label(9, model).view() == std::string_view{"10:SD"}, "channel 10 default aux source");
    check(labels::channel_label(17, model).view() == std::string_view{"18:None"}, "unset aux source is None");
    model.wing_tail_mix = 1;
    check(labels::channel_label(0, model).view() == std::string_view{"1:L.ELV"}, "elevon left label");
    check(labels::channel_label(1, model).view() == std::string_view{"2:R.ELV"}, "elevon right label");
    model.wing_tail_mix = 2;
    check(labels::channel_label(3, model).view() == std::string_view{"4:R.VTL"}, "v-tail right label");
    model.wing_tail_mix = 3;
    check(labels::channel_label(0, model).view() == std::string_view{"1:L.AIL"}, "flaperon left label");
    check(labels::channel_label(5, model).view() == std::string_view{"6:R.AIL"}, "flaperon right label");

    // Channel reverse toggles the active model bit and saves on cancel.
    mn::State rev{};
    st.active_model.channel_reverse = 0;
    rev.screen = Screen::ChannelReverse;
    mn::update(rev, mn::Input{0, 0, 0}, st);
    press_in(rev, mn::kKeyOk, 600);
    check(st.active_model.channel_reverse == 1u, "OK must toggle the selected channel");
    press_in(rev, mn::kKeyDown, 610);
    press_in(rev, mn::kKeyOk, 620);
    check(st.active_model.channel_reverse == 3u, "second channel must toggle independently");
    press_in(rev, mn::kKeyOk, 630);
    check(st.active_model.channel_reverse == 1u, "second OK must clear the bit again");
    const Event saved = press_in(rev, mn::kKeyCancel, 640);
    check(saved.action == Action::SaveActiveModel && rev.screen == Screen::Main, "cancel must save and return");

    // Channel monitor wraps through its three pages in both directions.
    mn::State mon{};
    mon.screen = Screen::ChannelMonitor;
    mn::update(mon, mn::Input{0, 0, 0}, st);
    press_in(mon, mn::kKeyUp, 700);
    check(mon.page == 2, "up from first monitor page must wrap to last");
    press_in(mon, mn::kKeyDown, 710);
    check(mon.page == 0, "down from last monitor page must wrap to first");
    press_in(mon, mn::kKeyCancel, 720);
    check(mon.screen == Screen::Main, "cancel must leave the monitor");

    // Analog diagnostics toggles between its two pages.
    mn::State diag{};
    diag.screen = Screen::DiagAnalog;
    mn::update(diag, mn::Input{0, 0, 0}, st);
    press_in(diag, mn::kKeyOk, 800);
    check(diag.page == 1, "OK must advance the diagnostic page");
    press_in(diag, mn::kKeyUp, 810);
    check(diag.page == 0, "up must go back a diagnostic page");

    // Monitor and ADC gauges clamp to their full scale instead of overflowing.
    logic::display::Framebuffer gauges{};
    mn::Live live{};
    live.rf.fill(3000);
    live.adc.fill(9999);
    live.mcu_name = "STM32F072VB";
    mn::State gs{};
    gs.screen = Screen::ChannelMonitor;
    gauges.clear();
    mn::render(gauges, gs, names, st, live);
    check(lit_pixels(gauges) > 0, "monitor must draw gauges for out-of-range values");
    gs.screen = Screen::DiagAnalog;
    gauges.clear();
    mn::render(gauges, gs, names, st, live);
    check(lit_pixels(gauges) > 0, "diagnostics must draw clamped ADC gauges");
    gs.screen = Screen::SystemInfo;
    gauges.clear();
    mn::render(gauges, gs, names, st, live);
    check(lit_pixels(gauges) > 0, "system info must render the MCU name");
}

void calib_test() {
    using namespace ferrox::logic;
    using calib::Input;
    using calib::Wizard;
    using calib::Step;

    const auto frame = [](Wizard& w, storage::RadioConfig& radio, const input::Raw& raw, uint16_t pressed,
                          uint16_t held, uint16_t dt) {
        return calib::update(w, Input{pressed, held, raw, dt}, radio);
    };

    storage::RadioConfig radio = storage::default_storage().radio;
    radio.ext_adc = 0;
    radio.gimbal_type = 0;
    for (auto& mode : radio.adc_modes) {
        mode = static_cast<uint8_t>(input::AdcMode::TwoPos);
    }
    radio.adc_modes[0] = static_cast<uint8_t>(input::AdcMode::Pot);
    radio.adc_modes[1] = static_cast<uint8_t>(input::AdcMode::Pot);
    const auto pot1_before = radio.aux_pots[1];

    input::Raw center{};
    center.fill(storage::kAdcCenter);

    Wizard w{};
    check(!calib::active(w), "calib: wizard starts inactive");
    calib::start(w);
    check(calib::active(w) && w.step == Step::Center, "calib: start enters Center");

    check(!frame(w, radio, center, logic::kKeyOk, logic::kKeyOk, 10) && w.step == Step::Center,
          "calib: OK held from the menu must not capture");
    frame(w, radio, center, 0, 0, 10);
    frame(w, radio, center, logic::kKeyOk, logic::kKeyOk, 10);
    check(w.step == Step::Limits, "calib: OK captures centre and moves to Limits");
    check(w.stick_center[0] == storage::kAdcCenter && w.pot_center[0] == storage::kAdcCenter,
          "calib: centre values captured");
    frame(w, radio, center, 0, 0, 10);
    check(!frame(w, radio, center, logic::kKeyOk, logic::kKeyOk, 10) && w.step == Step::Limits,
          "calib: OK rejected without travel");

    input::Raw lo = center;
    input::Raw hi = center;
    for (std::size_t i = 0; i < 4; ++i) {
        lo[i] = 1000;
        hi[i] = 3000;
    }
    lo[4] = 1000;
    hi[4] = 3000;
    lo[5] = 2000;
    hi[5] = 2100;
    frame(w, radio, lo, 0, 0, 10);
    frame(w, radio, hi, 0, 0, 10);
    const bool saved = frame(w, radio, hi, logic::kKeyOk, logic::kKeyOk, 10);
    check(saved && w.step == Step::Complete, "calib: OK with travel saves and enters Complete");
    check(radio.sticks[0].center == 2048 && radio.sticks[0].min == 1017 && radio.sticks[0].max == 2985,
          "calib: stick endpoints use the 63/64 margin");
    check(radio.aux_pots[0].center == 2048 && radio.aux_pots[0].min == 1017 && radio.aux_pots[0].max == 2985,
          "calib: pot with enough travel is saved");
    check(radio.aux_pots[1].min == pot1_before.min && radio.aux_pots[1].max == pot1_before.max,
          "calib: pot with span below 400 keeps its old calibration");

    frame(w, radio, hi, 0, 0, 500);
    check(w.step == Step::Complete, "calib: Complete holds for 1200 ms");
    frame(w, radio, hi, 0, 0, 500);
    frame(w, radio, hi, 0, 0, 500);
    check(!calib::active(w), "calib: Complete ends after 1200 ms");

    Wizard c{};
    calib::start(c);
    frame(c, radio, center, 0, 0, 10);
    frame(c, radio, center, calib::kKeyCancel, calib::kKeyCancel, 10);
    check(!calib::active(c), "calib: cancel in Center leaves the wizard");

    for (const std::size_t count : {std::size_t{1}, std::size_t{3}, std::size_t{7}}) {
        storage::RadioConfig r = storage::default_storage().radio;
        r.ext_adc = count > 6 ? 1 : 0;
        for (std::size_t ch = 0; ch < 10; ++ch) {
            r.adc_modes[ch] = static_cast<uint8_t>(ch < count ? input::AdcMode::Pot : input::AdcMode::TwoPos);
        }
        check(calib::configured_pots(r).count == count, "calib: configured pot count");
        Wizard lw{};
        calib::start(lw);
        lw.step = Step::Limits;
        logic::display::Framebuffer fb{};
        fb.clear();
        calib::render(fb, lw, center, r, 0);
        check(lit_pixels(fb) > 0, "calib: limits screen must draw");
    }

    logic::menu::State ms{};
    logic::menu::open(ms);
    ms.main.selected = logic::menu::kItemCalibration;
    storage::RadioStorage menu_storage = storage::default_storage();
    const auto ev0 = logic::menu::update(ms, logic::menu::Input{0, 0, 0}, menu_storage);
    const auto ev = logic::menu::update(ms, logic::menu::Input{logic::kKeyOk, logic::kKeyOk, 0}, menu_storage);
    check(ev0.action == logic::menu::Action::None, "menu: release frame must not act");
    check(ev.action == logic::menu::Action::StartCalibration && ms.screen == logic::menu::Screen::Closed,
          "menu: Calibration closes the menu and requests the wizard");
}

void dual_rate_test() {
    namespace mn = logic::menu;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    st.radio.ext_switches = 0;
    st.radio.adc_modes = {3, 3, 3, 3, 0, 0, 0, 0, 0, 0};  // SixPos on SA..SD: not usable for dual rate
    uint8_t two_pos_code = 0;
    for (unsigned v = 0; v < 256; ++v) {
        if (logic::input::resolve_mode(0, static_cast<uint8_t>(v)) == logic::input::AdcMode::TwoPos) {
            two_pos_code = static_cast<uint8_t>(v);
            break;
        }
    }
    check(!mn::is_dr_switch_available(1, st.radio), "dr: SA must be unavailable on a default input");
    check(mn::is_dr_switch_available(0, st.radio), "dr: switch 0 is always available");
    st.radio.adc_modes[0] = two_pos_code;
    check(mn::is_dr_switch_available(1, st.radio), "dr: SA must be available as a two-position input");
    check(!mn::is_dr_switch_available(5, st.radio), "dr: SE needs the external switch board");
    st.radio.ext_switches = 1;
    check(mn::is_dr_switch_available(6, st.radio), "dr: SF available with the external board");
    check(mn::step_dr_switch(st.radio, 0, true) == 6, "dr: previous from 0 wraps to SF");
    st.radio.ext_switches = 0;
    check(mn::step_dr_switch(st.radio, 0, false) == 1, "dr: next from 0 skips unavailable SE/SF");
    check(mn::step_dr_switch(st.radio, 0, true) == 1, "dr: previous from 0 skips unavailable SE/SF");

    mn::State ms{};
    mn::open(ms);
    ms.main.selected = mn::kItemDualRate;
    mn::update(ms, mn::Input{0, 0, 0}, st);
    auto tap = [&](uint16_t key, uint32_t now) {
        const Event e = mn::update(ms, mn::Input{key, key, now}, st);
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        return e;
    };

    tap(mn::kKeyOk, 10);
    check(ms.screen == Screen::DualRate, "dr: Dual Rate main item must open the screen");
    tap(mn::kKeyOk, 20);
    check(ms.editing, "dr: OK must start editing");
    tap(mn::kKeyOk, 30);
    check(!ms.editing, "dr: OK must leave edit mode");
    tap(mn::kKeyDown, 40);
    tap(mn::kKeyDown, 50);
    tap(mn::kKeyOk, 60);
    tap(mn::kKeyUp, 70);
    check(st.active_model.dr_high[0] == 100, "dr: Hi Rate must clamp at 100");
    tap(mn::kKeyDown, 80);
    check(st.active_model.dr_high[0] == 95, "dr: Down must step Hi Rate by 5");
    tap(mn::kKeyOk, 90);
    check(!ms.editing && ms.dual_rate.selected == 2, "dr: Hi Rate row stays selected after editing");

    tap(mn::kKeyUp, 100);
    tap(mn::kKeyOk, 110);
    tap(mn::kKeyDown, 120);
    check(ms.axis == 1, "dr: axis row must toggle to Pitch");
    tap(mn::kKeyOk, 130);
    tap(mn::kKeyDown, 140);
    tap(mn::kKeyOk, 150);
    tap(mn::kKeyDown, 160);
    tap(mn::kKeyDown, 170);
    check(st.active_model.dr_high[1] == 90 && st.active_model.dr_high[0] == 95,
          "dr: edits must apply to the selected axis only");
    tap(mn::kKeyOk, 180);
    tap(mn::kKeyDown, 190);
    tap(mn::kKeyDown, 200);
    check(ms.dual_rate.selected == 4, "dr: navigation must reach Hi Expo");

    tap(mn::kKeyOk, 210);
    for (uint32_t t = 220; t < 500; t += 10) {
        tap(mn::kKeyDown, t);
    }
    check(st.active_model.expo_high[1] == -100, "dr: Hi Expo must clamp at -100");

    const Event cancel_edit = tap(mn::kKeyCancel, 510);
    check(cancel_edit.action == Action::None && !ms.editing && ms.screen == Screen::DualRate,
          "dr: cancel while editing must only leave edit mode");
    const Event saved = tap(mn::kKeyCancel, 520);
    check(saved.action == Action::SaveActiveModel && ms.screen == Screen::Main, "dr: cancel must save and return");

    logic::display::Framebuffer fb{};
    fb.clear();
    mn::ModelNames names{};
    mn::State view{};
    mn::open(view);
    view.screen = Screen::DualRate;
    mn::render(fb, view, names, st, mn::Live{});
    check(lit_pixels(fb) > 0, "dr: dual rate screen must draw");
}

void throttle_curve_test() {
    namespace cv = logic::curve;
    namespace mn = logic::menu;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    const cv::Points identity{0, 25, 50, 75, 100};
    check(cv::evaluate(0, 5, false, identity) == 0, "curve: 5pt input 0 must give 0");
    check(cv::evaluate(125, 5, false, identity) == 125, "curve: 5pt linear midpoint of first segment");
    check(cv::evaluate(250, 5, false, identity) == 250, "curve: 5pt knot must be exact");
    check(cv::evaluate(1000, 5, false, identity) == 1000, "curve: 5pt input 1000 must give 1000");
    check(cv::evaluate(1500, 5, false, identity) == 1000, "curve: input above 1000 must clamp");
    check(cv::evaluate(250, 5, true, identity) == 250, "curve: spline must pass through the knots");
    check(cv::evaluate(750, 5, true, identity) == 750, "curve: spline knot at 750");

    cv::Points flat{};
    flat.fill(50);
    check(cv::evaluate(333, 5, false, flat) == 500 && cv::evaluate(333, 5, true, flat) == 500,
          "curve: a flat curve must stay flat");

    const cv::Points nine{0, 13, 25, 38, 50, 63, 75, 88, 100};
    check(cv::evaluate(500, 9, false, nine) == 500, "curve: 9pt knot must be exact");
    check(cv::evaluate(1000, 9, true, nine) == 1000, "curve: 9pt spline end must be 1000");

    const cv::Points seven{0, 17, 33, 50, 67, 83, 100};
    check(cv::evaluate(333, 7, false, seven) == 330, "curve: 7pt knot at one third");
    check(cv::evaluate(500, 7, true, seven) == 500, "curve: 7pt spline knot at one half");

    cv::Points c = identity;
    uint8_t pts = 5;
    cv::advance_mode(c, pts);
    check(pts == 7 && c[0] == 0 && c[3] == 50 && c[6] == 100, "curve: 5 -> 7 must resample evenly");
    cv::advance_mode(c, pts);
    check(pts == 9 && c[4] == 50 && c[8] == 100, "curve: 7 -> 9 must resample evenly");
    cv::advance_mode(c, pts);
    check(pts == 5 && c[1] == 25 && c[4] == 100, "curve: 9 -> 5 must keep every second point");
    check(cv::to_percent(5) == 1 && cv::to_percent(4) == 0 && cv::to_percent(1000) == 100,
          "curve: to_percent must round half up and clamp");

    check(mn::curve_point_label(0, 50).view() == "P01:  50%", "curve: label must right-align the value");
    check(mn::curve_point_label(1, 5).view() == "P02:   5%", "curve: label of one digit");
    check(mn::curve_point_label(8, 100).view() == "P09: 100%", "curve: label of three digits");

    logic::storage::RadioStorage st = logic::storage::default_storage();
    mn::State ms{};
    mn::open(ms);
    ms.main.selected = mn::kItemThrottleCurve;
    mn::update(ms, mn::Input{0, 0, 0}, st);
    auto tap = [&](uint16_t key, uint32_t now) {
        const Event e = mn::update(ms, mn::Input{key, key, now}, st);
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        return e;
    };

    tap(mn::kKeyOk, 10);
    check(ms.screen == Screen::ThrottleCurve, "curve: Throttle Curve main item must open the screen");
    tap(mn::kKeyOk, 20);
    check(st.active_model.thr_curve_pts == 7, "curve: OK on Pts must switch 5 -> 7");
    tap(mn::kKeyDown, 30);
    tap(mn::kKeyOk, 40);
    check(st.active_model.thr_curve_smooth == 1, "curve: OK on Crv must toggle smooth");
    tap(mn::kKeyDown, 50);
    tap(mn::kKeyOk, 60);
    check(ms.editing && ms.throttle_curve.selected == 2, "curve: OK on a point must start editing P1");
    for (uint32_t t = 70; t < 300; t += 10) {
        tap(mn::kKeyDown, t);
    }
    check(st.active_model.thr_curve[0] == 0, "curve: Down must clamp the point at 0");
    tap(mn::kKeyUp, 310);
    check(st.active_model.thr_curve[0] == 1, "curve: Up must step the point by 1");
    for (int i = 0; i < 7; ++i) {
        tap(mn::kKeyOk, 320 + static_cast<uint32_t>(i) * 10);
    }
    check(!ms.editing && ms.throttle_curve.selected == 8, "curve: OK on the last point must leave edit mode");
    tap(mn::kKeyUp, 500);
    check(ms.throttle_curve.selected == 7, "curve: navigation must move between points");
    const Event saved = tap(mn::kKeyCancel, 510);
    check(saved.action == Action::SaveActiveModel && ms.screen == Screen::Main,
          "curve: cancel must save and return to main");

    logic::display::Framebuffer fb{};
    fb.clear();
    mn::ModelNames names{};
    mn::State view{};
    mn::open(view);
    view.screen = Screen::ThrottleCurve;
    view.throttle_curve.selected = 2;
    mn::render(fb, view, names, st, mn::Live{});
    check(lit_pixels(fb) > 0, "curve: throttle curve screen must draw");
}

void mixer_menu_test() {
    namespace mn = logic::menu;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    st.active_model.mixes[0].source = 4;  // Thr: the first mix line starts on a usable source
    st.radio.adc_modes = {3, 0, 0, 0, 3, 3, 0, 0, 0, 0};  // SA and VRA/VRB unusable: stepping must skip them
    mn::State ms{};
    mn::open(ms);
    uint32_t now = 0;
    auto tap = [&](uint16_t key) {
        now += 10;
        const Event e = mn::update(ms, mn::Input{key, key, now}, st);
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        now += 1;
        return e;
    };

    ms.main.selected = mn::kItemWingMixer;
    mn::update(ms, mn::Input{0, 0, 0}, st);
    tap(mn::kKeyOk);
    check(ms.screen == Screen::WingMixer, "mix: Wing & Mixer main item must open the screen");

    const Event tmpl = tap(mn::kKeyOk);
    check(tmpl.action == Action::SaveActiveModel && st.active_model.wing_tail_mix == 1,
          "mix: OK on the template row must cycle the template and save");
    check(mn::wing_gvars_row(st.active_model) == 2 && mn::wing_row_count(st.active_model) == 35,
          "mix: Diff template must add a row before GVars");
    tap(mn::kKeyDown);
    tap(mn::kKeyOk);
    check(ms.editing, "mix: Diff row must start editing");
    for (int i = 0; i < 25; ++i) {
        tap(mn::kKeyDown);
    }
    check(st.active_model.template_diff == -100, "mix: Diff must clamp at -100");
    tap(mn::kKeyOk);
    check(!ms.editing, "mix: OK must leave Diff edit mode");

    tap(mn::kKeyDown);
    check(ms.wing.selected == 2, "mix: GVars row must follow the Diff row");
    tap(mn::kKeyOk);
    check(ms.screen == Screen::GlobalVars, "mix: GVars row must open the screen");
    tap(mn::kKeyOk);
    for (int i = 0; i < 25; ++i) {
        tap(mn::kKeyUp);
    }
    check(st.active_model.gvars[0] == 1000, "mix: GV1 must clamp at 1000");
    const Event gv_done = tap(mn::kKeyCancel);
    check(gv_done.action == Action::SaveActiveModel && !ms.editing && ms.screen == Screen::GlobalVars,
          "mix: cancel while editing a GV must save and only leave edit mode");
    const Event gv_back = tap(mn::kKeyCancel);
    check(gv_back.action == Action::SaveActiveModel && ms.screen == Screen::Main,
          "mix: GVars cancel must save and return to Main");

    ms.main.selected = mn::kItemWingMixer;
    tap(mn::kKeyOk);
    check(ms.screen == Screen::WingMixer, "mix: Wing must reopen from Main");
    for (int i = 0; i < 3; ++i) {
        tap(mn::kKeyDown);
    }
    check(ms.wing.selected == 3, "mix: row 3 must be the first mix line");
    tap(mn::kKeyOk);
    check(ms.screen == Screen::MixerLine && ms.mix_idx == 0, "mix: OK on a mix row must open the line editor");

    tap(mn::kKeyDown);  // Source
    tap(mn::kKeyOk);
    tap(mn::kKeyUp);
    check(st.active_model.mixes[0].source == 8, "mix: source step must skip unusable pots and switches");
    tap(mn::kKeyOk);

    const Event back = tap(mn::kKeyCancel);
    check(back.action == Action::None && ms.screen == Screen::WingMixer && ms.wing.selected == 3,
          "mix: cancel must return to the mix row it came from without saving");

    tap(mn::kKeyOk);
    tap(mn::kKeyOk);
    tap(mn::kKeyDown);
    check(st.active_model.mixes[0].target_ch == 18, "mix: target must wrap from 0 to 18");
    tap(mn::kKeyOk);
    for (int i = 0; i < 5; ++i) {
        tap(mn::kKeyDown);
    }
    st.radio.adc_modes.fill(3);  // no switch input usable: the switch condition must stay
    tap(mn::kKeyOk);
    tap(mn::kKeyUp);
    check(st.active_model.mixes[0].switch_sel == 0, "mix: no switch condition is available, so the value stays");
    tap(mn::kKeyOk);
    tap(mn::kKeyDown);
    tap(mn::kKeyOk);
    tap(mn::kKeyDown);
    check(st.active_model.mixes[0].mode == 2, "mix: mode must wrap from 0 to 2");
    tap(mn::kKeyOk);

    tap(mn::kKeyCancel);
    tap(mn::kKeyCancel);
    check(ms.screen == Screen::Main, "mix: Wing cancel must return to Main");

    logic::display::Framebuffer fb{};
    mn::ModelNames names{};
    for (const Screen sc : {Screen::WingMixer, Screen::MixerLine, Screen::GlobalVars}) {
        fb.clear();
        mn::State view{};
        mn::open(view);
        view.screen = sc;
        mn::render(fb, view, names, st, mn::Live{});
        check(lit_pixels(fb) > 0, "mix: mixer screens must draw");
    }
}

void aux_menu_test() {
    namespace mn = logic::menu;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    st.radio.adc_modes = {3, 0, 0, 0, 3, 3, 0, 0, 0, 0};  // SA, VRA, VRB unusable: stepping must skip them
    st.active_model.aux_channels[0] = 4;                   // Yaw
    st.active_model.aux_channels[1] = 8;                   // SB
    mn::State ms{};
    mn::open(ms);
    uint32_t now = 0;
    auto tap = [&](uint16_t key) {
        now += 10;
        const Event e = mn::update(ms, mn::Input{key, key, now}, st);
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        now += 1;
        return e;
    };

    ms.main.selected = mn::kItemAuxChannels;
    mn::update(ms, mn::Input{0, 0, 0}, st);
    tap(mn::kKeyOk);
    check(ms.screen == Screen::AuxChannels, "aux: Aux Channels main item must open the screen");

    tap(mn::kKeyOk);
    check(ms.editing, "aux: OK on CH5 must start editing");
    tap(mn::kKeyUp);
    check(st.active_model.aux_channels[0] == 8, "aux: source step up must skip unusable VRA, VRB and SA");
    tap(mn::kKeyOk);
    check(!ms.editing, "aux: OK must leave edit mode");

    tap(mn::kKeyDown);
    check(ms.aux.selected == 1, "aux: Down must select CH6");
    tap(mn::kKeyOk);
    tap(mn::kKeyDown);
    check(st.active_model.aux_channels[1] == 4, "aux: source step down must skip unusable SA, VRB and VRA");
    const Event cancel_edit = tap(mn::kKeyCancel);
    check(cancel_edit.action == Action::None && !ms.editing && ms.screen == Screen::AuxChannels,
          "aux: cancel while editing must only leave edit mode");

    const Event back = tap(mn::kKeyCancel);
    check(back.action == Action::SaveActiveModel && ms.screen == Screen::Main,
          "aux: cancel from the list must save and return to Main");
}

void radio_setup_test() {
    namespace mn = logic::menu;
    using mn::Action;
    using mn::Event;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    st.radio.ext_adc = 0;
    mn::State ms{};
    mn::open(ms);
    uint32_t now = 0;
    auto tap = [&](uint16_t key) {
        now += 10;
        const Event e = mn::update(ms, mn::Input{key, key, now}, st);
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        now += 1;
        return e;
    };

    ms.main.selected = mn::kItemRadioSetup;
    mn::update(ms, mn::Input{0, 0, 0}, st);
    tap(mn::kKeyOk);
    check(ms.screen == Screen::RadioSetup, "radio: Radio Setup main item must open the screen");

    const Event thr = tap(mn::kKeyOk);
    check(thr.action == Action::SaveRadioConfig && st.radio.throttle_trim == 1,
          "radio: throttle trim must cycle and request a save");

    ms.radio.selected = 4;
    tap(mn::kKeyOk);
    check(st.radio.backlight_brightness == 1, "radio: brightness must wrap from 10 to 1");

    ms.radio.selected = 5;
    tap(mn::kKeyOk);
    check(st.radio.lcd_contrast == 40, "radio: contrast must step by 3");
    st.radio.lcd_contrast = 50;
    tap(mn::kKeyOk);
    check(st.radio.lcd_contrast == 20, "radio: contrast must wrap from 50 to 20");

    ms.radio.selected = 6;
    tap(mn::kKeyOk);
    check(st.radio.vbat_warn_deci == 45, "radio: battery warning must step by 0.1 V");

    ms.radio.selected = 3;
    tap(mn::kKeyOk);
    check(st.radio.backlight_timeout == 1, "radio: backlight timeout must cycle");

    ms.radio.selected = 8;
    tap(mn::kKeyOk);
    check(st.radio.ext_module_pwr == 1, "radio: PC13 power must toggle");

    ms.radio.selected = 14;
    tap(mn::kKeyOk);
    check(st.radio.trainer_config == mn::kTrainerPpmOut, "radio: PPM out must set its bit");
    ms.radio.selected = 15;
    tap(mn::kKeyOk);
    check(st.radio.trainer_config == (mn::kTrainerPpmOut | mn::kTrainerPpmIn), "radio: trainer in must set its bit");
    ms.radio.selected = 14;
    tap(mn::kKeyOk);
    check(st.radio.trainer_config == mn::kTrainerPpmIn, "radio: PPM out must clear only its bit");

    ms.radio.selected = mn::kRadioInputRow;
    tap(mn::kKeyOk);
    check(ms.screen == Screen::InputSetup && ms.input_setup.selected == 0,
          "radio: Input Setup row must open the Input Modes screen");

    const Event mode = tap(mn::kKeyOk);
    check(mode.action == Action::SaveRadioConfig && st.radio.adc_modes[0] == 1,
          "radio: input mode must cycle and request a save");
    st.radio.adc_modes[0] = 6;
    tap(mn::kKeyOk);
    check(st.radio.adc_modes[0] == 0, "radio: input mode must wrap after INST-TRIM");

    tap(mn::kKeyCancel);
    check(ms.screen == Screen::RadioSetup && ms.radio.selected == mn::kRadioInputRow && ms.radio.scroll == 10,
          "radio: cancel in Input Modes must return to row 13");
    const Event back = tap(mn::kKeyCancel);
    check(back.action == Action::None && ms.screen == Screen::Main, "radio: cancel must return to the main menu");
}

void afhds2a_test() {
    namespace af = logic::afhds2a;

    std::array<uint16_t, af::kNumChannels> ch{1500, 988, 2012, 1123, 1456, 1789, 1357, 1642, 1890,
                                              1050, 1555, 1999, 1001, 1620, 1234, 1987, 1432, 1876};
    const af::Packet sticks = af::build_sticks(0x12345678, 0x9ABCDEF0, ch);
    check(sticks[0] == af::kPacketSticks, "afhds2a: sticks command byte");
    check(sticks[37] == 0x00, "afhds2a: sticks trailer byte");

    // Decode as iBUS does (Betaflight rx/ibus.c): 12-bit slots plus extended nibbles.
    std::array<uint16_t, af::kNumChannels> dec{};
    for (std::size_t i = 0; i < 14; ++i) {
        dec[i] = static_cast<uint16_t>(sticks[9 + i * 2] | ((sticks[10 + i * 2] & 0x0F) << 8));
    }
    for (std::size_t ext = 0; ext < 4; ++ext) {
        const std::size_t off = (ext * 3) * 2 + 1 + 9;
        dec[14 + ext] = static_cast<uint16_t>(((sticks[off] & 0xF0) >> 4) | (sticks[off + 2] & 0xF0) |
                                              ((sticks[off + 4] & 0xF0) << 4));
    }
    bool sticks_ok = true;
    for (std::size_t i = 0; i < ch.size(); ++i) {
        sticks_ok = sticks_ok && dec[i] == ch[i];
    }
    check(sticks_ok, "afhds2a: 18-channel sticks round trip");

    // Legacy 14-channel decoder masks the low 12 bits of each slot.
    std::array<uint16_t, af::kNumChannels> ch14{1500, 1000, 2000, 1200, 1400, 1600, 1800, 1100, 1300, 1500, 1700, 1900, 1050, 1950};
    const af::Packet legacy = af::build_sticks(0x11223344, 0x55667788, [&] {
        auto a = ch;
        for (std::size_t i = 0; i < 14; ++i) a[i] = ch14[i];
        return a;
    }());
    bool legacy_ok = true;
    for (std::size_t i = 0; i < 14; ++i) {
        const auto raw16 = static_cast<uint16_t>(legacy[9 + i * 2] | (legacy[10 + i * 2] << 8));
        legacy_ok = legacy_ok && (raw16 & 0x0FFF) == ch14[i];
    }
    check(legacy_ok, "afhds2a: legacy 14-channel decoding");

    const af::Packet fs = af::build_failsafe(0xAABBCCDD, 0);
    check(fs[0] == af::kPacketFailsafe, "afhds2a: failsafe command byte");
    check((fs[9 + 2 * 2] | (fs[10 + 2 * 2] << 8)) == af::kFailsafeThrottleUs, "afhds2a: failsafe throttle is 988us");
    bool hold_ok = true;
    for (std::size_t c = 0; c < 14; ++c) {
        if (c != 2) hold_ok = hold_ok && fs[9 + c * 2] == 0xFF && fs[10 + c * 2] == 0xFF;
    }
    check(hold_ok, "afhds2a: failsafe holds non-throttle channels");

    af::Link link{0x11223344};
    link.rx_id = 0x55667788;
    af::Packet st = af::build_settings(link.tx_id, link.rx_id, link.settings);
    check(st[0] == af::kPacketSettings && st[9] == 0xFD && st[10] == 0xFF, "afhds2a: settings header");
    check(st[11] == 0x32 && st[12] == 0x00 && st[13] == 0x00, "afhds2a: default 50 Hz PWM");
    check(st[18] == 0x05 && st[19] == 0xDC && st[21] == 0xDE, "afhds2a: center 1500us and iBUS default");
    link.set_rx_settings(400, true, true);
    st = af::build_settings(link.tx_id, link.rx_id, link.settings);
    check(st[11] == 0x90 && st[12] == 0x01 && st[13] == 0x01 && st[21] == 0xDD, "afhds2a: 400 Hz PPM SBUS");
    check(link.next == af::PacketKind::Settings, "afhds2a: settings change queues a settings packet");
    link.set_rx_settings(30, false, false);
    check(link.settings.rate_hz == 50, "afhds2a: rate clamped to 50 Hz");
    link.set_rx_settings(500, false, false);
    check(link.settings.rate_hz == 400, "afhds2a: rate clamped to 400 Hz");

    link.set_rf_power(4);
    check(link.rf_power_idx == 0, "afhds2a: out-of-range RF power falls back to index 0");
    link.set_rf_power(3);
    check(link.rf_power_idx == 3, "afhds2a: RF power index 3 kept");

    check(af::rssi_from_raw(0) == 100 && af::rssi_from_raw(200) == 0, "afhds2a: RSSI clamping");
    check(af::rssi_from_raw(100) == 96, "afhds2a: RSSI formula 256 - raw*8/5");

    // Hopping table: all in band and separated by at least 5 channels.
    const af::HoppingTable hop = af::hopping_table(0x12345678);
    bool hop_ok = true;
    for (std::size_t i = 0; i < hop.size(); ++i) {
        hop_ok = hop_ok && hop[i] >= 1 && hop[i] <= 164;
        for (std::size_t j = 0; j < i; ++j) {
            const int d = int(hop[i]) - int(hop[j]);
            hop_ok = hop_ok && (d < 0 ? -d : d) >= 5;
        }
    }
    check(hop_ok, "afhds2a: hopping table in band with channel separation");

    // Bind confirmation carries the RX ID and the state machine accepts the reply.
    af::Link bl{0x12345678};
    bl.binding = true;
    af::Frame reply{};
    reply[0] = af::kPacketBind2;
    reply[5] = 0x78;
    reply[6] = 0x56;
    reply[7] = 0x34;
    reply[8] = 0x12;
    reply[9] = 0x01;
    check(bl.on_bind_reply(reply) && bl.rx_id == 0x12345678 && bl.bind_step == af::BindStep::Confirm,
          "afhds2a: bind reply moves to confirmation with RX ID");
    const af::Packet confirm = af::build_bind(bl.tx_id, bl.rx_id, af::BindStep::Confirm, hop);
    check(confirm[0] == af::kPacketBind2 && confirm[9] == 0x02 && confirm[5] == 0x78,
          "afhds2a: bind confirmation packet");

    // Downlink handling: TX ID filter, sensor decoding, requests.
    af::Link tl{0x12345678};
    tl.rx_id = 0x9ABCDEF0;
    af::Frame rx{};
    rx[0] = 0xAA;
    rx[1] = 0x78;
    rx[2] = 0x56;
    rx[3] = 0x34;
    rx[4] = 0x12;
    rx[5] = 0xF0;
    rx[6] = 0xDE;
    rx[7] = 0xBC;
    rx[8] = 0x9A;
    rx[10] = 0x09;
    rx[11] = 0x01;
    rx[12] = 25;
    rx[14] = 0x41;
    rx[15] = 0x02;
    rx[16] = 185;
    check(tl.on_downlink(rx, 0, 0) == af::RxEvent::Data && tl.telemetry.connected, "afhds2a: downlink accepted");
    check(tl.telemetry.vspeed_cms == 250 && tl.telemetry.has_vspeed, "afhds2a: vario 2.5 m/s");
    check(tl.telemetry.alt_m == 185 && tl.telemetry.alt_dm == 1850, "afhds2a: altitude 185 m");

    rx[1] = 0x00;
    check(tl.on_downlink(rx, 0, 0) == af::RxEvent::Ignored, "afhds2a: foreign TX ID ignored");
    rx[1] = 0x78;

    af::Link tt{0x12345678};
    tt.rx_id = 0x9ABCDEF0;
    af::Frame rx2 = rx;
    rx2[10] = 0x01;
    rx2[11] = 0x01;
    rx2[12] = 0xEE;
    rx2[13] = 0x02;
    rx2[14] = 0x02;
    rx2[15] = 0x02;
    rx2[16] = 0x9A;
    rx2[17] = 0x10;
    tt.on_downlink(rx2, 0, 7500);
    check(tt.telemetry.temp_c_x10 == 350 && tt.telemetry.last_temp_ms == 7500, "afhds2a: temperature 35.0 C");
    check(tt.telemetry.rpm == 4250 && tt.telemetry.last_rpm_ms == 7500, "afhds2a: RPM 4250");

    af::Frame req = rx;
    req[9] = 0xFD;
    check(tt.on_downlink(req, 0, 0) == af::RxEvent::RequestFailsafe && tt.next == af::PacketKind::Failsafe,
          "afhds2a: failsafe request queues failsafe packet");
}

int main() {
    ring_test();
    crsf_framer_test();
    crsf_menu_test();
    storage_test();
    mixer_test();
    input_test();
    safety_test();
    display_test();
    dashboard_test();
    menu_test();
    calib_test();
    dual_rate_test();
    throttle_curve_test();
    mixer_menu_test();
    aux_menu_test();
    radio_setup_test();
    afhds2a_test();
    if (failures == 0) {
        std::puts("all host logic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
