#include <array>
#include <cstdint>
#include <cstdio>
#include <span>

#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/crsf.hpp"
#include "ferrox/logic/crsf_menu.hpp"
#include "ferrox/logic/mixer.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/trim.hpp"
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

}  // namespace

int main() {
    ring_test();
    crsf_framer_test();
    crsf_menu_test();
    storage_test();
    mixer_test();
    if (failures == 0) {
        std::puts("all host logic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
