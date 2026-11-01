#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "ferrox/logic/a7105.hpp"
#include "ferrox/logic/usb.hpp"
#include "ferrox/logic/bind.hpp"
#include "ferrox/logic/afhds2a.hpp"
#include "ferrox/logic/afhds2a_radio.hpp"
#include "ferrox/logic/buzzer_inputs.hpp"
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
#include "ferrox/logic/model_copy.hpp"
#include "ferrox/logic/protocol.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/logic/safety.hpp"
#include "ferrox/logic/storage.hpp"
#include "ferrox/logic/switches.hpp"
#include "ferrox/logic/telemetry.hpp"
#include "ferrox/logic/trim.hpp"
#include "ferrox/logic/trainer.hpp"
#include "ferrox/logic/vario.hpp"
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
    const std::size_t len = crsf::build_rc_frame(channels, {false}, {false}, frame);
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
    namespace crsf = logic::crsf;

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

    static_assert(menu::next_param_for_folder(crsf::ParamId{0}, menu::FolderId{0}, menu::ParamCount{3}, [] {
        std::array<uint8_t, menu::kMaxParamMap> m{};
        m.fill(menu::kNoParent);
        m[1] = 0;
        m[2] = 7;
        m[3] = 0;
        return m;
    }(), true) == 1);

    menu::Loader loader;
    const auto a0 = loader.start(crsf::DeviceAddr{0xEE}, menu::ParamCount{2}, 0);
    check(a0.kind == menu::Action::Kind::Read && a0.param == 0, "loader must start with param 0");

    // Param 0 root folder with child 1, sent as a single frame.
    const std::array<uint8_t, 6> root{0xFF, 0x0B, 'R', 0x00, 0x01, 0xFF};
    auto step = loader.on_entry(crsf::DeviceAddr{0xEE}, crsf::ParamId{0}, menu::ChunksRemain{0}, root, 10);
    check(step.kind == menu::Action::Kind::Read && step.param == 1, "root folder must read child param 1");

    // Multi-chunk param 1: first chunk requests the rest, second completes it.
    const std::array<uint8_t, 3> part1{0x00, 0x00, 'T'};
    step = loader.on_entry(crsf::DeviceAddr{0xEE}, crsf::ParamId{1}, menu::ChunksRemain{1}, part1, 20);
    check(step.kind == menu::Action::Kind::Read && step.arg == 1, "first chunk must request chunk 1");
    const std::array<uint8_t, 2> part2{0x00, 0x00};
    step = loader.on_entry(crsf::DeviceAddr{0xEE}, crsf::ParamId{1}, menu::ChunksRemain{0}, part2, 30);
    check(step.kind == menu::Action::Kind::Done && loader.state == menu::Loader::State::Ready,
          "last param in root must finish loading");
    check(loader.entries[1] && loader.entries[1]->name.len == 1, "param 1 must be stored");

    // Retry path on a fresh load with no reply: retries, then give up on param 1.
    menu::Loader retry;
    retry.start(crsf::DeviceAddr{0xEE}, menu::ParamCount{1}, 0);
    retry.on_entry(crsf::DeviceAddr{0xEE}, crsf::ParamId{0}, menu::ChunksRemain{0}, root, 10);
    step = retry.tick(10 + 500);
    check(step.kind == menu::Action::Kind::Read && step.param == 1, "timeout must retry param 1");
    for (int i = 0; i < 10 && step.kind != menu::Action::Kind::Done; ++i) {
        step = retry.tick(retry.next_req_ms);
    }
    check(step.kind == menu::Action::Kind::Done && retry.state == menu::Loader::State::Ready,
          "retries exhausted must finish loading");

    menu::Command cmd;
    cmd.start(2, 0);
    cmd.on_status(crsf::ParamId{2}, menu::CmdStatus::ConfirmationNeeded, menu::TimeoutByte{0}, 10);
    check(cmd.kind == menu::Command::Kind::WaitingConfirm, "confirmation-needed must wait for confirm");
    const auto confirm = cmd.confirm(20);
    check(confirm.kind == menu::Action::Kind::Write && confirm.arg == 4, "confirm must write 4");
    cmd.on_status(crsf::ParamId{2}, menu::CmdStatus::Progress, menu::TimeoutByte{5}, 30);
    check(cmd.kind == menu::Command::Kind::Running && cmd.poll_interval_ms == 500, "progress runs with 500 ms poll");
    const auto poll = cmd.tick(30 + 500);
    check(poll.kind == menu::Action::Kind::Write && poll.arg == 6, "running command must poll");
    cmd.on_status(crsf::ParamId{2}, menu::CmdStatus::Ready, menu::TimeoutByte{0}, 600);
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
static_assert(logic::sysinfo::firmware_line(logic::sysinfo::Build{{"0.27.0"}, {"unknown"}}).view() ==
              "v0.27.0 (unknown)");
static_assert(logic::sysinfo::storage_line(logic::storage::EepromStatus{}).view() == "Storage:  None (No EEPROM)");

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
        return mx::compute_channels({roll, pitch, thr, yaw}, pots, sw, m, no_trim, 0);
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
    mx::TimerLatch latched{};
    check(!mx::is_timer_active(0, 500, latched, sw_default, {true}), "timer off");
    check(!mx::is_timer_active(1, -950, latched, sw_default, {true}) &&
              mx::is_timer_active(1, -850, latched, sw_default, {true}),
          "THs threshold");
    latched.value = false;
    check(!mx::is_timer_active(2, -1000, latched, sw_default, {true}), "THt idle not latched");
    check(mx::is_timer_active(2, -500, latched, sw_default, {true}), "THt latches on throttle");
    check(mx::is_timer_active(2, -1000, latched, sw_default, {true}), "THt keeps running at idle");
    check(mx::is_timer_active(3, -1000, latched, sw_default, {true}), "timer always on");
    check(!mx::is_timer_active(3, 0, latched, sw_default, {false}), "timer inhibited when disarmed");
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
    check(detent.normalize_detent({2048}, {in::kDetentDeadband}) == 0, "detent center is zero");
    check(detent.normalize_detent({2048 - 15}, {in::kDetentDeadband}) == 0, "detent low edge is zero");
    check(detent.normalize_detent({2048 + 15}, {in::kDetentDeadband}) == 0, "detent high edge is zero");
    check(detent.normalize_detent({1000}, {in::kDetentDeadband}) == -1000, "detent min is -1000");
    check(detent.normalize_detent({3000}, {in::kDetentDeadband}) == 1000, "detent max is +1000");

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
    fb.set_pixel({5, 9}, true);
    check(fb.data()[1 * 128 + 5] == 0x02 && lit_pixels(fb) == 1, "set_pixel page/bit placement");
    fb.set_pixel({-1, 0}, true);
    fb.set_pixel({128, 0}, true);
    fb.set_pixel({0, 64}, true);
    check(lit_pixels(fb) == 1, "set_pixel ignores out-of-bounds");

    fb.clear();
    fb.fill_rect({{-5, -5}, {10, 10}}, true);
    check(lit_pixels(fb) == 25, "fill_rect clips to the screen");

    fb.clear();
    fb.draw_rect({{0, 0}, {10, 6}}, true);
    check(lit_pixels(fb) == 28, "draw_rect outline pixel count");

    fb.clear();
    fb.draw_char_4x6({0, 0}, 'A', false);
    check(lit_pixels(fb) == 10 && (fb.data()[0] & 0x02) != 0, "4x6 'A' glyph");

    fb.fill_rect({{0, 0}, {4, 6}}, true);
    fb.draw_char_4x6({0, 0}, 'A', true);
    check(lit_pixels(fb) == 14, "inverted glyph clears lit pixels only");

    fb.clear();
    fb.draw_char_4x6({0, 0}, '\x7f', false);
    fb.draw_char_6x10({0, 0}, '\t', false);
    check(lit_pixels(fb) == 0, "non-printable characters are skipped");

    fb.clear();
    fb.draw_char_6x10({0, 0}, 'I', false);
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
    widgets::draw_list_row(fb, {1}, true, "A", "", {0});
    check((fb.data()[2 * 128 + 2] & 0x80) != 0 && (fb.data()[3 * 128 + 125] & 0x80) != 0 &&
              (fb.data()[2 * 128 + 126] & 0x80) == 0 && (fb.data()[4 * 128 + 2] & 0x01) == 0,
          "selected list row highlight 2..125 x 23..31");

    fb.clear();
    widgets::draw_scrollbar(fb, {0}, {4}, {14, 40});
    check((fb.data()[1 * 128 + 126] & 0x40) != 0 && (fb.data()[1 * 128 + 125] & 0x40) != 0 &&
              (fb.data()[5 * 128 + 125] & 0x10) == 0,
          "scrollbar track and thumb at top");

    fb.clear();
    widgets::draw_progress_bar(fb, {{0, 0}, {102, 8}}, {0}, {0});
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
            const auto m = logic::dashboard::adaptive_metrics({sw}, {pots});
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

    const auto ten_sw_two_pots = logic::dashboard::adaptive_metrics({10}, {2});
    check(ten_sw_two_pots.compact && ten_sw_two_pots.spacing == 8 && ten_sw_two_pots.pot_start_x == 85,
          "10 switches + 2 pots layout matches Rust math");

    // Smoke test: every page renders into the framebuffer without touching out-of-range memory.
    logic::input::InputState in{};
    in.battery_mv = 7400;
    logic::storage::RadioStorage st{};
    logic::trim::Controller trims{};
    std::array<uint16_t, logic::kNumChannels> chs{};
    logic::dashboard::View view{in, st, trims, chs, true, {}, false, false, false, 0};

    {
        logic::afhds2a::Telemetry linked{};
        linked.connected = true;
        linked.rssi = 87;
        linked.last_rx_voltage_ms = 1000;
        linked.rx_voltage_mv = 4950;
        using logic::dashboard::View;
        const View bind_view{in, st, trims, chs, false, {}, true, true, false, 1000};
        const View link_view{in, st, trims, chs, false, linked, false, true, true, 1500};
        const View stale_view{in, st, trims, chs, false, linked, false, true, true, 9000};
        const View down_view{in, st, trims, chs, false, {}, false, false, false, 0};
        const View crsf_view{in, st, trims, chs, true, {}, false, false, false, 0};
        check(logic::dashboard::rf_status_fixed(bind_view) == "BIND", "dashboard shows BIND while binding");
        check(logic::dashboard::rf_status_fixed(link_view).empty(), "dashboard shows RSSI when linked");
        check(logic::dashboard::rf_status_fixed(down_view) == "NO RF", "dashboard shows NO RF without radio");
        check(logic::dashboard::rf_status_fixed(crsf_view) == "CRSF", "dashboard shows CRSF for CRSF models");
        check(logic::dashboard::afhds_link_line(link_view) == "AFHDS2A: CONNECTED", "model page shows link state");
        check(logic::dashboard::sensor_fresh(link_view, linked.last_rx_voltage_ms), "fresh telemetry is not stale");
        check(!logic::dashboard::sensor_fresh(stale_view, linked.last_rx_voltage_ms), "telemetry older than 2.5 s is stale");

        logic::dashboard::SessionStats stats{};
        logic::afhds2a::Telemetry sample{};
        logic::dashboard::track_session(stats, sample);
        check(!stats.seen, "session stats ignore disconnected samples");
        sample.connected = true;
        sample.rssi = 80;
        sample.rx_voltage_mv = 5000;
        logic::dashboard::track_session(stats, sample);
        sample.rssi = 60;
        sample.rx_voltage_mv = 0;
        logic::dashboard::track_session(stats, sample);
        sample.rssi = 90;
        sample.rx_voltage_mv = 4800;
        logic::dashboard::track_session(stats, sample);
        check(stats.min_rssi == 60, "session minimum RSSI tracks lowest sample");
        check(stats.min_rx_v_mv == 4800, "session minimum RX voltage ignores zero samples");
    }

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
        mn::step_list(p, 12, mn::Rows{mn::kMainVisible}, mn::Dir::Down);
    }
    check(p.selected == 3 && p.scroll == 1, "scroll must follow selection past the visible window");

    // Key repeat: immediate on press, then after 300 ms every 70 ms while held.
    mn::Repeat r{};
    check(mn::repeat_step(r, mn::KeyState{true, true}, 0), "press must step immediately");
    check(!mn::repeat_step(r, mn::KeyState{false, true}, 299), "no repeat before 300 ms");
    check(mn::repeat_step(r, mn::KeyState{false, true}, 300), "first repeat at 300 ms");
    check(!mn::repeat_step(r, mn::KeyState{false, true}, 369), "no repeat before 70 ms interval");
    check(mn::repeat_step(r, mn::KeyState{false, true}, 370), "repeat at 70 ms interval");
    check(!mn::repeat_step(r, mn::KeyState{false, false}, 400) && !r.active, "release must stop repeat");

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
                                Screen::ChannelMonitor, Screen::DiagAnalog, Screen::SystemInfo,
                                Screen::VarioSetup}) {
        s.screen = screen;
        fb.clear();
        mn::render(fb, s, names, st, mn::Live{});
    }

    check(logic::glyphs::kMenuGlyphs[0][4] == 194 && logic::glyphs::kMenuGlyphs[11][13] == 194,
          "menu glyph table must match the Rust bitmaps");
    {
        logic::display::Framebuffer icons{};
        logic::glyphs::draw_glyph_12x12(icons, {0, 0}, logic::glyphs::kMenuGlyphs[0], true);
        const int lit = lit_pixels(icons);
        icons.clear();
        logic::glyphs::draw_glyph_12x12(icons, {0, 0}, logic::glyphs::kMenuGlyphs[0], false);
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

    // Diagnostics pages 2-4 wrap around in both directions.
    mn::State dp{};
    dp.screen = Screen::DiagAnalog;
    mn::update(dp, mn::Input{0, 0, 0}, st);
    press_in(dp, mn::kKeyUp, 900);
    check(dp.page == 4, "up from the first page must wrap to the EEPROM page");
    press_in(dp, mn::kKeyDown, 910);
    check(dp.page == 0, "down from the EEPROM page must wrap to page 1");
    for (int i = 0; i < 3; ++i) {
        press_in(dp, mn::kKeyOk, 920 + i * 10);
    }
    check(dp.page == 3, "OK must reach the P7 page");

    // Helper values for the ext switch, P7 and EEPROM pages.
    check(mn::switch_fill(logic::SwitchPos::Down) == 30 && mn::switch_fill(logic::SwitchPos::Up) == 0 &&
              mn::switch_fill(logic::SwitchPos::Mid) == 0,
          "switch bar fills only when the switch is down");
    check(mn::switch_pos_label(logic::SwitchPos::Mid) == "UP (HIGH)", "mid switch must read as UP");
    check(mn::p7_value(1234, true).view() == "1234", "enabled P7 row shows the raw count");
    check(mn::p7_value(9999, true).view() == "4095", "P7 raw count clamps to full scale");
    check(mn::p7_value(1234, false).view() == " OFF", "disabled P7 row shows OFF");
    check(mn::adc_fill(0) == 0 && mn::adc_fill(9999) == mn::kGaugeMaxFill, "ADC gauge fill clamps");
    check(mn::eeprom_addr_line(0x50).view() == "ADDR: 0x50 (FT24C128A 16KB)", "EEPROM address line");
    check(mn::eeprom_state_label(logic::storage::EepromStatus{0x50, true, true, false}) == "STATUS: READY (HEALTHY)",
          "healthy EEPROM status");
    check(mn::eeprom_state_label(logic::storage::EepromStatus{0x50, true, false, false}) == "STATUS: READ ERR (FAIL)",
          "EEPROM read failure status");
    check(mn::eeprom_state_label(logic::storage::EepromStatus{0x50, true, true, true}) == "STATUS: WRITE ERR (FAIL)",
          "EEPROM write error takes precedence");

    // System info Firmware and Storage rows, as in Rust diag.rs update_system_info.
    const logic::sysinfo::Build build{{"0.27.0"}, {"abc1234"}};
    check(logic::sysinfo::firmware_line(build).view() == "v0.27.0 (abc1234)", "firmware row");
    check(logic::sysinfo::storage_line(logic::storage::EepromStatus{0x50, true, true, false}).view() ==
              "Storage:  I2C2 @ 0x50 (OK)",
          "storage row when healthy");
    check(logic::sysinfo::storage_line(logic::storage::EepromStatus{0x5A, true, false, false}).view() ==
              "Storage:  I2C2 @ 0x5A (ERR)",
          "storage row after a failed boot read");
    check(logic::sysinfo::storage_line(logic::storage::EepromStatus{0x50, true, true, true}).view() ==
              "Storage:  I2C2 @ 0x50 (ERR)",
          "storage row after a failed write");
    check(logic::sysinfo::storage_line(logic::storage::EepromStatus{}).view() == "Storage:  None (No EEPROM)",
          "storage row when no EEPROM answers");
    check(logic::sysinfo::storage_line(logic::storage::EepromStatus{0x50, true, true, true}).len == 27,
          "longest storage row fits its buffer");
    mn::State sysst{};
    sysst.screen = Screen::SystemInfo;
    logic::display::Framebuffer sysfb{};
    mn::Live syslive{};
    syslive.mcu_name = "STM32F072VB";
    syslive.build = build;
    syslive.eeprom = logic::storage::EepromStatus{0x50, true, true, false};
    mn::render(sysfb, sysst, mn::ModelNames{}, logic::storage::default_storage(), syslive);
    const int detected_pixels = lit_pixels(sysfb);
    syslive.eeprom = logic::storage::EepromStatus{};
    sysfb.clear();
    mn::render(sysfb, sysst, mn::ModelNames{}, logic::storage::default_storage(), syslive);
    check(detected_pixels > 0 && lit_pixels(sysfb) > 0 && detected_pixels != lit_pixels(sysfb),
          "system info must draw the storage row from the EEPROM status");

    // Pages 2-4 must render and differ from each other.
    logic::display::Framebuffer ext{};
    mn::Live el{};
    el.adc.fill(4095);
    el.switches.se = logic::SwitchPos::Down;
    el.eeprom = logic::storage::EepromStatus{0x50, true, true, false};
    mn::State ep{};
    ep.screen = Screen::DiagAnalog;
    std::array<std::size_t, 3> page_pixels{};
    for (std::size_t p = 2; p <= 4; ++p) {
        ep.page = static_cast<uint8_t>(p);
        ext.clear();
        mn::render(ext, ep, names, st, el);
        page_pixels[p - 2] = lit_pixels(ext);
    }
    check(page_pixels[0] > 0 && page_pixels[1] > 0 && page_pixels[2] > 0, "diag pages 2-4 must draw");
    check(page_pixels[0] != page_pixels[1] && page_pixels[1] != page_pixels[2], "diag pages 2-4 must differ");
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

void value_cues_test() {
    namespace mn = logic::menu;
    using logic::buzzer::Cue;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    st.radio.ext_switches = 0;
    st.radio.adc_modes = {3, 3, 3, 3, 0, 0, 0, 0, 0, 0};  // SA..SD unusable as dual-rate switches
    mn::State ms{};
    mn::update(ms, mn::Input{0, 0, 0}, st);
    uint32_t now = 0;
    // One key press: returns the cue raised on the press tick, then releases the key.
    auto press = [&](uint16_t key, logic::auto_select::Changes changes = {}) {
        now += 10;
        mn::update(ms, mn::Input{key, key, now, changes}, st);
        const Cue cue = ms.cue;
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        now += 1;
        return cue;
    };

    ms.screen = Screen::DualRate;
    ms.editing = true;
    ms.dual_rate.selected = 2;
    st.active_model.dr_high[0] = 80;
    check(press(mn::kKeyUp) == Cue::ValueStep && st.active_model.dr_high[0] == 85,
          "value cues: dual-rate high steps up with a tone");
    check(press(mn::kKeyDown) == Cue::ValueStep && st.active_model.dr_high[0] == 80,
          "value cues: dual-rate high steps down with a tone");
    st.active_model.dr_high[0] = 100;
    check(press(mn::kKeyUp) == Cue::None && st.active_model.dr_high[0] == 100,
          "value cues: no tone when dual-rate is clamped");

    ms.dual_rate.selected = 0;
    st.active_model.dr_switch = 0;
    check(press(0, {std::nullopt, uint8_t{1}}) == Cue::None && st.active_model.dr_switch == 0,
          "value cues: unavailable dual-rate switch is silent");
    check(press(0, {std::nullopt, uint8_t{0}}) == Cue::ValueSelect,
          "value cues: dual-rate switch selection chimes at 2400 Hz");
    check(press(mn::kKeyDown) == Cue::ValueStep, "value cues: dual-rate switch cycling tones");
    ms.dual_rate.selected = 1;
    check(press(mn::kKeyDown) == Cue::ValueStep && ms.axis == 1, "value cues: axis cycling tones");

    ms.screen = Screen::ThrottleCurve;
    ms.editing = false;
    ms.throttle_curve.selected = 3;
    check(press(mn::kKeyDown) == Cue::ValueStep && ms.throttle_curve.selected == 4,
          "value cues: throttle curve cursor tones");
    ms.editing = true;
    st.active_model.thr_curve[1] = 100;
    check(press(mn::kKeyUp) == Cue::ValueStep, "value cues: throttle point tones on every press");

    ms.screen = Screen::WingMixer;
    ms.editing = true;
    st.active_model.wing_tail_mix = 1;
    st.active_model.template_diff = 100;
    check(press(mn::kKeyUp) == Cue::None, "value cues: no tone when the template diff is clamped");
    check(press(mn::kKeyDown) == Cue::ValueStep && st.active_model.template_diff == 95,
          "value cues: template diff tones when it moves");

    ms.screen = Screen::GlobalVars;
    ms.editing = true;
    ms.gvars.selected = 0;
    st.active_model.gvars[0] = 1000;
    check(press(mn::kKeyUp) == Cue::None, "value cues: no tone when the global variable is clamped");
    check(press(mn::kKeyDown) == Cue::ValueStep && st.active_model.gvars[0] == 950,
          "value cues: global variable tones when it moves");

    ms.screen = Screen::MixerLine;
    ms.editing = true;
    ms.mix_idx = 0;
    ms.mix_line.selected = 0;
    check(press(mn::kKeyUp) == Cue::ValueStep, "value cues: mixer target channel tones");
    ms.mix_line.selected = 3;
    st.active_model.mixes[0].weight = 100;
    check(press(mn::kKeyUp) == Cue::None && st.active_model.mixes[0].weight == 100,
          "value cues: no tone when the mixer weight is clamped");
    check(press(mn::kKeyDown) == Cue::ValueStep && st.active_model.mixes[0].weight == 95,
          "value cues: mixer weight tones when it moves");
    ms.screen = Screen::DiagAnalog;
    ms.editing = false;
    ms.page = 0;
    now += 10;
    mn::update(ms, mn::Input{mn::kKeyDown, mn::kKeyDown, now}, st);
    check(ms.nav_tick && ms.cue == Cue::None && ms.page == 1, "value cues: diag page change uses the nav tick");
    mn::update(ms, mn::Input{0, 0, now + 1}, st);
    check(!ms.nav_tick && ms.cue == Cue::None, "value cues: tones clear on the next tick");
}

void cancel_click_test() {
    namespace mn = logic::menu;
    using logic::buzzer::Cue;
    using mn::Screen;

    logic::storage::RadioStorage st = logic::storage::default_storage();
    uint32_t now = 0;
    // Cancel from a fresh menu state on `from`: returns the screen and cue of the cancel tick.
    auto cancel = [&](Screen from, bool editing, Cue& cue) {
        mn::State ms{};
        ms.screen = from;
        ms.editing = editing;
        now += 10;
        mn::update(ms, mn::Input{mn::kKeyCancel, mn::kKeyCancel, now}, st);
        cue = ms.cue;
        mn::update(ms, mn::Input{0, 0, now + 1}, st);
        check(ms.cue == Cue::None, "cancel click: tone clears on the next tick");
        return ms.screen;
    };
    auto leaves_with_click = [&](Screen from, Screen to, const char* what) {
        Cue cue{Cue::None};
        check(cancel(from, false, cue) == to && cue == Cue::Click, what);
    };

    leaves_with_click(Screen::ModelSelect, Screen::Main, "cancel click: model select returns to main");
    leaves_with_click(Screen::ModelSetup, Screen::Main, "cancel click: model setup returns to main");
    leaves_with_click(Screen::ProtocolSetup, Screen::ModelSetup, "cancel click: protocol setup returns to model setup");
    leaves_with_click(Screen::VarioSetup, Screen::ModelSetup, "cancel click: vario setup returns to model setup");
    leaves_with_click(Screen::TelemSetup, Screen::ModelSetup, "cancel click: telemetry setup returns to model setup");
    leaves_with_click(Screen::DualRate, Screen::Main, "cancel click: dual rate returns to main");
    leaves_with_click(Screen::ThrottleCurve, Screen::Main, "cancel click: throttle curve returns to main");
    leaves_with_click(Screen::WingMixer, Screen::Main, "cancel click: wing mixer returns to main");
    leaves_with_click(Screen::MixerLine, Screen::WingMixer, "cancel click: mixer line returns to wing mixer");
    leaves_with_click(Screen::GlobalVars, Screen::Main, "cancel click: global variables return to main");
    leaves_with_click(Screen::AuxChannels, Screen::Main, "cancel click: aux channels return to main");
    leaves_with_click(Screen::RadioSetup, Screen::Main, "cancel click: radio setup returns to main");
    leaves_with_click(Screen::InputSetup, Screen::RadioSetup, "cancel click: input setup returns to radio setup");
    leaves_with_click(Screen::ChannelReverse, Screen::Main, "cancel click: channel reverse returns to main");
    leaves_with_click(Screen::ChannelMonitor, Screen::Main, "cancel click: channel monitor returns to main");
    leaves_with_click(Screen::DiagAnalog, Screen::Main, "cancel click: analog diagnostics return to main");
    leaves_with_click(Screen::SystemInfo, Screen::Main, "cancel click: system info returns to main");

    Cue edit_cue{Cue::None};
    check(cancel(Screen::DualRate, true, edit_cue) == Screen::DualRate && edit_cue == Cue::None,
          "cancel click: leaving edit mode stays silent, as in Rust");
    check(cancel(Screen::MixerLine, true, edit_cue) == Screen::MixerLine && edit_cue == Cue::None,
          "cancel click: mixer line edit cancel stays silent");
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
    const af::Packet sticks = af::build_sticks(af::TxId{0x12345678}, af::RxId{0x9ABCDEF0}, ch);
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
    const af::Packet legacy = af::build_sticks(af::TxId{0x11223344}, af::RxId{0x55667788}, [&] {
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

    const af::Packet fs = af::build_failsafe(af::TxId{0xAABBCCDD}, af::RxId{0});
    check(fs[0] == af::kPacketFailsafe, "afhds2a: failsafe command byte");
    check((fs[9 + 2 * 2] | (fs[10 + 2 * 2] << 8)) == af::kFailsafeThrottleUs, "afhds2a: failsafe throttle is 988us");
    bool hold_ok = true;
    for (std::size_t c = 0; c < 14; ++c) {
        if (c != 2) hold_ok = hold_ok && fs[9 + c * 2] == 0xFF && fs[10 + c * 2] == 0xFF;
    }
    check(hold_ok, "afhds2a: failsafe holds non-throttle channels");

    af::Link link{af::TxId{0x11223344}};
    link.rx_id = af::RxId{0x55667788};
    af::Packet st = af::build_settings(link.tx_id, link.rx_id, link.settings);
    check(st[0] == af::kPacketSettings && st[9] == 0xFD && st[10] == 0xFF, "afhds2a: settings header");
    check(st[11] == 0x32 && st[12] == 0x00 && st[13] == 0x00, "afhds2a: default 50 Hz PWM");
    check(st[18] == 0x05 && st[19] == 0xDC && st[21] == 0xDE, "afhds2a: center 1500us and iBUS default");
    link.set_rx_settings(400, {true}, {true});
    st = af::build_settings(link.tx_id, link.rx_id, link.settings);
    check(st[11] == 0x90 && st[12] == 0x01 && st[13] == 0x01 && st[21] == 0xDD, "afhds2a: 400 Hz PPM SBUS");
    check(link.next == af::PacketKind::Settings, "afhds2a: settings change queues a settings packet");
    link.set_rx_settings(30, {false}, {false});
    check(link.settings.rate_hz == 50, "afhds2a: rate clamped to 50 Hz");
    link.set_rx_settings(500, {false}, {false});
    check(link.settings.rate_hz == 400, "afhds2a: rate clamped to 400 Hz");

    link.set_rf_power(4);
    check(link.rf_power_idx == 0, "afhds2a: out-of-range RF power falls back to index 0");
    link.set_rf_power(3);
    check(link.rf_power_idx == 3, "afhds2a: RF power index 3 kept");

    check(af::rssi_from_raw(0) == 100 && af::rssi_from_raw(200) == 0, "afhds2a: RSSI clamping");
    check(af::rssi_from_raw(100) == 96, "afhds2a: RSSI formula 256 - raw*8/5");

    // Hopping table: all in band and separated by at least 5 channels.
    const af::HoppingTable hop = af::hopping_table(af::TxId{0x12345678});
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
    af::Link bl{af::TxId{0x12345678}};
    bl.binding = true;
    af::Frame reply{};
    reply[0] = af::kPacketBind2;
    reply[5] = 0x78;
    reply[6] = 0x56;
    reply[7] = 0x34;
    reply[8] = 0x12;
    reply[9] = 0x01;
    check(bl.on_bind_reply(reply) && bl.rx_id.value == 0x12345678 && bl.bind_step == af::BindStep::Confirm,
          "afhds2a: bind reply moves to confirmation with RX ID");
    const af::Packet confirm = af::build_bind(bl.tx_id, bl.rx_id, af::BindStep::Confirm, hop);
    check(confirm[0] == af::kPacketBind2 && confirm[9] == 0x02 && confirm[5] == 0x78,
          "afhds2a: bind confirmation packet");

    // Downlink handling: TX ID filter, sensor decoding, requests.
    af::Link tl{af::TxId{0x12345678}};
    tl.rx_id = af::RxId{0x9ABCDEF0};
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

    af::Link tt{af::TxId{0x12345678}};
    tt.rx_id = af::RxId{0x9ABCDEF0};
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

void vario_test() {
    namespace vr = logic::vario;

    const vr::Config climb_cfg{.switch_index = 1,
                               .climb_deadband = {2},
                               .sink_deadband = {5},
                               .sink_mode = vr::SinkMode::Pulse,
                               .motor_mute = true};

    // Weak lift +50 cm/s: 1000..1300 Hz, pulse >= 200 ms.
    vr::State v{};
    vr::Tone tone_weak = v.update({.now = {1000},
                                   .alt_m = {100},
                                   .alt_dm = {1000},
                                   .vspeed = {50},
                                   .has_native_vspeed = true,
                                   .telem_connected = true,
                                   .switch_active = true,
                                   .throttle = {1000}},
                                  climb_cfg);
    check(!tone_weak.is_silent(), "vario: weak lift audible");
    check(tone_weak.freq_hz >= 1000 && tone_weak.freq_hz <= 1300, "vario: weak lift pitch 1000..1300 Hz");
    check(tone_weak.pulse_ms >= 200, "vario: weak lift pulse >= 200 ms");

    // Strong lift converges to +500 cm/s: higher pitch, shorter pulses.
    vr::Tone tone_strong = tone_weak;
    for (uint32_t i = 1; i <= 16; ++i) {
        tone_strong = v.update({.now = {1000 + i * 50},
                                .alt_m = {static_cast<int16_t>(100 + i)},
                                .alt_dm = {static_cast<int16_t>(1000 + i * 10)},
                                .vspeed = {500},
                                .has_native_vspeed = true,
                                .telem_connected = true,
                                .switch_active = true,
                                .throttle = {1000}},
                               climb_cfg);
    }
    check(!tone_strong.is_silent(), "vario: strong lift audible");
    check(tone_strong.freq_hz >= 2500, "vario: strong lift pitch >= 2500 Hz");
    check(tone_strong.pulse_ms <= 80, "vario: strong lift pulse <= 80 ms");
    check(tone_strong.freq_hz > tone_weak.freq_hz && tone_strong.pulse_ms < tone_weak.pulse_ms,
          "vario: strong lift higher and faster than weak");

    // Deadband: +10 cm/s (climb threshold 20) and -30 cm/s (sink threshold 50) are silent.
    vr::State db{};
    check(db.update({.now = {1000},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {10},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = true,
                     .throttle = {1000}},
                    climb_cfg)
                  .is_silent(),
          "vario: level flight inside climb deadband is silent");
    check(db.update({.now = {1100},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {-30},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = true,
                     .throttle = {1000}},
                    climb_cfg)
                  .is_silent(),
          "vario: mild sink inside sink deadband is silent");

    // Heavy sink: Mute is silent, Pulse and Continuous are audible with their cadence.
    vr::Config sink_cfg = climb_cfg;
    sink_cfg.sink_mode = vr::SinkMode::Mute;
    vr::State sv{};
    check(sv.update({.now = {1000},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {-150},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = true,
                     .throttle = {1000}},
                    sink_cfg)
                  .is_silent(),
          "vario: heavy sink in Mute mode is silent");

    sink_cfg.sink_mode = vr::SinkMode::Pulse;
    const vr::Tone tone_pulse = sv.update({.now = {1100},
                                           .alt_m = {95},
                                           .alt_dm = {950},
                                           .vspeed = {-150},
                                           .has_native_vspeed = true,
                                           .telem_connected = true,
                                           .switch_active = true,
                                           .throttle = {1000}},
                                          sink_cfg);
    check(!tone_pulse.is_silent(), "vario: heavy sink in Pulse mode audible");
    check(tone_pulse.pulse_ms == 160 && tone_pulse.pause_ms == 360, "vario: sink pulse cadence 160/360 ms");
    check(tone_pulse.freq_hz <= 450, "vario: sink pitch at or below 450 Hz");

    sink_cfg.sink_mode = vr::SinkMode::Continuous;
    const vr::Tone tone_cont = sv.update({.now = {1200},
                                          .alt_m = {90},
                                          .alt_dm = {900},
                                          .vspeed = {-150},
                                          .has_native_vspeed = true,
                                          .telem_connected = true,
                                          .switch_active = true,
                                          .throttle = {1000}},
                                         sink_cfg);
    check(!tone_cont.is_silent() && tone_cont.pause_ms == 0, "vario: continuous sink has no pause");
    check(tone_cont.pulse_ms == 1000, "vario: continuous sink pulse 1000 ms");

    // Motor lockout above 1100 us; gliding (1000 us) stays audible.
    vr::State th{};
    check(th.update({.now = {1000},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {300},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = true,
                     .throttle = {1500}},
                    climb_cfg)
                  .is_silent(),
          "vario: motor climb muted");
    check(!th.update({.now = {1100},
                      .alt_m = {105},
                      .alt_dm = {1050},
                      .vspeed = {300},
                      .has_native_vspeed = true,
                      .telem_connected = true,
                      .switch_active = true,
                      .throttle = {1000}},
                     climb_cfg)
                   .is_silent(),
          "vario: gliding climb audible");

    // Derived vspeed from altitude: +20 dm in 1000 ms gives more than 40 cm/s after filtering.
    vr::Config motor_cfg = climb_cfg;
    motor_cfg.motor_mute = false;
    vr::State dv{};
    dv.update({.now = {0},
               .alt_m = {100},
               .alt_dm = {1000},
               .vspeed = {0},
               .has_native_vspeed = false,
               .telem_connected = true,
               .switch_active = true,
               .throttle = {1000}},
              motor_cfg);
    const vr::Tone tone_derived = dv.update({.now = {1000},
                                             .alt_m = {102},
                                             .alt_dm = {1020},
                                             .vspeed = {0},
                                             .has_native_vspeed = false,
                                             .telem_connected = true,
                                             .switch_active = true,
                                             .throttle = {1000}},
                                            motor_cfg);
    check(!tone_derived.is_silent(), "vario: derived climb audible");
    check(dv.vspeed.value > 40, "vario: derived vspeed above 40 cm/s");

    // Native vspeed 200 cm/s decays toward 0 when the sensor reports level flight.
    vr::State nv{};
    nv.update({.now = {0},
               .alt_m = {100},
               .alt_dm = {1000},
               .vspeed = {200},
               .has_native_vspeed = true,
               .telem_connected = true,
               .switch_active = true,
               .throttle = {1000}},
              motor_cfg);
    check(nv.vspeed.value == 200, "vario: native init takes first reading");
    for (uint32_t i = 1; i <= 10; ++i) {
        nv.update({.now = {i * 100},
                   .alt_m = {100},
                   .alt_dm = {1000},
                   .vspeed = {0},
                   .has_native_vspeed = true,
                   .telem_connected = true,
                   .switch_active = true,
                   .throttle = {1000}},
                  motor_cfg);
    }
    check(nv.vspeed.value < 15, "vario: native vspeed decays to near zero");
    check(nv.update({.now = {1100},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {0},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = true,
                     .throttle = {1000}},
                    motor_cfg)
                  .is_silent(),
          "vario: decayed vspeed is silent");

    // Disconnect resets state and is silent.
    check(nv.update({.now = {1200},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {300},
                     .has_native_vspeed = true,
                     .telem_connected = false,
                     .switch_active = true,
                     .throttle = {1000}},
                    motor_cfg)
                  .is_silent() &&
              nv.vspeed.value == 0 && !nv.initialized,
          "vario: disconnect resets state");

    // Switch off is silent but still filters vspeed.
    vr::State sw{};
    check(sw.update({.now = {0},
                     .alt_m = {100},
                     .alt_dm = {1000},
                     .vspeed = {300},
                     .has_native_vspeed = true,
                     .telem_connected = true,
                     .switch_active = false,
                     .throttle = {1000}},
                    climb_cfg)
                  .is_silent() &&
              sw.vspeed.value == 300,
          "vario: inactive switch is silent");

    check(vr::sink_mode_from_u8(0) == vr::SinkMode::Mute && vr::sink_mode_from_u8(1) == vr::SinkMode::Pulse &&
              vr::sink_mode_from_u8(2) == vr::SinkMode::Continuous && vr::sink_mode_from_u8(9) == vr::SinkMode::Mute,
          "vario: sink mode from u8");
    check(vr::kSilent.is_silent() && vr::Tone{1000, 100, 100} == vr::Tone{1000, 100, 100},
          "vario: silent tone and tone equality");
}

static_assert(logic::a7105::crc_ok(0x00));
static_assert(!logic::a7105::crc_ok(0x20));
static_assert(logic::a7105::power_from_index(0) == 0x1F);
static_assert(logic::a7105::power_from_index(1) == 0x0D);
static_assert(logic::a7105::power_from_index(2) == 0x02);
static_assert(logic::a7105::power_from_index(3) == 0x00);
static_assert(logic::a7105::power_from_index(7) == 0x1F);
static_assert(logic::a7105::cmd_read(0x10) == 0x50);
static_assert(logic::a7105::cmd_write(0x45) == 0x05);
static_assert(logic::a7105::kAfhds2aInit.size() == 50);
static_assert(logic::a7105::kAfhds2aInit[0] == 0xFF);
static_assert(logic::a7105::kAfhds2aInit[1] == 0xE2);
static_assert(logic::a7105::kAfhds2aInit[11] == 0x19);
static_assert(logic::a7105::kAfhds2aInit[12] == 0x01);
static_assert(logic::a7105::kAfhds2aInit[16] == 0x9E);

void a7105_radio_test() {
    namespace af = logic::afhds2a;
    namespace a7 = logic::a7105;

    af::Heartbeat gate{};
    bool gate_ok = true;
    for (int i = 0; i < 29; ++i) {
        gate_ok = gate_ok && gate.allow_tx(0);
    }
    check(gate_ok, "heartbeat: first 29 unchanged ticks still transmit");
    check(!gate.allow_tx(0), "heartbeat: 30th unchanged tick stops TX");
    check(gate.allow_tx(5), "heartbeat: new heartbeat resumes TX");
    check(gate.stale == 0, "heartbeat: stale count reset");

    af::Radio bind(af::TxId{0x12345678});
    check(!bind.binding(), "radio: starts in normal mode");
    const auto bind_power = bind.set_bind_mode(true);
    check(bind_power && *bind_power == a7::kBindPower, "radio: bind mode uses bind power");
    check(bind.binding(), "radio: bind mode active");

    const auto t1 = bind.on_timer_tick();
    check(t1.channel == 0x0D, "bind: odd packet uses channel 0x0D");
    check(t1.packet[0] == af::kPacketBind1, "bind: phase 1 sends 0xBB");
    const auto g1 = bind.on_gio2_event();
    check(g1.step == af::GioStep::StartRx && g1.front == af::FrontEnd::Off,
          "bind: TX done starts RX with front-end off");
    check(bind.link.bind_step == af::BindStep::Phase2, "bind: phase 1 -> 2");
    auto cycle = [&bind] {
        bind.on_timer_tick();
        return bind.on_gio2_event();
    };
    cycle();
    check(bind.link.bind_step == af::BindStep::Phase3, "bind: phase 2 -> 3");
    cycle();
    check(bind.link.bind_step == af::BindStep::Phase1, "bind: phase 3 -> 1");
    check(bind.on_timer_tick().channel == 0x8C, "bind: even packet uses channel 0x8C");

    af::Frame bind_reply{};
    bind_reply[0] = af::kPacketBind2;
    bind_reply[9] = 0x01;
    bind_reply[5] = 0x78;
    bind_reply[6] = 0x56;
    bind_reply[7] = 0x34;
    bind_reply[8] = 0x12;
    bind.on_rx_frame(bind_reply, 0, 0);
    check(bind.link.rx_id.value == 0x12345678, "bind: receiver answer captures RX ID");
    check(bind.link.bind_step == af::BindStep::Confirm, "bind: receiver answer enters confirm");

    for (int i = 0; i < 3; ++i) {
        bind.on_timer_tick();
    }
    check(bind.binding(), "bind: three confirm packets keep binding");
    const auto done = bind.on_timer_tick();
    check(!bind.binding() && done.power_reg && *done.power_reg == a7::kPowerHigh,
          "bind: fourth confirm leaves bind mode with high power");
    check(bind.link.bind_done, "bind: bind_done after confirm");
    const auto saved = bind.take_rx_save();
    check(saved && saved->value == 0x12345678, "bind: RX ID queued for persistence");
    check(!bind.take_rx_save(), "bind: save flag cleared after take");

    af::Radio normal(af::TxId{0x12345678});
    normal.set_rx_id(af::RxId{0x11223344});
    bool sticks_before_failsafe = true;
    for (uint32_t i = 1; i <= 1568; ++i) {
        sticks_before_failsafe = sticks_before_failsafe && normal.on_timer_tick().packet[0] == af::kPacketSticks;
    }
    check(sticks_before_failsafe, "normal: sticks frames before the failsafe period");
    check(normal.on_timer_tick().packet[0] == af::kPacketFailsafe, "normal: failsafe frame at packet 1569");
    check(normal.on_timer_tick().packet[0] == af::kPacketSticks, "normal: sticks resume after failsafe");

    af::Radio lossy(af::TxId{0x12345678});
    lossy.link.telemetry.connected = true;
    lossy.link.telemetry.rssi = 80;
    for (int i = 0; i < 52; ++i) {
        lossy.on_timer_tick();
    }
    check(lossy.link.telemetry.connected, "loss: 52 missed frames keep the link");
    lossy.on_timer_tick();
    check(!lossy.link.telemetry.connected && lossy.link.telemetry.rssi == 0,
          "loss: 53rd missed frame drops the link");

    af::Radio rxs(af::TxId{0xAABBCCDD});
    rxs.set_rx_id(af::RxId{0});
    check(!rxs.link.bind_done && rxs.link.rx_id.value == 0xFFFF'FFFFu, "rx id: zero becomes placeholder");
    af::Frame downlink{};
    downlink[0] = 0xAC;
    downlink[1] = 0xDD;
    downlink[2] = 0xCC;
    downlink[3] = 0xBB;
    downlink[4] = 0xAA;
    downlink[5] = 0x78;
    downlink[6] = 0x56;
    downlink[7] = 0x34;
    downlink[8] = 0x12;
    rxs.on_rx_frame(downlink, 0x20, 1000);
    check(rxs.link.rx_id.value == 0x12345678, "rx id: placeholder replaced by downlink ID");
    check(rxs.link.telemetry.connected, "rx id: downlink marks link connected");
    const auto rx_saved = rxs.take_rx_save();
    check(rx_saved && rx_saved->value == 0x12345678, "rx id: captured ID requests save");
    check(!rxs.take_rx_save(), "rx id: save flag cleared after take");
}

void crsf_diag_test() {
    namespace tm = logic::telemetry;
    namespace crsf = logic::crsf;

    const auto none = tm::crsf_diag(crsf::Telemetry{}, {.value = 1000});
    check(none.lq.view() == "LQ: ---%", "crsf diag: no data shows LQ placeholder");
    check(none.rssi.view() == "RS: ---dB", "crsf diag: no data shows RS placeholder");
    check(none.snr.view() == "SNR: --dB", "crsf diag: no data shows SNR placeholder");
    check(none.ant.view() == "ANT: -", "crsf diag: disconnected antenna");
    check(none.pwr.view() == "PWR: ---", "crsf diag: no data shows PWR placeholder");
    check(none.rate == "---", "crsf diag: no data shows rate placeholder");
    check(none.bat_label.view() == "BAT:", "crsf diag: no data shows BAT label");
    check(none.bat.view() == "----", "crsf diag: no data shows battery placeholder");
    check(none.cap.view() == "----", "crsf diag: no data shows capacity placeholder");

    crsf::Telemetry t{};
    t.connected = true;
    t.last_link_ms = 1000;
    t.last_vbat_ms = 1000;
    t.uplink_link_quality = 99;
    t.uplink_rssi_1 = -50;
    t.uplink_snr = 5;
    t.tx_power_mw = 100;
    t.rf_mode = 3;
    t.active_antenna = 1;
    t.rx_battery_mv = 7400;
    t.cell_count = 2;
    t.rx_capacity_mah = 1234;

    const auto live = tm::crsf_diag(t, {.value = 1500});
    check(live.lq.view() == "LQ: 99%", "crsf diag: live link quality");
    check(live.rssi.view() == "RS: -050dB", "crsf diag: live RSSI is absolute, zero padded");
    check(live.snr.view() == "SNR:+05dB", "crsf diag: live SNR is signed");
    check(live.ant.view() == "ANT: 2", "crsf diag: antenna 2 when index is 1");
    check(live.pwr.view() == "PWR: 100mW", "crsf diag: live TX power");
    check(live.rate == "100Hz", "crsf diag: rate name from rf mode");
    check(live.bat_label.view() == "2S: ", "crsf diag: cell count label");
    check(live.bat.view() == "7.40V", "crsf diag: live RX battery");
    check(live.cap.view() == " 1234", "crsf diag: capacity right aligned in 5 columns");

    const auto stale = tm::crsf_diag(t, {.value = 5000});
    check(stale.lq.view() == "LQ: 99%*", "crsf diag: stale link gets star");
    check(stale.pwr.view() == "PWR: 100mW*", "crsf diag: stale power gets star");
    check(stale.bat.view() == "7.40V*", "crsf diag: stale battery gets star");
    check(stale.cap.view() == " 1234*", "crsf diag: stale capacity gets star");
    check(stale.rate == "100Hz", "crsf diag: stale rate has no star");
    check(stale.ant.view() == "ANT: 2", "crsf diag: antenna has no star");

    crsf::Telemetry down = t;
    down.connected = false;
    const auto dropped = tm::crsf_diag(down, {.value = 1500});
    check(dropped.ant.view() == "ANT: -", "crsf diag: disconnected link shows no antenna");
    check(dropped.lq.view() == "LQ: 99%*", "crsf diag: disconnected link is stale");

    crsf::Telemetry no_power = t;
    no_power.tx_power_mw = 0;
    no_power.cell_count = 1;
    no_power.last_vbat_ms = 0;
    const auto unpowered = tm::crsf_diag(no_power, {.value = 1500});
    check(unpowered.pwr.view() == "PWR: ---", "crsf diag: zero power shows placeholder");
    check(unpowered.bat_label.view() == "BAT:", "crsf diag: single cell shows BAT label");
    check(unpowered.bat.view() == "----", "crsf diag: no battery data shows placeholder");

    crsf::Telemetry ten_cells = t;
    ten_cells.cell_count = 10;
    check(tm::crsf_diag(ten_cells, {.value = 1500}).bat_label.view() == "BAT:",
          "crsf diag: ten cells shows BAT label");
}

void telemetry_test() {
    namespace tm = logic::telemetry;
    namespace fmt = logic::format;

    check(tm::rf_mode_name(0) == "4Hz", "telemetry: rf mode 0 is 4Hz");
    check(tm::rf_mode_name(2) == "50Hz", "telemetry: rf mode 2 is 50Hz");
    check(tm::rf_mode_name(7) == "250Hz", "telemetry: rf mode 7 is 250Hz");
    check(tm::rf_mode_name(9) == "500Hz", "telemetry: rf mode 9 is 500Hz");
    check(tm::rf_mode_name(13) == "F1000", "telemetry: rf mode 13 is F1000");
    check(tm::rf_mode_name(14) == "---", "telemetry: rf mode 14 is unknown");
    check(tm::rf_mode_name(99) == "---", "telemetry: rf mode 99 is unknown");

    check(fmt::format_fixed({.value = 5, .decimals = 1}).view() == "0.5", "telemetry: 5 with 1 decimal is 0.5");
    check(fmt::format_fixed({.value = 420, .decimals = 2, .suffix = "V"}).view() == "4.20V",
          "telemetry: 420 with 2 decimals is 4.20V");
    check(fmt::format_fixed({.value = -15, .decimals = 1}).view() == "-1.5", "telemetry: -15 with 1 decimal");
    check(fmt::format_fixed({.value = 0}).view() == "0", "telemetry: zero prints 0");
    check(fmt::format_fixed({.value = -3, .pad_digits = 2, .pad_zero = true, .sign = fmt::SignMode::Force,
                             .suffix = "dB"})
                  .view() == "-03dB",
          "telemetry: negative zero-padded sign");
    check(fmt::format_fixed({.value = 7, .sign = fmt::SignMode::Force}).view() == "+7",
          "telemetry: forced plus sign");
    check(fmt::format_fixed({.value = 99, .pad_digits = 3, .suffix = "%"}).view() == " 99%",
          "telemetry: space padding");

    // Freshness: never seen, fresh, stale at 2500 ms, disconnected, and unsigned wrap-around.
    check(tm::check_sensor_status({.value = 0}, {.value = 5000}, true) == tm::SensorStatus{},
          "telemetry: never seen is neither fresh nor stale");
    check(tm::check_sensor_status({.value = 1000}, {.value = 1500}, true) == tm::SensorStatus{true, false},
          "telemetry: 500 ms old is fresh");
    check(tm::check_sensor_status({.value = 1000}, {.value = 3499}, true) == tm::SensorStatus{true, false},
          "telemetry: 2499 ms old is fresh");
    check(tm::check_sensor_status({.value = 1000}, {.value = 3500}, true) == tm::SensorStatus{true, true},
          "telemetry: 2500 ms old is stale");
    check(tm::check_sensor_status({.value = 1000}, {.value = 1500}, false) == tm::SensorStatus{true, true},
          "telemetry: disconnected is stale");
    check(tm::check_sensor_status({.value = 0xFFFFFF00u}, {.value = 0x100u}, true) == tm::SensorStatus{true, false},
          "telemetry: freshness survives tick wrap-around");

    logic::crsf::Telemetry c{};
    c.connected = true;
    c.last_vbat_ms = 1000;
    c.rx_battery_mv = 4200;
    c.last_link_ms = 1000;
    c.uplink_link_quality = 99;
    c.uplink_snr = -3;
    c.rf_mode = 2;
    c.active_antenna = 1;
    c.rx_current_ma = 0;
    c.rx_capacity_mah = 1234;
    c.tx_power_mw = 100;
    c.last_vspd_ms = 1000;
    c.vspeed_cms = -250;
    c.last_alt_ms = 1000;
    c.baro_alt_m = 42;
    c.last_cells_ms = 1000;
    c.cell_count = 4;
    c.cell_min_mv = 3950;
    c.last_rpm_ms = 1000;
    c.rpm = 150000;
    c.last_temp_ms = 1000;
    c.temp_c_x10 = -15;
    c.last_gps_ms = 1000;
    c.gps_groundspeed_kmh_x10 = 255;
    c.gps_satellites = 12;
    c.gps_heading_cdeg = 9050;
    c.last_att_ms = 1000;
    c.yaw_cdeg = -300;
    c.pitch_cdeg = 150;
    c.roll_cdeg = -50;

    logic::afhds2a::Telemetry a{};
    a.connected = true;
    a.last_rx_voltage_ms = 1000;
    a.rx_voltage_mv = 12600;
    a.rssi = 120;
    a.packets_received = 456;
    a.last_vspd_ms = 1000;
    a.vspeed_cms = 150;
    a.last_alt_ms = 1000;
    a.alt_m = 42;

    const tm::Sources crsf_src{.source = tm::Source::Crsf,
                               .crsf = c,
                               .afhds = a,
                               .tx_battery = {.value = 3700},
                               .packets_sent = {.value = 123},
                               .timer = {}};
    const tm::Sources afhds_src{.source = tm::Source::Afhds2a,
                                .crsf = c,
                                .afhds = a,
                                .tx_battery = {.value = 3700},
                                .packets_sent = {.value = 123},
                                .timer = "12:34"};
    const auto tile_at = [](uint8_t slot, uint32_t now, const tm::Sources& s) {
        return tm::sensor_tile({.value = slot}, {.value = now}, s);
    };
    const auto is = [](const tm::Tile& t, std::string_view label, std::string_view value) {
        return t.label == label && t.value.view() == value;
    };

    check(is(tile_at(0, 1500, crsf_src), "RX BAT", "4.20V"), "telemetry: CRSF RX battery tile");
    check(tile_at(0, 4000, crsf_src).stale && is(tile_at(0, 4000, crsf_src), "RX BAT", "4.20V*"),
          "telemetry: stale RX battery gets '*'");
    check(is(tile_at(0, 1500, afhds_src), "RX BAT", "12.60V"), "telemetry: AFHDS RX battery tile");
    check(is(tile_at(1, 1500, crsf_src), "LINK Q", " 99%"), "telemetry: CRSF link quality padded to 3");
    check(is(tile_at(1, 1500, afhds_src), "RSSI", "100%"), "telemetry: AFHDS RSSI clamped to 100");
    check(is(tile_at(2, 1500, crsf_src), "TX BAT", "3.70V"), "telemetry: TX battery tile");
    check(is(tile_at(3, 1500, crsf_src), "CURRENT", "0.0A"), "telemetry: CRSF current without draw");
    check(is(tile_at(3, 1500, afhds_src), "LINK", "OK"), "telemetry: AFHDS link OK");
    check(is(tile_at(4, 1500, crsf_src), "CAP mAh", "1234"), "telemetry: CRSF capacity");
    check(is(tile_at(5, 1500, crsf_src), "PWR mW", "100"), "telemetry: CRSF power");
    check(is(tile_at(6, 1500, crsf_src), "SNR dB", "-03dB"), "telemetry: CRSF SNR");
    check(is(tile_at(6, 1500, afhds_src), "SNR dB", "--dB"), "telemetry: AFHDS SNR placeholder");
    check(is(tile_at(7, 1500, crsf_src), "RATE", "50Hz"), "telemetry: CRSF rate name");
    check(is(tile_at(8, 1500, crsf_src), "ANTENNA", "ANT 2"), "telemetry: CRSF antenna 2");
    check(is(tile_at(9, 1500, crsf_src), "PKTS TX", "123"), "telemetry: packets sent");
    check(is(tile_at(10, 1500, afhds_src), "PKTS RX", "456"), "telemetry: packets received");
    check(is(tile_at(11, 1500, afhds_src), "TIMER", "12:34"), "telemetry: timer text");
    check(is(tile_at(11, 1500, crsf_src), "TIMER", "00:00"), "telemetry: timer placeholder");
    check(is(tile_at(12, 1500, crsf_src), "VSPD", "-2.5m/s"), "telemetry: CRSF vertical speed");
    check(is(tile_at(12, 1500, afhds_src), "VSPD", "+1.5m/s"), "telemetry: AFHDS vertical speed");
    check(is(tile_at(13, 1500, crsf_src), "ALT", "42m"), "telemetry: altitude");
    check(is(tile_at(14, 1500, crsf_src), "CELLS", "4S 3.95V"), "telemetry: CRSF cells with count");
    check(is(tile_at(14, 1500, afhds_src), "CELLS", "12.60V"), "telemetry: AFHDS cells fallback");
    check(is(tile_at(15, 1500, crsf_src), "RPM", "150k"), "telemetry: RPM in thousands");
    check(is(tile_at(16, 1500, crsf_src), "TEMP", "-1.5C"), "telemetry: temperature");
    check(is(tile_at(17, 1500, crsf_src), "GSPD", "25kmh"), "telemetry: ground speed");
    check(is(tile_at(18, 1500, crsf_src), "SATS", "12"), "telemetry: satellites");
    check(is(tile_at(19, 1500, crsf_src), "HDG", "90D"), "telemetry: GPS heading");
    check(is(tile_at(20, 1500, crsf_src), "PITCH", "+15D"), "telemetry: pitch");
    check(is(tile_at(21, 1500, crsf_src), "ROLL", "-5D"), "telemetry: roll");
    check(tile_at(21, 1500, afhds_src).value.view() == "---", "telemetry: AFHDS attitude placeholder");
    check(tile_at(22, 1500, crsf_src).slot_off && is(tile_at(22, 1500, crsf_src), "SLOT OFF", ""),
          "telemetry: unknown slot is off");

    c.last_gps_ms = 0;
    check(is(tile_at(19, 1500, crsf_src), "HDG", "330D"), "telemetry: heading falls back to yaw");
    c.last_vbat_ms = 0;
    check(is(tile_at(0, 1500, crsf_src), "RX BAT", "----"), "telemetry: never-seen battery placeholder");
    check(!tile_at(0, 1500, crsf_src).stale, "telemetry: never-seen is not stale");
    c.connected = false;
    check(tile_at(12, 1500, crsf_src).stale && tile_at(12, 1500, crsf_src).value.view() == "-2.5m/s*",
          "telemetry: disconnected CRSF tile is stale");
}

void buzzer_test() {
    using namespace ferrox::logic;
    using namespace ferrox::logic::buzzer;

    static_assert(pwm_timing(FreqHz{100}) == PwmTiming{9999, 5000});
    static_assert(trim_note(TrimStep{25}).freq == FreqHz{2500});

    struct Measured {
        int tones;
        uint32_t ms;
    };
    // Counts off-to-on edges and total length until the player goes idle.
    const auto measure = [](Player& p) {
        int tones = 0;
        uint32_t ms = 0;
        bool was_on = false;
        while (p.playing() && ms < 10000) {
            const bool on = p.output().value != 0;
            if (on && !was_on) {
                ++tones;
            }
            was_on = on;
            p.tick(Millis{1});
            ++ms;
        }
        return Measured{tones, ms};
    };

    for (const ToneStyle style : {ToneStyle::Simple, ToneStyle::Rich}) {
        check(pattern_for(Cue::None, style).notes.empty(), "buzzer: None has no pattern");
        for (uint8_t c = static_cast<uint8_t>(Cue::Click); c <= static_cast<uint8_t>(Cue::ValueSelect); ++c) {
            const auto pattern = pattern_for(static_cast<Cue>(c), style);
            check(!pattern.notes.empty(), "buzzer: every cue has a pattern");
            for (const Note& n : pattern.notes) {
                check(n.freq.value >= kMinAudibleHz && n.duration.value > 0, "buzzer: preset notes are audible");
            }
        }
    }
    check(pattern_for(Cue::Click, ToneStyle::Simple).notes[0].freq == FreqHz{kBeepDefaultHz},
          "buzzer: click uses the default beep frequency");
    check(pattern_for(Cue::Welcome, ToneStyle::Simple).notes.size() == 1 &&
              pattern_for(Cue::Welcome, ToneStyle::Simple).notes[0] == pattern_for(Cue::Click, ToneStyle::Simple).notes[0],
          "buzzer: Simple welcome is a click");
    check(pattern_for(Cue::Welcome, ToneStyle::Rich).notes.size() == 4, "buzzer: Rich welcome has four notes");
    check(pattern_for(Cue::WarnBattery, ToneStyle::Simple).repeats == Repeats{1}, "buzzer: Simple battery repeats once");
    check(pattern_for(Cue::WarnCritical, ToneStyle::Simple).repeats == Repeats{2}, "buzzer: Simple critical repeats twice");
    check(pattern_for(Cue::WarnBattery, ToneStyle::Rich).repeats == Repeats{0}, "buzzer: Rich battery plays once");
    check(pattern_for(Cue::ValueStep, ToneStyle::Simple).notes.size() == 1 &&
              pattern_for(Cue::ValueStep, ToneStyle::Simple).notes[0] == pattern_for(Cue::ValueStep, ToneStyle::Rich).notes[0] &&
              pattern_for(Cue::ValueStep, ToneStyle::Rich).notes[0].freq.value == 2200 &&
              pattern_for(Cue::ValueStep, ToneStyle::Rich).notes[0].duration.value == 20,
          "buzzer: menu value step is a 2200 Hz 20 ms tone in both styles");
    check(pattern_for(Cue::ValueSelect, ToneStyle::Simple).notes.size() == 1 &&
              pattern_for(Cue::ValueSelect, ToneStyle::Simple).notes[0] == pattern_for(Cue::ValueSelect, ToneStyle::Rich).notes[0] &&
              pattern_for(Cue::ValueSelect, ToneStyle::Rich).notes[0].freq.value == 2400 &&
              pattern_for(Cue::ValueSelect, ToneStyle::Rich).notes[0].duration.value == 40,
          "buzzer: menu value select is a 2400 Hz 40 ms tone in both styles");
    check(pattern_for(Cue::InstantTrim, ToneStyle::Simple).notes.size() == 1 &&
              pattern_for(Cue::InstantTrim, ToneStyle::Simple).notes[0] == pattern_for(Cue::InstantTrim, ToneStyle::Rich).notes[0] &&
              pattern_for(Cue::InstantTrim, ToneStyle::Simple).notes[0].freq == FreqHz{2400} &&
              pattern_for(Cue::InstantTrim, ToneStyle::Simple).repeats == Repeats{2} &&
              pattern_for(Cue::InstantTrim, ToneStyle::Rich).repeats == Repeats{2},
          "buzzer: instant trim is a 2400 Hz tone twice in both styles");
    check(tone_style_from_u8(0) == ToneStyle::Simple && tone_style_from_u8(1) == ToneStyle::Rich &&
              tone_style_from_u8(7) == ToneStyle::Rich,
          "buzzer: style byte maps to Simple or Rich");

    check(trim_note(TrimStep{0}).freq == FreqHz{2000}, "buzzer: trim centre is 2000 Hz");
    check(trim_note(TrimStep{-25}).freq == FreqHz{1500}, "buzzer: trim low end is 1500 Hz");
    check(trim_note(TrimStep{127}).freq == FreqHz{2500}, "buzzer: trim high end clamps to 2500 Hz");
    check(trim_note(TrimStep{-128}).freq == FreqHz{1500}, "buzzer: trim low extreme clamps to 1500 Hz");
    check(trim_note(TrimStep{1}).duration == DurationMs{25}, "buzzer: trim beep lasts 25 ms");

    check(pwm_timing(FreqHz{2250}) == (PwmTiming{443, 222}), "buzzer: 2250 Hz PWM timing");
    check(pwm_timing(FreqHz{0}) == (PwmTiming{65534, 32767}), "buzzer: zero frequency gives the slowest period");
    check(pwm_timing(FreqHz{65535}) == (PwmTiming{14, 7}), "buzzer: high frequency clamps to 10 ticks");
    check(pwm_timing(FreqHz{1}) == (PwmTiming{65534, 32767}), "buzzer: low frequency clamps to 65535 ticks");

    check(battery_low(BatteryMv{6999}, WarnDeci{70}), "buzzer: 6.999 V is below a 7.0 V threshold");
    check(!battery_low(BatteryMv{7000}, WarnDeci{70}), "buzzer: 7.000 V is not below a 7.0 V threshold");
    check(!battery_low(BatteryMv{0}, WarnDeci{0}), "buzzer: zero threshold never warns");

    BatteryWarnTimer bt{};
    check(!bt.step(false, Millis{0}) && bt.elapsed == ElapsedMs{7000}, "buzzer: battery not low rests at 7 s");
    check(!bt.step(true, Millis{500}) && bt.elapsed == ElapsedMs{7500}, "buzzer: battery low accumulates");
    check(!bt.step(true, Millis{500}) && bt.elapsed == ElapsedMs{8000}, "buzzer: battery low reaches 8 s");
    check(bt.step(true, Millis{500}) && bt.elapsed == ElapsedMs{}, "buzzer: battery fires at 8 s and resets");

    check(rssi_alarm(Rssi{40}, true) == RssiAlarm::None, "buzzer: RSSI 40 is no alarm");
    check(rssi_alarm(Rssi{39}, true) == RssiAlarm::Low, "buzzer: RSSI 39 is low");
    check(rssi_alarm(Rssi{20}, true) == RssiAlarm::Low, "buzzer: RSSI 20 is still low");
    check(rssi_alarm(Rssi{19}, true) == RssiAlarm::Critical, "buzzer: RSSI 19 is critical");
    check(rssi_alarm(Rssi{0}, true) == RssiAlarm::Critical, "buzzer: RSSI 0 is critical");
    check(rssi_alarm(Rssi{0}, false) == RssiAlarm::None, "buzzer: no telemetry gives no RSSI alarm");

    RssiWarnTimer rt{};
    check(!rt.step(RssiAlarm::Critical, Millis{1000}) && !rt.step(RssiAlarm::Critical, Millis{1000}) &&
              !rt.step(RssiAlarm::Critical, Millis{1000}),
          "buzzer: critical RSSI is quiet for the first 3 s");
    check(rt.step(RssiAlarm::Critical, Millis{1000}), "buzzer: critical RSSI fires at 3 s");
    RssiWarnTimer rl{};
    for (int i = 0; i < 6; ++i) {
        check(!rl.step(RssiAlarm::Low, Millis{1000}), "buzzer: low RSSI is quiet before 6 s");
    }
    check(rl.step(RssiAlarm::Low, Millis{1000}), "buzzer: low RSSI fires at 6 s");
    rl.step(RssiAlarm::Low, Millis{1000});
    check(!rl.step(RssiAlarm::None, Millis{1000}) && rl.elapsed == ElapsedMs{},
          "buzzer: RSSI recovery resets the timer");

    InactivityTimer it{};
    bool early = false;
    for (int i = 0; i < 599; ++i) {
        early = it.step(false, Millis{1000}) || early;
    }
    check(!early, "buzzer: no inactivity beep before 10 minutes");
    check(!it.step(true, Millis{1000}) && it.idle == ElapsedMs{}, "buzzer: user input resets inactivity");
    InactivityTimer idle{};
    int first = 0;
    for (int i = 1; i <= 700 && first == 0; ++i) {
        if (idle.step(false, Millis{1000})) {
            first = i;
        }
    }
    check(first == 630, "buzzer: first inactivity beep 30 s after the 10 minute limit");

    PreflightTimer pf{};
    check(!pf.step(Millis{799}), "buzzer: preflight waits 800 ms");
    check(pf.step(Millis{800}), "buzzer: preflight fires at 800 ms");
    check(!pf.step(Millis{1000}) && pf.step(Millis{1600}), "buzzer: preflight repeats every 800 ms");

    check(arm_cue(ArmEdge{SwitchIndex{0}, true, false, false}) == Cue::None, "buzzer: arm switch 0 is not armable");
    check(arm_cue(ArmEdge{SwitchIndex{23}, true, false, false}) == Cue::None, "buzzer: arm switch 23 is not armable");
    check(arm_cue(ArmEdge{SwitchIndex{1}, true, false, false}) == Cue::Armed, "buzzer: arming chimes");
    check(arm_cue(ArmEdge{SwitchIndex{22}, false, true, false}) == Cue::Disarmed, "buzzer: disarming chimes");
    check(arm_cue(ArmEdge{SwitchIndex{5}, true, false, true}) == Cue::None, "buzzer: no arm chime while a menu is open");
    check(arm_cue(ArmEdge{SwitchIndex{5}, true, true, false}) == Cue::None, "buzzer: no arm chime without a change");

    Player warn{};
    warn.play_cue(Cue::WarnCritical, ToneStyle::Simple);
    const Measured critical = measure(warn);
    check(critical.tones == 3 && critical.ms == 320, "buzzer: critical warning plays 3 times with pauses");

    Player battery{};
    battery.play_cue(Cue::WarnBattery, ToneStyle::Rich);
    const Measured rich_battery = measure(battery);
    check(rich_battery.tones == 3 && rich_battery.ms == 270, "buzzer: Rich battery melody has no trailing pause");

    Player arm{};
    arm.play_cue(Cue::Armed, ToneStyle::Rich);
    const Measured armed = measure(arm);
    check(armed.tones == 2 && armed.ms == 130, "buzzer: Rich arm chime lasts 130 ms");

    Player welcome{};
    welcome.play(Pattern{tables::kWelcome, Repeats{1}});
    const Measured twice = measure(welcome);
    check(twice.tones == 7 && twice.ms == 1320,
          "buzzer: repeats play the whole list again, joined without a gap as the last pause is zero");

    constexpr std::array<Note, 2> zero_pause{{{FreqHz{2000}, DurationMs{10}, PauseMs{0}},
                                              {FreqHz{2100}, DurationMs{10}, PauseMs{0}}}};
    Player seq{};
    seq.play(Pattern{zero_pause, Repeats{0}});
    for (int i = 0; i < 10; ++i) {
        seq.tick(Millis{1});
    }
    check(seq.output() == FreqHz{2100}, "buzzer: zero pause advances to the next note");
    seq.tick(Millis{10});
    check(!seq.playing() && seq.output() == FreqHz{}, "buzzer: pattern ends after its last note");

    Player trim{};
    trim.play_trim(TrimStep{0});
    check(trim.output() == FreqHz{2000}, "buzzer: trim plays at 2000 Hz");
    const Measured trim_run = measure(trim);
    check(trim_run.tones == 1 && trim_run.ms == 25, "buzzer: trim beep is 25 ms");

    Player vario{};
    vario.set_vario(Tone{2000, 100, 200});
    check(vario.is_vario_active() && vario.output() == FreqHz{2000}, "buzzer: vario starts on");
    vario.tick(Millis{100});
    check(vario.output() == FreqHz{}, "buzzer: vario pauses after its pulse");
    vario.tick(Millis{200});
    check(vario.output() == FreqHz{2000}, "buzzer: vario pulses again after the pause");

    Player drone{};
    drone.set_vario(Tone{2000, 100, 0});
    drone.tick(Millis{1000});
    check(drone.output() == FreqHz{2000}, "buzzer: continuous vario stays on");

    Player same{};
    same.set_vario(Tone{2000, 100, 200});
    same.tick(Millis{50});
    same.set_vario(Tone{2000, 100, 200});
    same.tick(Millis{50});
    check(same.output() == FreqHz{}, "buzzer: re-setting the same vario tone keeps its phase");
    same.set_vario(Tone{2500, 100, 200});
    check(same.output() == FreqHz{2500}, "buzzer: a new vario tone restarts the pulse");

    Player over{};
    over.set_vario(Tone{2000, 100, 0});
    over.play_cue(Cue::Click, ToneStyle::Simple);
    check(over.output() == FreqHz{kBeepDefaultHz}, "buzzer: tones override the vario");
    over.tick(Millis{15});
    check(!over.playing() && over.output() == FreqHz{2000}, "buzzer: vario resumes after a tone");

    over.set_vario(vario::kSilent);
    check(!over.is_vario_active() && over.output() == FreqHz{}, "buzzer: silent vario is off");

    Player off{};
    off.set_vario(Tone{2000, 100, 0});
    off.set_enabled(false);
    off.play_cue(Cue::Click, ToneStyle::Simple);
    check(!off.enabled() && !off.playing() && off.output() == FreqHz{}, "buzzer: disabled buzzer is silent");
    off.set_enabled(true);
    check(off.output() == FreqHz{}, "buzzer: disabling clears the vario until the caller sets it again");
    off.set_vario(Tone{2000, 100, 0});
    check(off.output() == FreqHz{2000}, "buzzer: vario plays again once enabled and set");

    warn.play_cue(Cue::WarnCritical, ToneStyle::Simple);
    warn.stop();
    check(!warn.playing() && warn.output() == FreqHz{}, "buzzer: stop silences the pattern");

    calib::Wizard cw{};
    calib::start(cw);
    check(cw.cue == Cue::CalibStart, "buzzer: calibration start chimes");
    storage::RadioConfig radio = storage::default_storage().radio;
    input::Raw center{};
    center.fill(storage::kAdcCenter);
    calib::update(cw, calib::Input{0, 0, center, 10}, radio);
    check(cw.cue == Cue::None, "buzzer: cue clears on the next update");
    calib::update(cw, calib::Input{calib::kKeyOk, calib::kKeyOk, center, 10}, radio);
    check(cw.cue == Cue::CalibAdvance, "buzzer: centre capture chimes");
    calib::update(cw, calib::Input{0, 0, center, 10}, radio);
    calib::update(cw, calib::Input{calib::kKeyOk, calib::kKeyOk, center, 10}, radio);
    check(cw.cue == Cue::CalibReject, "buzzer: limits without travel rejects");
    calib::update(cw, calib::Input{calib::kKeyCancel, calib::kKeyCancel, center, 10}, radio);
    check(cw.cue == Cue::Click, "buzzer: cancel in limits clicks");

    using namespace ferrox::logic::buzzer_inputs;
    const input::Raw idle_raw{};
    Switches sw_idle{};
    ActivityTracker act{};
    act.prime(idle_raw, sw_idle);
    check(!act.step(idle_raw, sw_idle, 0), "inactivity: primed idle sample is not activity");
    input::Raw raw_small = idle_raw;
    raw_small[0] = 30;
    raw_small[6] = 40;
    check(!act.step(raw_small, sw_idle, 0), "inactivity: deflections at the thresholds are not activity");
    raw_small[0] = 61;
    check(act.step(raw_small, sw_idle, 0), "inactivity: stick past 30 counts is activity");
    check(!act.step(raw_small, sw_idle, 0), "inactivity: activity only counts the sample it changes");
    raw_small[6] = 81;
    check(act.step(raw_small, sw_idle, 0), "inactivity: pot past 40 counts is activity");
    check(act.step(raw_small, sw_idle, 0x0004) && !act.step(raw_small, sw_idle, 0), "inactivity: any key is activity");
    raw_small[2] = 1000;
    check(act.step(raw_small, sw_idle, 0), "inactivity: channel 2 movement is activity");
    raw_small[7] = 0;
    raw_small[4] = 2000;
    check(!act.step(raw_small, sw_idle, 0), "inactivity: unused raw channels are ignored");
    Switches sw_moved = sw_idle;
    sw_moved.sa = SwitchPos::Down;
    check(act.step(raw_small, sw_moved, 0), "inactivity: a switch change is activity");

    InactivityTimer idle_timer{};
    int beeps = 0;
    for (uint32_t t = 0; t < 700000; t += 20) {
        beeps += idle_timer.step(false, Millis{20}) ? 1 : 0;
    }
    check(beeps > 0 && beeps <= 3, "inactivity: beeps once the 10 minute limit passes, with a 30 s cadence");
    check(!idle_timer.step(true, Millis{20}) && idle_timer.idle.value == 0, "inactivity: activity resets the timer");

    check(!vario_switch_active(0, sw_idle) && vario_switch_active(1, sw_idle),
          "vario: setting 0 is off and setting 1 is always on");
    Switches sw_arm{};
    sw_arm.sa = SwitchPos::Down;
    check(vario_switch_active(2, sw_arm) == mixer::eval_arm_switch(1, sw_arm) &&
              vario_switch_active(23, sw_arm) == mixer::eval_arm_switch(22, sw_arm),
          "vario: settings 2..=23 map to arm conditions 1..=22");
    check(!vario_switch_active(24, sw_arm), "vario: out-of-range settings are off");

    PreflightTimer preflight_timer{};
    check(!preflight_timer.step(Millis{799}) && preflight_timer.step(Millis{800}) && !preflight_timer.step(Millis{1000}),
          "buzzer: preflight warns every 800 ms");
}

void trim_controller_test() {
    using logic::trim::Axis;
    using logic::trim::Controller;
    using logic::trim::Event;
    using Kind = Event::Kind;
    using logic::vario::Millis;

    const Controller defaults{};
    check(defaults.values.roll == 0 && defaults.values.pitch == 0 && defaults.values.throttle == 0 &&
              defaults.values.yaw == 0 && !defaults.last_active && !defaults.throttle_enabled,
          "trim: defaults are centred, no active axis and throttle locked");

    check(logic::trim::axis_letter(Axis::Roll) == 'A' && logic::trim::axis_letter(Axis::Pitch) == 'E' &&
              logic::trim::axis_letter(Axis::Throttle) == 'T' && logic::trim::axis_letter(Axis::Yaw) == 'R',
          "trim: axis footer letters match Rust format_trim");
    check(logic::format::format_trim(logic::trim::axis_letter(Axis::Roll), 5).view() == "TRM A:+05",
          "trim: footer text shows the signed value");

    Controller c{};
    check(c.update(0x01, Millis{0}) == Event{Kind::Step, 1}, "trim: roll+ press steps to +1");
    check(c.values.roll == 1 && c.last_active == Axis::Roll && c.active_ms == Controller::kActiveMs,
          "trim: stepped axis is shown for 1.5 s");
    check(logic::trim::value_of(c.values, Axis::Roll) == 1, "trim: value_of reads the stepped axis");

    bool quiet = true;
    for (int i = 0; i < 6; ++i) {
        quiet = quiet && c.update(0x01, Millis{50}) == Event{};
    }
    check(quiet && c.values.roll == 1, "trim: no auto-repeat before the 350 ms hold delay");
    check(c.update(0x01, Millis{50}) == Event{}, "trim: hold delay reached, first repeat waits 90 ms");
    check(c.update(0x01, Millis{50}) == Event{Kind::Step, 2}, "trim: auto-repeat steps every 90 ms");
    check(c.update(0x01, Millis{50}) == Event{}, "trim: repeat timer restarts after a repeat");

    check(c.update(0, Millis{50}) == Event{} && c.hold_ms == 0 && c.repeat_ms == 0,
          "trim: release clears hold and repeat state");
    check(c.update(0x01, Millis{0}) == Event{Kind::Step, 3}, "trim: a new press steps at once after release");
    c.update(0, Millis{0});

    check(c.update(0x06, Millis{0}) == Event{Kind::Step, 2} && c.values.roll == 2 && c.values.pitch == 0,
          "trim: simultaneous presses use the lowest bit only");
    c.update(0, Millis{0});

    check(c.update(0x100, Millis{0}) == Event{} && c.values.roll == 2, "trim: key bits above 7 are ignored");

    c.values.pitch = 1;
    check(c.update(0x08, Millis{0}) == Event{Kind::Center} && c.values.pitch == 0,
          "trim: a step to zero plays the centre cue");
    c.update(0, Millis{0});

    c.values.yaw = 25;
    c.last_active.reset();
    c.active_ms = 0;
    check(c.update(0x40, Millis{0}) == Event{Kind::Limit} && c.values.yaw == 25 && !c.last_active,
          "trim: stepping past +25 plays the limit cue and changes nothing");
    c.update(0, Millis{0});
    c.values.yaw = -25;
    check(c.update(0x80, Millis{0}) == Event{Kind::Limit} && c.values.yaw == -25,
          "trim: stepping past -25 plays the limit cue");
    c.update(0, Millis{0});

    check(c.update(0x10, Millis{0}) == Event{Kind::Limit} && c.values.throttle == 0,
          "trim: throttle trim is locked while disabled");
    c.update(0, Millis{0});
    c.throttle_enabled = true;
    check(c.update(0x10, Millis{0}) == Event{Kind::Step, 1} && c.values.throttle == 1,
          "trim: throttle trim steps once enabled");
    c.update(0, Millis{0});

    const int8_t roll_before = c.values.roll;
    check(c.update(0x43, Millis{0}) == Event{} && c.values.roll == roll_before,
          "trim: DFU combination locks trim processing");
    check(c.update(0x01, Millis{0}).kind == Kind::Step,
          "trim: a key still held after the DFU lockout steps as a new press");
    c.update(0, Millis{0});

    c.last_active = Axis::Pitch;
    c.active_ms = Controller::kActiveMs;
    check(c.update(0, Millis{1000}) == Event{} && c.last_active == Axis::Pitch && c.active_ms == 500,
          "trim: active timer counts down");
    check(c.update(0, Millis{500}) == Event{} && !c.last_active && c.active_ms == 0,
          "trim: active axis clears when the timer runs out");
}

void trim_autosave_test() {
    using logic::trim::AutoSave;
    using logic::trim::TrimValues;
    using logic::vario::Millis;

    const Millis tick_ms{20};
    // Ticks until the trigger fires, counting the firing tick. Returns 0 if it does not fire within 1000 ticks.
    const auto ticks_until_save = [&](AutoSave& s, const TrimValues& v, bool armed) {
        for (int i = 1; i <= 1000; ++i) {
            if (s.tick(v, armed, tick_ms)) {
                return i;
            }
        }
        return 0;
    };

    AutoSave s{};
    TrimValues v{};
    s.reset(v);
    check(!s.tick(v, false, tick_ms) && !s.dirty && s.cooldown_ms == 0, "trim save: no change, no save");

    v.roll = 1;
    check(!s.tick(v, false, tick_ms) && s.dirty && s.cooldown_ms == AutoSave::kDelayMs,
          "trim save: a change marks the trims dirty and starts the 2000 ms cooldown");
    check(ticks_until_save(s, v, false) == 100, "trim save: saves 2000 ms after the change, not earlier");
    s.saved(true);
    check(!s.dirty, "trim save: a successful write clears dirty");

    v.pitch = -3;
    s.tick(v, false, tick_ms);
    for (int i = 0; i < 50; ++i) {
        s.tick(v, false, tick_ms);
    }
    v.pitch = -4;
    s.tick(v, false, tick_ms);
    check(ticks_until_save(s, v, false) == 100, "trim save: a second change restarts the cooldown");
    s.saved(true);

    v.yaw = 5;
    s.tick(v, false, tick_ms);
    check(ticks_until_save(s, v, true) == 0 && s.dirty, "trim save: no save while armed");
    check(ticks_until_save(s, v, true) == 0 && s.dirty, "trim save: still no save while armed");
    check(s.tick(v, false, tick_ms) && s.dirty, "trim save: disarming saves the pending trims at once");
    s.saved(false);
    check(s.dirty, "trim save: a failed write keeps the trims dirty");
    check(s.tick(v, false, tick_ms), "trim save: a failed write is retried on the next tick");
    s.saved(true);

    int writes = 0;
    for (int i = 0; i < 5000; ++i) {
        if (s.tick(v, false, tick_ms)) {
            ++writes;
            s.saved(true);
        }
    }
    check(writes == 0, "trim save: idle loop never writes again");

    int burst_writes = 0;
    for (int i = 0; i < 1000; ++i) {
        v.roll = static_cast<int8_t>(i % 2 == 0 ? 2 : 3);
        if (s.tick(v, false, tick_ms)) {
            ++burst_writes;
            s.saved(true);
        }
    }
    check(burst_writes == 0, "trim save: continuous changes never write while the trims keep moving");
    check(ticks_until_save(s, v, false) == 100, "trim save: one write after a burst ends");
    s.saved(true);

    v.roll = 9;
    s.reset(v);
    check(!s.tick(v, false, tick_ms) && !s.dirty, "trim save: reset to loaded trims is not a change");
    s.reset(TrimValues{});
    check(!s.tick(v, false, tick_ms) && s.dirty, "trim save: a change after reset starts the cooldown");
    check(ticks_until_save(s, v, false) == 100, "trim save: a change after reset is saved after 2000 ms");
}

void instant_trim_test() {
    using logic::trim::Axis;
    using logic::trim::Controller;
    using logic::trim::InstantEdge;
    using logic::trim::InstantResult;
    using logic::trim::InstantSticks;
    using logic::trim::instant_step;
    using logic::trim::instant_switch_down;
    using logic::trim::kInstantTrimRaw;
    namespace storage = logic::storage;
    namespace input = logic::input;

    check(instant_step(0) == 0 && instant_step(19) == 0 && instant_step(20) == 1 && instant_step(59) == 1 &&
              instant_step(60) == 2 && instant_step(-19) == 0 && instant_step(-20) == -1 &&
              instant_step(-60) == -2 && instant_step(1000) == 25 && instant_step(-1000) == -25,
          "instant trim: stick steps round half away from zero, 40 counts per step");

    // Sticks deflected: roll +10, pitch -5, yaw 0. Roll is the axis shown in the footer.
    Controller c{};
    check(c.instant_trim(InstantSticks{400, -200, 0}) == InstantResult::Adjusted && c.values.roll == 10 &&
              c.values.pitch == -5 && c.values.yaw == 0 && c.last_active == Axis::Roll &&
              c.active_ms == Controller::kActiveMs,
          "instant trim: deflected sticks add the rounded steps and show the roll footer");

    Controller pitch_first{};
    check(pitch_first.instant_trim(InstantSticks{0, 100, 50}) == InstantResult::Adjusted &&
              pitch_first.values.pitch == 3 && pitch_first.values.yaw == 1 &&
              pitch_first.last_active == Axis::Pitch,
          "instant trim: footer picks pitch before yaw when roll is unchanged");

    Controller yaw_only{};
    check(yaw_only.instant_trim(InstantSticks{0, 0, -50}) == InstantResult::Adjusted &&
              yaw_only.values.yaw == -1 && yaw_only.last_active == Axis::Yaw,
          "instant trim: footer shows yaw when only yaw moves");

    Controller clamp{};
    clamp.values = {24, -24, 0, 0};
    check(clamp.instant_trim(InstantSticks{1000, -1000, 0}) == InstantResult::Adjusted && clamp.values.roll == 25 &&
              clamp.values.pitch == -25,
          "instant trim: steps are clamped to +-25");

    Controller throttle{};
    throttle.throttle_enabled = true;
    throttle.values.throttle = 7;
    throttle.instant_trim(InstantSticks{400, 0, 0});
    check(throttle.values.throttle == 7, "instant trim: throttle is never changed");

    // Sticks centred with trims set: all three reset, footer shows roll, and the Reset cue is used.
    Controller reset{};
    reset.values = {5, -3, 9, 4};
    check(reset.instant_trim(InstantSticks{0, 0, 0}) == InstantResult::Reset && reset.values.roll == 0 &&
              reset.values.pitch == 0 && reset.values.yaw == 0 && reset.values.throttle == 9 &&
              reset.last_active == Axis::Roll && reset.active_ms == Controller::kActiveMs,
          "instant trim: centred sticks reset roll, pitch and yaw, keep throttle");

    // Small deflection inside the rounding dead zone with trims set counts as centred, as in Rust.
    Controller dead_zone{};
    dead_zone.values = {2, 0, 0, 0};
    check(dead_zone.instant_trim(InstantSticks{19, 0, 0}) == InstantResult::Reset && dead_zone.values.roll == 0,
          "instant trim: deflection that rounds to zero resets set trims");

    Controller idle{};
    check(idle.instant_trim(InstantSticks{0, 0, 0}) == InstantResult::None && idle.active_ms == 0 &&
              !idle.last_active,
          "instant trim: centred sticks with zero trims do nothing");

    InstantEdge edge{};
    check(edge.update(true) && !edge.update(true) && !edge.update(false) && edge.update(true),
          "instant trim: fires once per press, on the rising edge");

    storage::RadioConfig radio = storage::default_storage().radio;
    input::Raw raw{};
    check(!instant_switch_down(radio, raw), "instant trim: default modes never fire");

    radio.adc_modes[4] = 6;  // VRA as InstantTrim, reads raw index 6
    raw[6] = kInstantTrimRaw - 1;
    check(!instant_switch_down(radio, raw), "instant trim: switch below 2048 is up");
    raw[6] = kInstantTrimRaw;
    check(instant_switch_down(radio, raw), "instant trim: switch at 2048 is down");

    radio.adc_modes[6] = 6;  // VRC as InstantTrim, reads raw index 11 (external ADC)
    raw = {};
    raw[11] = 4000;
    check(!instant_switch_down(radio, raw), "instant trim: external ADC channels are ignored without ext ADC");
    radio.ext_adc = 1;
    check(instant_switch_down(radio, raw), "instant trim: external ADC channels count with ext ADC");
}

void flight_timer_test() {
    using logic::flight_timer::Config;
    using logic::flight_timer::Input;
    using logic::flight_timer::Modal;
    using logic::flight_timer::Seconds;
    using logic::flight_timer::State;
    using logic::buzzer::Cue;
    using logic::vario::Millis;

    constexpr auto kOff = Cue::None;
    static_assert(logic::format::format_timer(300, true).view() == "+05:00");
    static_assert(logic::format::format_timer(65, false).view() == "01:05");
    static_assert(logic::format::format_timer(6000, false).view() == "99:00");
    check(logic::format::format_timer(0, false).view() == "00:00", "timer format: zero");

    auto make = [](uint16_t limit, uint8_t source, uint8_t arm, uint32_t dt_ms) {
        Input in{};
        in.model = {0};
        in.config = Config{.limit = {limit}, .source = {source}, .arm = {arm}};
        in.dt = Millis{dt_ms};
        return in;
    };
    // Always-running source (3), no arm switch, so the tick logic is isolated.
    const Config always{.limit = {3}, .source = {3}, .arm = {0}};
    auto tick = [](State& s, Input in) { return logic::flight_timer::update(s, in); };

    {
        State s{};
        Input in = make(3, 3, 0, 1000);
        in.model = {1};
        check(tick(s, in) == Cue::TimerCountdown && s.remaining.value == 2, "timer: first tick resets to limit");
        check(logic::flight_timer::display(s, always) == logic::flight_timer::Display{{2}, false},
              "timer: display shows remaining");
    }

    {
        State s{};
        Input in = make(3, 3, 0, 1000);
        check(tick(s, in) == Cue::TimerCountdown, "timer: countdown beep at 2 s");
        check(tick(s, in) == Cue::TimerCountdown, "timer: countdown beep at 1 s");
        check(tick(s, in) == Cue::TimerElapsed && s.remaining.value == 0, "timer: elapsed alarm at zero");
        check(logic::flight_timer::display(s, always) == logic::flight_timer::Display{{0}, true},
              "timer: expired display shows zero with plus");
        check(tick(s, in) == kOff && s.elapsed.value == 1, "timer: counts up after expiry");
        check(logic::flight_timer::display(s, always) == logic::flight_timer::Display{{1}, true},
              "timer: expired display counts elapsed");
    }

    {
        State s{};
        Input in = make(120, 3, 0, 1000);
        Cue last = kOff;
        bool minute_early = false;
        for (int i = 1; i <= 60; ++i) {
            const Cue c = tick(s, in);
            if (i < 60 && c == Cue::TimerMinute) {
                minute_early = true;
            }
            last = c;
        }
        check(!minute_early && last == Cue::TimerMinute && s.remaining.value == 60,
              "timer: minute beep at each full minute");
    }

    {
        State s{};
        Input in = make(0, 3, 0, 1000);
        check(tick(s, in) == kOff && s.elapsed.value == 1, "timer: no limit counts elapsed");
        check(logic::flight_timer::display(s, Config{.limit = {0}, .source = {3}, .arm = {0}}) ==
                  logic::flight_timer::Display{{1}, false},
              "timer: no limit shows elapsed without expired");
    }

    {
        State s{};
        check(!logic::flight_timer::display(s, Config{}).has_value(), "timer: source off hides footer");
        Input in = make(10, 1, 0, 1000);
        in.throttle = -1000;
        in.model = {0};
        tick(s, in);
        check(s.remaining.value == 10, "timer: throttle source stays idle at low stick");
        in.throttle = 0;
        tick(s, in);
        check(s.remaining.value == 9, "timer: throttle source runs above low stick");
    }

    {
        // Switch-based arm: pick a switch state that is armed and one that is not.
        logic::Switches up{};
        up.sa = logic::SwitchPos::Up;
        logic::Switches down{};
        down.sa = logic::SwitchPos::Down;
        const bool up_armed = logic::flight::is_armed(1, up);
        const logic::Switches on_sw = up_armed ? up : down;
        const logic::Switches off_sw = up_armed ? down : up;

        State s{};
        Input in = make(10, 3, 1, 1000);
        in.switches = off_sw;
        tick(s, in);
        check(s.remaining.value == 10, "timer: disarmed holds the timer");

        in.switches = on_sw;
        in.was_armed = false;
        tick(s, in);
        check(s.remaining.value == 9, "timer: arming resets then runs");
        in.was_armed = true;
        tick(s, in);
        tick(s, in);
        check(s.remaining.value == 7, "timer: armed ticks count down");

        in.switches = off_sw;
        in.was_armed = true;
        tick(s, in);
        check(s.remaining.value == 7 && !s.latched.value, "timer: disarming stops the count");

        in.switches = on_sw;
        in.was_armed = false;
        tick(s, in);
        check(s.remaining.value == 9, "timer: rearming resets to the limit");
    }

    {
        State s{};
        Input in = make(300, 0, 0, 500);
        in.model = {0};
        tick(s, in);
        s.remaining = {50};
        in.cancel_held = true;
        check(tick(s, in) == kOff && s.cancel_hold_ms == 500, "timer cancel: hold accumulates");
        check(logic::flight_timer::modal(s) == Modal{false, 50}, "timer cancel: progress modal at half hold");
        check(tick(s, in) == Cue::TimerReset && s.remaining.value == 300, "timer cancel: hold 1 s resets");
        check(s.toast_ms == 800 && s.cooldown_ms == 1500 && s.cancel_waiting_release,
              "timer cancel: reset starts toast, cooldown and release guard");
        check(logic::flight_timer::modal(s) == Modal{true, 100}, "timer cancel: confirmation toast shown");
        check(tick(s, in) == kOff && s.cancel_hold_ms == 0, "timer cancel: held key does not repeat");
        check(tick(s, in) == kOff, "timer cancel: still held during cooldown");
        in.cancel_held = false;
        check(tick(s, in) == kOff && !s.cancel_waiting_release, "timer cancel: release clears the guard");
        in.cancel_held = true;
        check(tick(s, in) == kOff && s.cancel_hold_ms == 500, "timer cancel: second hold waits out cooldown");
        check(tick(s, in) == Cue::TimerReset, "timer cancel: second hold resets after cooldown");
    }

    {
        State s{};
        Input in = make(300, 0, 0, 500);
        in.model = {0};
        in.menu_active = true;
        in.cancel_held = true;
        tick(s, in);
        check(tick(s, in) == kOff && s.cancel_hold_ms == 0 && s.toast_ms == 0, "timer cancel: menu blocks the hold");
    }

    {
        State s{};
        Input in = make(300, 0, 0, 80);
        in.model = {0};
        in.cancel_held = true;
        tick(s, in);
        check(!logic::flight_timer::modal(s).has_value(), "timer cancel: modal hidden for short taps");
        in.dt = Millis{500};
        tick(s, in);
        check(logic::flight_timer::modal(s) == Modal{false, 58}, "timer cancel: modal progress rounds down");
    }
}

void bind_test() {
    using logic::bind::Action;
    using logic::bind::Controller;
    using logic::bind::Input;
    using logic::bind::Link;
    using logic::buzzer::Cue;
    using logic::vario::Millis;

    constexpr uint16_t kBind = logic::kKeyBind;
    constexpr uint16_t kCancel = logic::safety::kKeyCancel;
    constexpr Link kLinked{true, false};
    constexpr Link kBinding{true, true};

    static_assert(logic::bind::binding_shown(kBinding));
    static_assert(!logic::bind::binding_shown({false, true}));
    static_assert(!logic::bind::binding_shown(kLinked));

    auto make = [](Link link, uint16_t keys, uint16_t pressed, uint32_t dt_ms, bool menu = false) {
        Input in{};
        in.link = link;
        in.menu_active = menu;
        in.keys = keys;
        in.pressed = pressed;
        in.dt = Millis{dt_ms};
        return in;
    };

    Controller c{};
    check(logic::bind::update(c, make(kLinked, kBind, kBind, 500)).action == Action::None, "bind: no start at 500 ms");
    check(logic::bind::update(c, make(kLinked, kBind, 0, 499)).action == Action::None, "bind: no start at 999 ms");
    const auto started = logic::bind::update(c, make(kLinked, kBind, 0, 1));
    check(started.action == Action::Start && started.cue == ferrox::logic::buzzer::Cue::BindHold,
          "bind: starts at 1000 ms hold with the bind hold chirp");
    check(logic::bind::update(c, make(kLinked, kBind, 0, 300)).action == Action::None, "bind: start not repeated");
    logic::bind::update(c, make(kLinked, 0, 0, 10));
    check(c == Controller{}, "bind: release resets hold");

    Controller menu_c{};
    logic::bind::update(menu_c, make(kLinked, kBind, kBind, 600));
    logic::bind::update(menu_c, make(kLinked, kBind, 0, 2000, true));
    check(menu_c == Controller{}, "bind: menu open resets hold");
    check(logic::bind::update(menu_c, make(kLinked, kBind, 0, 500)).action == Action::None,
          "bind: menu open then hold restarts from zero");

    Controller crsf_c{};
    check(logic::bind::update(crsf_c, make({false, false}, kBind, kBind, 2000)).action == Action::None,
          "bind: no start without AFHDS 2A");

    Controller stop_c{};
    const auto stop_bind = logic::bind::update(stop_c, make(kBinding, 0, kBind, 10));
    check(stop_bind == logic::bind::Event{Action::Stop, Cue::Click}, "bind: BIND press stops binding with click");
    const auto stop_cancel = logic::bind::update(stop_c, make(kBinding, kCancel, 0, 10));
    check(stop_cancel == logic::bind::Event{Action::Stop, Cue::Click}, "bind: CANCEL stops binding with click");
    const auto idle = logic::bind::update(stop_c, make(kBinding, 0, 0, 10));
    check(idle == logic::bind::Event{}, "bind: idle while binding does nothing");

    // Footer precedence: bind > trim > timer > default.
    logic::input::InputState in{};
    logic::storage::RadioStorage st{};
    logic::trim::Controller trims{};
    std::array<uint16_t, logic::kNumChannels> chs{};
    using logic::dashboard::FooterKind;
    {
        logic::dashboard::View v{in, st, trims, chs, false, {}, true, false, false, 0};
        check(logic::dashboard::footer_kind(v) == FooterKind::Bind, "footer: bind shown while binding");
        trims.last_active = logic::trim::Axis::Roll;
        check(logic::dashboard::footer_kind(v) == FooterKind::Bind, "footer: bind beats active trim");
    }
    {
        logic::dashboard::View v{in, st, trims, chs, false, {}, false, false, false, 0};
        check(logic::dashboard::footer_kind(v) == FooterKind::Trim, "footer: trim shown when not binding");
        trims.last_active.reset();
        check(logic::dashboard::footer_kind(v) == FooterKind::Default, "footer: default when idle");
    }
}

void model_copy_test() {
    namespace mc = logic::model_copy;
    namespace mn = logic::menu;
    namespace st = logic::storage;
    using Dev = test_eeprom::FakeEeprom;
    using mn::Action;

    // Picker: opens on the slot after the source and wraps in both directions.
    check(mc::open(mc::SrcSlot{2}).dst == mc::DstSlot{3}, "copy picker must open on source + 1");
    check(mc::open(mc::SrcSlot{19}).dst == mc::DstSlot{0}, "copy picker must wrap from the last slot");
    mc::Picker w = mc::open(mc::SrcSlot{0});
    mc::update(w, mc::Keys{.down = true});
    check(w.dst == mc::DstSlot{0}, "down from slot 1 must reach slot 0");
    mc::update(w, mc::Keys{.down = true});
    check(w.dst == mc::DstSlot{19}, "down from slot 0 must wrap to slot 19");
    mc::update(w, mc::Keys{.up = true});
    check(w.dst == mc::DstSlot{0}, "up from slot 19 must wrap to slot 0");
    check(mc::update(w, mc::Keys{.ok = true}) == mc::Outcome::Copy, "OK must confirm the copy");
    check(mc::update(w, mc::Keys{.cancel = true}) == mc::Outcome::Cancelled, "ESC must cancel the copy");
    check(mc::update(w, mc::Keys{.up = true, .cancel = true}) == mc::Outcome::Cancelled,
          "ESC must win over a same-tick move");
    check(mc::update(w, mc::Keys{}) == mc::Outcome::Choosing, "idle tick must keep choosing");

    check(std::string_view{mc::target_label(mc::DstSlot{0}).view()} == "Copy -> M01", "label must show slot 1 as M01");
    check(std::string_view{mc::target_label(mc::DstSlot{19}).view()} == "Copy -> M20",
          "label must show slot 20 as M20");

    // Copy writes the in-memory active model to the destination slot only.
    st::RadioStorage rs = st::default_storage();
    rs.radio.active_model = 2;
    rs.active_model.name = {'C', 'O', 'P', 'Y', ' ', ' ', ' ', ' ', ' ', ' '};
    rs.active_model.timer_secs = 90;
    check(mc::active_slot(rs) == mc::SrcSlot{2}, "active slot must follow the radio config");
    rs.radio.active_model = 99;
    check(mc::active_slot(rs) == mc::SrcSlot{0}, "out-of-range active slot must clamp to 0");
    rs.radio.active_model = 2;

    check(!mc::copy_model<Dev>(rs, mc::SrcSlot{3}, mc::DstSlot{5}), "copy from a non-active slot must fail");
    check(!mc::copy_model<Dev>(rs, mc::SrcSlot{2}, mc::DstSlot{20}), "copy to an out-of-range slot must fail");
    check(mc::copy_model<Dev>(rs, mc::SrcSlot{2}, mc::DstSlot{5}), "copy must succeed");
    st::ModelConfig got{};
    check(st::load_slot<Dev>(st::model_slot_addr(5).value(), got) == st::Status::Ok && got.name[0] == 'C' &&
              got.timer_secs == 90,
          "copy must write the active model to the destination slot");
    check(rs.radio.active_model == 2 && rs.active_model.name[0] == 'C',
          "copy must leave the active model unchanged");

    Dev::fail_writes = true;
    check(!mc::copy_model<Dev>(rs, mc::SrcSlot{2}, mc::DstSlot{6}), "failing EEPROM must report copy failure");
    Dev::fail_writes = false;

    // Menu flow: Copy row is reached by wrapping up from Name, OK opens the picker, OK confirms.
    mn::State s{};
    s.screen = mn::Screen::ModelSetup;
    auto press = [&](uint16_t key, uint32_t now) {
        const mn::Event e = mn::update(s, mn::Input{key, key, now}, rs);
        mn::update(s, mn::Input{0, 0, now + 1}, rs);
        return e;
    };
    press(mn::kKeyUp, 10);
    check(s.setup.selected == 8 && s.setup.scroll == 5, "up from Name must wrap to the Reset row and scroll");
    press(mn::kKeyUp, 15);
    press(mn::kKeyUp, 16);
    press(mn::kKeyUp, 17);
    press(mn::kKeyUp, 18);
    check(s.setup.selected == 4, "up from Reset must reach the Copy row");
    const mn::Event open_ev = press(mn::kKeyOk, 20);
    check(open_ev.action == Action::None && s.editing && s.copy.dst == mc::DstSlot{3},
          "OK on Copy must open the picker at source + 1");
    press(mn::kKeyUp, 30);
    check(s.copy.dst == mc::DstSlot{4}, "up on the picker must move the destination");
    const mn::Event cancel_ev = press(mn::kKeyCancel, 40);
    check(cancel_ev.action == Action::None && !s.editing, "ESC on the picker must cancel without a copy");
    press(mn::kKeyOk, 50);
    press(mn::kKeyUp, 60);
    const mn::Event copy_ev = press(mn::kKeyOk, 70);
    check(copy_ev.action == Action::CopyActiveModel && copy_ev.index == 4 && !s.editing,
          "OK on the picker must request a copy to the chosen slot");
}

void protocol_reset_test() {
    namespace pr = logic::protocol;
    namespace mn = logic::menu;
    namespace st = logic::storage;
    using mn::Action;

    check(pr::from_raw(0) == pr::Protocol::Afhds2a && pr::from_raw(1) == pr::Protocol::Crsf,
          "stored 0 and 1 must map to AFHDS2A and CRSF");
    check(pr::from_raw(7) == pr::Protocol::Afhds2a, "out-of-range stored protocol must fall back to AFHDS2A");
    check(pr::to_raw(pr::Protocol::Crsf) == 1 && pr::toggle(pr::toggle(pr::Protocol::Crsf)) == pr::Protocol::Crsf,
          "toggle must alternate and round-trip");
    check(std::string_view{pr::label(pr::Protocol::Afhds2a)} == "AFHDS2A" &&
              std::string_view{pr::label(pr::Protocol::Crsf)} == "CRSF",
          "labels must match the Rust menu text");
    check(pr::link_for(pr::Protocol::Crsf) == pr::Link{true, true},
          "CRSF must enable the UART and mute the A7105");
    check(pr::link_for(pr::Protocol::Afhds2a) == pr::Link{false, false},
          "AFHDS2A must disable the UART and leave the A7105 active");

    const st::ModelConfig def = st::default_model_config(3);
    st::ModelConfig reset = pr::reset_model(pr::ModelSlot{3});
    check(reset.rf_protocol == 0 && reset.rx_id == 0 && reset.timer_secs == def.timer_secs &&
              reset.name == def.name,
          "reset must restore the factory model for the slot");

    // Menu flow: Protocol row opens the sub-screen, OK edits, UP/DOWN toggle, OK saves.
    st::RadioStorage rs = st::default_storage();
    rs.radio.active_model = 2;
    rs.active_model.rf_protocol = 0;
    mn::State s{};
    s.screen = mn::Screen::ModelSetup;
    s.setup.selected = 7;
    s.setup.scroll = 4;
    auto press = [&](uint16_t key, uint32_t now) {
        const mn::Event e = mn::update(s, mn::Input{key, key, now}, rs);
        mn::update(s, mn::Input{0, 0, now + 1}, rs);
        return e;
    };

    press(mn::kKeyOk, 10);
    check(s.screen == mn::Screen::ProtocolSetup && !s.editing, "OK on Protocol must open the sub-screen");
    press(mn::kKeyOk, 20);
    check(s.editing && rs.active_model.rf_protocol == 0, "OK on the sub-screen must start editing");
    press(mn::kKeyUp, 30);
    check(rs.active_model.rf_protocol == 1, "UP while editing must switch to CRSF");
    press(mn::kKeyDown, 40);
    check(rs.active_model.rf_protocol == 0, "DOWN while editing must switch back to AFHDS2A");
    press(mn::kKeyUp, 50);
    const mn::Event save_ev = press(mn::kKeyOk, 60);
    check(save_ev.action == Action::SaveActiveModel && !s.editing && rs.active_model.rf_protocol == 1,
          "second OK must save the CRSF selection and stop editing");
    press(mn::kKeyCancel, 70);
    check(s.screen == mn::Screen::ModelSetup && s.setup.selected == 7 && s.setup.scroll == 4,
          "ESC must return to Model Setup on the Protocol row");

    // Cancel while editing keeps the toggled value in RAM without saving, then cancel leaves the sub-screen.
    s.setup.selected = 7;
    press(mn::kKeyOk, 80);
    press(mn::kKeyOk, 90);
    press(mn::kKeyUp, 100);
    const mn::Event cancel_edit = press(mn::kKeyCancel, 110);
    check(cancel_edit.action == Action::None && !s.editing && rs.active_model.rf_protocol == 0,
          "ESC while editing must stop editing without saving");
    check(s.screen == mn::Screen::ProtocolSetup, "ESC while editing must stay on the sub-screen");
    press(mn::kKeyUp, 115);
    check(rs.active_model.rf_protocol == 0, "UP after leaving edit must not change the protocol");
    press(mn::kKeyCancel, 120);
    check(s.screen == mn::Screen::ModelSetup, "ESC on the sub-screen must return to Model Setup");

    // Reset row restores the slot defaults in place and asks the caller to apply them.
    rs.active_model.rf_protocol = 1;
    rs.active_model.rx_id = 0x1234;
    rs.active_model.name = {'X', 'X', 'X', 0, 0, 0, 0, 0, 0, 0};
    s.setup.selected = 8;
    const mn::Event reset_ev = press(mn::kKeyOk, 130);
    check(reset_ev.action == Action::ResetActiveModel && rs.active_model.rf_protocol == 0 &&
              rs.active_model.rx_id == 0 && rs.active_model.name == st::default_model_config(2).name,
          "OK on Reset must restore the active model defaults and request a reset");

    s.setup.selected = 8;
    press(mn::kKeyUp, 140);
    check(s.setup.selected == 7, "UP from Reset must reach the Protocol row");
}

void vario_setup_test() {
    namespace mn = logic::menu;
    namespace st = logic::storage;
    namespace lb = logic::labels;
    namespace fmt = logic::format;
    using mn::Action;

    check(std::string_view{lb::vario_sink_mode_label(0)} == "MUTE" &&
              std::string_view{lb::vario_sink_mode_label(1)} == "PULSE" &&
              std::string_view{lb::vario_sink_mode_label(2)} == "DRONE",
          "sink mode labels must match the Rust vario screen");
    check(std::string_view{lb::vario_motor_mute_label(1)} == "ENABLED" &&
              std::string_view{lb::vario_motor_mute_label(0)} == "DISABLED",
          "motor mute labels must match the Rust vario screen");
    check(fmt::format_deadband_ms(10, false).view() == "+1.0m/s" &&
              fmt::format_deadband_ms(5, true).view() == "-0.5m/s",
          "deadband text must show tenths of m/s with climb and sink signs");

    // Deadband and sink mode cycles, including Rust's odd-start sink sequence.
    check(mn::next_climb_deadband(10) == 1 && mn::next_climb_deadband(1) == 2, "climb deadband must wrap 1..10");
    check(mn::next_sink_deadband(20) == 2 && mn::next_sink_deadband(5) == 7 && mn::next_sink_deadband(19) == 21,
          "sink deadband must step by 2 and wrap as Rust does");

    st::RadioStorage rs = st::default_storage();
    rs.radio.active_model = 2;
    rs.active_model.vario_switch = 0;
    rs.active_model.vario_climb_deadband = 10;
    rs.active_model.vario_sink_deadband = 5;
    rs.active_model.vario_sink_mode = 1;
    rs.active_model.vario_motor_mute = 1;
    mn::State s{};
    s.screen = mn::Screen::ModelSetup;
    s.setup.selected = 5;
    s.setup.scroll = 2;
    auto press = [&](uint16_t key, uint32_t now) {
        const mn::Event e = mn::update(s, mn::Input{key, key, now}, rs);
        mn::update(s, mn::Input{0, 0, now + 1}, rs);
        return e;
    };

    press(mn::kKeyOk, 10);
    check(s.screen == mn::Screen::VarioSetup && s.vario.selected == 0 && s.vario.scroll == 0 && !s.editing,
          "OK on Vario must open the sub-screen at the top");
    check(rs.active_model.vario_switch == 0, "switch row must start from OFF");

    // Climb deadband: each OK cycles and saves.
    press(mn::kKeyDown, 20);
    const mn::Event climb_ev = press(mn::kKeyOk, 30);
    check(climb_ev.action == Action::SaveActiveModel && rs.active_model.vario_climb_deadband == 1,
          "OK on climb deadband must wrap to 0.1 m/s and save");
    press(mn::kKeyDown, 40);
    press(mn::kKeyOk, 50);
    check(rs.active_model.vario_sink_deadband == 7, "OK on sink deadband must step to 0.7 m/s");

    // Sink mode cycles Pulse -> Drone -> Mute; motor mute toggles.
    press(mn::kKeyDown, 60);
    press(mn::kKeyOk, 70);
    check(rs.active_model.vario_sink_mode == 2, "sink mode must cycle from pulse to continuous");
    press(mn::kKeyOk, 80);
    check(rs.active_model.vario_sink_mode == 0, "sink mode must wrap to mute");
    press(mn::kKeyDown, 90);
    const mn::Event mute_ev = press(mn::kKeyOk, 100);
    check(mute_ev.action == Action::SaveActiveModel && rs.active_model.vario_motor_mute == 0,
          "OK on motor mute must toggle off and save");
    press(mn::kKeyOk, 110);
    check(rs.active_model.vario_motor_mute == 1, "OK on motor mute must toggle back on");

    // Switch row: OFF cycles only to ON while no arm condition is available (no switch inputs configured).
    press(mn::kKeyUp, 120);
    press(mn::kKeyUp, 130);
    press(mn::kKeyUp, 140);
    press(mn::kKeyUp, 150);
    check(s.vario.selected == 0, "UP must move back to the switch row");
    press(mn::kKeyOk, 160);
    check(s.editing, "OK on the switch row must start editing");
    press(mn::kKeyUp, 170);
    check(rs.active_model.vario_switch == 1, "UP while editing must select ON");
    const mn::Event switch_done = press(mn::kKeyOk, 180);
    check(switch_done.action == Action::SaveActiveModel && !s.editing && rs.active_model.vario_switch == 1,
          "OK while editing must save the switch and stop editing");

    // ESC from the sub-screen returns to Model Setup on the Vario row.
    press(mn::kKeyCancel, 190);
    check(s.screen == mn::Screen::ModelSetup && s.setup.selected == 5 && s.setup.scroll == 2,
          "ESC must return to Model Setup on the Vario row");

    // Rendering and the available-switch rule.
    logic::display::Framebuffer fb{};
    s.screen = mn::Screen::VarioSetup;
    s.vario = mn::ListPos{};
    mn::render(fb, s, mn::ModelNames{}, rs, mn::Live{});
    check(mn::is_vario_switch_available(0, rs.radio) && mn::is_vario_switch_available(1, rs.radio) &&
              !mn::is_vario_switch_available(24, rs.radio),
          "OFF and ON must always be available and codes past 23 must not");
}

void trainer_test() {
    namespace tr = ferrox::logic::trainer;
    using tr::PulseUs;

    constexpr uint32_t kTotal = [] {
        tr::PpmGenerator g;
        g.update_channels(std::array<uint16_t, 8>{1000, 1500, 2000, 1500, 1200, 1800, 1000, 2000});
        uint32_t total = 0;
        for (int i = 0; i < 18; ++i) {
            total += g.next_interval().duration.value;
        }
        return total;
    }();
    static_assert(kTotal == tr::kFrameTotalUs, "18 generator intervals must sum to 22.5 ms");

    {
        tr::PpmDecoder d;
        check(!d.is_signal_valid(), "decoder starts without a valid signal");
        d.process_pulse(PulseUs{10000});
        check(!d.is_signal_valid(), "sync alone does not validate a frame");
        const std::array<uint16_t, 8> want{1000, 1200, 1500, 1800, 2000, 1100, 1600, 1900};
        for (uint16_t ch : want) {
            d.process_pulse(PulseUs{ch});
        }
        d.process_pulse(PulseUs{10000});
        check(d.is_signal_valid(), "clean 8-channel frame validates the signal");
        check(d.frame_count() == 1, "one frame counted");
        check(d.channels() && *d.channels() == want, "decoded channels match the frame");
    }
    {
        tr::PpmDecoder d;
        d.process_pulse(PulseUs{10000});
        for (uint16_t ch : {1500, 1500, 400, 1500, 1500}) {
            d.process_pulse(PulseUs{ch});
        }
        d.process_pulse(PulseUs{10000});
        check(!d.is_signal_valid(), "out-of-range glitch aborts the frame");
        check(d.frame_count() == 0, "aborted frame is not counted");
    }
    {
        tr::PpmDecoder d;
        d.process_pulse(PulseUs{10000});
        for (int i = 0; i < 4; ++i) {
            d.process_pulse(PulseUs{1500});
        }
        d.process_pulse(PulseUs{10000});
        d.tick(tr::DeltaMs{50});
        check(d.is_signal_valid(), "signal still valid 50 ms after the frame");
        d.tick(tr::DeltaMs{60});
        check(!d.is_signal_valid(), "signal lost after the 100 ms timeout");
        check(!d.channels().has_value(), "no channels once the signal is lost");
    }
    {
        tr::PpmDecoder d;
        d.process_pulse(PulseUs{10000});
        for (uint16_t ch : {2140, 900, 1500, 1500}) {
            d.process_pulse(PulseUs{ch});
        }
        d.process_pulse(PulseUs{10000});
        check(d.channels() && (*d.channels())[0] == 2012, "pulse above 2012 us clamps to 2012");
        check(d.channels() && (*d.channels())[1] == 988, "pulse of 900 us clamps up to the 988 us decode floor");
    }
    {
        tr::PpmGenerator g;
        g.update_channels(std::array<uint16_t, 8>{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500});
        const auto first = g.next_interval();
        check(first.duration.value == 400 && !first.high, "frame starts with a 400 us low sync");
        const auto second = g.next_interval();
        check(second.duration.value == 1100 && second.high, "1500 us channel is a 1100 us high after sync");
    }
    {
        tr::PpmGenerator g;
        g.update_channels(std::array<uint16_t, 8>{2500, 500, 1500, 1500, 1500, 1500, 1500, 1500});
        g.next_interval();
        const auto high = g.next_interval();
        check(high.duration.value == 1700, "channel above 2100 us clamps to 2100 us");
        g.next_interval();
        check(g.next_interval().duration.value == 500, "channel below 900 us clamps to 900 us high minimum");
    }
    {
        tr::Handover h;
        const std::array<uint16_t, 18> instr{1500, 1500, 1000, 1500, 1111, 1222, 1333, 1444};
        const tr::InChannels student{1100, 1200, 1300, 1400, 1, 1, 1, 1};

        auto out = h.process(false, student, instr);
        check(h.state() == tr::State::Disabled && out == instr, "switch off passes the instructor frame through");

        out = h.process(true, std::nullopt, instr);
        check(h.state() == tr::State::SignalLost && out == instr, "student signal lost keeps the instructor frame");

        auto deflected = instr;
        deflected[0] = 1600;
        out = h.process(true, student, deflected);
        check(h.state() == tr::State::InstructorOverride && out == deflected,
              "instructor roll deflection overrides the student");

        out = h.process(true, student, instr);
        check(h.state() == tr::State::StudentActive, "centred instructor lets the student fly");
        check(out[0] == 1100 && out[1] == 1200 && out[2] == 1300 && out[3] == 1400,
              "first 4 channels come from the student");
        check(out[4] == 1111 && out[7] == 1444, "channels above 4 stay with the instructor");
    }
    {
        const std::array<uint16_t, 4> centred{1500, 1500, 1000, 1500};
        const std::array<uint16_t, 4> minor_drift{1530, 1480, 1030, 1520};
        const std::array<uint16_t, 4> roll_takeover{1560, 1500, 1000, 1500};
        const std::array<uint16_t, 4> pitch_takeover{1500, 1440, 1000, 1500};
        const std::array<uint16_t, 4> yaw_takeover{1500, 1500, 1000, 1445};
        const std::array<uint16_t, 4> throttle_edge{1500, 1500, 1050, 1500};
        const std::array<uint16_t, 4> throttle_up{1500, 1500, 1051, 1500};
        check(!tr::is_instructor_deflected(centred), "centred sticks are not deflected");
        check(!tr::is_instructor_deflected(minor_drift), "drift within 50 us is not deflection");
        check(tr::is_instructor_deflected(roll_takeover), "roll beyond 50 us is deflection");
        check(tr::is_instructor_deflected(pitch_takeover), "pitch beyond 50 us is deflection");
        check(tr::is_instructor_deflected(yaw_takeover), "yaw beyond 50 us is deflection");
        check(!tr::is_instructor_deflected(throttle_edge), "throttle at 1050 us is not deflection");
        check(tr::is_instructor_deflected(throttle_up), "throttle above 1050 us is deflection");
    }
}

void auto_select_test() {
    namespace as = logic::auto_select;
    namespace mn = logic::menu;
    using mn::Screen;

    as::Tracker tracker{};
    as::Sample s{};
    check(!tracker.step(s).sw_cond.has_value(), "auto: idle sample reports no switch change");
    s.switches.sa = logic::SwitchPos::Down;
    check(tracker.step(s).sw_cond == 2, "auto: SA moving to Down reports condition 2");
    check(!tracker.step(s).sw_cond.has_value(), "auto: a held switch reports no change");
    s.vr1 = 400;
    check(tracker.step(s).vr1, "auto: VR1 moving 400 counts is detected");
    check(!tracker.step(s).vr1, "auto: a held VR1 reports no change");

    logic::storage::RadioStorage st = logic::storage::default_storage();
    uint8_t two_pos_code = 0;
    for (unsigned v = 0; v < 256; ++v) {
        if (logic::input::resolve_mode(0, static_cast<uint8_t>(v)) == logic::input::AdcMode::TwoPos) {
            two_pos_code = static_cast<uint8_t>(v);
            break;
        }
    }
    st.radio.adc_modes[0] = two_pos_code;

    const as::Changes vr1_moved{.vr1 = true};
    const as::Changes vr2_moved{.vr2 = true};
    const as::Changes sa_moved{.sw_cond = 1};
    const as::Changes dr_sa{.dr_switch = 1};

    mn::State ms{};
    mn::open(ms);
    ms.screen = Screen::AuxChannels;
    ms.aux = mn::ListPos{};
    ms.editing = true;
    st.active_model.aux_channels[0] = 4;
    mn::update(ms, mn::Input{0, 0, 0}, st);  // consumes the release wait that open() arms
    mn::update(ms, mn::Input{0, 0, 0, vr1_moved}, st);
    check(st.active_model.aux_channels[0] == mn::kSourceVra, "auto: aux row takes VRA when VR1 moves");
    mn::update(ms, mn::Input{0, 0, 1, vr2_moved}, st);
    check(st.active_model.aux_channels[0] == mn::kSourceVrb, "auto: aux row takes VRB when VR2 moves");
    mn::update(ms, mn::Input{0, 0, 2}, st);
    mn::update(ms, mn::Input{mn::kKeyCancel, mn::kKeyCancel, 3}, st);
    check(!ms.editing, "auto: cancel leaves aux edit mode");

    ms.screen = Screen::DualRate;
    ms.dual_rate = mn::ListPos{};
    ms.editing = true;
    st.active_model.dr_switch = 0;
    mn::update(ms, mn::Input{0, 0, 0, dr_sa}, st);
    check(st.active_model.dr_switch == 1, "auto: dual rate switch follows SA when usable");
    st.radio.adc_modes[0] = 3;  // SixPos: not usable for dual rate
    st.active_model.dr_switch = 0;
    mn::update(ms, mn::Input{mn::kKeyUp, mn::kKeyUp, 10, dr_sa}, st);
    check(st.active_model.dr_switch == 0, "auto: unusable DR switch change is ignored and blocks up/down");
    st.radio.adc_modes[0] = two_pos_code;

    ms.screen = Screen::ModelSetup;
    ms.setup.selected = static_cast<uint8_t>(mn::SetupRow::ArmSwitch);
    ms.editing = true;
    st.active_model.arm_switch = 0;
    mn::update(ms, mn::Input{0, 0, 0, sa_moved}, st);
    check(st.active_model.arm_switch == 1, "auto: arm switch follows SA when usable");
    st.radio.adc_modes[0] = 3;  // SixPos: not usable for arm switch
    st.active_model.arm_switch = 0;
    mn::update(ms, mn::Input{0, 0, 1, sa_moved}, st);
    check(st.active_model.arm_switch == 0, "auto: unusable arm switch change is ignored");
    st.radio.adc_modes[0] = two_pos_code;

    ms.screen = Screen::VarioSetup;
    ms.vario = mn::ListPos{};
    ms.editing = true;
    st.active_model.vario_switch = 0;
    mn::update(ms, mn::Input{0, 0, 0, sa_moved}, st);
    check(st.active_model.vario_switch == 2, "auto: vario switch stores condition + 1");

    ms.screen = Screen::MixerLine;
    ms.mix_idx = 0;
    ms.mix_line.selected = 5;
    ms.editing = true;
    st.active_model.mixes[0].switch_sel = 0;
    mn::update(ms, mn::Input{0, 0, 0, sa_moved}, st);
    check(st.active_model.mixes[0].switch_sel == 1, "auto: mixer switch row follows SA when usable");

    ms.mix_line.selected = 1;
    st.active_model.mixes[0].source = 0;
    mn::update(ms, mn::Input{0, 0, 1, vr2_moved}, st);
    const bool vrb_usable = mn::is_source_available(mn::kSourceVrb, st.radio, st.active_model);
    check(st.active_model.mixes[0].source == (vrb_usable ? mn::kSourceVrb : 0),
          "auto: mixer source takes VRB only when it is usable");
}

void telem_slots_test() {
    namespace mn = logic::menu;
    namespace st = logic::storage;
    namespace tm = logic::telemetry;
    using mn::Action;

    {
        logic::input::InputState in{};
        logic::storage::RadioStorage rs{};
        logic::trim::Controller trims{};
        std::array<uint16_t, logic::kNumChannels> chs{};
        rs.active_model.telem_slots = {0, 1, 2, 5};
        logic::dashboard::View v{in, rs, trims, chs, false, {}, false, false, false, 0};
        logic::dashboard::PageState ps{};
        ps.page = 4;
        logic::display::Framebuffer fb{};
        logic::dashboard::render(fb, ps, v);
        auto lit = [&](int x, int y) {
            return (fb.data()[static_cast<std::size_t>((y / 8) * 128 + x)] >> (y % 8)) & 1u;
        };
        check(lit(64, 20) == 1 && lit(64, 45) == 1, "custom telem page: vertical divider drawn");
        check(lit(30, 31) == 1 && lit(100, 31) == 1, "custom telem page: horizontal divider drawn");
    }

    check(tm::telem_sensor_name(0) == "RX BAT" && tm::telem_sensor_name(1) == "LQ/RSSI" &&
              tm::telem_sensor_name(22) == "OFF" && tm::telem_sensor_name(99) == "OFF",
          "telemetry slot names must match the Rust sensor list");
    check(tm::next_telem_sensor(21) == 22 && tm::next_telem_sensor(22) == 0,
          "telemetry slot OK must step through all sensors and wrap after OFF");
    check(!tm::is_custom_telem_layout({0, 1, 2, 3}) && tm::is_custom_telem_layout({0, 1, 2, 4}) &&
              tm::is_custom_telem_layout({22, 1, 2, 3}),
          "only non-default slots must select the custom telemetry page");
    check(tm::tile_origin(0).x == 4 && tm::tile_origin(0).y == 14 && tm::tile_origin(1).x == 68 &&
              tm::tile_origin(2).y == 33 && tm::tile_origin(3).x == 68 && tm::tile_origin(3).y == 33,
          "custom telemetry tiles must sit on the 2x2 grid of Rust");

    st::RadioStorage rs = st::default_storage();
    rs.radio.active_model = 1;
    mn::State s{};
    s.screen = mn::Screen::ModelSetup;
    s.setup.selected = mn::kItemTelemRow;
    s.setup.scroll = 3;
    auto press = [&](uint16_t key, uint32_t now) {
        const mn::Event e = mn::update(s, mn::Input{key, key, now}, rs);
        mn::update(s, mn::Input{0, 0, now + 1}, rs);
        return e;
    };

    check(static_cast<std::size_t>(mn::SetupRow::Telemetry) == mn::kItemTelemRow &&
              mn::kSetupRows == 9 && static_cast<std::size_t>(mn::SetupRow::Protocol) == 7,
          "Telemetry must be the row before Protocol on the model setup screen");

    press(mn::kKeyOk, 10);
    check(s.screen == mn::Screen::TelemSetup && s.telem.selected == 0 && s.telem.scroll == 0,
          "OK on Telemetry must open the slot editor at the top");
    check(rs.active_model.telem_slots == std::array<uint8_t, 4>{0, 1, 2, 3}, "default slots must be unchanged on open");

    const mn::Event e = press(mn::kKeyOk, 20);
    check(e.action == Action::SaveActiveModel && rs.active_model.telem_slots[0] == 1,
          "OK on a slot must cycle its sensor and save the model");

    press(mn::kKeyDown, 30);
    check(s.telem.selected == 1, "down must move to the next slot");
    press(mn::kKeyOk, 40);
    check(rs.active_model.telem_slots[1] == 2 && rs.active_model.telem_slots[0] == 1,
          "only the selected slot must change");

    rs.active_model.telem_slots[2] = 22;
    press(mn::kKeyDown, 50);
    press(mn::kKeyOk, 60);
    check(s.telem.selected == 2 && rs.active_model.telem_slots[2] == 0, "OK must wrap from OFF to the first sensor");

    press(mn::kKeyCancel, 70);
    check(s.screen == mn::Screen::ModelSetup && s.setup.selected == mn::kItemTelemRow,
          "ESC must return to the Telemetry row of model setup");
}

void wiring_cues_test() {
    namespace mn = ferrox::logic::menu;
    using ferrox::logic::buzzer::Cue;
    using ferrox::logic::buzzer::pattern_for;
    using ferrox::logic::buzzer::PreflightWarning;
    using ferrox::logic::buzzer::Repeats;
    using ferrox::logic::buzzer::ToneStyle;

    const auto simple_bind = pattern_for(Cue::BindHold, ToneStyle::Simple);
    check(simple_bind.notes.size() == 1 && simple_bind.notes[0].freq.value == 2400 &&
              simple_bind.notes[0].duration.value == 150 && simple_bind.repeats == Repeats{0},
          "wiring: bind hold is one 2400 Hz 150 ms tone in both styles");
    check(pattern_for(Cue::BindHold, ToneStyle::Rich).notes[0].freq.value == 2400, "wiring: bind hold matches in Rich");
    check(pattern_for(Cue::PreflightExit, ToneStyle::Simple).notes[0].freq.value == 2200 &&
              pattern_for(Cue::PreflightExit, ToneStyle::Simple).notes[0].duration.value == 40,
          "wiring: preflight exit is a 2200 Hz 40 ms tone");
    check(pattern_for(Cue::RxSave, ToneStyle::Rich).repeats == Repeats{2}, "wiring: RX save repeats twice");
    check(pattern_for(Cue::RxSave, ToneStyle::Simple).notes[0].freq.value == 2400 &&
              pattern_for(Cue::RxSave, ToneStyle::Simple).notes[0].duration.value == 70,
          "wiring: RX save is a 2400 Hz 70 ms tone");
    const auto watchdog = pattern_for(Cue::WatchdogRecover, ToneStyle::Simple);
    check(watchdog.notes[0].freq.value == 2600 && watchdog.notes[0].duration.value == 60 &&
              watchdog.notes[0].pause.value == 40 && watchdog.repeats == Repeats{3},
          "wiring: watchdog chirp is 2600 Hz 60 ms, 40 ms pause, three times");

    PreflightWarning warning{};
    check(warning.exit_cue() == Cue::None, "wiring: no preflight exit chirp without a warning");
    warning.unsafe();
    check(warning.exit_cue() == Cue::PreflightExit, "wiring: preflight exit chirp after a warning");

    ferrox::logic::buzzer_inputs::PotCenterTracker<6> pots{};
    const std::array<int16_t, 2> far{{100, -100}};
    const std::array<int16_t, 2> near{{0, 10}};
    check(!pots.step(far), "wiring: pots starting away from centre do not click");
    check(pots.step(near), "wiring: pots leaving the deadband click");
    check(!pots.step(near), "wiring: no repeat click while inside the deadband");
    check(!pots.step(std::array<int16_t, 2>{{30, 10}}), "wiring: moving out of the deadband does not click");

    auto st = ferrox::logic::storage::default_storage();
    mn::State menu{};
    menu.screen = mn::Screen::Main;
    mn::update(menu, mn::Input{mn::kKeyDown, mn::kKeyDown, 0}, st);
    check(menu.nav_tick, "wiring: a list step sets the nav tick");
    mn::update(menu, mn::Input{0, 0, 1}, st);
    check(!menu.nav_tick, "wiring: the nav tick clears on the next tick");
}

void usb_logic_test() {
    namespace u = ferrox::logic::usb;
    using M = u::UsbMode;
    const auto le16 = [](std::span<const uint8_t> d, std::size_t i) {
        return static_cast<unsigned>(d[i] | (d[i + 1] << 8));
    };
    const auto contains = [](std::span<const uint8_t> hay, std::span<const uint8_t> needle) {
        return std::search(hay.begin(), hay.end(), needle.begin(), needle.end()) != hay.end();
    };
    // True when the sub-descriptors tile the blob exactly (each length byte is non-zero and lands on the next one).
    const auto tiles = [](std::span<const uint8_t> d) {
        std::size_t i = 0;
        while (i < d.size()) {
            if (d[i] == 0) return false;
            i += d[i];
        }
        return i == d.size();
    };

    check(u::mode_from_stored(0) == M::Off && u::mode_from_stored(3) == M::Composite &&
              u::mode_from_stored(4) == M::Off && u::mode_from_stored(255) == M::Off,
          "usb: stored mode maps 0..3 and falls back to Off");
    check(u::has_hid(M::Joystick) && !u::has_hid(M::Serial) && u::has_hid(M::Composite) && !u::has_hid(M::Off),
          "usb: HID is used by joystick and composite only");
    check(!u::has_cdc(M::Joystick) && u::has_cdc(M::Serial) && u::has_cdc(M::Composite) && !u::has_cdc(M::Off),
          "usb: CDC is used by serial and composite only");
    check(u::is_sim_mode(M::Joystick, true) && !u::is_sim_mode(M::Joystick, false) &&
              !u::is_sim_mode(M::Serial, true) && !u::is_sim_mode(M::Composite, true) &&
              !u::is_sim_mode(M::Off, true),
          "usb: sim mode needs the joystick mode and a connected host");
    check(u::interval_elapsed({110}, {10}, {100}) && !u::interval_elapsed({109}, {10}, {100}),
          "usb: interval elapses at the boundary");
    check(u::interval_elapsed({5}, {0xFFFFFFF0u}, {20}), "usb: interval survives the millisecond counter wrapping");

    for (M m : {M::Joystick, M::Serial, M::Composite}) {
        const auto d = u::device_descriptor(m).view();
        check(d.size() == 18 && d[0] == 18 && d[1] == 1 && le16(d, 2) == 0x0200 && d[17] == 1,
              "usb: device descriptor header and single configuration");
        check(d[7] == 64, "usb: control endpoint packet size is 64");
    }
    {
        const auto joy = u::device_descriptor(M::Joystick).view();
        const auto ser = u::device_descriptor(M::Serial).view();
        const auto comp = u::device_descriptor(M::Composite).view();
        check(le16(joy, 8) == 0x1209 && le16(joy, 10) == 0x4F54, "usb: joystick VID/PID");
        check(le16(ser, 8) == 0x0483 && le16(ser, 10) == 0x5740, "usb: serial VID/PID");
        check(le16(comp, 8) == 0x1209 && le16(comp, 10) == 0x4968, "usb: composite VID/PID");
        check(joy[4] == 0 && ser[4] == 2 && comp[4] == 0xEF && comp[5] == 2 && comp[6] == 1,
              "usb: device class codes (composite uses IAD)");
    }

    {
        const auto cfg = u::config_descriptor(M::Joystick).view();
        check(cfg.size() == 34 && le16(cfg, 2) == 34 && cfg[4] == 1 && tiles(cfg.subspan(9)),
              "usb: joystick configuration tiles exactly");
        check(contains(cfg, std::to_array<uint8_t>({0x22, 56, 0})), "usb: HID report descriptor length is 56");
        check(contains(cfg, std::to_array<uint8_t>({7, 0x05, 0x81, 0x03, 18, 0, 10})),
              "usb: HID IN endpoint is 18 bytes every 10 ms");
    }
    {
        const auto cfg = u::config_descriptor(M::Serial).view();
        check(cfg.size() == 75 && le16(cfg, 2) == 75 && cfg[4] == 2 && tiles(cfg.subspan(9)),
              "usb: serial configuration tiles exactly");
        check(contains(cfg, std::to_array<uint8_t>({0x05, 0x24, 0x06, 0, 1})), "usb: CDC union uses interfaces 0 and 1");
        check(contains(cfg, std::to_array<uint8_t>({7, 0x05, 0x02, 0x02, 64, 0, 0})) &&
                  contains(cfg, std::to_array<uint8_t>({7, 0x05, 0x82, 0x02, 64, 0, 0})) &&
                  contains(cfg, std::to_array<uint8_t>({7, 0x05, 0x83, 0x03, 16, 0, 255})),
              "usb: CDC bulk OUT/IN and notify endpoints");
    }
    {
        const auto cfg = u::config_descriptor(M::Composite).view();
        check(cfg.size() == 100 && le16(cfg, 2) == 100 && cfg[4] == 3 && tiles(cfg.subspan(9)),
              "usb: composite configuration tiles exactly");
        check(contains(cfg, std::to_array<uint8_t>({8, 0x0B, 1, 2, 0x02, 0x02, 0x00, 0})),
              "usb: IAD starts at the CDC control interface after HID");
        check(contains(cfg, std::to_array<uint8_t>({9, 0x04, 0, 0, 1, 0x03, 0, 0, 0})),
              "usb: HID interface is number 0 in composite");
    }

    {
        const auto s0 = u::string_descriptor(M::Joystick, 0).view();
        check(s0.size() == 4 && s0[2] == 0x09 && s0[3] == 0x04, "usb: language ID string is en-US");
        const auto s2 = u::string_descriptor(M::Joystick, 2).view();
        check(s2.size() == 32 && s2[0] == 32 && s2[1] == 3 && s2[2] == 'F' && s2[3] == 0 && s2[30] == 'k',
              "usb: product string is UTF-16LE of the mode name");
        check(u::string_descriptor(M::Serial, 3).size == 22, "usb: serial string length");
        check(u::string_descriptor(M::Serial, 4).size == 0, "usb: unknown string index is empty");
    }
    {
        const auto rd = u::kGamepadReportDescriptor;
        check(rd.size() == 56 && rd[0] == 0x05 && rd[1] == 0x01 && rd[54] == 0xC0 && rd[55] == 0xC0,
              "usb: gamepad report descriptor framing");
    }

    {
        std::array<uint16_t, ferrox::logic::kNumChannels> chs{};
        chs.fill(1500);
        ferrox::logic::Switches sw{};
        auto r = u::build_gamepad_report(chs, sw);
        check(r[0] == 0 && r[1] == 0 && r[2] == 0x00 && r[3] == 0x04 && r[16] == 0x00 && r[17] == 0x04,
              "usb: centred channels give zero buttons and axis 1024");
        chs[0] = 988;
        chs[1] = 2012;
        chs[2] = 5000;
        chs[3] = 100;
        r = u::build_gamepad_report(chs, sw);
        check(r[2] == 0 && r[3] == 0 && r[4] == 0xFF && r[5] == 0x07 && r[6] == 0xFF && r[7] == 0x07 &&
                  r[8] == 0 && r[9] == 0,
              "usb: axes clamp to 0..2047");
        chs.fill(1500);
        chs[13] = 1800;
        sw.sa = ferrox::logic::SwitchPos::Down;
        sw.sb = ferrox::logic::SwitchPos::Mid;
        sw.se = ferrox::logic::SwitchPos::Down;
        sw.sh = ferrox::logic::SwitchPos::Down;
        r = u::build_gamepad_report(chs, sw);
        check(r[0] == (9731 & 0xFF) && r[1] == (9731 >> 8), "usb: switches and CH14 aux map to the expected buttons");
    }

    check(u::parse_command("") == u::Command::Empty && u::parse_command(" \r\n\t") == u::Command::Empty,
          "usb cli: blank lines are ignored");
    check(u::parse_command("  HeLp \r") == u::Command::Help, "usb cli: case-insensitive and trimmed");
    check(u::parse_command("status") == u::Command::Status && u::parse_command("channels") == u::Command::Channels,
          "usb cli: status and channels");
    check(u::parse_command("eeprom") == u::Command::Storage && u::parse_command("storage") == u::Command::Storage,
          "usb cli: storage and eeprom alias");
    check(u::parse_command("telem") == u::Command::Telem && u::parse_command("telem off") == u::Command::TelemOff,
          "usb cli: single frame and telemetry off");
    check(u::parse_command("stream") == u::Command::Stream && u::parse_command("telem on") == u::Command::Stream &&
              u::parse_command("telem stream") == u::Command::Stream,
          "usb cli: stream aliases");
    check(u::parse_command("dfu") == u::Command::Dfu && u::parse_command("reboot bootloader") == u::Command::Dfu &&
              u::parse_command("reboot-bootloader") == u::Command::Dfu,
          "usb cli: DFU aliases");
    check(u::parse_command("reboot") == u::Command::Reboot, "usb cli: reboot");
    check(u::parse_command("telem  on") == u::Command::Unknown && u::parse_command("reboot now") == u::Command::Unknown,
          "usb cli: inner spacing must match and unknown words are rejected");

    {
        const std::array<uint16_t, 3> ch{1500, 988, 2012};
        std::array<char, 64> buf{};
        const std::size_t n = u::format_channels_json(ch, buf);
        check(std::string_view(buf.data(), n) == "{\"ch\":[1500,988,2012]}\r\n",
              "usb json: channel dump matches the Rust format");
        std::array<char, 8> small{};
        check(u::format_channels_json(ch, small) == 0, "usb json: overflow returns zero bytes");
    }
    {
        const std::array<uint16_t, 2> ch{1500, 988};
        std::array<char, 64> buf{};
        const std::size_t n = u::format_telemetry_json(7404, ch, buf);
        check(std::string_view(buf.data(), n) == "{\"vbat\":7.40,\"ch\":[1500,988]}\r\n",
              "usb telemetry: battery voltage with two decimals");
        std::array<char, 64> buf2{};
        const std::size_t n2 = u::format_telemetry_json(6905, ch, buf2);
        check(std::string_view(buf2.data(), n2) == "{\"vbat\":6.91,\"ch\":[1500,988]}\r\n",
              "usb telemetry: millivolt rounding");
        std::array<char, 64> buf3{};
        check(u::format_telemetry_json(5003, ch, buf3) > 0 &&
                  std::string_view(buf3.data()) == "{\"vbat\":5.00,\"ch\":[1500,988]}\r\n",
              "usb telemetry: zero-padded hundredths");
    }
    {
        u::ByteRing<4> ring{};
        check(ring.push_text("abc") && ring.size() == 3, "usb ring: text fits");
        check(!ring.push_text("xy") && ring.size() == 3, "usb ring: text is all-or-nothing");
        check(ring.push_all(std::to_array<uint8_t>({'d', 'e'})) == 1 && ring.size() == 4, "usb ring: push_all stops when full");
        std::array<uint8_t, 3> out{};
        check(ring.pop_into(out) == 3 && out[0] == 'a' && out[2] == 'c', "usb ring: FIFO order");
        check(ring.push_text("fg") && ring.size() == 3, "usb ring: wraps around");
        std::array<uint8_t, 8> rest{};
        const std::size_t r = ring.pop_into(rest);
        check(r == 3 && rest[0] == 'd' && rest[1] == 'f' && rest[2] == 'g', "usb ring: wrapped contents");
        ring.clear();
        check(ring.size() == 0 && ring.free_space() == 4, "usb ring: clear");
    }
    {
        u::LineEditor ed{};
        using A = u::LineEditor::Action;
        check(ed.feed('h') == A::Echo && ed.feed('i') == A::Echo, "usb line: echoes printable bytes");
        check(ed.feed(0x7F) == A::Erase && ed.line() == "h", "usb line: DEL erases one char");
        check(ed.feed(0x08) == A::Erase && ed.line().empty(), "usb line: backspace erases");
        check(ed.feed(0x08) == A::None, "usb line: backspace on empty line is ignored");
        ed.feed('s');
        ed.feed('s');
        check(ed.feed('\r') == A::EndOfLine && ed.line() == "ss", "usb line: CR ends line");
        check(ed.feed('\n') == A::None, "usb line: LF after CR collapses");
        ed.clear();
        check(ed.feed('\n') == A::EndOfLine && ed.feed('\n') == A::EndOfLine,
              "usb line: bare LF lines and repeated LF both end lines");
        ed.clear();
        for (std::size_t i = 0; i < u::LineEditor::kCapacity; ++i) ed.feed('x');
        check(ed.feed('y') == A::None && ed.line().size() == u::LineEditor::kCapacity, "usb line: capacity is bounded");
    }
    check(u::kHelpText.size() > 0 && u::kLineCoding.size() == 7 && u::kLineCoding[0] == 0x00 && u::kLineCoding[1] == 0xC2,
          "usb line coding: 115200 baud");
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
    auto_select_test();
    throttle_curve_test();
    mixer_menu_test();
    value_cues_test();
    cancel_click_test();
    aux_menu_test();
    radio_setup_test();
    afhds2a_test();
    a7105_radio_test();
    vario_test();
    telemetry_test();
    crsf_diag_test();
    buzzer_test();
    wiring_cues_test();
    usb_logic_test();
    trim_controller_test();
    trim_autosave_test();
    instant_trim_test();
    flight_timer_test();
    bind_test();
    model_copy_test();
    protocol_reset_test();
    vario_setup_test();
    trainer_test();
    telem_slots_test();
    if (failures == 0) {
        std::puts("all host logic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
