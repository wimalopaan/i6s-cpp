#include <array>
#include <cstdint>
#include <cstdio>
#include <span>

#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/crsf.hpp"
#include "ferrox/logic/crsf_menu.hpp"
#include "ferrox/logic/power.hpp"
#include "ferrox/util/ring.hpp"

using namespace ferrox;

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

}  // namespace

int main() {
    ring_test();
    crsf_framer_test();
    crsf_menu_test();
    if (failures == 0) {
        std::puts("all host logic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
