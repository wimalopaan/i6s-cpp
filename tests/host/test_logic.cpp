#include <array>
#include <cstdint>
#include <cstdio>
#include <span>

#include "ferrox/logic/brr.hpp"
#include "ferrox/logic/crsf.hpp"
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

}  // namespace

int main() {
    ring_test();
    crsf_framer_test();
    if (failures == 0) {
        std::puts("all host logic tests passed");
    }
    return failures == 0 ? 0 : 1;
}
