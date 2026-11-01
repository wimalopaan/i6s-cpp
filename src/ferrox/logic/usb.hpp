#pragma once

// Pure USB logic (Rust `usb/mod.rs`, `usb/hid.rs`, `usb/serial.rs`): mode selection,
// device, configuration and string descriptors, the 18-byte gamepad report, CLI
// command parsing and the channel dump. Register access lives in `hw/usb.hpp`.

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "ferrox/logic/channels.hpp"
#include "ferrox/logic/switches.hpp"

namespace ferrox::logic::usb {

enum class UsbMode : uint8_t { Off = 0, Joystick = 1, Serial = 2, Composite = 3 };

inline constexpr uint8_t kModeCount = 4;
inline constexpr uint16_t kAuxButtonThresholdUs = 1750;
inline constexpr uint32_t kHidIntervalMs = 10;
inline constexpr uint32_t kTelemetryIntervalMs = 100;
inline constexpr std::size_t kReportSize = 18;
inline constexpr std::size_t kHidAxes = 8;
inline constexpr uint32_t kAxisMax = 2047;

// Strong types so the vendor and product ids cannot be swapped at a call site.
struct VendorId {
    uint16_t value = 0;
    constexpr bool operator==(const VendorId&) const = default;
};

struct ProductId {
    uint16_t value = 0;
    constexpr bool operator==(const ProductId&) const = default;
};

struct Millis {
    uint32_t value = 0;
    constexpr bool operator==(const Millis&) const = default;
};

// Mode values outside the menu range fall back to Off, matching the storage validation.
constexpr UsbMode mode_from_stored(const uint8_t raw) {
    return raw < kModeCount ? static_cast<UsbMode>(raw) : UsbMode::Off;
}

constexpr bool has_hid(const UsbMode m) {
    return m == UsbMode::Joystick || m == UsbMode::Composite;
}

constexpr bool has_cdc(const UsbMode m) {
    return m == UsbMode::Serial || m == UsbMode::Composite;
}

// Rust `is_sim_mode`: sim mode is the joystick mode with a configured host. The main loop then mutes
// the RF link, stops CRSF and PPM out (see main.cpp).
constexpr bool is_sim_mode(const UsbMode m, const bool connected) {
    return m == UsbMode::Joystick && connected;
}

constexpr bool interval_elapsed(const Millis now, const Millis since, const Millis interval) {
    return now.value - since.value >= interval.value;
}

// Fixed-capacity byte blob for descriptors. Writes past capacity fail at compile time in constexpr use.
template <std::size_t N>
struct Blob {
    std::array<uint8_t, N> bytes{};
    std::size_t size = 0;

    constexpr void put(const uint8_t v) {
        bytes[size++] = v;
    }

    constexpr void put16(const uint16_t v) {
        put(static_cast<uint8_t>(v & 0xFF));
        put(static_cast<uint8_t>(v >> 8));
    }

    constexpr void put_all(std::span<const uint8_t> data) {
        for (uint8_t v : data) {
            put(v);
        }
    }

    constexpr std::span<const uint8_t> view() const {
        return {bytes.data(), size};
    }
};

struct DeviceIdentity {
    VendorId vid{};
    ProductId pid{};
    std::string_view product{};
    std::string_view serial{};
};

constexpr DeviceIdentity identity(const UsbMode m) {
    switch (m) {
        case UsbMode::Serial:
            return {VendorId{0x0483}, ProductId{0x5740}, "FS-i6X Serial", "FS-I6X-VCP"};
        case UsbMode::Composite:
            return {VendorId{0x1209}, ProductId{0x4968}, "FS-i6X Radio", "FS-I6X-COMP"};
        case UsbMode::Joystick:
        case UsbMode::Off:
            break;
    }
    return {VendorId{0x1209}, ProductId{0x4F54}, "FS-i6X Joystick", "FS-I6X-SIM"};
}

// Interface numbers in the configuration. HID goes first in composite mode, then the CDC pair.
struct Layout {
    uint8_t hid_if = 0;
    uint8_t ctrl_if = 0;
    uint8_t data_if = 0;
    uint8_t count = 0;
};

constexpr Layout layout(const UsbMode m) {
    switch (m) {
        case UsbMode::Joystick:
            return {0, 0, 0, 1};
        case UsbMode::Serial:
            return {0, 0, 1, 2};
        case UsbMode::Composite:
            return {0, 1, 2, 3};
        case UsbMode::Off:
            break;
    }
    return {};
}

// Endpoint addresses shared by every mode so the hardware code is mode-agnostic.
inline constexpr uint8_t kEpHidIn = 0x81;
inline constexpr uint8_t kEpCdcOut = 0x02;
inline constexpr uint8_t kEpCdcIn = 0x82;
inline constexpr uint8_t kEpCdcNotify = 0x83;
inline constexpr uint16_t kControlPacketSize = 64;
inline constexpr uint16_t kCdcPacketSize = 64;
inline constexpr uint16_t kHidPacketSize = 18;
inline constexpr uint16_t kNotifyPacketSize = 16;

// Joystick HID report descriptor (Rust `GAMEPAD_REPORT_DESC`): 16 buttons, 8 axes at 0..2047.
inline constexpr auto kGamepadReportDescriptor = std::to_array<uint8_t>({
    0x05, 0x01, 0x09, 0x04, 0xA1, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x15, 0x00,
    0x25, 0x01, 0x95, 0x10, 0x75, 0x01, 0x81, 0x02,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32,
    0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x09, 0x37,
    0x09, 0x36, 0x16, 0x00, 0x00, 0x26, 0xFF, 0x07,
    0x75, 0x10, 0x95, 0x08, 0x81, 0x02, 0xC0, 0xC0,
});

constexpr Blob<18> device_descriptor(const UsbMode m) {
    const DeviceIdentity id = identity(m);
    std::array<uint8_t, 3> cls{0x00, 0x00, 0x00};
    if (m == UsbMode::Serial) {
        cls = {0x02, 0x00, 0x00};
    } else if (m == UsbMode::Composite) {
        // Interface association descriptor for the CDC pair.
        cls = {0xEF, 0x02, 0x01};
    }
    Blob<18> d{};
    d.put(18);
    d.put(0x01);
    d.put16(0x0200);
    d.put_all(cls);
    d.put(static_cast<uint8_t>(kControlPacketSize));
    d.put16(id.vid.value);
    d.put16(id.pid.value);
    d.put16(0x0100);
    d.put(1);
    d.put(2);
    d.put(3);
    d.put(1);
    return d;
}

constexpr Blob<100> config_descriptor(const UsbMode m) {
    const Layout lay = layout(m);
    Blob<100> d{};
    d.put(9);
    d.put(0x02);
    d.put16(0);  // wTotalLength, patched below
    d.put(lay.count);
    d.put(1);
    d.put(0);
    d.put(0x80);
    d.put(50);

    if (has_hid(m)) {
        d.put_all(std::to_array<uint8_t>({9, 0x04, lay.hid_if, 0, 1, 0x03, 0, 0, 0}));
        d.put_all(std::to_array<uint8_t>({0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22}));
        d.put16(static_cast<uint16_t>(kGamepadReportDescriptor.size()));
        d.put_all(std::to_array<uint8_t>({7, 0x05, kEpHidIn, 0x03}));
        d.put16(kHidPacketSize);
        d.put(static_cast<uint8_t>(kHidIntervalMs));
    }

    if (has_cdc(m)) {
        d.put_all(std::to_array<uint8_t>({8, 0x0B, lay.ctrl_if, 2, 0x02, 0x02, 0x00, 0}));
        d.put_all(std::to_array<uint8_t>({9, 0x04, lay.ctrl_if, 0, 1, 0x02, 0x02, 0x01, 0}));
        d.put_all(std::to_array<uint8_t>({0x05, 0x24, 0x00, 0x10, 0x01}));
        d.put_all(std::to_array<uint8_t>({0x05, 0x24, 0x01, 0x00, lay.data_if}));
        d.put_all(std::to_array<uint8_t>({0x04, 0x24, 0x02, 0x02}));
        d.put_all(std::to_array<uint8_t>({0x05, 0x24, 0x06, lay.ctrl_if, lay.data_if}));
        d.put_all(std::to_array<uint8_t>({7, 0x05, kEpCdcNotify, 0x03}));
        d.put16(kNotifyPacketSize);
        d.put(255);
        d.put_all(std::to_array<uint8_t>({9, 0x04, lay.data_if, 0, 2, 0x0A, 0x00, 0x00, 0}));
        d.put_all(std::to_array<uint8_t>({7, 0x05, kEpCdcOut, 0x02}));
        d.put16(kCdcPacketSize);
        d.put(0);
        d.put_all(std::to_array<uint8_t>({7, 0x05, kEpCdcIn, 0x02}));
        d.put16(kCdcPacketSize);
        d.put(0);
    }

    d.bytes[2] = static_cast<uint8_t>(d.size & 0xFF);
    d.bytes[3] = static_cast<uint8_t>(d.size >> 8);
    return d;
}

// String descriptors as UTF-16LE. Index 0 is the language list; unknown indices give an empty blob.
constexpr Blob<64> string_descriptor(const UsbMode m, const uint8_t index) {
    Blob<64> d{};
    if (index == 0) {
        d.put_all(std::to_array<uint8_t>({4, 0x03, 0x09, 0x04}));
        return d;
    }
    const DeviceIdentity id = identity(m);
    std::string_view text;
    switch (index) {
        case 1:
            text = "FlySky";
            break;
        case 2:
            text = id.product;
            break;
        case 3:
            text = id.serial;
            break;
        default:
            return d;
    }
    d.put(static_cast<uint8_t>(2 + 2 * text.size()));
    d.put(0x03);
    for (char c : text) {
        d.put(static_cast<uint8_t>(c));
        d.put(0);
    }
    return d;
}

// Rust `us_to_axis`: maps a channel pulse onto the 11-bit axis range with rounding.
constexpr uint16_t us_to_axis(const uint16_t us) {
    const uint32_t clamped = us < kChannelMinUs ? kChannelMinUs : (us > kChannelMaxUs ? kChannelMaxUs : us);
    const uint32_t span = kChannelSpanUs;
    const uint32_t axis = ((clamped - kChannelMinUs) * kAxisMax + span / 2) / span;
    return static_cast<uint16_t>(axis > kAxisMax ? kAxisMax : axis);
}

constexpr bool is_at(const SwitchPos actual, const SwitchPos want) {
    return actual == want;
}

// Rust `build_gamepad_report`: 16 buttons first (little-endian), then 8 axes from CH1..CH8.
constexpr std::array<uint8_t, kReportSize> build_gamepad_report(std::span<const uint16_t, kNumChannels> chs,
                                                               const Switches& sw) {
    uint16_t btns = 0;
    if (is_at(sw.sa, SwitchPos::Down)) btns |= 1u << 0;
    if (is_at(sw.sb, SwitchPos::Mid)) btns |= 1u << 1;
    if (is_at(sw.sb, SwitchPos::Down)) btns |= 1u << 2;
    if (is_at(sw.sc, SwitchPos::Mid)) btns |= 1u << 3;
    if (is_at(sw.sc, SwitchPos::Down)) btns |= 1u << 4;
    if (is_at(sw.sd, SwitchPos::Down)) btns |= 1u << 5;
    for (std::size_t i = 0; i < 4; ++i) {
        if (chs[10 + i] > kAuxButtonThresholdUs) btns |= static_cast<uint16_t>(1u << (6 + i));
    }
    if (is_at(sw.se, SwitchPos::Down)) btns |= 1u << 10;
    if (is_at(sw.sf, SwitchPos::Down)) btns |= 1u << 11;
    if (is_at(sw.sg, SwitchPos::Down)) btns |= 1u << 12;
    if (is_at(sw.sh, SwitchPos::Down)) btns |= 1u << 13;

    std::array<uint8_t, kReportSize> out{};
    out[0] = static_cast<uint8_t>(btns & 0xFF);
    out[1] = static_cast<uint8_t>(btns >> 8);
    for (std::size_t i = 0; i < kHidAxes; ++i) {
        const uint16_t axis = us_to_axis(chs[i]);
        out[2 + 2 * i] = static_cast<uint8_t>(axis & 0xFF);
        out[3 + 2 * i] = static_cast<uint8_t>(axis >> 8);
    }
    return out;
}

enum class Command : uint8_t {
    Empty,
    Help,
    Status,
    Channels,
    Storage,
    Telem,
    Stream,
    TelemOff,
    Dfu,
    Reboot,
    Unknown,
};

constexpr bool is_space(const char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
}

constexpr std::string_view trim(std::string_view s) {
    while (!s.empty() && is_space(s.front())) s.remove_prefix(1);
    while (!s.empty() && is_space(s.back())) s.remove_suffix(1);
    return s;
}

constexpr char to_lower(const char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

constexpr bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (to_lower(a[i]) != to_lower(b[i])) return false;
    }
    return true;
}

// Rust `handle_command`: case-insensitive, surrounding whitespace ignored.
constexpr Command parse_command(std::string_view line) {
    const std::string_view c = trim(line);
    if (c.empty()) return Command::Empty;
    if (iequals(c, "help")) return Command::Help;
    if (iequals(c, "status")) return Command::Status;
    if (iequals(c, "channels")) return Command::Channels;
    if (iequals(c, "storage") || iequals(c, "eeprom")) return Command::Storage;
    if (iequals(c, "telem")) return Command::Telem;
    if (iequals(c, "stream") || iequals(c, "telem on") || iequals(c, "telem stream")) return Command::Stream;
    if (iequals(c, "telem off")) return Command::TelemOff;
    if (iequals(c, "dfu") || iequals(c, "reboot bootloader") || iequals(c, "reboot-bootloader")) {
        return Command::Dfu;
    }
    if (iequals(c, "reboot")) return Command::Reboot;
    return Command::Unknown;
}

// Bounded appender for CDC text. Overflow sets `ok` to false instead of writing past the end.
struct TextWriter {
    std::span<char> out;
    std::size_t pos = 0;
    bool ok = true;

    constexpr void put(std::string_view s) {
        if (!ok || pos + s.size() > out.size()) {
            ok = false;
            return;
        }
        for (char c : s) {
            out[pos++] = c;
        }
    }

    constexpr void put_uint(uint32_t v) {
        char digits[10]{};
        std::size_t n = 0;
        do {
            digits[n++] = static_cast<char>('0' + v % 10);
            v /= 10;
        } while (v != 0);
        while (n > 0) {
            put(std::string_view{&digits[--n], 1});
        }
    }
};

// Rust `channels` command: `{"ch":[...]}\r\n`. Returns 0 if the text does not fit.
constexpr std::size_t format_channels_json(std::span<const uint16_t> chs, std::span<char> out) {
    TextWriter w{out};
    w.put("{\"ch\":[");
    for (std::size_t i = 0; i < chs.size(); ++i) {
        if (i > 0) w.put(",");
        w.put_uint(chs[i]);
    }
    w.put("]}\r\n");
    return w.ok ? w.pos : 0;
}

// Same JSON shape as `format_channels_json`, prefixed with the battery voltage (two decimals).
constexpr std::size_t format_telemetry_json(const uint16_t battery_mv, std::span<const uint16_t> chs, std::span<char> out) {
    TextWriter w{out};
    const uint32_t centivolts = (static_cast<uint32_t>(battery_mv) + 5) / 10;
    w.put("{\"vbat\":");
    w.put_uint(centivolts / 100);
    w.put(".");
    if (centivolts % 100 < 10) w.put("0");
    w.put_uint(centivolts % 100);
    w.put(",\"ch\":[");
    for (std::size_t i = 0; i < chs.size(); ++i) {
        if (i > 0) w.put(",");
        w.put_uint(chs[i]);
    }
    w.put("]}\r\n");
    return w.ok ? w.pos : 0;
}

inline constexpr std::string_view kCliPrompt = "i6x> ";
inline constexpr std::string_view kHelpText =
    "Commands:\r\n"
    "  help      - Show this help\r\n"
    "  status    - Firmware & battery info\r\n"
    "  channels  - Dump RF channel values (CH1..CH18)\r\n"
    "  storage   - Dump EEPROM bus, address & I/O diagnostics\r\n"
    "  telem     - Dump single telemetry frame\r\n"
    "  stream    - Start continuous telemetry streaming (any key to stop)\r\n"
    "  dfu       - Reboot into STM32 factory ROM DFU bootloader\r\n"
    "  reboot    - Reboot transmitter\r\n";

// CDC line coding reported to the host: 115200 baud, 1 stop bit, no parity, 8 data bits.
inline constexpr auto kLineCoding = std::to_array<uint8_t>({0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08});

/// Fixed-capacity FIFO of bytes. Overflow is refused rather than overwriting.
template <std::size_t N>
struct ByteRing {
    static_assert(N > 0, "ring needs storage");
    std::array<uint8_t, N> buf{};
    std::size_t head = 0;
    std::size_t count = 0;

    constexpr std::size_t size() const { return count; }
    constexpr std::size_t free_space() const { return N - count; }
    constexpr void clear() {
        head = 0;
        count = 0;
    }

    constexpr bool push(const uint8_t v) {
        if (count == N) return false;
        buf[(head + count) % N] = v;
        ++count;
        return true;
    }

    // All-or-nothing: returns false and stores nothing if `s` does not fit.
    constexpr bool push_text(std::string_view s) {
        if (s.size() > free_space()) return false;
        for (char c : s) {
            push(static_cast<uint8_t>(c));
        }
        return true;
    }

    // Pushes as many bytes as fit; returns the count stored.
    constexpr std::size_t push_all(std::span<const uint8_t> src) {
        std::size_t n = 0;
        while (n < src.size() && push(src[n])) ++n;
        return n;
    }

    constexpr std::size_t pop_into(std::span<uint8_t> dst) {
        std::size_t n = 0;
        while (n < dst.size() && count > 0) {
            dst[n++] = buf[head];
            head = (head + 1) % N;
            --count;
        }
        return n;
    }
};

/// Line editing for the CLI: echo, backspace, and CR/LF/CRLF line ends (a CRLF pair ends one line).
struct LineEditor {
    enum class Action : uint8_t { None, Echo, Erase, EndOfLine };
    static constexpr std::size_t kCapacity = 64;

    std::array<char, kCapacity> buf{};
    std::size_t len = 0;
    bool last_cr = false;

    constexpr Action feed(const uint8_t b) {
        if (b == '\r' || b == '\n') {
            const bool second_of_pair = b == '\n' && last_cr;
            last_cr = b == '\r';
            return second_of_pair ? Action::None : Action::EndOfLine;
        }
        last_cr = false;
        if (b == 0x08 || b == 0x7F) {
            if (len == 0) return Action::None;
            --len;
            return Action::Erase;
        }
        if (len == kCapacity) return Action::None;
        buf[len++] = static_cast<char>(b);
        return Action::Echo;
    }

    constexpr std::string_view line() const { return {buf.data(), len}; }

    // Keeps the CR/LF pairing state so the LF of a CRLF sent after a line is still collapsed.
    constexpr void clear() { len = 0; }
};

}  // namespace ferrox::logic::usb
