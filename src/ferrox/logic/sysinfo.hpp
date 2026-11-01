#pragma once

#include <cstddef>
#include <string_view>

#include "ferrox/logic/format.hpp"
#include "ferrox/logic/storage.hpp"

namespace ferrox::logic::sysinfo {

struct FirmwareVersion {
    std::string_view text{};  // without the leading "v", e.g. "0.27.0"
    friend constexpr bool operator==(const FirmwareVersion&, const FirmwareVersion&) = default;
};

struct GitHash {
    std::string_view text{};  // short commit hash, or "unknown" when git was unavailable at build time
    friend constexpr bool operator==(const GitHash&, const GitHash&) = default;
};

struct Build {
    FirmwareVersion version{};
    GitHash hash{};
    friend constexpr bool operator==(const Build&, const Build&) = default;
};

inline constexpr std::size_t kFirmwareLineMax = 32;
inline constexpr std::size_t kStorageLineMax = 27;

// "v0.27.0 (abc1234)", the value of the Firmware row.
constexpr format::Text<kFirmwareLineMax> firmware_line(const Build& build) {
    format::Text<kFirmwareLineMax> t;
    t.push('v');
    for (const char c : build.version.text) {
        t.push(c);
    }
    for (const char c : std::string_view{" ("}) {
        t.push(c);
    }
    for (const char c : build.hash.text) {
        t.push(c);
    }
    t.push(')');
    return t;
}

// "Storage:  I2C2 @ 0x50 (OK)", or "Storage:  None (No EEPROM)" when no chip answered the probe.
// The EEPROM bus is always I2C2 in this port, so the bus name is fixed.
constexpr format::Text<kStorageLineMax> storage_line(const storage::EepromStatus& e) {
    format::Text<kStorageLineMax> t;
    for (const char c : std::string_view{"Storage:  "}) {
        t.push(c);
    }
    if (!e.detected) {
        for (const char c : std::string_view{"None (No EEPROM)"}) {
            t.push(c);
        }
        return t;
    }
    for (const char c : std::string_view{"I2C2 @ 0x"}) {
        t.push(c);
    }
    t.push(format::detail::hex_digit(e.addr >> 4));
    t.push(format::detail::hex_digit(e.addr));
    // The write error is checked first: a failed write leaves the status "(ERR)" even if the boot read worked.
    t.push(' ');
    for (const char c : std::string_view{e.write_err || !e.read_ok ? "(ERR)" : "(OK)"}) {
        t.push(c);
    }
    return t;
}

}  // namespace ferrox::logic::sysinfo
