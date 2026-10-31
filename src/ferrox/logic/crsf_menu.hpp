#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "ferrox/logic/crsf.hpp"

namespace ferrox::logic::crsf::menu {

inline constexpr uint8_t kNoParent = 0xFF;
inline constexpr std::size_t kTextLen = 32;
inline constexpr std::size_t kMaxParams = 48;
inline constexpr std::size_t kMaxParamMap = 96;
inline constexpr std::size_t kChunkBufSize = 256;
inline constexpr uint32_t kCmdStartTimeoutMs = 1500;
inline constexpr uint32_t kCmdRunTimeoutMs = 8000;
inline constexpr uint32_t kCmdDoneMs = 2500;
inline constexpr uint32_t kCmdDefaultPollMs = 250;

enum class Type : uint8_t {
    Uint8 = 0x00,
    Int8 = 0x01,
    Uint16 = 0x02,
    Int16 = 0x03,
    Float = 0x08,
    Select = 0x09,
    String = 0x0A,
    Folder = 0x0B,
    Info = 0x0C,
    Command = 0x0D,
    Back = 0x0E,
};

enum class CmdStatus : uint8_t {
    Ready = 0,
    Start = 1,
    Progress = 2,
    ConfirmationNeeded = 3,
    Confirm = 4,
    Cancel = 5,
    Poll = 6,
};

template <std::size_t N>
struct Text {
    std::array<char, N> data{};
    uint8_t len = 0;

    constexpr bool operator==(const Text&) const = default;
};

/// Index of the first NUL at or after start, or chunk.size() if none.
constexpr std::size_t find_nul(std::span<const uint8_t> c, std::size_t start) {
    std::size_t end = start;
    while (end < c.size() && c[end] != 0) {
        ++end;
    }
    return end;
}

/// Copies the NUL-terminated string at start (truncated to N) and returns the index of its NUL.
template <std::size_t N>
constexpr std::size_t copy_text(std::span<const uint8_t> c, std::size_t start, Text<N>& out) {
    const std::size_t end = find_nul(c, start);
    const std::size_t n = std::min(end - start, N);
    for (std::size_t i = 0; i < n; ++i) {
        out.data[i] = static_cast<char>(c[start + i]);
    }
    out.len = static_cast<uint8_t>(n);
    return end;
}

/// One parsed parameter entry. Float is recognised but its value is not decoded.
struct Entry {
    uint8_t parent = kNoParent;
    Type type = Type::Back;
    bool hidden = false;
    Text<kTextLen> name{};
    /// Select: ';'-separated options. Uint8/Int8/Uint16/Int16: unit. String/Info/Command: text.
    Text<kTextLen> text{};
    int32_t value = 0;
    int32_t min = 0;
    int32_t max = 0;
    /// Command only: timeout in units of 100 ms (0 means default).
    uint8_t cmd_timeout = 0;

    constexpr bool operator==(const Entry&) const = default;
};

/// Parses a reassembled PARAM_ENTRY payload (parent, type, name, type-specific data).
constexpr std::optional<Entry> parse_entry(std::span<const uint8_t> c) {
    if (c.size() < 3) {
        return std::nullopt;
    }
    Entry e{};
    e.parent = c[0];
    e.type = static_cast<Type>(c[1] & 0x7F);
    e.hidden = (c[1] & 0x80) != 0;
    const std::size_t rest = copy_text(c, 2, e.name) + 1;
    if (rest >= c.size()) {
        return e;
    }

    switch (e.type) {
        case Type::Select: {
            const std::size_t end = copy_text(c, rest, e.text);
            uint8_t count = 1;
            for (std::size_t i = rest; i < end; ++i) {
                if (c[i] == ';') {
                    ++count;
                }
            }
            e.min = 0;
            e.max = count - 1;
            if (end + 1 < c.size()) {
                e.value = c[end + 1];
            }
            break;
        }
        case Type::Uint8:
        case Type::Int8:
            if (rest + 3 <= c.size()) {
                if (e.type == Type::Uint8) {
                    e.value = c[rest];
                    e.min = c[rest + 1];
                    e.max = c[rest + 2];
                } else {
                    e.value = static_cast<int8_t>(c[rest]);
                    e.min = static_cast<int8_t>(c[rest + 1]);
                    e.max = static_cast<int8_t>(c[rest + 2]);
                }
                if (rest + 4 < c.size()) {
                    copy_text(c, rest + 4, e.text);
                }
            }
            break;
        case Type::Uint16:
        case Type::Int16:
            if (rest + 6 <= c.size()) {
                const auto be16 = [&](std::size_t i) {
                    return static_cast<uint16_t>((c[i] << 8) | c[i + 1]);
                };
                if (e.type == Type::Uint16) {
                    e.value = be16(rest);
                    e.min = be16(rest + 2);
                    e.max = be16(rest + 4);
                } else {
                    e.value = static_cast<int16_t>(be16(rest));
                    e.min = static_cast<int16_t>(be16(rest + 2));
                    e.max = static_cast<int16_t>(be16(rest + 4));
                }
                if (rest + 8 < c.size()) {
                    copy_text(c, rest + 8, e.text);
                }
            }
            break;
        case Type::String:
        case Type::Info:
            copy_text(c, rest, e.text);
            break;
        case Type::Command:
            e.value = c[rest];
            if (rest + 1 < c.size()) {
                e.cmd_timeout = c[rest + 1];
            }
            if (rest + 2 < c.size()) {
                copy_text(c, rest + 2, e.text);
            }
            break;
        case Type::Float:
        case Type::Folder:
        case Type::Back:
            break;
        default:
            return std::nullopt;
    }
    return e;
}

/// Clamps a value to the entry's [min, max] range. Non-numeric types return the input unchanged.
constexpr int32_t clamp_value(const Entry& e, int32_t v) {
    return std::clamp(v, e.min, e.max);
}

/// Next value of a select entry (wraps around). Other types are unchanged.
constexpr int32_t cycled_value(const Entry& e) {
    if (e.type != Type::Select || e.max < 0) {
        return e.value;
    }
    return (e.value + 1) % (e.max + 1);
}

/// Encodes a value for PARAM_WRITE. Returns the number of payload bytes (0 if the type is not writable).
constexpr std::size_t encode_value(const Entry& e, int32_t v, std::array<uint8_t, 2>& out) {
    const int32_t x = clamp_value(e, v);
    switch (e.type) {
        case Type::Select:
        case Type::Uint8:
        case Type::Int8:
            out[0] = static_cast<uint8_t>(x);
            return 1;
        case Type::Uint16:
        case Type::Int16:
            out[0] = static_cast<uint8_t>((x >> 8) & 0xFF);
            out[1] = static_cast<uint8_t>(x & 0xFF);
            return 2;
        default:
            return 0;
    }
}

/// Reassembles a multi-chunk parameter stream into a fixed buffer.
struct Reassembler {
    enum class Result : uint8_t { Ignored, NeedMore, Complete };

    std::array<uint8_t, kChunkBufSize> buf{};
    std::size_t len = 0;
    uint8_t param = 0;
    uint8_t current_chunk = 0;
    uint8_t expect_remain = 0;

    constexpr void reset() {
        len = 0;
        current_chunk = 0;
        expect_remain = 0;
    }

    constexpr std::span<const uint8_t> bytes() const { return {buf.data(), len}; }

    constexpr Result feed(uint8_t param_id, uint8_t chunks_remain, std::span<const uint8_t> data) {
        if (current_chunk > 0 && chunks_remain != expect_remain) {
            return Result::Ignored;
        }
        if (current_chunk == 0 || param != param_id) {
            len = 0;
            param = param_id;
        }
        const std::size_t n = std::min(data.size(), buf.size() - len);
        std::copy_n(data.begin(), n, buf.begin() + len);
        len += n;
        if (chunks_remain > 0) {
            ++current_chunk;
            expect_remain = chunks_remain - 1;
            return Result::NeedMore;
        }
        return Result::Complete;
    }
};

/// Next parameter id after current_id that belongs to target_folder. Legacy devices without
/// a root folder (no parent info) return every unmapped parameter.
constexpr std::optional<uint8_t> next_param_for_folder(uint8_t current_id, uint8_t target_folder,
                                                       uint8_t param_count,
                                                       const std::array<uint8_t, kMaxParamMap>& parent_map,
                                                       bool has_root_folder) {
    for (uint16_t id = static_cast<uint16_t>(current_id) + 1; id <= param_count; ++id) {
        const uint8_t parent = id < kMaxParamMap ? parent_map[id] : kNoParent;
        if (parent == target_folder || (!has_root_folder && parent == kNoParent)) {
            return static_cast<uint8_t>(id);
        }
    }
    return std::nullopt;
}

/// True if now has reached or passed deadline (wrap-safe for 32-bit millisecond counters).
constexpr bool due(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

/// Frame the caller must transmit, or nothing.
struct Action {
    enum class Kind : uint8_t { None, Read, Write, Done };

    Kind kind = Kind::None;
    uint8_t param = 0;
    /// Read: chunk index. Write: value byte(s) as payload value (for commands).
    uint8_t arg = 0;
};

/// Loads all parameters of one device in folder order, with retries and chunk handling.
struct Loader {
    enum class State : uint8_t { Idle, Loading, Ready };

    State state = State::Idle;
    uint8_t device = 0;
    uint8_t param_count = 0;
    uint8_t folder = 0;
    bool has_root_folder = true;
    uint8_t loading_id = 0;
    uint8_t retry = 0;
    uint32_t next_req_ms = 0;
    Reassembler chunks{};
    std::array<uint8_t, kMaxParamMap> parent_map{};
    std::array<std::optional<Entry>, kMaxParams> entries{};
    /// Set to the parameter id whose entry was completed by the last on_entry call.
    std::optional<uint8_t> completed{};

    constexpr uint32_t timeout() const { return device == kAddrCrsfReceiver ? 1000 : 500; }

    constexpr Action start(uint8_t dev, uint8_t count, uint32_t now) {
        device = dev;
        param_count = count;
        folder = 0;
        has_root_folder = true;
        parent_map.fill(kNoParent);
        entries = {};
        state = State::Loading;
        loading_id = 0;
        retry = 0;
        chunks.reset();
        next_req_ms = now + timeout();
        return {Action::Kind::Read, 0, 0};
    }

    constexpr void stop() {
        state = State::Idle;
        chunks.reset();
    }

    /// Handles a PARAM_ENTRY frame. origin is the sender address, data is the chunk payload.
    constexpr Action on_entry(uint8_t origin, uint8_t param_id, uint8_t chunks_remain,
                              std::span<const uint8_t> data, uint32_t now) {
        completed.reset();
        if (origin != device) {
            return {};
        }
        if (state == State::Loading && param_id != loading_id) {
            return {};
        }
        switch (chunks.feed(param_id, chunks_remain, data)) {
            case Reassembler::Result::Ignored:
                return {};
            case Reassembler::Result::NeedMore:
                retry = 0;
                next_req_ms = now + timeout();
                return {Action::Kind::Read, param_id, chunks.current_chunk};
            case Reassembler::Result::Complete:
                break;
        }

        const auto parsed = parse_entry(chunks.bytes());
        if (parsed && parsed->type == Type::Folder) {
            // Folder entries list their child ids after the name, terminated by 0xFF.
            if (param_id == 0) {
                has_root_folder = true;
            }
            const auto c = chunks.bytes();
            for (std::size_t i = find_nul(c, 2) + 1; i < c.size() && c[i] != 0xFF; ++i) {
                if (c[i] < kMaxParamMap) {
                    parent_map[c[i]] = param_id;
                }
            }
        }
        chunks.reset();
        retry = 0;
        completed = param_id;
        if (parsed) {
            if (param_id < kMaxParamMap) {
                parent_map[param_id] = parsed->parent;
            }
            if (param_id < kMaxParams) {
                entries[param_id] = parsed;
            }
        }
        if (state == State::Loading && param_id == loading_id) {
            return advance(now, param_id);
        }
        return {};
    }

    /// Retry and timeout handling. Call periodically with the current millisecond time.
    constexpr Action tick(uint32_t now) {
        if (state != State::Loading || !due(now, next_req_ms)) {
            return {};
        }
        const uint8_t max_retries = loading_id == 0 ? 2 : 4;
        if (retry < max_retries) {
            ++retry;
            next_req_ms = now + timeout();
            return {Action::Kind::Read, loading_id, chunks.current_chunk};
        }

        // Retries exhausted: give up on this parameter (packet lost over the air).
        retry = 0;
        chunks.reset();
        if (loading_id == 0) {
            // No root folder support: fall back to legacy sequential discovery from id 1.
            has_root_folder = false;
            if (param_count >= 1) {
                loading_id = 1;
                next_req_ms = now + timeout();
                return {Action::Kind::Read, 1, 0};
            }
            state = State::Ready;
            return {Action::Kind::Done, 0, 0};
        }
        return advance(now, loading_id);
    }

    /// Continues with the next parameter in the current folder, or finishes.
    constexpr Action advance(uint32_t now, uint8_t after_id) {
        const auto next = next_param_for_folder(after_id, folder, param_count, parent_map, has_root_folder);
        if (!next) {
            state = State::Ready;
            return {Action::Kind::Done, 0, 0};
        }
        loading_id = *next;
        retry = 0;
        chunks.reset();
        next_req_ms = now + timeout();
        return {Action::Kind::Read, *next, 0};
    }

    /// Enters a folder and reloads its children. Returns nothing if the folder is not known.
    constexpr Action enter_folder(uint8_t folder_id, uint32_t now) {
        if (state != State::Ready) {
            return {};
        }
        folder = folder_id;
        state = State::Loading;
        const auto first = next_param_for_folder(0, folder, param_count, parent_map, has_root_folder);
        if (!first) {
            state = State::Ready;
            return {Action::Kind::Done, 0, 0};
        }
        loading_id = *first;
        retry = 0;
        chunks.reset();
        next_req_ms = now + timeout();
        return {Action::Kind::Read, loading_id, 0};
    }
};

/// Command (type COMMAND) state machine. Produces writes for start, confirm, cancel and poll.
struct Command {
    enum class Kind : uint8_t { Idle, Starting, WaitingConfirm, Running, Completed };

    Kind kind = Kind::Idle;
    uint8_t param = 0;
    uint32_t deadline_ms = 0;
    uint32_t poll_at_ms = 0;
    uint32_t poll_interval_ms = kCmdDefaultPollMs;

    constexpr Action start(uint8_t id, uint32_t now) {
        param = id;
        kind = Kind::Starting;
        deadline_ms = now + kCmdStartTimeoutMs;
        return {Action::Kind::Write, id, static_cast<uint8_t>(CmdStatus::Start)};
    }

    constexpr Action confirm(uint32_t now) {
        if (kind != Kind::WaitingConfirm) {
            return {};
        }
        kind = Kind::Starting;
        deadline_ms = now + kCmdStartTimeoutMs;
        return {Action::Kind::Write, param, static_cast<uint8_t>(CmdStatus::Confirm)};
    }

    constexpr Action cancel() {
        if (kind == Kind::Idle) {
            return {};
        }
        const uint8_t id = param;
        kind = Kind::Idle;
        return {Action::Kind::Write, id, static_cast<uint8_t>(CmdStatus::Cancel)};
    }

    /// Applies a status frame for the given command parameter.
    constexpr void on_status(uint8_t id, uint8_t status, uint8_t timeout_byte, uint32_t now) {
        if (id != param || (kind != Kind::Starting && kind != Kind::Running)) {
            return;
        }
        poll_interval_ms = timeout_byte > 0 ? static_cast<uint32_t>(timeout_byte) * 100 : kCmdDefaultPollMs;
        switch (static_cast<CmdStatus>(status)) {
            case CmdStatus::ConfirmationNeeded:
                kind = Kind::WaitingConfirm;
                break;
            case CmdStatus::Progress:
                kind = Kind::Running;
                poll_at_ms = now + poll_interval_ms;
                deadline_ms = now + kCmdRunTimeoutMs;
                break;
            case CmdStatus::Ready:
                kind = Kind::Completed;
                deadline_ms = now + kCmdDoneMs;
                break;
            default:
                break;
        }
    }

    /// Timeouts and periodic polling while the command runs.
    constexpr Action tick(uint32_t now) {
        switch (kind) {
            case Kind::Starting:
                if (due(now, deadline_ms)) {
                    kind = Kind::Idle;
                }
                break;
            case Kind::Running:
                if (due(now, deadline_ms)) {
                    kind = Kind::Idle;
                } else if (due(now, poll_at_ms)) {
                    poll_at_ms = now + poll_interval_ms;
                    return {Action::Kind::Write, param, static_cast<uint8_t>(CmdStatus::Poll)};
                }
                break;
            case Kind::Completed:
                if (due(now, deadline_ms)) {
                    kind = Kind::Idle;
                }
                break;
            default:
                break;
        }
        return {};
    }
};

}  // namespace ferrox::logic::crsf::menu
