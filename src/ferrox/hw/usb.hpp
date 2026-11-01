#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/gpio.hpp"
#include "ferrox/hw/keys.hpp"
#include "ferrox/logic/usb.hpp"

namespace ferrox::hw::usb {

namespace lu = logic::usb;
using lu::UsbMode;

// PA11: USB DM, PA12: USB DP (dedicated USB pins, AF0).
using Dm = Pin<Port::A, 11>;
using Dp = Pin<Port::A, 12>;

// USB SRAM: 1 KB. Each 16-bit field sits on a 32-bit word, so the logical
// PMA byte offset `o` maps to hardware address kPmaBase + 2 * o.
inline constexpr uintptr_t kPmaBase = 0x4000'6000u;
inline constexpr uint16_t kBtableOff = 0;
inline constexpr uint16_t kEp0TxOff = 32;
inline constexpr uint16_t kEp0RxOff = 96;
inline constexpr uint16_t kRxCountControl = 0x8400;  // 64-byte buffer (BLSIZE=1, NUM_BLOCK=1)
inline constexpr uint8_t kHidEp = 1;
inline constexpr uint16_t kHidPmaOff = 160;
inline constexpr uint16_t kCdcInPmaOff = 192;
inline constexpr uint16_t kCdcOutPmaOff = 256;
inline constexpr uint16_t kNotifyPmaOff = 320;
inline constexpr uint8_t kCdcEp = 2;      // bulk IN (0x82) and bulk OUT (0x02) share EP2
inline constexpr uint8_t kNotifyEp = 3;   // interrupt IN (0x83)
inline constexpr uint8_t kCdcPacket = 64;
inline constexpr uint32_t kTelemIntervalMs = 100;
inline constexpr uint32_t kResetGraceMs = 200;
inline constexpr std::size_t kTelemBufSize = 160;

inline constexpr uint8_t kIrqPriorityRaw = 0x40;
inline constexpr uint8_t kNoAddress = 0xFF;
inline constexpr uint32_t kSpinCycles = 4000;

inline constexpr uint16_t kEpKeepMask = USB_EP_T_FIELD | USB_EP_KIND | 0x000FU;
inline constexpr uint16_t kEpCtrMask = USB_EP_CTR_RX | USB_EP_CTR_TX;
inline constexpr uint16_t kTxStatMask = USB_EP_TX_VALID;
inline constexpr uint16_t kRxStatMask = USB_EP_RX_VALID;

enum class Dir : uint8_t { Rx, Tx };
enum class Stage : uint8_t { Idle, DataIn, DataOut, StatusIn, StatusOut };
enum class Reset : uint8_t { None, Reboot, Dfu };

struct Setup {
    uint8_t type;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
};

inline constinit UsbMode g_mode = UsbMode::Off;
inline constinit uint8_t g_config = 0;
inline constinit uint8_t g_pending_addr = kNoAddress;
inline constinit Stage g_stage = Stage::Idle;
inline constinit bool g_zlp = false;
inline constinit std::span<const uint8_t> g_ctl_data{};
inline constinit lu::Blob<100> g_ctl_store{};
inline constinit std::array<uint8_t, lu::kReportSize> g_last_report{};
inline constinit lu::Millis g_hid_last{};
inline constinit uint8_t g_hid_idle = 0;
// RX is filled from the ISR and drained by poll(); TX, the editor and the streaming state are main-context only.
inline constinit lu::ByteRing<128> g_rx{};
inline constinit lu::ByteRing<512> g_tx{};
inline constinit lu::LineEditor g_line{};
inline constinit bool g_streaming = false;
inline constinit bool g_prompted = false;
inline constinit lu::Millis g_stream_last{};
inline constinit Reset g_reset = Reset::None;
inline constinit lu::Millis g_reset_at{};

inline volatile uint16_t& ep_reg(const uint8_t n) {
    return reinterpret_cast<volatile uint16_t*>(USB)[2u * n];
}

inline volatile uint16_t& pma(const uint16_t off) {
    return *reinterpret_cast<volatile uint16_t*>(kPmaBase + 2u * off);
}

inline void spin(const uint32_t cycles) {
    for (uint32_t i = 0; i < cycles; ++i) {
        __NOP();
    }
}

// CTR bits are rc_w0 (write 1 to keep), DTOG and STAT bits toggle on write 1.
inline void ep_set_status(const uint8_t n, const uint16_t stat_mask, const uint16_t status) {
    volatile uint16_t& r = ep_reg(n);
    const uint16_t cur = r;
    r = static_cast<uint16_t>((cur & kEpKeepMask) | kEpCtrMask | ((cur ^ status) & stat_mask));
}

inline void ep_set_dtog(const uint8_t n, const uint16_t dtog_bit, const bool one) {
    volatile uint16_t& r = ep_reg(n);
    const uint16_t cur = r;
    const bool is_one = (cur & dtog_bit) != 0;
    r = static_cast<uint16_t>((cur & kEpKeepMask) | kEpCtrMask | (is_one != one ? dtog_bit : 0));
}

// Clears the CTR flag for `dir` and keeps the other direction's flag.
inline void ep_ack(const uint8_t n, const Dir dir) {
    volatile uint16_t& r = ep_reg(n);
    const uint16_t cur = r;
    r = static_cast<uint16_t>((cur & kEpKeepMask) | (dir == Dir::Rx ? USB_EP_CTR_TX : USB_EP_CTR_RX));
}

inline void pma_write(const uint16_t off, std::span<const uint8_t> src) {
    for (std::size_t i = 0; i < src.size(); i += 2) {
        const uint16_t lo = src[i];
        const uint16_t hi = i + 1 < src.size() ? src[i + 1] : 0;
        pma(static_cast<uint16_t>(off + i)) = static_cast<uint16_t>(lo | (hi << 8));
    }
}

inline void pma_read(const uint16_t off, std::span<uint8_t> dst) {
    for (std::size_t i = 0; i < dst.size(); i += 2) {
        const uint16_t w = pma(static_cast<uint16_t>(off + i));
        dst[i] = static_cast<uint8_t>(w & 0xFF);
        if (i + 1 < dst.size()) {
            dst[i + 1] = static_cast<uint8_t>(w >> 8);
        }
    }
}

// BTABLE entry for endpoint n lives at logical offset 8 * n.
inline void set_buffers(const uint8_t n, const uint16_t tx_off, const uint16_t rx_off, const uint16_t rx_count) {
    const auto base = static_cast<uint16_t>(8u * n);
    pma(base) = tx_off;
    pma(static_cast<uint16_t>(base + 2)) = 0;
    pma(static_cast<uint16_t>(base + 4)) = rx_off;
    pma(static_cast<uint16_t>(base + 6)) = rx_count;
}

inline void ep_send(const uint8_t n, const uint16_t tx_off, std::span<const uint8_t> data) {
    pma_write(tx_off, data);
    pma(static_cast<uint16_t>(8u * n + 2)) = static_cast<uint16_t>(data.size());
    ep_set_status(n, kTxStatMask, USB_EP_TX_VALID);
}

inline void ep0_send_next() {
    const auto n = std::min<std::size_t>(g_ctl_data.size(), lu::kControlPacketSize);
    ep_send(0, kEp0TxOff, g_ctl_data.first(n));
    g_ctl_data = g_ctl_data.subspan(n);
}

inline void ep0_stall() {
    ep_set_status(0, kTxStatMask, USB_EP_TX_STALL);
    ep_set_status(0, kRxStatMask, USB_EP_RX_STALL);
}

// Data stage IN: the host asked for `wanted` bytes of `data`.
inline void ep0_send_data(std::span<const uint8_t> data, const uint16_t wanted) {
    const auto len = std::min<std::size_t>(data.size(), wanted);
    g_ctl_data = data.first(len);
    g_zlp = len > 0 && len < wanted && len % lu::kControlPacketSize == 0;
    g_stage = Stage::DataIn;
    ep_set_dtog(0, USB_EP_DTOG_TX, true);
    ep0_send_next();
}

inline void ep0_status_in() {
    g_stage = Stage::StatusIn;
    ep_set_dtog(0, USB_EP_DTOG_TX, true);
    ep_send(0, kEp0TxOff, {});
}

inline void ep0_expect_out(const Stage stage) {
    g_stage = stage;
    ep_set_dtog(0, USB_EP_DTOG_RX, true);
    ep_set_status(0, kRxStatMask, USB_EP_RX_VALID);
}

template <std::size_t N>
inline void ep0_stage_blob(const lu::Blob<N>& blob, const uint16_t wanted) {
    static_assert(N <= 100, "control staging buffer too small");
    g_ctl_store = {};
    g_ctl_store.put_all(blob.view());
    ep0_send_data(g_ctl_store.view(), wanted);
}

inline void open_cdc() {
    ep_reg(kCdcEp) = kCdcEp;
    set_buffers(kCdcEp, kCdcInPmaOff, kCdcOutPmaOff, kRxCountControl);
    ep_set_status(kCdcEp, kTxStatMask, USB_EP_TX_NAK);
    ep_set_status(kCdcEp, kRxStatMask, USB_EP_RX_VALID);

    ep_reg(kNotifyEp) = static_cast<uint16_t>(USB_EP_INTERRUPT | kNotifyEp);
    set_buffers(kNotifyEp, kNotifyPmaOff, 0, 0);
    ep_set_status(kNotifyEp, kTxStatMask, USB_EP_TX_NAK);
}

inline void open_hid() {
    ep_reg(kHidEp) = static_cast<uint16_t>(USB_EP_INTERRUPT | kHidEp);
    set_buffers(kHidEp, kHidPmaOff, 0, 0);
    ep_set_status(kHidEp, kTxStatMask, USB_EP_TX_NAK);
}

inline void configure_endpoints() {
    if (g_config == 0) {
        return;
    }
    if (lu::has_hid(g_mode)) {
        open_hid();
    }
    if (lu::has_cdc(g_mode)) {
        open_cdc();
    }
}

inline void handle_get_descriptor(const Setup& s) {
    const auto type = static_cast<uint8_t>(s.value >> 8);
    const auto index = static_cast<uint8_t>(s.value & 0xFF);
    switch (type) {
        case 1: ep0_stage_blob(lu::device_descriptor(g_mode), s.length); break;
        case 2: ep0_stage_blob(lu::config_descriptor(g_mode), s.length); break;
        case 3: ep0_stage_blob(lu::string_descriptor(g_mode, index), s.length); break;
        case 0x22:  // HID report descriptor
            if (lu::has_hid(g_mode)) {
                ep0_send_data(std::span<const uint8_t>(lu::kGamepadReportDescriptor), s.length);
            } else {
                ep0_stall();
            }
            break;
        default: ep0_stall(); break;
    }
}

inline void handle_standard(const Setup& s) {
    switch (s.request) {
        case 0x00: {  // GET_STATUS
            g_ctl_store = {};
            g_ctl_store.put(0);
            g_ctl_store.put(0);
            ep0_send_data(g_ctl_store.view(), s.length);
            break;
        }
        case 0x05:  // SET_ADDRESS: applied after the status stage
            g_pending_addr = static_cast<uint8_t>(s.value & 0x7F);
            ep0_status_in();
            break;
        case 0x06: handle_get_descriptor(s); break;
        case 0x08: {  // GET_CONFIGURATION
            g_ctl_store = {};
            g_ctl_store.put(g_config);
            ep0_send_data(g_ctl_store.view(), s.length);
            break;
        }
        case 0x09:  // SET_CONFIGURATION
            g_config = static_cast<uint8_t>(s.value & 0xFF);
            configure_endpoints();
            ep0_status_in();
            break;
        case 0x0A: {  // GET_INTERFACE
            g_ctl_store = {};
            g_ctl_store.put(0);
            ep0_send_data(g_ctl_store.view(), s.length);
            break;
        }
        case 0x0B:  // SET_INTERFACE
            ep0_status_in();
            break;
        default: ep0_stall(); break;
    }
}

// CDC-ACM control requests. Line coding is accepted and not applied: the UART-facing rate is irrelevant on USB.
inline void handle_cdc(const Setup& s) {
    switch (s.request) {
        case 0x20:  // SET_LINE_CODING: 7-byte data stage from the host
            if (s.length == lu::kLineCoding.size()) {
                ep0_expect_out(Stage::DataOut);
            } else {
                ep0_stall();
            }
            break;
        case 0x21:  // GET_LINE_CODING
            ep0_send_data(std::span<const uint8_t>(lu::kLineCoding), s.length);
            break;
        case 0x22:  // SET_CONTROL_LINE_STATE (DTR/RTS not used)
            ep0_status_in();
            break;
        default: ep0_stall(); break;
    }
}

// HID class requests (interface recipient). Unknown requests stall.
inline void handle_hid(const Setup& s) {
    if (!lu::has_hid(g_mode)) {
        ep0_stall();
        return;
    }
    switch (s.request) {
        case 0x01:  // GET_REPORT: last gamepad report
            ep0_send_data(g_last_report, s.length);
            break;
        case 0x02:  // GET_IDLE
            g_ctl_store = {};
            g_ctl_store.put(g_hid_idle);
            ep0_send_data(g_ctl_store.view(), s.length);
            break;
        case 0x0A:  // SET_IDLE
            g_hid_idle = static_cast<uint8_t>(s.value >> 8);
            ep0_status_in();
            break;
        case 0x0B:  // SET_PROTOCOL (report protocol only)
            ep0_status_in();
            break;
        default: ep0_stall(); break;
    }
}

// Class requests route by interface number: the CDC control interface, otherwise HID.
inline void handle_class(const Setup& s) {
    const auto iface = static_cast<uint8_t>(s.index & 0xFF);
    if (lu::has_cdc(g_mode) && iface == lu::layout(g_mode).ctrl_if) {
        handle_cdc(s);
    } else {
        handle_hid(s);
    }
}

inline void ep0_setup() {
    ep_set_status(0, kRxStatMask, USB_EP_RX_NAK);
    ep_set_status(0, kTxStatMask, USB_EP_TX_NAK);

    uint8_t raw[8];
    pma_read(kEp0RxOff, raw);
    const Setup s{raw[0],
                  raw[1],
                  static_cast<uint16_t>(raw[2] | (raw[3] << 8)),
                  static_cast<uint16_t>(raw[4] | (raw[5] << 8)),
                  static_cast<uint16_t>(raw[6] | (raw[7] << 8))};

    const uint8_t kind = (s.type >> 5) & 0x3;
    if (kind == 0) {
        handle_standard(s);
    } else if (kind == 1) {
        handle_class(s);
    } else {
        ep0_stall();
    }
}

inline void ep0_rx_done() {
    if (g_stage == Stage::StatusOut) {
        g_stage = Stage::Idle;
    } else if (g_stage == Stage::DataOut) {
        ep0_status_in();
    }
    ep_set_status(0, kRxStatMask, USB_EP_RX_NAK);
}

inline void ep0_tx_done() {
    switch (g_stage) {
        case Stage::DataIn:
            if (!g_ctl_data.empty()) {
                ep0_send_next();
            } else if (g_zlp) {
                g_zlp = false;
                ep_send(0, kEp0TxOff, {});
            } else {
                ep0_expect_out(Stage::StatusOut);
            }
            break;
        case Stage::StatusIn:
            g_stage = Stage::Idle;
            if (g_pending_addr != kNoAddress) {
                USB->DADDR = static_cast<uint16_t>(USB_DADDR_EF | g_pending_addr);
                g_pending_addr = kNoAddress;
            }
            ep_set_status(0, kTxStatMask, USB_EP_TX_NAK);
            break;
        default:
            ep_set_status(0, kTxStatMask, USB_EP_TX_NAK);
            break;
    }
}

inline void on_reset() {
    USB->BTABLE = kBtableOff;
    ep_reg(0) = USB_EP_CONTROL;
    set_buffers(0, kEp0TxOff, kEp0RxOff, kRxCountControl);
    ep_set_status(0, kRxStatMask, USB_EP_RX_VALID);
    ep_set_status(0, kTxStatMask, USB_EP_TX_NAK);
    USB->DADDR = USB_DADDR_EF;
    g_config = 0;
    g_pending_addr = kNoAddress;
    g_stage = Stage::Idle;
    g_zlp = false;
    g_ctl_data = {};
}

// Bulk OUT: copy the packet into the RX ring (bytes beyond its free space are dropped), then re-arm.
// Bulk IN completion only needs its CTR flag cleared; poll() queues the next chunk.
inline void cdc_isr(const uint16_t reg) {
    if ((reg & USB_EP_CTR_RX) != 0) {
        const auto count = static_cast<std::size_t>(pma(static_cast<uint16_t>(8u * kCdcEp + 6)) & 0x3FFu);
        const std::size_t n = std::min<std::size_t>(count, kCdcPacket);
        std::array<uint8_t, kCdcPacket> buf{};
        pma_read(kCdcOutPmaOff, std::span<uint8_t>(buf).first(n));
        g_rx.push_all(std::span<const uint8_t>(buf).first(n));
        ep_ack(kCdcEp, Dir::Rx);
        ep_set_status(kCdcEp, kRxStatMask, USB_EP_RX_VALID);
    }
    if ((reg & USB_EP_CTR_TX) != 0) {
        ep_ack(kCdcEp, Dir::Tx);
    }
}

/// USB interrupt body: resets, control transfers on EP0, and class endpoints.
inline void on_irq() {
    for (uint8_t guard = 0; guard < 16; ++guard) {
        const uint16_t istr = USB->ISTR;
        if ((istr & USB_ISTR_RESET) != 0) {
            USB->ISTR = static_cast<uint16_t>(~USB_ISTR_RESET);
            on_reset();
            continue;
        }
        const uint16_t misc = USB_ISTR_SUSP | USB_ISTR_WKUP | USB_ISTR_ERR | USB_ISTR_PMAOVR;
        if ((istr & misc) != 0) {
            USB->ISTR = static_cast<uint16_t>(~(istr & misc));
        }
        if ((istr & USB_ISTR_CTR) == 0) {
            break;
        }
        const auto ep_id = static_cast<uint8_t>(istr & USB_ISTR_EP_ID);
        if (ep_id == kCdcEp) {
            cdc_isr(ep_reg(ep_id));
            continue;
        }
        if (ep_id != 0) {
            // Class endpoints: the IN completion sets STAT back to NAK, so poll() can send the next report.
            const uint16_t reg = ep_reg(ep_id);
            if ((reg & USB_EP_CTR_TX) != 0) {
                ep_ack(ep_id, Dir::Tx);
            }
            if ((reg & USB_EP_CTR_RX) != 0) {
                ep_ack(ep_id, Dir::Rx);
            }
            continue;
        }
        const uint16_t ep = ep_reg(0);
        if ((ep & USB_EP_CTR_RX) != 0) {
            ep_ack(0, Dir::Rx);
            if ((ep & USB_EP_SETUP) != 0) {
                ep0_setup();
            } else {
                ep0_rx_done();
            }
        }
        if ((ep_reg(0) & USB_EP_CTR_TX) != 0) {
            ep_ack(0, Dir::Tx);
            ep0_tx_done();
        }
    }
}

/// Sends the gamepad report on the HID IN endpoint every kHidIntervalMs when the previous one has been taken.
inline void poll_hid(const lu::Millis now, std::span<const uint16_t, logic::kNumChannels> chs, const logic::Switches& sw) {
    if (!lu::has_hid(g_mode) || g_config == 0) {
        return;
    }
    if (!lu::interval_elapsed(now, g_hid_last, lu::Millis{lu::kHidIntervalMs})) {
        return;
    }
    if ((ep_reg(kHidEp) & USB_EP_TX_VALID) != USB_EP_TX_NAK) {
        return;
    }
    g_last_report = lu::build_gamepad_report(chs, sw);
    g_hid_last = now;
    ep_send(kHidEp, kHidPmaOff, g_last_report);
}

// Pops from the RX ring with interrupts masked, so the bulk OUT ISR cannot push mid-pop.
inline std::size_t rx_take(std::span<uint8_t> dst) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const std::size_t n = g_rx.pop_into(dst);
    if ((primask & 1u) == 0u) {
        __enable_irq();
    }
    return n;
}

inline void cdc_pump_tx() {
    if (g_tx.size() == 0 || (ep_reg(kCdcEp) & USB_EP_TX_VALID) != USB_EP_TX_NAK) {
        return;
    }
    std::array<uint8_t, kCdcPacket> chunk{};
    const std::size_t n = g_tx.pop_into(chunk);
    ep_send(kCdcEp, kCdcInPmaOff, std::span<const uint8_t>(chunk).first(n));
}

inline void cdc_reset_state() {
    g_rx.clear();
    g_tx.clear();
    g_line.clear();
    g_streaming = false;
    g_prompted = false;
    g_reset = Reset::None;
}

inline void request_reset(const Reset kind, const lu::Millis now) {
    g_reset = kind;
    g_reset_at = now;
}

[[noreturn]] inline void perform_reset() {
    if (g_reset == Reset::Dfu) {
        *reinterpret_cast<volatile uint32_t*>(keys::kDfuFlagAddr) = keys::kDfuMagic;
    }
    NVIC_SystemReset();
}

// Returns true when the CLI should print its prompt again.
inline bool run_command(const lu::Command cmd, const lu::Millis now,
                        std::span<const uint16_t, logic::kNumChannels> chs, const uint16_t battery_mv) {
    std::array<char, kTelemBufSize> buf{};
    const auto push_buf = [&buf](std::size_t n) {
        g_tx.push_text(std::string_view(buf.data(), n));
    };
    switch (cmd) {
        case lu::Command::Empty:
            return true;
        case lu::Command::Help:
            g_tx.push_text(lu::kHelpText);
            return true;
        case lu::Command::Status:
            push_buf(lu::format_telemetry_json(battery_mv, {}, buf));
            return true;
        case lu::Command::Channels:
            push_buf(lu::format_channels_json(chs, buf));
            return true;
        case lu::Command::Storage:
            g_tx.push_text("Storage diagnostics are not available on this port\r\n");
            return true;
        case lu::Command::Telem:
            push_buf(lu::format_telemetry_json(battery_mv, chs, buf));
            return true;
        case lu::Command::Stream:
            g_tx.push_text("[Streaming telemetry @ 10Hz. Press any key to stop.]\r\n");
            g_streaming = true;
            g_stream_last = now;
            return false;
        case lu::Command::TelemOff:
            g_streaming = false;
            g_tx.push_text("Telemetry streaming disabled\r\n");
            return true;
        case lu::Command::Dfu:
            g_tx.push_text("Rebooting to STM32 DFU bootloader...\r\n");
            request_reset(Reset::Dfu, now);
            return false;
        case lu::Command::Reboot:
            g_tx.push_text("Rebooting...\r\n");
            request_reset(Reset::Reboot, now);
            return false;
        case lu::Command::Unknown:
            g_tx.push_text("Unknown command. Type 'help'\r\n");
            return true;
    }
    return true;
}

inline void handle_rx_byte(const uint8_t b, const lu::Millis now,
                           std::span<const uint16_t, logic::kNumChannels> chs, const uint16_t battery_mv) {
    const auto action = g_line.feed(b);
    if (g_streaming) {
        // Any key ends streaming; the LF of a CRLF pair is not a keypress.
        if (action == lu::LineEditor::Action::None) {
            return;
        }
        g_line.clear();
        g_streaming = false;
        g_tx.push_text("\r\n");
        g_tx.push_text(lu::kCliPrompt);
        return;
    }
    switch (action) {
        case lu::LineEditor::Action::Echo:
            g_tx.push(b);
            break;
        case lu::LineEditor::Action::Erase:
            g_tx.push_text("\b \b");
            break;
        case lu::LineEditor::Action::EndOfLine: {
            g_tx.push_text("\r\n");
            const lu::Command cmd = lu::parse_command(g_line.line());
            g_line.clear();
            if (run_command(cmd, now, chs, battery_mv)) {
                g_tx.push_text(lu::kCliPrompt);
            }
            break;
        }
        case lu::LineEditor::Action::None:
            break;
    }
}

// CLI and telemetry streaming on the CDC pair, in main context. Shares no state with the ISR except the RX ring.
inline void poll_cdc(const lu::Millis now, std::span<const uint16_t, logic::kNumChannels> chs, const uint16_t battery_mv) {
    if (!lu::has_cdc(g_mode) || g_config == 0) {
        cdc_reset_state();
        return;
    }
    if (!g_prompted) {
        g_tx.push_text(lu::kCliPrompt);
        g_prompted = true;
    }
    std::array<uint8_t, kCdcPacket> in{};
    for (std::size_t n = rx_take(in); n != 0; n = rx_take(in)) {
        for (std::size_t i = 0; i < n; ++i) {
            handle_rx_byte(in[i], now, chs, battery_mv);
        }
    }
    if (g_streaming && lu::interval_elapsed(now, g_stream_last, lu::Millis{kTelemIntervalMs})) {
        g_stream_last = now;
        std::array<char, kTelemBufSize> buf{};
        const std::size_t n = lu::format_telemetry_json(battery_mv, chs, buf);
        g_tx.push_text(std::string_view(buf.data(), n));
    }
    cdc_pump_tx();
    if (g_reset != Reset::None) {
        const bool drained = g_tx.size() == 0 && (ep_reg(kCdcEp) & USB_EP_TX_VALID) == USB_EP_TX_NAK;
        if (drained || lu::interval_elapsed(now, g_reset_at, lu::Millis{kResetGraceMs})) {
            perform_reset();
        }
    }
}

/// Per-loop service: HID report at its rate, CDC CLI and telemetry streaming.
inline void poll(const lu::Millis now, std::span<const uint16_t, logic::kNumChannels> chs, const logic::Switches& sw,
                 const uint16_t battery_mv) {
    poll_hid(now, chs, sw);
    poll_cdc(now, chs, battery_mv);
}

/// True while the joystick mode has a configured host (Rust `usb::is_sim_mode`).
inline bool is_sim_mode() {
    return lu::is_sim_mode(g_mode, g_config != 0);
}

/// Starts the device stack in `mode` (pull-up on, interrupts enabled).
inline void start(const UsbMode mode) {
    g_mode = mode;

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    RCC->APB1RSTR |= RCC_APB1RSTR_USBRST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;

    Dm::set_pull(Pull::None);
    Dp::set_pull(Pull::None);
    Dm::set_alt_function({0});
    Dp::set_alt_function({0});
    Dm::set_mode(Mode::Alternate);
    Dp::set_mode(Mode::Alternate);

    USB->CNTR = 0;
    spin(kSpinCycles);
    USB->CNTR = USB_CNTR_FRES;
    spin(kSpinCycles);
    USB->CNTR = 0;
    USB->ISTR = 0;
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM | USB_CNTR_ERRM;

    NVIC_ClearPendingIRQ(USB_IRQn);
    reinterpret_cast<volatile uint8_t*>(&NVIC->IP[0])[USB_IRQn] = kIrqPriorityRaw;
    NVIC_EnableIRQ(USB_IRQn);

    USB->BCDR = USB_BCDR_DPPU;
}

/// Detaches from the host (pull-up off), powers the block down and gates its clock.
inline void stop() {
    NVIC_DisableIRQ(USB_IRQn);
    USB->BCDR = 0;
    USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
    RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
    Dm::set_mode(Mode::Input);
    Dp::set_mode(Mode::Input);
    g_mode = UsbMode::Off;
    g_config = 0;
    cdc_reset_state();
}

/// Re-inits the stack when the stored mode changes (Radio Setup row); no-op otherwise.
inline void set_mode(const UsbMode mode) {
    if (mode == g_mode) {
        return;
    }
    if (g_mode != UsbMode::Off) {
        stop();
    }
    if (mode != UsbMode::Off) {
        start(mode);
    }
}

}  // namespace ferrox::hw::usb
