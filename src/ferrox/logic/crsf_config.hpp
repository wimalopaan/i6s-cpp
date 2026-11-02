#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "ferrox/logic/crsf_menu.hpp"
#include "ferrox/logic/format.hpp"
#include "ferrox/logic/framebuffer.hpp"
#include "ferrox/logic/widgets.hpp"

// UI of the "Configure Module" screen: browses and edits the parameter tree of the CRSF device
// (ELRS / TBS) that the parameter loader has discovered.
namespace ferrox::logic::crsf::config {

inline constexpr std::size_t kVisible = 4;
inline constexpr std::size_t kMaxDepth = 4;

/// What the caller has to do on the CRSF link after an update.
struct Request {
    enum class Kind : uint8_t { None, Restart, Write, EnterFolder, StartCommand, ConfirmCommand, CancelCommand };

    Kind kind{Kind::None};
    uint8_t param{0};
    int32_t value{0};
};

struct State {
    uint8_t selected{0};
    uint8_t scroll{0};
    uint8_t folder{0};
    std::array<uint8_t, kMaxDepth> stack{};  // folders above the current one
    uint8_t depth{0};
    bool editing{false};
    int32_t edit_val{0};
};

struct Keys {
    bool up{false};
    bool down{false};
    bool ok{false};
    bool cancel{false};
};

struct Result {
    Request req{};
    bool leave{false};     // ESC at the root folder: close the screen
    bool nav_tick{false};  // list step, for the navigation click
    bool click{false};
};

struct Items {
    std::array<uint8_t, menu::kMaxParams> ids{};
    std::size_t count{0};
};

constexpr bool is_numeric(const menu::Type t) {
    return t == menu::Type::Uint8 || t == menu::Type::Int8 || t == menu::Type::Uint16 || t == menu::Type::Int16;
}

constexpr bool is_editable(const menu::Type t) {
    return t == menu::Type::Select || is_numeric(t);
}

/// Visible parameters of a folder in id order. Parameter 0 is the root folder itself.
constexpr Items items_in_folder(const menu::Loader& l, const uint8_t folder) {
    Items items;
    for (std::size_t id = 1; id < menu::kMaxParams; ++id) {
        const auto& entry = l.entries[id];
        if (!entry || entry->hidden || entry->type == menu::Type::Back) {
            continue;
        }
        const bool legacy_top = !l.has_root_folder && entry->parent == menu::kNoParent && folder == 0;
        if (entry->parent == folder || legacy_top) {
            items.ids[items.count++] = static_cast<uint8_t>(id);
        }
    }
    return items;
}

template <std::size_t N>
constexpr std::string_view view_of(const menu::Text<N>& t) {
    return {t.data.data(), t.len};
}

/// Option `index` of a Select entry whose options are separated by ';'.
constexpr std::string_view select_option(const menu::Entry& e, const int32_t index) {
    const std::string_view all = view_of(e.text);
    std::size_t start = 0;
    for (int32_t i = 0; i < index; ++i) {
        const std::size_t semi = all.find(';', start);
        if (semi == std::string_view::npos) {
            return {};
        }
        start = semi + 1;
    }
    const std::size_t end = std::min(all.find(';', start), all.size());
    return {all.data() + start, end - start};
}

constexpr void step(State& s, const std::size_t count, const bool down) {
    if (down) {
        if (s.selected + 1u < count) {
            ++s.selected;
            if (s.selected >= s.scroll + kVisible) {
                s.scroll = static_cast<uint8_t>(s.selected - kVisible + 1);
            }
        } else {
            s.selected = 0;
            s.scroll = 0;
        }
    } else if (s.selected > 0) {
        --s.selected;
        if (s.selected < s.scroll) {
            s.scroll = s.selected;
        }
    } else {
        s.selected = static_cast<uint8_t>(count - 1);
        s.scroll = static_cast<uint8_t>(count > kVisible ? count - kVisible : 0);
    }
}

constexpr void edit_step(State& s, const menu::Entry& e, const Keys& k, Result& r) {
    if (k.up == k.down) {
        return;
    }
    if (e.type == menu::Type::Select) {
        const int32_t max = std::max<int32_t>(e.max, 0);
        s.edit_val = k.down ? (s.edit_val >= max ? 0 : s.edit_val + 1) : (s.edit_val <= 0 ? max : s.edit_val - 1);
        r.nav_tick = true;
    } else {
        const int32_t next = std::clamp(s.edit_val + (k.up ? 1 : -1), e.min, e.max);
        r.nav_tick = next != s.edit_val;
        s.edit_val = next;
    }
}

constexpr Result update(State& s, const Keys& k, const menu::Loader* l, const menu::Command* c) {
    Result r;
    const auto cmd = c ? c->kind : menu::Command::Kind::Idle;
    using Kind = menu::Command::Kind;
    if (cmd == Kind::WaitingConfirm) {
        if (k.cancel) {
            r.req.kind = Request::Kind::CancelCommand;
            r.click = true;
        } else if (k.ok) {
            r.req.kind = Request::Kind::ConfirmCommand;
            r.click = true;
        }
        return r;
    }
    if (cmd == Kind::Starting || cmd == Kind::Running) {
        if (k.cancel) {
            r.req.kind = Request::Kind::CancelCommand;
            r.click = true;
        }
        return r;
    }

    if (l == nullptr || l->state != menu::Loader::State::Ready) {
        if (k.ok && (l == nullptr || l->state == menu::Loader::State::Idle)) {
            r.req.kind = Request::Kind::Restart;
            r.click = true;
        } else if (k.cancel) {
            r.leave = true;
        }
        return r;
    }

    const Items items = items_in_folder(*l, s.folder);
    if (s.selected >= items.count) {
        s.selected = 0;
        s.scroll = 0;
        s.editing = false;
    }

    if (s.editing) {
        const auto& e = *l->entries[items.ids[s.selected]];
        if (k.ok) {
            r.req = {Request::Kind::Write, items.ids[s.selected], s.edit_val};
            s.editing = false;
            r.click = true;
        } else if (k.cancel) {
            s.editing = false;
            r.click = true;
        } else {
            edit_step(s, e, k, r);
        }
        return r;
    }

    if (items.count > 0 && k.up != k.down) {
        step(s, items.count, k.down);
        r.nav_tick = true;
    }

    if (k.ok && items.count > 0) {
        const uint8_t id = items.ids[s.selected];
        const auto& e = *l->entries[id];
        if (e.type == menu::Type::Folder) {
            if (s.depth < kMaxDepth) {
                s.stack[s.depth++] = s.folder;
                s.folder = id;
                s.selected = 0;
                s.scroll = 0;
                r.req = {Request::Kind::EnterFolder, id, 0};
                r.click = true;
            }
        } else if (is_editable(e.type)) {
            s.editing = true;
            s.edit_val = e.value;
            r.click = true;
        } else if (e.type == menu::Type::Command) {
            r.req = {Request::Kind::StartCommand, id, 0};
            r.click = true;
        }
    } else if (k.cancel) {
        r.click = true;
        if (s.depth > 0) {
            s.folder = s.stack[--s.depth];
            s.selected = 0;
            s.scroll = 0;
            r.req = {Request::Kind::EnterFolder, s.folder, 0};
        } else {
            r.leave = true;
        }
    }
    return r;
}

namespace detail {

constexpr void push_int(format::Text<24>& t, const int32_t v) {
    if (v < 0) {
        t.push('-');
    }
    uint32_t mag = v < 0 ? 0u - static_cast<uint32_t>(v) : static_cast<uint32_t>(v);
    std::array<char, 10> tmp{};
    std::size_t n = 0;
    do {
        tmp[n++] = static_cast<char>('0' + mag % 10);
        mag /= 10;
    } while (mag != 0);
    while (n > 0) {
        t.push(tmp[--n]);
    }
}

constexpr void push_text(format::Text<24>& t, const std::string_view sv) {
    for (const char ch : sv) {
        t.push(ch);
    }
}

constexpr void draw_row(display::Framebuffer& fb, const std::size_t slot, const bool selected, const std::string_view name,
                        const std::string_view value) {
    const std::size_t max_name = value.empty() ? 19 : std::max<std::size_t>(20 - std::min<std::size_t>(value.size(), 15) - 1, 5);
    widgets::draw_list_row(fb, {static_cast<int>(slot)}, selected, name.substr(0, max_name), value, {0});
}

constexpr format::Text<24> select_text(const menu::Entry& e, const int32_t index, const bool edit) {
    std::string_view opt = select_option(e, index);
    // Long options lose their trailing "(...)" detail first.
    const std::size_t paren = opt.find('(');
    if (paren != std::string_view::npos && paren > 0 && (edit || view_of(e.name).size() + opt.size() > 18)) {
        opt = opt.substr(0, paren);
    }
    while (!opt.empty() && opt.back() == ' ') {
        opt.remove_suffix(1);
    }
    format::Text<24> t;
    if (edit) {
        push_text(t, "< ");
    }
    push_text(t, opt.substr(0, edit ? 18 : 22));
    if (edit) {
        push_text(t, " >");
    }
    return t;
}

constexpr format::Text<24> number_text(const menu::Entry& e, const int32_t v, const bool edit) {
    format::Text<24> t;
    if (edit) {
        push_text(t, "< ");
    }
    push_int(t, v);
    if (e.text.len > 0) {
        t.push(' ');
        push_text(t, view_of(e.text).substr(0, 6));
    }
    if (edit) {
        push_text(t, " >");
    }
    return t;
}

constexpr void draw_param(display::Framebuffer& fb, const State& s, const menu::Command* c, const std::size_t slot,
                          const std::size_t index, const uint8_t id, const menu::Entry& e) {
    const bool selected = index == s.selected;
    const bool edit = selected && s.editing;
    const std::string_view name = view_of(e.name);
    switch (e.type) {
        case menu::Type::Folder:
            draw_row(fb, slot, selected, name.substr(0, 18), ">");
            break;
        case menu::Type::Select: {
            const auto t = select_text(e, edit ? s.edit_val : e.value, edit);
            draw_row(fb, slot, selected, name, t.view());
            break;
        }
        case menu::Type::Uint8:
        case menu::Type::Int8:
        case menu::Type::Uint16:
        case menu::Type::Int16: {
            const auto t = number_text(e, edit ? s.edit_val : e.value, edit);
            draw_row(fb, slot, selected, name, t.view());
            break;
        }
        case menu::Type::Command: {
            using Kind = menu::Command::Kind;
            const bool active = c != nullptr && c->param == id && c->kind != Kind::Idle;
            std::string_view text = name;
            if (active) {
                text = c->kind == Kind::Completed ? std::string_view{"OK"} : std::string_view{"Executing..."};
            }
            format::Text<24> t;
            t.push('[');
            push_text(t, text.substr(0, 18));
            t.push(']');
            draw_row(fb, slot, selected, t.view(), {});
            break;
        }
        case menu::Type::Info:
        case menu::Type::String: {
            const std::string_view info = view_of(e.text);
            if (info.empty()) {
                draw_row(fb, slot, selected, name, {});
            } else if (info.size() > 7 || name.size() + info.size() > 17) {
                const std::size_t len = std::min<std::size_t>(info.size(), 15);
                const int val_x = 124 - static_cast<int>(len) * 4;
                const std::size_t max_name = static_cast<std::size_t>(std::max((val_x - 6) / 6, 4));
                draw_row(fb, slot, selected, name.substr(0, max_name), {});
                fb.draw_str_4x6({val_x, 14 + static_cast<int>(slot) * 9 + 2}, info.substr(0, len), selected);
            } else {
                draw_row(fb, slot, selected, name, info);
            }
            break;
        }
        default:
            draw_row(fb, slot, selected, name, {});
            break;
    }
}

constexpr void draw_confirm(display::Framebuffer& fb, const menu::Loader& l, const uint8_t id) {
    fb.fill_rect({{10, 16}, {108, 32}}, false);
    fb.draw_rect({{10, 16}, {108, 32}}, true);
    format::Text<24> q;
    push_text(q, "Run: [");
    if (id < menu::kMaxParams && l.entries[id]) {
        push_text(q, view_of(l.entries[id]->name).substr(0, 8));
    }
    push_text(q, "]?");
    const int x = 10 + std::max((108 - static_cast<int>(q.len) * 6) / 2, 0);
    fb.draw_str_6x10({x, 20}, q.view(), false);
    fb.draw_str_4x6({20, 37}, "[OK] Yes  [ESC] No", false);
    widgets::draw_footer(fb, "[OK] Confirm   [ESC] Cancel");
}

}  // namespace detail

constexpr void render(display::Framebuffer& fb, const State& s, const menu::Loader* l, const menu::Command* c) {
    using Kind = menu::Command::Kind;
    if (l == nullptr || l->state == menu::Loader::State::Idle) {
        widgets::draw_header(fb, "CRSF CONFIG");
        fb.draw_str_6x10({4, 19}, "Searching module...", false);
        fb.draw_str_4x6({4, 33}, "Listening for pings (1Hz)", false);
        widgets::draw_footer(fb, "[OK] Scan   [ESC] Back");
        return;
    }
    if (l->state == menu::Loader::State::Loading) {
        widgets::draw_header(fb, "CRSF CONFIG");
        fb.draw_str_6x10({12, 21}, "Loading...", false);
        widgets::draw_footer(fb, "Please wait...  [ESC] Abort");
        return;
    }

    if (s.folder > 0 && s.folder < menu::kMaxParams && l->entries[s.folder]) {
        widgets::draw_header(fb, view_of(l->entries[s.folder]->name).substr(0, 20));
    } else {
        widgets::draw_header(fb, "CRSF CONFIG");
    }

    const Items items = items_in_folder(*l, s.folder);
    if (items.count == 0) {
        fb.draw_str_6x10({12, 21}, "No parameters", false);
        widgets::draw_footer(fb, s.depth > 0 ? "[ESC] Up" : "[ESC] Back");
        return;
    }

    for (std::size_t slot = 0; slot < kVisible; ++slot) {
        const std::size_t index = s.scroll + slot;
        if (index >= items.count) {
            break;
        }
        detail::draw_param(fb, s, c, slot, index, items.ids[index], *l->entries[items.ids[index]]);
    }
    widgets::draw_scrollbar(fb, {static_cast<int>(s.selected)}, {static_cast<int>(items.count)}, {14, 36});

    const uint8_t sel_id = s.selected < items.count ? items.ids[s.selected] : 0;
    if (c != nullptr && c->kind == Kind::WaitingConfirm) {
        detail::draw_confirm(fb, *l, c->param);
    } else if (s.editing) {
        widgets::draw_footer(fb, "[OK] Confirm   [ESC] Cancel");
    } else {
        const auto& e = *l->entries[sel_id];
        const std::string_view back = s.depth > 0 ? "[ESC] Up" : "[ESC] Back";
        if (e.type == menu::Type::Folder) {
            widgets::draw_footer_split(fb, "[OK] Enter", back);
        } else if (is_editable(e.type)) {
            widgets::draw_footer_split(fb, "[OK] Edit", back);
        } else if (e.type == menu::Type::Command) {
            widgets::draw_footer_split(fb, "[OK] Run", back);
        } else {
            widgets::draw_footer(fb, back);
        }
    }
}

}  // namespace ferrox::logic::crsf::config
