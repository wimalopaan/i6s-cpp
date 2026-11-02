#pragma once

#include <cstdint>

#include "wm/arch/regs.hpp"

namespace wm::arch {

/// RAII interrupt mask: saves PRIMASK, disables IRQs, restores on scope exit.
class CriticalSection {
public:
    CriticalSection() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~CriticalSection() { __set_PRIMASK(primask_); }

    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t primask_;
};

}  // namespace wm::arch
