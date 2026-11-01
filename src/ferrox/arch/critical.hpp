#pragma once

#include <cstdint>

#include "ferrox/arch/regs.hpp"

namespace ferrox::arch {

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

}  // namespace ferrox::arch
