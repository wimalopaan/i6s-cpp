#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/uart.hpp"
#include "ferrox/time/systick.hpp"

extern "C" {

void SysTick_Handler() {
    ferrox::time::on_tick();
}

void USART2_IRQHandler() {
    ferrox::hw::uart::on_irq();
}

}  // extern "C"
