#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/rf.hpp"
#include "ferrox/hw/uart.hpp"
#include "ferrox/time/systick.hpp"

extern "C" {

void SysTick_Handler() {
    ferrox::time::on_tick();
}

void USART2_IRQHandler() {
    ferrox::hw::uart::on_irq();
}

void EXTI2_3_IRQHandler() {
    ferrox::hw::rf::on_exti2_3();
}

void TIM16_IRQHandler() {
    ferrox::hw::rf::on_tim16();
}

}  // extern "C"
