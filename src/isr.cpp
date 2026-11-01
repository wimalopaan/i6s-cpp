#include "ferrox/arch/regs.hpp"
#include "ferrox/hw/rf.hpp"
#include "ferrox/hw/trainer.hpp"
#include "ferrox/hw/uart.hpp"
#include "ferrox/time/systick.hpp"
#if FERROX_USE_USB
#include "ferrox/hw/usb.hpp"
#endif

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

void TIM15_IRQHandler() {
    ferrox::hw::trainer::on_tim15();
}

#if FERROX_USE_USB
void USB_IRQHandler() {
    ferrox::hw::usb::on_irq();
}
#endif

}  // extern "C"
