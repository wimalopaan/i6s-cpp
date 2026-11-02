#include "wm/arch/regs.hpp"
#include "wm/hw/rf.hpp"
#include "wm/hw/trainer.hpp"
#include "wm/hw/uart.hpp"
#include "wm/time/systick.hpp"
#if WM_USE_USB
#include "wm/hw/usb.hpp"
#endif

extern "C" {

void SysTick_Handler() {
    wm::time::on_tick();
}

void USART2_IRQHandler() {
    wm::hw::uart::on_irq();
}

void EXTI2_3_IRQHandler() {
    wm::hw::rf::on_exti2_3();
}

void TIM16_IRQHandler() {
    wm::hw::rf::on_tim16();
}

void TIM15_IRQHandler() {
    wm::hw::trainer::on_tim15();
}

#if WM_USE_USB
void USB_IRQHandler() {
    wm::hw::usb::on_irq();
}
#endif

}  // extern "C"
