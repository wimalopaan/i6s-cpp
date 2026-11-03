#include "wm/arch/regs.hpp"
#include "wm/hw/rf.hpp"
#include "wm/hw/uart.hpp"
#include "wm/time/systick.hpp"
#if WM_USE_USB
#include "wm/hw/usb.hpp"
#endif

extern "C" {

void SysTick_Handler() {
    wm::time::on_tick();
}

#if WM_PCB_I6S
void USART1_IRQHandler() {
#else
void USART2_IRQHandler() {
#endif
    wm::hw::uart::on_irq();
}

void EXTI2_3_IRQHandler() {
    wm::hw::rf::on_exti2_3();
}

void TIM16_IRQHandler() {
    wm::hw::rf::on_tim16();
}

#if WM_USE_USB
void USB_IRQHandler() {
    wm::hw::usb::on_irq();
}
#endif

}  // extern "C"
