#include <TCAN330.h>
#include "pico/stdio.h"
#include "hardware/gpio.h"
void PIOx_IRQHandler()
{
    assert(TCAN330::instance != NULL);
    can2040_pio_irq_handler(TCAN330::instance->getCANBus());
}
void rxCallback(can2040_t* canbus, uint32_t notify, can2040_msg_t* msg)
{
    assert(TCAN330::instance != NULL);
    if (notify == CAN2040_NOTIFY_RX)
        TCAN330::instance->rxCallback(msg);
}
TCAN330::TCAN330(int rxPin, int txPin, int sPin, int shdnPin, std::function<void(can2040_msg_t*)> rxFunc,
    int clockFrequency)
{
    instance = this;
    this->rxFunc = rxFunc;
    can2040_setup(&canbus, 0);
    can2040_callback_config(&canbus, ::rxCallback);
    gpio_init(sPin);
    gpio_set_dir(sPin, GPIO_OUT);
    gpio_put(sPin, false);
    gpio_init(shdnPin);
    gpio_set_dir(shdnPin, GPIO_OUT);
    gpio_put(shdnPin, false);
    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_enabled(PIO0_IRQ_0, true);
    can2040_start(&canbus, clockFrequency, 1000000, rxPin, txPin);
}
void TCAN330::rxCallback(can2040_msg_t* msg)
{
    if (rxFunc != NULL)
    {
        rxFunc(msg);
    }
}
void TCAN330::sendMessage(can2040_msg_t* msg)
{
    can2040_transmit(&canbus, msg);
}
can2040_t* TCAN330::getCANBus()
{
    return &canbus;
}