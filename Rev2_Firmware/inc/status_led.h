#pragma once

#include "stm32g4xx.h"

namespace umnsvp::mppt {
class StatusLED {
   private:
    uint32_t led_pin;
    GPIO_TypeDef *led_port;

   public:
    StatusLED(GPIO_TypeDef *port, uint16_t pin);
    void init();
    void off();
    void on();
    void toggle();
};
}  // namespace umnsvp::mppt