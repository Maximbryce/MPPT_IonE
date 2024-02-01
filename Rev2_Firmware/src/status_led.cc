#include "status_led.h"

#include "pin_defines.h"

namespace umnsvp::mppt {

/**
 *
 * @param port The GPIO port for the pin
 * @param pin The pin number for the pin
 */
StatusLED::StatusLED(GPIO_TypeDef *port, uint16_t pin)
    : led_pin{pin}, led_port{port} {
}

/**
 * @brief Initializes the LED's with the given pin and port and turns it off by
 * default
 */
void StatusLED::init() {
    GPIO_InitTypeDef gpio;
    gpio.Pin = led_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(led_port, &gpio);
    // Turn off by default
    off();
}

/**
 * @brief Sets the LED GPIO pin high thus turning off the LED. This is because
 * the micro is a low side switch in the circuit
 */
void StatusLED::off() {
    HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_SET);
}

/**
 * @brief Sets the LED GPIO pin low turning on the LED. This is because
 * the micro is a low side switch in the circuit
 */
void StatusLED::on() {
    HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_RESET);
}

/**
 * @brief Toggles the LED GPIO
 *
 */
void StatusLED::toggle() {
    HAL_GPIO_TogglePin(led_port, led_pin);
}

}  // namespace umnsvp::mppt
