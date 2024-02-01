#pragma once
#include "stm32g4xx.h"
/*
 * This file contains all the pin defines for the various parts of the program
 */

/*
 * Pins used by the ADC peripheral
 */

namespace umnsvp::mppt {

/* Pins for battery channels */
constexpr uint16_t BATTERY_VOLTAGE_ADC_PIN = GPIO_PIN_7;
static GPIO_TypeDef* const BATTERY_VOLTAGE_ADC_PORT = GPIOE;

/* Pins for array channels */
constexpr uint16_t ARRAY_VOLTAGE_ADC_PIN = GPIO_PIN_6;
static GPIO_TypeDef* const ARRAY_VOLTAGE_ADC_PORT = GPIOA;
constexpr uint16_t SLOW_ARRAY_CURRENT_ADC_PIN = GPIO_PIN_0;
static GPIO_TypeDef* const SLOW_ARRAY_CURRENT_ADC_PORT = GPIOA;
constexpr uint16_t FAST_ARRAY_CURRENT_ADC_PIN = GPIO_PIN_14;
static GPIO_TypeDef* const FAST_ARRAY_CURRENT_ADC_PORT = GPIOB;

/* Pins for Temp sense channel */
static GPIO_TypeDef* const TEMP_ADC_PORT = GPIOE;
constexpr uint16_t TEMP_ADC_PIN = GPIO_PIN_10;

/* Pins for precharge sense channel */
static GPIO_TypeDef* const PRECHARGE_SENSE_PORT = GPIOE;
constexpr uint16_t PRECHARGE_SENSE_PIN = GPIO_PIN_11;

/*
 * Defines for op-amp peripheral
 */

/* Op-Amp 1 pins used for the buffer of battery voltage*/
constexpr uint16_t OPAMP1_VINP_PIN = GPIO_PIN_3;
static GPIO_TypeDef* const OPAMP1_VINP_PORT = GPIOA;
constexpr uint16_t OPAMP1_VOUT_PIN = GPIO_PIN_2;
static GPIO_TypeDef* const OPAMP1_VOUT_PORT GPIOA;

/* Op-Amp 2 pins used for the buffer of array voltage*/
constexpr uint16_t OPAMP2_VINP_PIN = GPIO_PIN_0;
static GPIO_TypeDef* const OPAMP2_VINP_PORT GPIOB;
constexpr uint16_t OPAMP2_VOUT_PIN = GPIO_PIN_6;
static GPIO_TypeDef* const OPAMP2_VOUT_PORT GPIOA;

/* Op-Amp 4 pins used for the buffer of precharge voltage*/
constexpr uint16_t OPAMP4_VINP_PIN = GPIO_PIN_11;
static GPIO_TypeDef* const OPAMP4_VINP_PORT GPIOB;
constexpr uint16_t OPAMP4_VOUT_PIN = GPIO_PIN_11;
static GPIO_TypeDef* const OPAMP4_VOUT_PORT GPIOB;

/*
 * Defines for the comparators
 */

/* Defines for COMP1 which is used for over-voltage on output to the battery */
constexpr uint16_t comp1_inp_pin = GPIO_PIN_1;
GPIO_TypeDef* const comp1_inp_port = GPIOA;

/* Defines for COMP2 which is used for over-voltage on input from the array */
constexpr uint16_t comp2_inp_pin = GPIO_PIN_7;
GPIO_TypeDef* const comp2_inp_port = GPIOA;

/* Defines for COMP3 which is used for over-current on output to the battery */
constexpr uint16_t comp3_inp_pin = GPIO_PIN_0;
GPIO_TypeDef* const comp3_inp_port = GPIOA;

/* Defines for COMP7 which is used for over-current on input from the array */
constexpr uint16_t comp7_inp_pin = GPIO_PIN_14;
GPIO_TypeDef* const comp7_inp_port = GPIOB;

/*
 * Defines for the gate driver
 */

/* Defines for the high side gate driver This is CH1N for PWM*/
constexpr uint16_t AUX_GATE_PIN = GPIO_PIN_12;
// constexpr uint16_t HIGH_GATE_PIN = GPIO_PIN_10;
static GPIO_TypeDef* const HIGH_GATE_PORT = GPIOC;

/* Defines for the low side gate driver. This is CH1 for PWM*/
constexpr uint16_t LOW_GATE_PIN = GPIO_PIN_6;
static GPIO_TypeDef* const LOW_GATE_PORT = GPIOB;

/*
 * Defines for status LED's
 */

/* Indicator 0 */
constexpr uint16_t INDICATOR0_PIN = GPIO_PIN_9;
static GPIO_TypeDef* const INDICATOR0_PORT = GPIOA;

/* Indicator 1 */
constexpr uint16_t INDICATOR1_PIN = GPIO_PIN_8;
static GPIO_TypeDef* const INDICATOR1_PORT = GPIOA;

/* Indicator 2 */
constexpr uint16_t INDICATOR2_PIN = GPIO_PIN_9;
static GPIO_TypeDef* const INDICATOR2_PORT = GPIOC;

/* Indicator 3 */
constexpr uint16_t INDICATOR3_PIN = GPIO_PIN_8;
static GPIO_TypeDef* const INDICATOR3_PORT = GPIOC;

/* Indicator 4 */
constexpr uint16_t INDICATOR4_PIN = GPIO_PIN_7;
static GPIO_TypeDef* const INDICATOR4_PORT = GPIOC;

/* Indicator 5 */
constexpr uint16_t INDICATOR5_PIN = GPIO_PIN_6;
static GPIO_TypeDef* const INDICATOR5_PORT = GPIOC;

/*
 * Defines for various hardware objects
 */

/* Defines for output relay and precharge relay */
constexpr uint16_t OUTPUT_RELAY_PIN = GPIO_PIN_2;
static GPIO_TypeDef* const OUTPUT_RELAY_PORT = GPIOC;
constexpr uint16_t PRECHARGE_RELAY_PIN = GPIO_PIN_1;
static GPIO_TypeDef* const PRECHARGE_RELAY_PORT = GPIOC;

/* Defines for the UART peripheral used for debuging*/
constexpr uint16_t UART_TX_PIN = GPIO_PIN_0;
constexpr uint16_t UART_RX_PIN = GPIO_PIN_1;
static GPIO_TypeDef* const UART_PORT = GPIOE;

}  // namespace umnsvp::mppt