#include <array>
#pragma once
#include "stm32g4xx.h"

namespace umnsvp::mppt::hardware {

/*Result struct*/

struct adc_results_t {
    uint32_t raw_value;
    float value;
    float conversion;
    float offset;
};

/* Battery measurements ADC define with channels */
static ADC_TypeDef* const BATTERY_Voltage_ADC = ADC3;
constexpr uint32_t BATTERY_VOLTAGE_CHANNEL = ADC_CHANNEL_4;

/* Array Voltage ADC define with channels */
static ADC_TypeDef* const ARRAY_VOLTAGE_ADC = ADC2;
constexpr uint32_t ARRAY_VOLTAGE_CHANNEL = ADC_CHANNEL_3;

/* Array Current ADC define with channels */
static ADC_TypeDef* const SLOW_ARRAY_CURRENT_ADC = ADC1;
constexpr uint32_t SLOW_ARRAY_CURRENT_CHANNEL = ADC_CHANNEL_1;

static ADC_TypeDef* const FAST_ARRAY_CURRENT_ADC = ADC4;
constexpr uint32_t FAST_ARRAY_CURRENT_CHANNEL = ADC_CHANNEL_4;

/* Misc measurements ADC define with channels */
static ADC_TypeDef* const MISC_ADC = ADC5;
constexpr uint32_t TEMP_CHANNEL = ADC_CHANNEL_14;
constexpr uint32_t PRECHARGE_CHANNEL = ADC_CHANNEL_15;

/* Measurement indexes*/
constexpr uint32_t ARRAY_VOLTAGE_ADC_INDEX = 0;
constexpr uint32_t BATTERY_VOLTAGE_ADC_INDEX = 1;
constexpr uint32_t SLOW_ARRAY_CURRENT_ADC_INDEX = 2;
constexpr uint32_t FAST_ARRAY_CURRENT_ADC_INDEX = 3;
constexpr uint32_t NUM_POWER_ADC_RESULTS = 4;

/* Measurement indexes for misc ADC's*/
constexpr uint32_t EXTERNAL_THERMISTOR_ADC_INDEX = 0;
constexpr uint32_t PRECHARGE_VOLTAGE_ADC_INDEX = 1;
constexpr uint32_t IC_TEMP_ADC_INDEX = 2;
constexpr uint32_t NUM_MISC_ADC_RESULTS = 3;

/* Reference voltage for the ADC */
constexpr float adcVref = 3.3;

/* This is the size of the result from the ADC, you need to divide results from
 * the ADC by this value then mutliply by Vref to get voltage */
constexpr uint32_t ADC_RESULT_SCALAR_12_BITS = 4096;

// Ratio of the voltage divider for the HV system
static constexpr float HV_VoltageDividerRatio = 4.22 / 200;

// Ration of voltage output on the HAL vs current through it
// This number is in V/A: 100mv/A
static constexpr float CurrentSenseCurrentRatio = 0.200;  // mV/A

// Offset of the HAL sensors
static constexpr float CURRENT_ADC_HAL_OFFSET =
    (0.1 * adcVref) / CurrentSenseCurrentRatio;

constexpr float HV_ADC_CONVERSION_RATIO =
    adcVref / ADC_RESULT_SCALAR_12_BITS / HV_VoltageDividerRatio;

constexpr float LV_ADC_CONVERSION_RATIO = adcVref / ADC_RESULT_SCALAR_12_BITS;

constexpr float CURRENT_ADC_CONVERSION_RATIO =
    adcVref / ADC_RESULT_SCALAR_12_BITS / CurrentSenseCurrentRatio;

void adc_init();
}  // namespace umnsvp::mppt::hardware
