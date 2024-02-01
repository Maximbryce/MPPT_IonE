#include "hardware.h"

#include <cmath>

#include "adc.h"
#include "comparator.h"
#include "dac.h"
#include "gate_driver.h"
#include "op_amp.h"
#include "pin_defines.h"

namespace umnsvp::mppt::hardware {

extern struct adc_results_t adc_power_results[NUM_POWER_ADC_RESULTS];
extern struct adc_results_t adc_misc_results[NUM_MISC_ADC_RESULTS];
extern std::array<uint16_t, 3> misc_measurements;

static GateDriver driver(FSW, DEADTIME);
static InternalDac internal_dac1(DAC1);
static InternalDac internal_dac3(DAC3);
static InternalDac internal_dac4(DAC4);

// Temperature scaling factors for converting from resistance to a
// temperature
static std::array<float, 6> tempCoefficients = {111.5,  -250.6, 335.6,
                                                -249.6, 90.98,  -12.55};

static constexpr float coilInductance = 118e-6;
// Temp static resistor in voltage divider
static constexpr float tempReistorBase = 10000;

// HV voltage scalaing
static constexpr float HVSlope = 1.028;
static constexpr float HVOffset = 0.09253;

void initGpios();
void setupComparatorReferences();

void hardware_init() {
    initGpios();
    // Start the ADC's
    adc_init();
    /* Intialize and start the gate driver */
    driver.init();
    driver.start();
    /* Initialize and start the op amp's */
    op_amp_init();
    /* Setup the DAC's for references to the op amps */
    setupComparatorReferences();
}

void initGpios() {
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };

    // Init output relay
    gpio.Pin = OUTPUT_RELAY_PIN;
    HAL_GPIO_Init(OUTPUT_RELAY_PORT, &gpio);

    // Init precharge relay
    gpio.Pin = PRECHARGE_RELAY_PIN;
    HAL_GPIO_Init(PRECHARGE_RELAY_PORT, &gpio);
}

void setupComparatorReferences() {
    // Initialize DAC1 and it's channels
    internal_dac1.init();
    internal_dac1.init_ch1();
    internal_dac1.start_ch1();

    // Initialize DAC3 and it's channels
    internal_dac3.init();
    internal_dac3.init_ch1();
    internal_dac3.init_ch2();
    internal_dac3.start_ch1();
    internal_dac3.start_ch2();

    // Initialize DAC4 and it's channels
    internal_dac4.init();
    internal_dac4.init_ch1();
    internal_dac4.start_ch1();

    // Initialize DAC 1 ch2 for external debugg
    internal_dac1.init_ch2(false);
    internal_dac1.start_ch2();
}

void set_debug_dac_voltage(float voltage) {
    internal_dac1.set_ch2_voltage(voltage);
}

float get_input_voltage() {
    return adc_power_results[ARRAY_VOLTAGE_ADC_INDEX].value;
}

float get_lightly_filtered_input_current() {
    return adc_power_results[FAST_ARRAY_CURRENT_ADC_INDEX].value;
}

float get_filtered_input_current() {
    return adc_power_results[SLOW_ARRAY_CURRENT_ADC_INDEX].value;
}

float get_output_voltage() {
    return adc_power_results[BATTERY_VOLTAGE_ADC_INDEX].value;
}

float calc_thermistor_temp(float resistance) {
    float resistance_scaled =
        resistance / 10000;  // scale the resistance so the polynomials
                             // don't get out of hand
    float temp = tempCoefficients[0];
    float rPoly = 1;
    for (unsigned int i = 1; i < tempCoefficients.size(); i++) {
        rPoly *= resistance_scaled;
        temp += tempCoefficients[i] * rPoly;
    }
    return temp;
}

float get_heatsink_temp() {
    return adc_misc_results[EXTERNAL_THERMISTOR_ADC_INDEX].value;
}

float get_bus_voltage() {
    return adc_misc_results[PRECHARGE_VOLTAGE_ADC_INDEX].value;
}

float get_internal_ic_temp() {
    return adc_misc_results[IC_TEMP_ADC_INDEX].value;
}

float calc_aux_duty_val() {
    float T2 =
        std::abs(adc_power_results[SLOW_ARRAY_CURRENT_ADC_INDEX].value) /
        std::max((static_cast<float>(
                     adc_power_results[BATTERY_VOLTAGE_ADC_INDEX].value / Lr)),
                 Vout_Lr_min);
    auto resonant_pulse_compare_width = T1 + T2 + 150e-9;
    return resonant_pulse_compare_width;
}

void set_duty(float duty_cycle) {
    driver.set_duty(duty_cycle, calc_aux_duty_val());
}

float get_duty() {
    return driver.get_duty();
}

void incrementDutyCycle(float increment) {
    driver.set_duty(driver.get_duty() + increment, calc_aux_duty_val());
}

void output_relay_close() {
    HAL_GPIO_WritePin(OUTPUT_RELAY_PORT, OUTPUT_RELAY_PIN, GPIO_PIN_SET);
}

void output_relay_open() {
    HAL_GPIO_WritePin(OUTPUT_RELAY_PORT, OUTPUT_RELAY_PIN, GPIO_PIN_RESET);
}

void precharge_relay_close() {
    HAL_GPIO_WritePin(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN, GPIO_PIN_SET);
}

void precharge_relay_open() {
    HAL_GPIO_WritePin(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN,
                      GPIO_PIN_RESET);
}

void enable_gate_driver() {
    driver.start();
}

void disable_gate_driver() {
    driver.stop();
}

void re_arm_pwm_break_input() {
    driver.clear_rearm_break_fault();
}

void set_fast_array_current_limit(float limit) {
    internal_dac4.set_ch1_voltage(CURRENT_ADC_HAL_OFFSET +
                                  limit * CurrentSenseCurrentRatio);
}

void set_array_voltage_limit(float limit) {
    internal_dac3.set_ch2_voltage(limit * HV_VoltageDividerRatio);
}

void set_slow_array_current_limit(float limit) {
    internal_dac3.set_ch1_voltage(CURRENT_ADC_HAL_OFFSET +
                                  limit * CurrentSenseCurrentRatio);
}

void set_battery_voltage_limit(float limit) {
    internal_dac1.set_ch1_voltage(limit * HV_VoltageDividerRatio);
}

void start_comparators() {
    // Startup the comparators to monitor the voltage
    comparator_init();
}

void convert_core_measurements() {
    for (uint32_t i = 0; i < NUM_POWER_ADC_RESULTS; i++) {
        adc_power_results[i].value =
            (adc_power_results[i].raw_value * adc_power_results[i].conversion) -
            adc_power_results[i].offset;
    }
}

void convert_misc_measurements() {
    for (uint32_t i = 0; i < NUM_MISC_ADC_RESULTS; i++) {
        adc_misc_results[i].raw_value = misc_measurements[i];
    }

    /* Do the bus HV measurement */
    adc_misc_results[PRECHARGE_VOLTAGE_ADC_INDEX].value =
        (adc_misc_results[PRECHARGE_VOLTAGE_ADC_INDEX].raw_value *
         adc_misc_results[PRECHARGE_VOLTAGE_ADC_INDEX].conversion) -
        adc_misc_results[PRECHARGE_VOLTAGE_ADC_INDEX].offset;

    /* Conversion for the external thermistor*/
    float therm_voltage =
        adc_misc_results[EXTERNAL_THERMISTOR_ADC_INDEX].raw_value *
        LV_ADC_CONVERSION_RATIO;
    float resistance =
        (tempReistorBase * adcVref / therm_voltage) - tempReistorBase;
    adc_misc_results[EXTERNAL_THERMISTOR_ADC_INDEX].value =
        calc_thermistor_temp(resistance);

    /* Conversion of internal temp*/

    const uint16_t *const ADC_TEMP_3V3_30C =
        reinterpret_cast<uint16_t *>(TEMPSENSOR_CAL1_ADDR);
    const uint16_t *const ADC_TEMP_3V3_110C =
        reinterpret_cast<uint16_t *>(TEMPSENSOR_CAL2_ADDR);
    const float CALIBRATION_REFERENCE_VOLTAGE = TEMPSENSOR_CAL_VREFANALOG;

    const float REFERENCE_VOLTAGE = 3300.0F;  // Vref internal in mV

    // scale constants to current reference voltage
    float adcCalTemp30C = static_cast<float>(*ADC_TEMP_3V3_30C);
    float adcCalTemp110C = static_cast<float>(*ADC_TEMP_3V3_110C);

    float float_converter_adc_val =
        (adc_misc_results[IC_TEMP_ADC_INDEX].raw_value) *
        (REFERENCE_VOLTAGE / CALIBRATION_REFERENCE_VOLTAGE);

    adc_misc_results[IC_TEMP_ADC_INDEX].value =
        (float_converter_adc_val - adcCalTemp30C) /
            (adcCalTemp110C - adcCalTemp30C) *
            (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) +
        30.0F;
}

// TODO: Clean this up to make it clearer since the flag is an event, while the
// READ_BIT for VALUE is a continous case
bool get_input_current_fast_comparator_output() {
    return (READ_BIT((getComp7Handle())->Instance->CSR, COMP_CSR_VALUE) ||
            __HAL_COMP_COMP7_EXTI_GET_FLAG());
}
bool get_input_current_slow_comparator_output() {
    return (READ_BIT((getComp3Handle())->Instance->CSR, COMP_CSR_VALUE) ||
            __HAL_COMP_COMP3_EXTI_GET_FLAG());
}
bool get_input_voltage_comparator_output() {
    return (READ_BIT((getComp2Handle())->Instance->CSR, COMP_CSR_VALUE) ||
            __HAL_COMP_COMP2_EXTI_GET_FLAG());
}
bool get_output_voltage_comparator_output() {
    return (READ_BIT((getComp1Handle())->Instance->CSR, COMP_CSR_VALUE) ||
            __HAL_COMP_COMP1_EXTI_GET_FLAG());
}

void clear_comparator_flags(void) {
    __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP3_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP7_EXTI_CLEAR_FLAG();
}

/* Do not put in more than 2^16 elements in here!!! */
float average(const uint16_t *data, uint16_t size) {
    /* only using a 32 bit value here to speed up processing time and
     * because 64 is simply not necessary as results from adc are at most 16
     * bit and it would require 2^16 inputs to overflow*/
    uint32_t sum = 0;
    for (uint16_t i = 0; i < size; i += 1) {
        sum = data[i] + sum;
    }
    // Divide by size/ (increment + sample_offset) bc you have to account
    // for skipping some
    return static_cast<float>((float)sum / size);
}

}  // namespace umnsvp::mppt::hardware
