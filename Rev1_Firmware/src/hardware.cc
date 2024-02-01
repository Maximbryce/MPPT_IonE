#include "hardware.h"

#include <cmath>

#include "comparator.h"
#include "op_amp.h"
#include "pin_defines.h"

namespace umnsvp::mppt {

Hardware::Hardware()
    : sense(),
      driver(gate_driver_frequency::kHz_20),
      internal_dac1(DAC1),
      internal_dac3(DAC3),
      internal_dac4(DAC4),
      inputVoltage(inputVoltageInternal),
      outputVoltage(outputVoltageInternal),
      inputCurrent(inputCurrentInternal),
      outputCurrent(outputCurrentInternal) {
}

void Hardware::init() {
    initGpios();
    /* Initialize and start the ADC's */
    sense.init();
    sense.start();
    /* Intialize and start the gate driver */
    driver.init();
    driver.start();
    /* Initialize and start the op amp's */
    op_amp::init_op_amp();
    op_amp::op_amp_start();
    /* Setup the DAC's for references to the op amps */
    setupComparatorReferences();
    /* Calibrate the HAL sensors by applying the offset here to the internal
     * variables*/
    HAL_Delay(100);
    updateHalOffsets();
}

void Hardware::initGpios() {
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

void Hardware::setupComparatorReferences() {
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

void Hardware::set_debug_dac_voltage(float voltage) {
    internal_dac1.set_ch2_voltage(voltage);
}

float Hardware::get_input_voltage() {
    auto measurements = sense.get_array_voltage_measurements();
    float average = measurements[0];
    return (average * adcVref / ADC_RESULT_SCALAR_ARRAY_VOLTAGE /
            HV_VoltageDividerRatio) *
               HVSlope +
           HVOffset;
}

float Hardware::get_average_input_current() {
    auto measurements = sense.get_buffered_array_current_measurements();
    float average = hardware::average(measurements.data(), measurements.size());
    return ((average * adcVref / ADC_RESULT_SCALAR_ARRAY_CURRENT) -
            arrayHalOffset) /
           CurrentSenseCurrentRatio;
}

float Hardware::get_input_current() {
    auto measurements = sense.get_array_current_measurements();
    float average = measurements[0];
    return ((average * adcVref / ADC_RESULT_SCALAR_ARRAY_CURRENT) -
            arrayHalOffset) /
           CurrentSenseCurrentRatio;
}

float Hardware::get_output_voltage() {
    auto measurements = sense.get_battery_voltage_measurements();
    float average = measurements[0];
    return (average * adcVref / ADC_RESULT_SCALAR_BAT_VOLTAGE /
            HV_VoltageDividerRatio) *
               HVSlope +
           HVOffset;
}

float Hardware::get_output_current() {
    auto measurements = sense.get_battery_current_measurements();
    float average = measurements[0];
    return ((average * adcVref / ADC_RESULT_SCALAR_BAT_CURRENT) -
            batteryHalOffset) /
           CurrentSenseCurrentRatio;
}
float Hardware::get_average_output_current() {
    auto measurements = sense.get_buffered_battery_current_measurements();
    float average = hardware::average(measurements.data(), measurements.size());
    return ((average * adcVref / ADC_RESULT_SCALAR_BAT_CURRENT) -
            arrayHalOffset) /
           CurrentSenseCurrentRatio;
}

float Hardware::calc_thermistor_temp(float resistance) {
    float resistance_scaled =
        resistance /
        10000;  // scale the resistance so the polynomials don't get out of hand
    float temp = tempCoefficients[0];
    float rPoly = 1;
    for (unsigned int i = 1; i < tempCoefficients.size(); i++) {
        rPoly *= resistance_scaled;
        temp += tempCoefficients[i] * rPoly;
    }
    return temp;
}

float Hardware::get_heatsink_temp() {
    auto measurements = sense.get_misc_measurements();
    // Calculate the voltage here first
    auto voltage =
        static_cast<float>(measurements[0] * adcVref / ADC_RESULT_SCALAR_MISC);
    // This is using the equation of a voltage divider
    float resistance = tempReistorBase * adcVref / voltage - tempReistorBase;
    return calc_thermistor_temp(resistance);
}

float Hardware::get_bus_voltage() {
    auto measurements = sense.get_misc_measurements();
    return (measurements[1] * adcVref / ADC_RESULT_SCALAR_MISC /
            HV_VoltageDividerRatio) *
               HVSlope +
           HVOffset;
}

float Hardware::get_array_hal_ref_voltage() {
    auto measurements = sense.get_misc_measurements();
    return static_cast<float>(measurements[2] * adcVref /
                              ADC_RESULT_SCALAR_MISC);
}

float Hardware::get_bat_hal_ref_voltage() {
    auto measurements = sense.get_misc_measurements();
    return static_cast<float>(measurements[3] * adcVref /
                              ADC_RESULT_SCALAR_MISC);
}

float Hardware::get_internal_ic_temp() {
    auto measurements = sense.get_misc_measurements();
    const uint16_t *const ADC_TEMP_3V3_30C =
        reinterpret_cast<uint16_t *>(TEMPSENSOR_CAL1_ADDR);
    const uint16_t *const ADC_TEMP_3V3_110C =
        reinterpret_cast<uint16_t *>(TEMPSENSOR_CAL2_ADDR);
    const float CALIBRATION_REFERENCE_VOLTAGE = TEMPSENSOR_CAL_VREFANALOG;

    const float REFERENCE_VOLTAGE = 3300.0F;  // Vref internal in mV

    // scale constants to current reference voltage
    float adcCalTemp30C = static_cast<float>(*ADC_TEMP_3V3_30C);
    float adcCalTemp110C = static_cast<float>(*ADC_TEMP_3V3_110C);

    uint16_t adcTempValue = (measurements[4] / 16) *
                            (REFERENCE_VOLTAGE / CALIBRATION_REFERENCE_VOLTAGE);

    float temperature = (static_cast<float>(adcTempValue) - adcCalTemp30C) /
                            (adcCalTemp110C - adcCalTemp30C) *
                            (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) +
                        30.0F;
    return temperature;
}

void Hardware::updateHalOffsets() {
    auto measurements_array = sense.get_array_current_measurements();
    arrayHalOffset =
        (measurements_array[0] * adcVref / ADC_RESULT_SCALAR_ARRAY_CURRENT);

    // Update Bat OFFSET
    auto measurements_bat = sense.get_battery_current_measurements();
    batteryHalOffset =
        (measurements_bat[0] * adcVref / ADC_RESULT_SCALAR_BAT_CURRENT);
}

void Hardware::set_duty(float duty_cycle) {
    driver.set_duty(duty_cycle);
}

float Hardware::get_duty() {
    return driver.get_duty();
}

void Hardware::incrementDutyCycle(float increment) {
    driver.set_duty(driver.get_duty() + increment);
}

void Hardware::output_relay_close() {
    HAL_GPIO_WritePin(OUTPUT_RELAY_PORT, OUTPUT_RELAY_PIN, GPIO_PIN_SET);
}

void Hardware::output_relay_open() {
    HAL_GPIO_WritePin(OUTPUT_RELAY_PORT, OUTPUT_RELAY_PIN, GPIO_PIN_RESET);
}

void Hardware::precharge_relay_close() {
    HAL_GPIO_WritePin(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN, GPIO_PIN_SET);
}

void Hardware::precharge_relay_open() {
    HAL_GPIO_WritePin(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN,
                      GPIO_PIN_RESET);
}

void Hardware::shut_down() {
    driver.stop();
    output_relay_open();
}

void Hardware::set_array_current_limit(float limit) {
    internal_dac4.set_ch1_voltage(arrayHalOffset +
                                  limit * CurrentSenseCurrentRatio);
}

void Hardware::set_array_voltage_limit(float limit) {
    internal_dac3.set_ch2_voltage(limit * HV_VoltageDividerRatio);
}

void Hardware::set_battery_current_limit(float limit) {
    internal_dac3.set_ch1_voltage(batteryHalOffset +
                                  limit * CurrentSenseCurrentRatio);
}

void Hardware::set_battery_voltage_limit(float limit) {
    internal_dac1.set_ch1_voltage(limit * HV_VoltageDividerRatio);
}

void Hardware::start_comparators() {
    // Startup the comparators to monitor the voltage
    comparator::init();
    comparator::start();
}

COMP_HandleTypeDef *Hardware::getComp1Handle() {
    return comparator::getComp1Handle();
}

COMP_HandleTypeDef *Hardware::getComp2Handle() {
    return comparator::getComp2Handle();
}

COMP_HandleTypeDef *Hardware::getComp3Handle() {
    return comparator::getComp3Handle();
}

COMP_HandleTypeDef *Hardware::getComp7Handle() {
    return comparator::getComp7Handle();
}

void Hardware::enableSynchronous() {
    driver.enableSynchronousMode();
}

void Hardware::disableSynchronous() {
    driver.disableSynchronousMode();
}

void Hardware::updateCoreMeasurements() {
    inputCurrentInternal = get_input_current();
    inputVoltageInternal = get_input_voltage();
    outputCurrentInternal = get_output_current();
    outputVoltageInternal = get_output_voltage();
}

void Hardware::updateConversionMode() {
    float freq = driver.get_freq();
    float ripple_current_aprox =
        (inputVoltage)*driver.get_duty() / (coilInductance * freq);

    // Multiply by 1.1 for a 10% tolerance on ripple, 1.2 is for hysteresis
    // Check if your in asychronous mode and need to transition to sychronous
    // mode
    float inputPower = inputCurrent * inputVoltage;
    // Above a power limit so see if you can transition
    if (driver.get_mode() == driver_mode::asynchronous) {
        if ((ripple_current_aprox * 1.2 / 2 <= inputCurrent) &&
            inputPower > minSynchPower_rising) {
            driver.enableSynchronousMode();
        }
    }
    // Check if your in sychronous mode and need to transition to
    // asychronous mode
    else if (driver.get_mode() == driver_mode::synchronous) {
        if ((ripple_current_aprox * 1.1 / 2 >= inputCurrent) ||
            inputPower <= minSynchPower_falling) {
            driver.disableSynchronousMode();
        }
    } else {
    }
}

/* Do not put in more than 2^16 elements in here!!! */
float hardware::average(const uint16_t *data, uint16_t size) {
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

}  // namespace umnsvp::mppt
