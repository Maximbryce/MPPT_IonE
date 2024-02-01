#include <array>

#include "adc.h"
#include "dac.h"
#include "gate_driver.h"
#include "stm32g4xx.h"

#pragma once

namespace umnsvp::mppt {

namespace hardware {
float average(const uint16_t* data, uint16_t size);
}

class Hardware {
   private:
    ADCSensing sense;
    GateDriver driver;
    InternalDac internal_dac1;
    InternalDac internal_dac3;
    InternalDac internal_dac4;

    float inputVoltageInternal;
    float outputVoltageInternal;
    float inputCurrentInternal;
    float outputCurrentInternal;

    // Temperature scaling factors for converting from resistance to a
    // temperature
    std::array<float, 6> tempCoefficients = {111.5,  -250.6, 335.6,
                                             -249.6, 90.98,  -12.55};

    static constexpr float coilInductance = 418e-6;
    // Temp static resistor in voltage divider
    static constexpr float tempReistorBase = 10000;
    // Ratio of the voltage divider for the HV system
    static constexpr float HV_VoltageDividerRatio = 4.22 / 200;
    // Ration of voltage output on the HAL vs current through it
    // This number is in V/A: 100mv/A
    static constexpr float CurrentSenseCurrentRatio = 200.0 / 1000.0;
    // Non const members bc the value may change on startup
    // TODO: Eventually move these offsets to hardware within the ADC peripheral
    float arrayHalOffset = 0;    // 0.1 * 3.3;
    float batteryHalOffset = 0;  // 0.1 * 3.3;

    // HV voltage scalaing
    static constexpr float HVSlope = 1.028;
    static constexpr float HVOffset = 0.09253;

    static constexpr float minSynchPower = 20.0f;
    static constexpr float minSynchPower_rising = minSynchPower * 1.15f;
    static constexpr float minSynchPower_falling = minSynchPower * 1.0f;

    void initGpios();
    void setupComparatorReferences();

    void calibrateOutputHal();
    void calibrateInputHal();

   public:
    static constexpr float maxPrechargeDiff = 2;
    static constexpr float minOutputVoltage = 20;

    void init();

    void start_comparators();

    /* Both of these access types for the core measurments exist. The references
     * are used in the algorithm and main loop to increase speed of those
     * methods. The regular getters exist in all other contexts to keep code
     * simpler and safer where speed is not important*/
    const float& inputVoltage;
    const float& outputVoltage;
    const float& inputCurrent;
    const float& outputCurrent;

    float get_output_voltage();
    float get_output_current();
    float get_average_output_current();
    float get_input_current();
    float get_average_input_current();
    float get_input_voltage();

    float get_heatsink_temp();
    float get_internal_ic_temp();
    float get_bus_voltage();
    float get_array_hal_ref_voltage();
    float get_bat_hal_ref_voltage();

    void updateHalOffsets();
    void updateCoreMeasurements();
    void updateConversionMode();

    float calc_thermistor_temp(float resistance);

    // Open and close for main output relay
    void output_relay_close();
    void output_relay_open();

    // open and close for precharge relay
    void precharge_relay_close();
    void precharge_relay_open();

    void set_array_voltage_limit(float limit);
    void set_array_current_limit(float limit);
    void set_battery_voltage_limit(float limit);
    void set_battery_current_limit(float limit);

    COMP_HandleTypeDef* getComp1Handle();
    COMP_HandleTypeDef* getComp2Handle();
    COMP_HandleTypeDef* getComp3Handle();
    COMP_HandleTypeDef* getComp7Handle();

    void set_duty(float duty_cycle);
    float get_duty();
    void set_debug_dac_voltage(float voltage);
    void incrementDutyCycle(float increment);
    void shut_down();

    void enableSynchronous();
    void disableSynchronous();

    Hardware();
};

}  // namespace umnsvp::mppt