#pragma once

#include "hardware.h"

namespace umnsvp::mppt::safety {

struct fault_info {
    bool input_voltage_fault;
    bool input_current_fault;
    bool output_voltage_fault;
    bool output_current_fault;
    float fault_value;
};

/*
 * The below limits are hardware checks for the comparators
 */

/* Voltage limits from the ADC's */
constexpr float max_input_voltage_hardware_limit = 150;
constexpr float max_output_voltage_hardware_limit = 150;

/* Current limits as seen by the HAL sensors */
constexpr float max_input_current_hardware_limit = 20;
constexpr float max_output_current_hardware_limit = 20;

/*
 * The below limits are for software checks
 */

/* Voltage limits from the ADC's */
constexpr float max_input_voltage_software_limit = 150;
constexpr float max_output_voltage_software_limit = 150;

/* Current limits as seen by the HAL sensors */
constexpr float max_input_current_software_limit = 10;
constexpr float max_output_current_software_limit = 10;

/* Temp limits for the heatsink in celcius*/
constexpr float max_heatsink_temp = 60;

void initialize_comparator_limits(Hardware* const mppt_hardware);
bool check_converter_safe(Hardware* const mppt_hardware, fault_info& fault);

}  // namespace umnsvp::mppt::safety