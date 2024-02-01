#pragma once

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
constexpr float max_input_voltage_hardware_limit = 90;
constexpr float max_output_voltage_hardware_limit = 150;

/* Current limits as seen by the HAL sensors */
constexpr float max_input_current_hardware_limit = 20;
constexpr float max_output_current_hardware_limit = 20;

/*
 * The below limits are for software checks
 */

/* Voltage limits from the ADC's */
constexpr float max_input_voltage_software_limit = 90;
constexpr float max_output_voltage_software_limit = 150;

/* Current limits as seen by the HAL sensors */
constexpr float max_input_current_software_limit = 20;
constexpr float max_output_current_software_limit = 20;

/* Temp limits for the heatsink in celcius*/
constexpr float max_heatsink_temp = 60;

void initialize_comparator_limits(void);
bool check_converter_safe(fault_info& fault);

}  // namespace umnsvp::mppt::safety