
#include "safety.h"

#include "SEGGER_RTT.h"
#include "hardware.h"
#include "pin_defines.h"
#include "status_led.h"

namespace umnsvp::mppt::safety {

/* Sets the various limits for the converter within the comparators */
// TODO: The problem with settings limits for currents is HAL offset, need to
// take offset into account
void initialize_comparator_limits(void) {
    // Set default values for the DAC's

    hardware::set_battery_voltage_limit(max_output_voltage_hardware_limit);
    hardware::set_array_voltage_limit(max_input_voltage_hardware_limit);

    hardware::set_slow_array_current_limit(max_output_current_hardware_limit);
    hardware::set_fast_array_current_limit(max_input_current_hardware_limit);

    // Now that the limits are correctly set, you can start the comparator
    hardware::start_comparators();
}

/*
 * checks to make sure the converter is operating safely
 */
bool check_converter_safe(fault_info &info) {
    float outputVoltage = hardware::get_output_voltage();
    bool over_voltage_output =
        (outputVoltage >= max_output_voltage_software_limit);

    float inputVoltage = hardware::get_input_voltage();
    bool over_voltage_input =
        (inputVoltage >= max_input_voltage_software_limit);

    float inputCurrent = hardware::get_filtered_input_current();
    bool over_current_input =
        (inputCurrent >= max_input_current_software_limit);

    float heatsinkTemp = hardware::get_heatsink_temp();
    bool over_temp = (heatsinkTemp >= max_heatsink_temp);

    return true;

    return !over_voltage_output && !over_voltage_input && !over_current_input &&
           !over_temp;
}

}  // namespace umnsvp::mppt::safety
