
#include "safety.h"

#include "SEGGER_RTT.h"
#include "pin_defines.h"
#include "status_led.h"

namespace umnsvp::mppt::safety {

/* Sets the various limits for the converter within the comparators */
// TODO: The problem with settings limits for currents is HAL offset, need to
// take offset into account
void initialize_comparator_limits(Hardware *const mppt_hardware) {
    // Set default values for the DAC's

    mppt_hardware->set_battery_voltage_limit(max_output_voltage_hardware_limit);
    mppt_hardware->set_array_voltage_limit(max_input_voltage_hardware_limit);

    mppt_hardware->set_battery_current_limit(max_output_current_hardware_limit);
    mppt_hardware->set_array_current_limit(max_input_current_hardware_limit);

    // Now that the limits are correctly set, you can start the comparator
    mppt_hardware->start_comparators();
}

/*
 * checks to make sure the converter is operating safely
 */
bool check_converter_safe(Hardware *const mppt_hardware, fault_info &info) {
    float outputVoltage = mppt_hardware->outputVoltage;
    bool over_voltage_output =
        (outputVoltage >= max_output_voltage_software_limit);

    float inputVoltage = mppt_hardware->inputVoltage;
    bool over_voltage_input =
        (inputVoltage >= max_input_voltage_software_limit);

    float outputCurrent = mppt_hardware->outputCurrent;
    bool over_current_output =
        (outputCurrent >= max_output_current_software_limit);

    float inputCurrent = mppt_hardware->inputCurrent;
    bool over_current_input =
        (inputCurrent >= max_input_current_software_limit);

    float heatsinkTemp = mppt_hardware->get_heatsink_temp();
    bool over_temp = (heatsinkTemp >= max_heatsink_temp);

    return !over_voltage_output && !over_voltage_input &&
           !over_current_output && !over_current_input && !over_temp;
}

}  // namespace umnsvp::mppt::safety
