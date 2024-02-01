#include "mppt.h"

#include "hardware.h"

namespace umnsvp::mppt::mppt {

static float prevCurrent = 0;
static float prevVoltage = 0;

static float prevPower = 0;
static int8_t direction = 1;  // TODO: replace by an enum

// #define IC
#define PD

algorithmOutput mppt_algorithm(void) {
    algorithmOutput algorithmResponse = algorithmOutput::noChange;
#if defined(IC)
    float curVoltage = hardware::->inputVoltage;
    float curCurrent = hardware::->inputCurrent;

    float deltaV = curVoltage - prevMaxPowerPointVoltage;
    float deltaI = curCurrent - prevMaxPowerPointCurrent;

    /* Check if you had a change in the input voltage */
    if (std::abs(deltaV) < zeroVoltageTolerance) {
        if (std::abs(deltaI) < zeroCurrentTolerance) {
            /* This state represents that you have no change in the maximum
             * power point */
            /* Don't update the maximum power point numbers bc you didn't update
             * the maximum power point if you where to update it could cause
             * bugs with deltaZero creepage */
            algorithmResponse = algorithmOutput::noChange;
        } else {
            /* This state represents a small change in current, but need to find
             * out which direction */
            if (deltaI > 0) {
                /* You increased your current input, attempt to increase input
                 * voltage */
                algorithmResponse = algorithmOutput::decreaseInputVoltage;
            } else {
                /* You decreased your current input, attempt to increase input
                 * voltage to find the maximum power point again */
                algorithmResponse = algorithmOutput::increaseInputVoltage;
            }
        }
    } else {
        /* This branch corresponds to a nonzero change in the input voltage */
        if (std::abs((curCurrent / curVoltage) + (deltaI / deltaV)) <
            zeroCurrentTolerance) {
            /* This corresponds to the slop of the conductance being the same as
             * the conductance, you are at the maximum power point */
            algorithmResponse = algorithmOutput::noChange;
        } else {
            if ((curCurrent / curVoltage) > (-1 * (deltaI / deltaV))) {
                /* Corresponds to being to the left of the maximum power point
                 */
                algorithmResponse = algorithmOutput::increaseInputVoltage;
            } else {
                /* Corresponds to being to the right of the maximum power point
                 */
                algorithmResponse = algorithmOutput::decreaseInputVoltage;
            }
        }
    }
    prevMaxPowerPointVoltage = curVoltage;
    prevMaxPowerPointCurrent = curCurrent;
#elif defined(PD)
    float curAverageVoltage = hardware::get_input_voltage();
    float curAverageCurrent = hardware::get_filtered_input_current();

    float curPower = curAverageCurrent * curAverageVoltage;

    if (curPower > prevPower) {
        if (curAverageVoltage > prevVoltage) {
            algorithmResponse = algorithmOutput::increaseInputVoltage;

        } else {
            algorithmResponse = algorithmOutput::decreaseInputVoltage;
        }
    } else {
        if (curAverageVoltage > prevVoltage) {
            algorithmResponse = algorithmOutput::decreaseInputVoltage;
        } else {
            algorithmResponse = algorithmOutput::increaseInputVoltage;
        }
    }

    prevPower = curPower;
    prevVoltage = curAverageVoltage;

#endif
    return algorithmResponse;
}

void mppt_controller() {
    // If timer has updated, you can start looking at the next
    // control loop run through. Only update algorithm if the
    // converter is active
    hardware::set_debug_dac_voltage(3.0);

    // Clear the flag
    algorithmOutput algorithm_action = mppt_algorithm();

    if (algorithm_action == algorithmOutput::decreaseInputVoltage) {
        hardware::incrementDutyCycle(0.005f);
    } else if (algorithm_action == algorithmOutput::increaseInputVoltage) {
        hardware::incrementDutyCycle(-0.005f);
    } else {
        // Do nothing
    }
}

}  // namespace umnsvp::mppt::mppt