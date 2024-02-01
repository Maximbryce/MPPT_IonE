#pragma once
#include "hardware.h"

/*
 * The current implemented algorithm is incremental conductance
 */

namespace umnsvp::mppt::algorithm {

enum class algorithmOutput : uint8_t
{
    increaseInputVoltage, /* Application wise this means decreasing duty */
    decreaseInputVoltage, /* Application wise this means increasing duty */
    noChange, /* Application wise this means not changing duty cycle */
};

/* These are the thresholds for what is considered a 'zero' change in current
 * for the algorithm. Changes below this threshold are considered noise */
constexpr float zeroVoltageTolerance = 0.10;
constexpr float zeroCurrentTolerance = 0.05;
// constexpr float zeroConductanceTolerance = 0.01;

algorithmOutput step_algorithm(Hardware *const mpptHardware);
}  // namespace umnsvp::mppt::algorithm