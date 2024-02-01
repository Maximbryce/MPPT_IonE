#pragma once
#include "stm32g4xx.h"

namespace umnsvp::mppt::mppt {

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

void mppt_controller(void);
void static_controller(void);
algorithmOutput mppt_algorithm(void);

}  // namespace umnsvp::mppt::mppt