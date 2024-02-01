#pragma once

#include "stm32g4xx.h"

namespace umnsvp::mppt::op_amp {

void init_pins();

void op_amp_start();

void init_op_amp();
}  // namespace umnsvp::mppt::op_amp