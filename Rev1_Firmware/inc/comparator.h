#pragma once
#include "stm32g4xx.h"
namespace umnsvp::mppt::comparator {

void init_pins();

void init();

void start();

COMP_HandleTypeDef* getComp1Handle();
COMP_HandleTypeDef* getComp2Handle();
COMP_HandleTypeDef* getComp3Handle();
COMP_HandleTypeDef* getComp7Handle();

}  // namespace umnsvp::mppt::comparator
