#pragma once
#include "stm32g4xx.h"

namespace umnsvp::mppt::timing {

void startCtrlTimer(void);
void startSysTimer(void);
void startHardwareTimer(void);

TIM_HandleTypeDef *getSysTimHandle();
TIM_HandleTypeDef *getCtrlTimHandle();
TIM_HandleTypeDef *getHardwareTimHandle();

}  // namespace umnsvp::mppt::timing