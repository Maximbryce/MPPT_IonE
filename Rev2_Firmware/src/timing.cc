#include "timing.h"

namespace umnsvp::mppt::timing {

TIM_TypeDef *const sysTimerInstance = TIM7;
TIM_TypeDef *const controlTimerInstance = TIM6;
TIM_TypeDef *const hardwareTimerInstance = TIM3;

TIM_HandleTypeDef sysTimerHandle = {0};
TIM_HandleTypeDef controlTimerHandle = {0};
TIM_HandleTypeDef hardwareTimerHandle = {0};

void startSysTimer() {
    /* For 160Mhz this will have the clock run at 1 kHrtz */
    __HAL_RCC_TIM7_CLK_ENABLE();
    sysTimerHandle.Instance = sysTimerInstance;
    sysTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sysTimerHandle.Init.Prescaler = 399;
    sysTimerHandle.Init.Period = 400;
    sysTimerHandle.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    sysTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&sysTimerHandle) != HAL_OK) {
        while (1)
            ;
    }

    HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 4, 3);
    HAL_NVIC_EnableIRQ(TIM7_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&sysTimerHandle);
    // ENABLE this timer in inerupt mode without enabling the irq, that
    // way we get the flag but not stop interuption
    __HAL_TIM_ENABLE_IT(&sysTimerHandle, TIM_IT_UPDATE);
}

void startCtrlTimer() {
    /* For 160Mhz this will have the clock run at 5000 Hrtz */
    __HAL_RCC_TIM6_CLK_ENABLE();
    controlTimerHandle.Instance = controlTimerInstance;
    controlTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    controlTimerHandle.Init.Prescaler = (3999);
    controlTimerHandle.Init.Period = 8;
    controlTimerHandle.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    controlTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&controlTimerHandle) != HAL_OK) {
        while (1)
            ;
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&controlTimerHandle);
    //  ENABLE this timer in inerupt mode without enabling the irq, that
    //  way we get the flag but not stop interuption
    __HAL_TIM_ENABLE_IT(&controlTimerHandle, TIM_IT_UPDATE);
}

void startHardwareTimer() {
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* For 160Mhz this will have the clock run at 5Khz Hrtz */
    __HAL_RCC_TIM3_CLK_ENABLE();
    hardwareTimerHandle.Instance = hardwareTimerInstance;
    hardwareTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hardwareTimerHandle.Init.Prescaler = 39;
    hardwareTimerHandle.Init.Period = 800;
    hardwareTimerHandle.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    hardwareTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&hardwareTimerHandle) != HAL_OK) {
        while (1)
            ;
    }

    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 3);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&hardwareTimerHandle,
                                              &sMasterConfig) != HAL_OK) {
        while (1)
            ;
    }

    HAL_TIM_Base_Start_IT(&hardwareTimerHandle);
    // ENABLE this timer in inerupt mode without enabling the irq, that
    // way we get the flag but not stop interuption
    __HAL_TIM_ENABLE_IT(&hardwareTimerHandle, TIM_IT_UPDATE);
}

TIM_HandleTypeDef *getSysTimHandle() {
    return &sysTimerHandle;
}

TIM_HandleTypeDef *getCtrlTimHandle() {
    return &controlTimerHandle;
}

TIM_HandleTypeDef *getHardwareTimHandle() {
    return &hardwareTimerHandle;
}
}  // namespace umnsvp::mppt::timing