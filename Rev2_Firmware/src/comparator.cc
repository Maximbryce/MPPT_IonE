#include "comparator.h"

#include "pin_defines.h"

namespace umnsvp::mppt::hardware {
// Handles defined up here bc they don't need to be in header
COMP_HandleTypeDef batteryVoltageComp;
COMP_HandleTypeDef slowArrayCurrentComp;
COMP_HandleTypeDef arrayVoltageComp;
COMP_HandleTypeDef fastArrayCurrentComp;

/* Function declerations */
void comparator_start();
void comparator_init_pins();

void comparator_init_pins() {
    // Define a default struct for all these pins
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,  // I don't think this is technically
                                        // necessary bc analog
    };

    // Initialize all the input+ pins for the comparators
    gpio.Pin = comp1_inp_pin;
    HAL_GPIO_Init(comp1_inp_port, &gpio);
    gpio.Pin = comp2_inp_pin;
    HAL_GPIO_Init(comp2_inp_port, &gpio);
    gpio.Pin = comp3_inp_pin;
    HAL_GPIO_Init(comp3_inp_port, &gpio);
    gpio.Pin = comp7_inp_pin;
    HAL_GPIO_Init(comp7_inp_port, &gpio);
}

void comparator_init() {
    comparator_init_pins();

    // Enable clock
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    COMP_InitTypeDef defaultInit = {
        // All input pluses are the first options
        .InputPlus = COMP_INPUT_PLUS_IO1,
        // Because this is just a trigger, no hysteresis
        .Hysteresis = COMP_HYSTERESIS_HIGH,  // Corresponds to around ~3.33 V of
                                             // hysteresis
                                             // on HV bus, 1.3A on current
        .OutputPol = COMP_OUTPUTPOL_NONINVERTED,
        .TriggerMode = COMP_TRIGGERMODE_IT_RISING,
    };

    // Set comparator 3 which is for current from the battery
    slowArrayCurrentComp.Init = defaultInit;
    slowArrayCurrentComp.Init.BlankingSrce =
        0;  // COMP_BLANKINGSRC_TIM8_OC5_COMP3;
    slowArrayCurrentComp.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
    slowArrayCurrentComp.Instance = COMP3;

    // Set comparator 1 which is for voltage from the battery
    batteryVoltageComp.Init = defaultInit;
    batteryVoltageComp.Init.BlankingSrce =
        0;  // COMP_BLANKINGSRC_TIM8_OC5_COMP1;
    batteryVoltageComp.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
    batteryVoltageComp.Instance = COMP1;

    // Set comparator 7 which is for current from the array
    fastArrayCurrentComp.Init = defaultInit;
    fastArrayCurrentComp.Init.BlankingSrce =
        0;  // COMP_BLANKINGSRC_TIM8_OC5_COMP7;
    fastArrayCurrentComp.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH1;
    fastArrayCurrentComp.Instance = COMP7;

    // Set comparator 2 which is for voltage from the array
    arrayVoltageComp.Init = defaultInit;
    arrayVoltageComp.Init.BlankingSrce = 0;  // COMP_BLANKINGSRC_TIM8_OC5_COMP2;
    arrayVoltageComp.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
    arrayVoltageComp.Instance = COMP2;

    // Initialize the comparators
    bool status1 =
        HAL_COMP_Init(&slowArrayCurrentComp) == HAL_StatusTypeDef::HAL_OK;
    bool status2 =
        HAL_COMP_Init(&batteryVoltageComp) == HAL_StatusTypeDef::HAL_OK;
    bool status3 =
        HAL_COMP_Init(&fastArrayCurrentComp) == HAL_StatusTypeDef::HAL_OK;
    bool status4 =
        HAL_COMP_Init(&arrayVoltageComp) == HAL_StatusTypeDef::HAL_OK;

    // Block if initialization fails
    if (!status1 | !status2 | !status3 | !status4) {
        while (1)
            ;
    }
    comparator_start();
}

void comparator_start() {
    //        bool status1 =
    //            HAL_COMP_Start(&slowArrayCurrentComp) ==
    //            HAL_StatusTypeDef::HAL_OK;
    bool status2 =
        HAL_COMP_Start(&batteryVoltageComp) == HAL_StatusTypeDef::HAL_OK;
    //        bool status3 =
    //            HAL_COMP_Start(&fastArrayCurrentComp) ==
    //            HAL_StatusTypeDef::HAL_OK;
    bool status4 =
        HAL_COMP_Start(&arrayVoltageComp) == HAL_StatusTypeDef::HAL_OK;

    // Block if initialization fails
    // ! status1 | !status3 |
    if (!status2 | !status4) {
        while (1)
            ;
    }

    __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP3_EXTI_CLEAR_FLAG();
    __HAL_COMP_COMP7_EXTI_CLEAR_FLAG();

    // Enable the interrupts and set the priorities. Priorities are high
    // HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
    // HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 2, 1);
    // HAL_NVIC_EnableIRQ(COMP7_IRQn);
    // HAL_NVIC_SetPriority(COMP7_IRQn, 2, 2);
}

COMP_HandleTypeDef *getComp1Handle() {
    return &batteryVoltageComp;
}

COMP_HandleTypeDef *getComp2Handle() {
    return &arrayVoltageComp;
}

COMP_HandleTypeDef *getComp3Handle() {
    return &slowArrayCurrentComp;
}

COMP_HandleTypeDef *getComp7Handle() {
    return &fastArrayCurrentComp;
}

}  // namespace umnsvp::mppt::hardware
