#include "op_amp.h"

#include "pin_defines.h"

namespace umnsvp::mppt::hardware {
OPAMP_HandleTypeDef opAmp1Handle = {};  // OpAmp for battery sense
OPAMP_HandleTypeDef opAmp2Handle = {};  // OpAmp for array sense
OPAMP_HandleTypeDef opAmp4Handle = {};  // OpAmp for bus sense

void op_amp_start() {
    if (HAL_OPAMP_Start(&opAmp1Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_OPAMP_Start(&opAmp2Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_OPAMP_Start(&opAmp4Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
};

void op_amp_init_pins() {
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };

    // Initialize the pins for the op amp
    gpio.Pin = OPAMP1_VINP_PIN;
    HAL_GPIO_Init(OPAMP1_VINP_PORT, &gpio);

    gpio.Pin = OPAMP1_VOUT_PIN;
    HAL_GPIO_Init(OPAMP1_VOUT_PORT, &gpio);

    gpio.Pin = OPAMP2_VINP_PIN;
    HAL_GPIO_Init(OPAMP2_VINP_PORT, &gpio);

    gpio.Pin = OPAMP2_VOUT_PIN;
    HAL_GPIO_Init(OPAMP2_VOUT_PORT, &gpio);

    gpio.Pin = OPAMP4_VINP_PIN;
    HAL_GPIO_Init(OPAMP4_VINP_PORT, &gpio);

    gpio.Pin = OPAMP4_VOUT_PIN;
    HAL_GPIO_Init(OPAMP4_VOUT_PORT, &gpio);
}

void op_amp_init() {
    op_amp_init_pins();

    OPAMP_InitTypeDef default_init = {
        .PowerMode = OPAMP_POWERMODE_HIGHSPEED,
        .Mode = OPAMP_FOLLOWER_MODE,
        .InternalOutput = DISABLE,
        .TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE,
        .UserTrimming = OPAMP_TRIMMING_FACTORY,
    };

    opAmp1Handle.Init = default_init;
    // Signal routing in page 782 of reference manual
    opAmp1Handle.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
    opAmp1Handle.Instance = OPAMP1;
    if (HAL_OPAMP_Init(&opAmp1Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;  // Catch an error
    }

    opAmp2Handle.Init = default_init;
    // Signal routing in page 782 of reference manual
    opAmp2Handle.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
    opAmp2Handle.Instance = OPAMP2;
    if (HAL_OPAMP_Init(&opAmp2Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;  // Catch an error
    }

    opAmp4Handle.Init = default_init;
    // Signal routing in page 782 of reference manual
    opAmp4Handle.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
    opAmp4Handle.Instance = OPAMP4;
    if (HAL_OPAMP_Init(&opAmp4Handle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;  // Catch an error
    }
    op_amp_start();
}
}  // namespace umnsvp::mppt::hardware