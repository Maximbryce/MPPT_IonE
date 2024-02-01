//
// Created by Maxim on 11/6/2022.
//

#include "gate_driver.h"

#include <cmath>

#include "pin_defines.h"

namespace umnsvp::mppt {
GateDriver::GateDriver(gate_driver_frequency freq)
    : frequency{freq}, cur_mode(driver_mode::off) {
}

void GateDriver::initGpios() {
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };

    gpio.Pin = HIGH_GATE_PIN;
    gpio.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(HIGH_GATE_PORT, &gpio);

    gpio.Pin = LOW_GATE_PIN;
    gpio.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(LOW_GATE_PORT, &gpio);
}

/**
 * Starts the PWM channel with a specified frequency and a 0% Duty
 * @param frequency
 */
void GateDriver::init() {
    initGpios();
    __HAL_RCC_TIM8_CLK_ENABLE();

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};

    timer.Instance = TIM8;
    timer.Init.Prescaler = 0;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = static_cast<uint16_t>(frequency);
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    timer.Init.RepetitionCounter = 0;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        while (1)
            ;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &sMasterConfig) !=
        HAL_OK) {
        while (1)
            ;
    }
    sConfigOC.OCMode = TIM_OCMODE_COMBINED_PWM1;
    sConfigOC.Pulse = 0;  // Will be loaded in later by duty cycle set functions
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_1) !=
        HAL_OK) {
        while (1)
            ;
    }

    // Config Channel 5 so it can be used as a blanking source for the
    // comparators
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse =
        static_cast<uint16_t>((uint16_t)frequency * blankingWindowPercentage);
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_5) !=
        HAL_OK) {
        while (1)
            ;
    }

    // Config channel 2 for timing of the ADC. The Pulse value Should always
    // be
    // half the value in the PWM compare register to happen halfway through
    // d period. It is 1 by default so the pulse always occurs even if the
    // value
    // of duty is 0
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_3) !=
        HAL_OK) {
        while (1)
            ;
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, TIM_CHANNEL_3);

    // Config channel 3 for timing of the ADC. The Pulse value Should always
    // be
    // 3/2 the value in the PWM compare register to happen halfway through
    // d' period. It is 1 by default so the pulse always occurs even if the
    // value
    // of duty is 0
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_4) !=
        HAL_OK) {
        while (1)
            ;
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, TIM_CHANNEL_4);

    sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
    sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
    sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &sBreakInputConfig) != HAL_OK) {
        while (1)
            ;
    }
    sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &sBreakInputConfig) != HAL_OK) {
        while (1)
            ;
    }
    sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP3;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &sBreakInputConfig) != HAL_OK) {
        while (1)
            ;
    }
    sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP7;

    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &sBreakInputConfig) != HAL_OK) {
        while (1)
            ;
    }

    // TODO: Implement the break funcationality with pins on PCB
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime =
        static_cast<uint32_t>(gate_driver_deadtime::ns_250);
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&timer, &sBreakDeadTimeConfig) !=
        HAL_OK) {
        while (1)
            ;
    }

    start_utility_timers();
    disableSynchronousMode();
}

void GateDriver::start_utility_timers() {
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_5);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_4);
}

/*
 * Change pin HIGH side gate to a GND'd pin to keep the MOSFET of and force
 * conduction through the body diode
 */
void GateDriver::disableSynchronousMode() {
    GPIO_InitTypeDef gpio = {
        .Pin = HIGH_GATE_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    stop();
    HAL_GPIO_Init(HIGH_GATE_PORT, &gpio);
    // Turn the GPIO off so the gate driver is also off
    HAL_GPIO_WritePin(HIGH_GATE_PORT, HIGH_GATE_PIN, GPIO_PIN_RESET);
    start();
    cur_mode = driver_mode::asynchronous;
}

/*
 * Change pin HIGH side gate to a timer pin with CH1N to make the converter
 * synchronous
 */
void GateDriver::enableSynchronousMode() {
    GPIO_InitTypeDef gpio = {
        .Pin = HIGH_GATE_PIN,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF4_TIM8,
    };
    stop();
    HAL_GPIO_Init(HIGH_GATE_PORT, &gpio);
    start();
    cur_mode = driver_mode::synchronous;
}

float GateDriver::get_duty() const {
    return static_cast<float>(timer.Instance->CCR1) /
           static_cast<float>(timer.Instance->ARR);
}

float GateDriver::get_freq() const {
    float sys_clock = HAL_RCC_GetSysClockFreq();
    return sys_clock / ((timer.Init.Prescaler + 1) * timer.Init.Period);
}

driver_mode GateDriver::get_mode() const {
    return cur_mode;
}

/**
 *
 * @return True or false depending on if the start was succesful
 */
bool GateDriver::start() {
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_1);
    return true;
}

void GateDriver::stop() {
    HAL_TIM_PWM_Stop(&timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&timer, TIM_CHANNEL_1);
}

void GateDriver::set_duty(float value) {
    if (value > max_duty) {
        value = max_duty;
    }

    if (value < min_duty) {
        value = 0;
    }

    auto compare_value = static_cast<uint16_t>(value * timer.Instance->ARR);
    auto halfway_d_period =
        static_cast<uint16_t>((value * timer.Instance->ARR) / 2);
    auto halfway_dp_period = static_cast<uint16_t>(
        (value * timer.Instance->ARR + timer.Instance->ARR) / 2);

    // Truncate the value and load it
    timer.Instance->CCR1 = 0xFFFF & compare_value;
    // Set the values for the compainion compare values for ADC timing. Minimum
    // value of 1 so the ADC always triggers
    timer.Instance->CCR3 = 0xFFFF & std::max(halfway_d_period, ((uint16_t)1));
    timer.Instance->CCR4 = 0xFFFF & std::max(halfway_dp_period, ((uint16_t)1));
    duty = value;
}

}  // namespace umnsvp::mppt
