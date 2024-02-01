//
// Created by Maxim on 11/6/2022.
//

#include "gate_driver.h"

#include <cmath>

#include "pin_defines.h"

namespace umnsvp::mppt {
GateDriver::GateDriver(float freq, float deadtime)
    : frequency{freq}, deadtime(deadtime), cur_mode(driver_mode::off) {
    deadtime_pulse_length = std::ceil(DEADTIME * 1 / FSW);
}

void GateDriver::initGpios() {
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };

    gpio.Pin = LOW_GATE_PIN;
    gpio.Alternate = GPIO_AF5_TIM8;
    HAL_GPIO_Init(LOW_GATE_PORT, &gpio);

    gpio.Pin = AUX_GATE_PIN;
    gpio.Alternate = GPIO_AF4_TIM8;
    gpio.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(HIGH_GATE_PORT, &gpio);
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

    float sys_clock = HAL_RCC_GetSysClockFreq();
    float period_counts = sys_clock / ((timer.Init.Prescaler + 1) * frequency);

    timer.Init.Period = period_counts;
    timer.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV4;  // For break filter timing
    timer.Init.RepetitionCounter = 0;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        while (1)
            ;
    }
    // sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
    // sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &sMasterConfig) !=
        HAL_OK) {
        while (1)
            ;
    }
    sConfigOC.OCMode = TIM_OCMODE_COMBINED_PWM2;
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

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Will be loaded in later by duty cycle set functions
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_2) !=
        HAL_OK) {
        while (1)
            ;
    }

    // Logic on this flipped for now bc wrong pin used
    sConfigOC.OCMode = TIM_OCMODE_COMBINED_PWM2;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_3) !=
        HAL_OK) {
        while (1)
            ;
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Will be loaded in later by duty cycle set
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, TIM_CHANNEL_4) !=
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
    // Break filter running at 40MHz.
    // Break timer currently ocnfigured for fsample = 40Mhz/8 = 5MHz
    // Then, 6 consecutive samples are required to trigger
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0x8;
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
}

void GateDriver::start_utility_timers() {
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_5);
}

float GateDriver::get_duty() const {
    return duty;
}

float GateDriver::get_freq() const {
    return frequency;
}

driver_mode GateDriver::get_mode() const {
    return cur_mode;
}

void GateDriver::clear_rearm_break_fault() {
    timer.Instance->BDTR = timer.Instance->BDTR | TIM_BDTR_BKDSRM;
}

/**
 *
 * @return True or false depending on if the start was succesful
 */
bool GateDriver::start() {
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_4);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_4);
    return true;
}

void GateDriver::stop() {
    HAL_TIM_PWM_Stop(&timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&timer, TIM_CHANNEL_4);
}

// TODO Global Reload? These register updates NEED to be synced
void GateDriver::set_duty(float value, float aux_pulse_length) {
    if (value > max_duty) {
        value = max_duty;
    }

    if (value < min_duty) {
        value = 0;
    }

    auto duty_pulse_counts = static_cast<uint16_t>(value * timer.Instance->ARR);
    auto aux_pulse_counts = static_cast<uint32_t>(aux_pulse_length * frequency *
                                                  timer.Instance->ARR);

    // CCR1 indicates start time of the Main pulse CCR2 is end time. CCR3 is
    // start time of the aux pulse CCR4 is end time of aux pulse

    timer.Instance->CCR3 = 0xFFFF & 0;  // Just start at zero CNT
    timer.Instance->CCR4 = 0xFFFF & std::max(aux_pulse_counts, 1UL);
    timer.Instance->CCR1 =
        0xFFFF & std::max(aux_pulse_counts + deadtime_pulse_length, 1UL);
    timer.Instance->CCR2 =
        0xFFFF &
        std::max(aux_pulse_counts + deadtime_pulse_length + duty_pulse_counts,
                 1UL);
    duty = value;
}

}  // namespace umnsvp::mppt
