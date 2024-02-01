/**
 * @file main.cc
 * @author Maxim Erickson (eric4190@umn.edu)
 * @brief Runs the main method for the mppt
 * @version 0.1
 * @date 2023-1-1
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main.h"

#include <functional>

#include "SEGGER_RTT.h"
#include "application.h"
#include "state_machine.h"

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
umnsvp::mppt::Application app;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableUCPDDeadBattery();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    SEGGER_RTT_Init();

    app.main();
    return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 80;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1)
            ;
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1)
            ;
    }
}

namespace umnsvp::mppt {

/* Precharge */
void precharge_on_entry_wrapper(State *prev_state);
State *precharge_wrapper();
void precharge_on_exit_wrapper(State *next_state);
/* Startup */
void startup_on_entry_wrapper(State *prev_state);
State *startup_wrapper();
void startup_on_exit_wrapper(State *next_state);
/* Idle */
void idle_on_entry_wrapper(State *prev_state);
State *idle_wrapper();
void idle_on_exit_wrapper(State *next_state);
/* Active */
void active_on_entry_wrapper(State *prev_state);
State *active_wrapper();
void active_on_exit_wrapper(State *next_state);
/* Killed */
void killed_on_entry_wrapper(State *prev_state);
State *killed_wrapper();
void killed_on_exit_wrapper(State *next_state);
/* Faulted */
void faulted_on_entry_wrapper(State *prev_state);
State *faulted_wrapper();
void faulted_on_exit_wrapper(State *next_state);

/* State definitions */
State application_states[converter_state::NUM_CONVERTER_STATES] = {
    State(NULL, &startup_wrapper, NULL,
          converter_state::STARTUP),  // STARTUP
    State(&precharge_on_entry_wrapper, &precharge_wrapper,
          precharge_on_exit_wrapper,
          converter_state::PRECHARGE),  // PRECHARGE
    State(&idle_on_entry_wrapper, &idle_wrapper, NULL,
          converter_state::IDLE),  // IDLE
    State(&active_on_entry_wrapper, &active_wrapper, NULL,
          converter_state::ACTIVE),  // ACTIVE
    State(&faulted_on_entry_wrapper, &faulted_wrapper, &faulted_on_exit_wrapper,
          converter_state::FAULTED),  // FAULTED
    State(killed_on_entry_wrapper, &killed_wrapper, NULL,
          converter_state::KILLED),  // KILLED
};

Machine application_state_machine(&application_states[STARTUP]);

State *startup_wrapper() {
    return app.startup();
}

State *precharge_wrapper() {
    return app.precharge();
}

State *idle_wrapper() {
    return app.idle();
}

State *killed_wrapper() {
    return app.killed();
}

State *faulted_wrapper() {
    return app.faulted();
}

State *active_wrapper() {
    return app.active();
}

void precharge_on_exit_wrapper(State *next_state) {
    app.precharge_on_exit(next_state);
}

void precharge_on_entry_wrapper(State *prev_state) {
    app.precharge_on_entry(prev_state);
}

void idle_on_entry_wrapper(State *prev_state) {
    app.idle_on_entry(prev_state);
}

void active_on_entry_wrapper(State *prev_state) {
    app.active_on_entry(prev_state);
}

void faulted_on_entry_wrapper(State *prev_state) {
    app.faulted_on_entry(prev_state);
}

void faulted_on_exit_wrapper(State *next_state) {
    app.faulted_on_exit(next_state);
}

void killed_on_entry_wrapper(State *prev_state) {
    app.killed_on_entry(prev_state);
}

}  // namespace umnsvp::mppt