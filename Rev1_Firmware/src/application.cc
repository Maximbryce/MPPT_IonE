#include "application.h"

#include "SEGGER_RTT.h"
#include "algorithm.h"
#include "pin_defines.h"

// #define TEST_MODE
// #define NO_RUN
// #define BATTERY_SIM

namespace umnsvp::mppt {
Application::Application()
    : mppt_hardware(),
      heartBeat(INDICATOR5_PORT, INDICATOR5_PIN),
      outputVoltageLED(INDICATOR2_PORT, INDICATOR2_PIN),
      outputCurrentLED(INDICATOR3_PORT, INDICATOR3_PIN),
      inputVoltageLED(INDICATOR0_PORT, INDICATOR0_PIN),
      inputCurrentLED(INDICATOR1_PORT, INDICATOR1_PIN),
      tempErrorLED(INDICATOR4_PORT, INDICATOR4_PIN),
      can_driver(FDCAN1, {can_rx_pin, can_tx_pin, can_port}),
      skylab2(can_driver, can::fifo::FIFO0) {
}

void Application::main() {
    init();
    cur_state = converter_state::idle;

    while (1) {
        mppt_hardware.set_debug_dac_voltage(1.0);
        /** BEGIN: Update the measurments and info for state transitions---**/
        mppt_hardware.updateCoreMeasurements();

        // Update whether or not the converter should be enabled/disabled
        if (skylab2.mppt_enable_buffer.pop()) {
            control_packet =
                skylab2.mppt_enable_buffer.output();  // Update control packet
        }
        /** END: Update the measurments and info for state transitions---**/

        /** BEGIN: Action on State Transitions*/
        mppt_hardware.set_debug_dac_voltage(2.0);

        switch (cur_state) {
            case converter_state::killed:
                /* State for turning off the converter in case of a fault*/
                disable_converter();
                updateErrorLEDs();
                break;

            case converter_state::idle:
                if (control_packet.enable) {
                    cur_state = converter_state::starting_up;
                    break;
                }
                disable_converter();
                break;

            case converter_state::starting_up:
                if (startup_errors >= 1) {
                    cur_state = converter_state::killed;
                    break;
                } else {
                    if (enable_converter()) {
                        cur_state = converter_state::active;
                        break;  // Go back to the begining of the loop because
                                // you changed states
                    } else {
                        startup_errors += 1;  // increment an error counter and
                                              // maybe try again later
                    }
                }
                break;

            case (converter_state::active):
                if (!safety::check_converter_safe(&mppt_hardware, curFault)) {
                    cur_state = converter_state::killed;
                    break;
                }
                if (!control_packet.enable) {
                    cur_state = converter_state::killed;
                }
                // mppt_hardware.updateConversionMode();
#if defined(TEST_MODE) || defined(NO_RUN) || defined(BATTERY_SIM)
#else
                // If timer has updated, you can start looking at the next
                // control loop run through. Only update algorithm if the
                // converter is active
                if (controlLoopTimer.Instance->SR == 0x01) {
                    mppt_hardware.set_debug_dac_voltage(3.0);

                    // Clear the flag
                    controlLoopTimer.Instance->SR &= ~0x00000001;
                    algorithm::algorithmOutput algorithm_action =
                        algorithm::step_algorithm(&mppt_hardware);

                    if (algorithm_action ==
                        algorithm::algorithmOutput::decreaseInputVoltage) {
                        mppt_hardware.incrementDutyCycle(0.005f);
                    } else if (algorithm_action == algorithm::algorithmOutput::
                                                       increaseInputVoltage) {
                        mppt_hardware.incrementDutyCycle(-0.005f);
                    } else {
                        // Do nothing
                    }
                    // Re-enable the timer to come back and
                    __HAL_TIM_ENABLE(&controlLoopTimer);
                }
#endif
                break;
            default:
                // UKNOWN STATE SHUTDOWN
                disable_converter();
                cur_state = converter_state::killed;
                break;
                /** END: Action on State Transitions   **/
        }
    }
}

void Application::init() {
    heartBeat.init();
    mppt_hardware.init();
    HAL_Delay(10);  // Wait 10 ms for voltages to stabalize
    safety::initialize_comparator_limits(&mppt_hardware);
    outputVoltageLED.init();
    outputCurrentLED.init();
    inputVoltageLED.init();
    inputCurrentLED.init();
    tempErrorLED.init();
    skylab2.init();
    startSysTimer();
}

void Application::disable_converter() {
    controlLoopTimer.Instance->SR &= ~0x00000001;
    __HAL_TIM_DISABLE(&controlLoopTimer);
    mppt_hardware.shut_down();
    cur_state = converter_state::killed;
}

bool Application::enable_converter() {
#if defined(TEST_MODE) || defined(NO_RUN)
    mppt_hardware.output_relay_close();
    HAL_Delay(2000);
#if !defined(NO_RUN)
    mppt_hardware.set_duty(0.667);  // Set an initial duty cycle
    HAL_Delay(20);
    mppt_hardware.enableSynchronous();
#endif
    return true;
#else
    // Attempt to precharge and fail if not
    bool precharge_status = precharge();
    if (!precharge_status) {
        return false;
    }
#if defined(BATTERY_SIM)
    mppt_hardware.set_duty(0.6);
    mppt_hardware.disableSynchronous();
#else
    mppt_hardware.set_duty(0.1);  // Set an initial duty cycle
    controlLoopTimer.Instance->SR &= ~0x00000001;
    __HAL_TIM_ENABLE(&controlLoopTimer);  // Start the control loop timer
#endif

    return precharge_status;
#endif
}

void Application::sendPowerTelemtry() {
    float icur = mppt_hardware.get_average_input_current();
    float ocur = mppt_hardware.get_average_output_current();
    float iv = mppt_hardware.inputVoltage;
    float ov = mppt_hardware.outputVoltage;

    skylab2::can_packet_mppt_measurements meausrements_packet = {
        .input_voltage = static_cast<uint16_t>(iv * 100),
        .input_current = static_cast<uint16_t>(icur * 1000),
        .output_voltage = static_cast<uint16_t>(ov * 100),
        .output_current = static_cast<uint16_t>(ocur * 1000),
    };

    skylab2.send_mppt_measurements(meausrements_packet);

    // TODO: THIS WILL MAX OUT IF POWER OVER 600W!!!
    skylab2::can_packet_mppt_power power_packet = {
        .input_power = static_cast<uint16_t>(iv * icur * 100),
        .output_power = static_cast<uint16_t>(ov * ocur * 100),
        .efficiency = (ov * ocur / (iv * icur) * 100),
    };
    skylab2.send_mppt_power(power_packet);

    skylab2::can_packet_mppt_temp temp_packet = {
        .ic_temp =
            static_cast<uint16_t>(mppt_hardware.get_internal_ic_temp() * 100),
        .heatsink_temp =
            static_cast<uint16_t>(mppt_hardware.get_heatsink_temp() * 100),
    };

    skylab2.send_mppt_temp(temp_packet);

    skylab2::can_packet_mppt_status status_packet = {0};
    status_packet.mppt_faults.input_voltage_fault =
        static_cast<uint8_t>(curFault.input_voltage_fault ? 1 : 0),
    status_packet.mppt_faults.input_current_fault =
        static_cast<uint8_t>(curFault.input_current_fault ? 1 : 0),
    status_packet.mppt_faults.output_voltage_fault =
        static_cast<uint8_t>(curFault.output_voltage_fault ? 1 : 0),
    status_packet.mppt_faults.output_current_fault =
        static_cast<uint8_t>(curFault.output_current_fault ? 1 : 0),
    status_packet.fault_value = curFault.fault_value,
    skylab2.send_mppt_status(status_packet);

    skylab2::can_packet_mppt_debug debug_packet = {0};
    debug_packet.duty = mppt_hardware.get_duty();
    skylab2.send_mppt_debug(debug_packet);
}

bool Application::precharge() {
    uint16_t precharge_timeout = 0;

    // Start precharging!!
    mppt_hardware.precharge_relay_close();

    do {
        HAL_Delay(25);
        precharge_timeout++;
        if (cur_state == converter_state::killed) {
            return false;
        }
        if (!safety::check_converter_safe(&mppt_hardware, curFault)) {
            mppt_hardware.precharge_relay_open();
            return false;
        }
        // If startup takes longer than 4 seconds stop precharging
        if (precharge_timeout > 80) {
            mppt_hardware.precharge_relay_open();
            return false;
        }
    } while (
        std::abs(mppt_hardware.get_bus_voltage() -
                     mppt_hardware.get_output_voltage() >
                 Hardware::maxPrechargeDiff) ||
        (mppt_hardware.get_output_voltage() < Hardware::minOutputVoltage) ||
        // Check the voltages at least 10 times
        (precharge_timeout < 10));

    // Close the output relay now that you are done
    mppt_hardware.output_relay_close();
    HAL_Delay(50);  // Wait for the relay to actually close
    // Stop precharging by opening the precharge relay
    mppt_hardware.precharge_relay_open();
    return true;
}

void Application::startSysTimer() {
    /* For 160Mhz this will have the clock run at 10 Hrtz */
    __HAL_RCC_TIM7_CLK_ENABLE();
    sysTimerHandle.Instance = sysTimerInstance;
    sysTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sysTimerHandle.Init.Prescaler = 39999;
    sysTimerHandle.Init.Period = 400;
    sysTimerHandle.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    sysTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&sysTimerHandle) != HAL_OK) {
        while (1)
            ;
    }

    /* This will have the clock run at 1Hz*/
    __HAL_RCC_TIM6_CLK_ENABLE();
    controlLoopTimer.Instance = controlTimerInstance;
    controlLoopTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    controlLoopTimer.Init.Prescaler = 39999;
    controlLoopTimer.Init.Period = 100;  // 62.6 ms for 250
    controlLoopTimer.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    controlLoopTimer.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&controlLoopTimer) != HAL_OK) {
        while (1)
            ;
    }

    // Set on pulse mode
    controlLoopTimer.Instance->CR1 |= TIM_CR1_OPM;

    HAL_NVIC_SetPriority(TIM7_DAC_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(TIM7_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&sysTimerHandle);
    // ENABLE this timer in inerupt mode without enabling the irq, that
    // way we get the flag but not stop interuption
    __HAL_TIM_ENABLE_IT(&sysTimerHandle, TIM_IT_UPDATE);
}

/**
 * @brief Turns on an error LED if the coresponding error has occured.
 *  Will NOT currently turn the LED off if the device exists the fault.
 */
void Application::updateErrorLEDs() {
    if (curFault.input_current_fault) {
        inputCurrentLED.on();
    }
    if (curFault.input_voltage_fault) {
        inputVoltageLED.on();
    }
    if (curFault.output_current_fault) {
        outputCurrentLED.on();
    }
    if (curFault.output_voltage_fault) {
        outputVoltageLED.on();
    }
}

void Application::sysTimerHandler() {
    mppt_hardware.set_debug_dac_voltage(3.0);
    sendPowerTelemtry();
    heartBeat.toggle();
    mppt_hardware.set_debug_dac_voltage(0);
}

FDCAN_HandleTypeDef *Application::getFdcanHandle() {
    return can_driver.get_handle();
}

TIM_HandleTypeDef *Application::getSysTimHandle() {
    return &sysTimerHandle;
}

void Application::fdcanRxHandler() {
    mppt_hardware.set_debug_dac_voltage(3.0);
    skylab2.main_bus_rx_handler();
    mppt_hardware.set_debug_dac_voltage(0);
}

void Application::fdcanTxHandler() {
    mppt_hardware.set_debug_dac_voltage(3.0);
    skylab2.main_bus_tx_handler();
    mppt_hardware.set_debug_dac_voltage(0);
}

}  // namespace umnsvp::mppt