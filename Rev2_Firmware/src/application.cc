#include "application.h"

#include <algorithm>

#include "SEGGER_RTT.h"
#include "controller.h"
#include "mppt.h"
#include "pin_defines.h"
#include "state_machine.h"
#include "timing.h"

#define TEST_MODE
// #define NO_RUN
// #define BATTERY_SIM

// TODO FAULT HANDLE ON WHILE(1)

namespace umnsvp::mppt {
extern State application_states[NUM_CONVERTER_STATES];
extern Machine application_state_machine;

Application::Application()
    : heartBeat(GPIOC, GPIO_PIN_6),
      can_driver(FDCAN1, {can_rx_pin, can_tx_pin, can_port}),
      skylab2(can_driver, can::fifo::FIFO0),
      output_voltage_controller(3.745e+4, 5.0, 1 / (5e+3)) {
}

void Application::main() {
    init();
    timing::startSysTimer();
    timing::startHardwareTimer();
    // Wait a few ms before starting ctrl of converter. If the
    // delay doesn't exist the ctrl might pre-empt hardware setup
    // with PWM causing instability
    HAL_Delay(5);
    timing::startCtrlTimer();
    while (1) {
    }
}

void Application::init() {
    heartBeat.init();
    hardware::hardware_init();
    HAL_Delay(10);  // Wait 10 ms for voltages to stabalize
    safety::initialize_comparator_limits();
    init_error_indicator_leds();
    skylab2.init();
}

void Application::init_error_indicator_leds() {
    GPIO_InitTypeDef gpio;
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(INDICATOR0_PORT, &gpio);
    HAL_GPIO_WritePin(INDICATOR0_PORT, INDICATOR0_PIN, GPIO_PIN_SET);

    gpio.Pin = INDICATOR1_PIN;
    HAL_GPIO_Init(INDICATOR1_PORT, &gpio);
    HAL_GPIO_WritePin(INDICATOR1_PORT, INDICATOR1_PIN, GPIO_PIN_SET);

    gpio.Pin = INDICATOR2_PIN;
    HAL_GPIO_Init(INDICATOR2_PORT, &gpio);
    HAL_GPIO_WritePin(INDICATOR2_PORT, INDICATOR2_PIN, GPIO_PIN_SET);

    gpio.Pin = INDICATOR3_PIN;
    HAL_GPIO_Init(INDICATOR3_PORT, &gpio);
    HAL_GPIO_WritePin(INDICATOR3_PORT, INDICATOR3_PIN, GPIO_PIN_SET);

    gpio.Pin = INDICATOR4_PIN;
    HAL_GPIO_Init(INDICATOR4_PORT, &gpio);
    HAL_GPIO_WritePin(INDICATOR4_PORT, INDICATOR4_PIN, GPIO_PIN_SET);
}

void Application::disable_converter() {
    hardware::disable_gate_driver();
    hardware::set_duty(0);
}

void Application::enable_converter() {
    hardware::enable_gate_driver();
    hardware::set_duty(0.5);
}

void Application::sendPowerTelemtry() {
    float icur = hardware::get_filtered_input_current();
    float iv = hardware::get_input_voltage();
    float ov = hardware::get_output_voltage();

    skylab2::can_packet_mppt_measurements meausrements_packet = {
        .input_voltage = static_cast<uint16_t>(iv * 100),
        .input_current = static_cast<uint16_t>(icur * 1000),
        .output_voltage = static_cast<uint16_t>(ov * 100),
    };

    skylab2.send_mppt_measurements(meausrements_packet);

    skylab2::can_packet_mppt_power power_packet = {
        .input_power = static_cast<uint16_t>(iv * icur * 100),
    };

    skylab2.send_mppt_power(power_packet);

    skylab2::can_packet_mppt_temp temp_packet = {
        .ic_temp =
            static_cast<uint16_t>(hardware::get_internal_ic_temp() * 100),
        .heatsink_temp =
            static_cast<uint16_t>(hardware::get_heatsink_temp() * 100),
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
    debug_packet.duty = hardware::get_duty();
    debug_packet.controller_ref =
        output_voltage_controller.get_current_reference();
    skylab2.send_mppt_debug(debug_packet);
}

State* Application::startup() {
    return &application_states[PRECHARGE];
}

void Application::precharge_on_entry(State* prev_state) {
    // Start precharging!!
    hardware::precharge_relay_close();
    precharge_timer = 0;
}

State* Application::precharge() {
    State* next_state = &application_states[PRECHARGE];

#if defined(TEST_MODE) || defined(NO_RUN)
    // Skip precharge and go to IDLE immedietly
    next_state = &application_states[IDLE];
#else
    precharge_timer++;
    if (!safety::check_converter_safe(curFault)) {
        hardware::precharge_relay_open();
        next_state = &application_states[FAULTED];
    }
    // If startup takes longer than 4 seconds stop precharging
    if (precharge_timer > 4000) {
        hardware::precharge_relay_open();
        next_state = &application_states[KILLED];
    }

    if (std::abs(hardware::get_bus_voltage() - hardware::get_output_voltage() >
                 hardware::maxPrechargeDiff) ||
        (hardware::get_output_voltage() < hardware::minOutputVoltage) ||
        // Check the voltages at least 100 ms
        (precharge_timer < 100)) {
        next_state = &application_states[IDLE];
    }
#endif
    return next_state;
}

void Application::precharge_on_exit(State* next_state) {
    if (*next_state == application_states[IDLE]) {
        hardware::output_relay_close();
    }
    // Stop precharging!!
    hardware::precharge_relay_open();
}

void Application::idle_on_entry(State* prev_state) {
    if (*prev_state == application_states[PRECHARGE]) {
        idle_min_wait = 300;
    }
    disable_converter();
    hardware::clear_comparator_flags();
    hardware::re_arm_pwm_break_input();
}

State* Application::idle() {
    State* next_state = &application_states[IDLE];
    if (idle_wait_timer < idle_min_wait) {
        idle_wait_timer++;
    } else {
        if (converter_enabled == true) {
            next_state = &application_states[ACTIVE];
        }
        idle_min_wait = 0;
        idle_wait_timer = 0;
    }

    return next_state;
}

void Application::active_on_entry(State* prev_state) {
    clear_fault_counter_timer = 0;
#if !defined(NO_RUN)
    enable_converter();
#endif
#if defined(TEST_MODE)
    output_voltage_controller.set_reference(60.0f);
#endif
}

State* Application::active() {
    State* next_state = &application_states[ACTIVE];
    if (!safety::check_converter_safe(curFault)) {
        next_state = &application_states[FAULTED];
    }
    if (input_current_slow_fault || input_current_fast_fault ||
        output_voltage_fault || input_voltage_fault) {
        next_state = &application_states[FAULTED];
    }
    // Clear fault counter after 2 Seconds of continuous running
    if (fault_counter > 0) {
        clear_fault_counter_timer++;
        if (clear_fault_counter_timer > 2000) {
            fault_counter = 0;
        }
    }
    if (converter_enabled == false) {
        next_state = &application_states[IDLE];
    }

    return next_state;
}

void Application::killed_on_entry(State* prev_state) {
    UNUSED(prev_state);
    disable_converter();
    hardware::output_relay_open();
}

State* Application::killed() {
    State* next_state = &application_states[KILLED];
    return next_state;
}

void Application::faulted_on_entry(State* prev_state) {
    // TODO CLEAR FAULTS
    UNUSED(prev_state);
    fault_counter++;
    disable_converter();
}

State* Application::faulted() {
    // Attempt to enter the IDLE state again and retry
    State* next_state = &application_states[IDLE];
    if (fault_counter >= max_fault_retry_count) {
        next_state = &application_states[KILLED];
    }
    return next_state;
}

void Application::faulted_on_exit(State* next_state) {
    UNUSED(next_state);
}

/**
 * @brief Turns on an error LED if the coresponding error has occured.
 *  Will NOT currently turn the LED off if the device exists the fault.
 */
void Application::updateErrorLEDs() {
    if (input_current_fast_fault || input_current_slow_fault) {
        HAL_GPIO_WritePin(INDICATOR1_PORT, INDICATOR1_PIN, GPIO_PIN_RESET);
        curFault.input_current_fault = true;
    } else {
        HAL_GPIO_WritePin(INDICATOR1_PORT, INDICATOR1_PIN, GPIO_PIN_SET);
    }
    if (input_voltage_fault) {
        HAL_GPIO_WritePin(INDICATOR0_PORT, INDICATOR0_PIN, GPIO_PIN_RESET);
        curFault.input_voltage_fault = true;
    } else {
        HAL_GPIO_WritePin(INDICATOR0_PORT, INDICATOR0_PIN, GPIO_PIN_SET);
    }
    if (output_voltage_fault) {
        HAL_GPIO_WritePin(INDICATOR2_PORT, INDICATOR2_PIN, GPIO_PIN_RESET);
        curFault.output_voltage_fault = true;
    } else {
        HAL_GPIO_WritePin(INDICATOR2_PORT, INDICATOR2_PIN, GPIO_PIN_SET);
    }
}

void Application::sample_debug_ctrl_signals(void) {
    if (skylab2.mppt_enable_buffer.pop()) {
        skylab2::can_packet_mppt_enable control_packet =
            skylab2.mppt_enable_buffer.output();  // Update control packet
        converter_enabled = control_packet.enable;
    }

    if (skylab2.mppt_debug_ctrl_output_voltage_buffer.pop()) {
        skylab2::can_packet_mppt_debug_ctrl_output_voltage dbg_control_packet =
            skylab2.mppt_debug_ctrl_output_voltage_buffer
                .output();  // Update control packet
        float new_reference = dbg_control_packet.output_voltage *
                              0.01;  // Multiply by conversion ratio
        if (new_reference < 1.1 * hardware::get_input_voltage()) {
            /* Do nothing if the voltage to be set is invalid*/
        } else {
            output_voltage_controller.set_reference(new_reference);
        }
    }
}

void Application::sample_comparator_faults() {
    input_current_slow_fault =
        hardware::get_input_current_slow_comparator_output();
    output_voltage_fault = hardware::get_output_voltage_comparator_output();
    input_voltage_fault = hardware::get_input_voltage_comparator_output();
    input_current_fast_fault =
        hardware::get_input_current_fast_comparator_output();
}

void Application::task10Hz() {
    hardware::set_debug_dac_voltage(0.75);
    sendPowerTelemtry();
    sample_debug_ctrl_signals();
    heartBeat.toggle();
    updateErrorLEDs();
    hardware::set_debug_dac_voltage(0.0);
}

void Application::task1kHz() {
    sample_comparator_faults();
    application_state_machine.step_machine();
}

void Application::sysTimerHandler() {
    static uint16_t counter = 0;

    hardware::set_debug_dac_voltage(1.0);

    counter++;
    if (counter >= 100) {
        task10Hz();
        counter = 0;
    }
    task1kHz();
    hardware::set_debug_dac_voltage(0);
}

// TODO: Make sure MPPT CAN RUN AT THIS FREQ!!!
void Application::ctrlTimerHandler() {
    hardware::set_debug_dac_voltage(3.0);
    if (*application_state_machine.get_current_state() ==
        application_states[ACTIVE]) {
#if (!defined(TEST_MODE) && !defined(NO_RUN) && !defined(BATTERY_SIM))
        mppt::mppt_controller();
#else
        // static_controller();
#endif
    }
    hardware::set_debug_dac_voltage(0.0);
}

void Application::hardwareTimerHandler() {
    hardware::set_debug_dac_voltage(2.0);
    hardware::convert_core_measurements();
    hardware::convert_misc_measurements();
    hardware::set_debug_dac_voltage(0.0);
}

void Application::static_controller() {
#if defined(TEST_MODE)
    hardware::set_debug_dac_voltage(3.0);
    float new_duty =
        output_voltage_controller.run(hardware::get_output_voltage());
    new_duty = std::clamp(new_duty, 0.1f, 0.8f);
    hardware::set_duty(new_duty);
#else
#endif
}

FDCAN_HandleTypeDef* Application::getFdcanHandle() {
    return can_driver.get_handle();
}

void Application::fdcanRxHandler() {
    skylab2.main_bus_rx_handler();
}

void Application::fdcanTxHandler() {
    skylab2.main_bus_tx_handler();
}

}  // namespace umnsvp::mppt