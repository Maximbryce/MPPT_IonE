#pragma once

#include "controller.h"
#include "fdcan.h"
#include "hardware.h"
#include "safety.h"
#include "skylab2_boards.h"
#include "state_machine.h"
#include "status_led.h"
#include "stm32g4xx.h"

namespace umnsvp {
namespace mppt {

enum converter_state
{
    STARTUP = 0,
    PRECHARGE = 1,
    IDLE = 2,
    ACTIVE = 3,
    FAULTED = 4,
    KILLED = 5,
    NUM_CONVERTER_STATES = 6
};

class Application {
   public:
    Application();

    /* Startup */
    void startup_on_entry(State *prev_state);
    State *startup();
    void startup_on_exit(State *next_state);
    /* Precharge */
    void precharge_on_entry(State *prev_state);
    State *precharge();
    void precharge_on_exit(State *next_state);
    /* turn on delays */
    void turn_on_delay_on_entry(State *prev_state);
    State *turn_on_delay();
    void turn_on_delay_on_exit(State *next_state);
    /* Idle */
    void idle_on_entry(State *prev_state);
    State *idle();
    void idle_on_exit(State *next_state);
    /* Active */
    void active_on_entry(State *prev_state);
    State *active();
    void active_on_exit(State *next_state);
    /* Faulted */
    void faulted_on_entry(State *prev_state);
    State *faulted();
    void faulted_on_exit(State *next_state);
    /* Killed */
    void killed_on_entry(State *prev_state);
    State *killed();
    void killed_on_exit(State *next_state);

    void main();
    void sysTimerHandler();
    void ctrlTimerHandler();
    void hardwareTimerHandler();

    void disable_converter();
    void enable_converter();

    void updateErrorLEDs();

    void fdcanRxHandler();

    void fdcanTxHandler();

    FDCAN_HandleTypeDef *getFdcanHandle();

    // Fault info if the software has hit a fault
    safety::fault_info curFault;

   private:
    // Default true bc it needs to work on startup without can signal
    bool converter_enabled = false;

    // Fault flags
    bool input_voltage_fault = false;
    bool input_current_fast_fault = false;
    bool input_current_slow_fault = false;
    bool output_voltage_fault = false;
    bool temperature_fault = false;

    // Counter on Number of sequential faults
    uint32_t fault_counter = 0;

    uint32_t precharge_timer = 0;
    uint32_t clear_fault_counter_timer = 0;
    uint32_t idle_wait_timer = 0;

    static constexpr uint32_t max_fault_retry_count = 1;
    uint32_t idle_min_wait = 0;

    // This CAN pinout is for the MPPT, adjust for other boards as needed
    static constexpr uint16_t can_rx_pin = GPIO_PIN_0;
    static constexpr uint16_t can_tx_pin = GPIO_PIN_1;
    GPIO_TypeDef *const can_port = GPIOD;
    StatusLED heartBeat;

    // CAN things
    can::fdcan_driver can_driver;
    skylab2::mppt_can skylab2;

    // Voltage Controller
    PIController output_voltage_controller;

    converter_state cur_state;

    void sample_debug_ctrl_signals(void);
    void sample_comparator_faults(void);

    void clear_comparator_faults(void);

    /* Timing stuff */
    void task10Hz();
    void task1kHz();

    void static_controller();
    void sendPowerTelemtry();

    void init();
    void init_error_indicator_leds();
};

}  // namespace mppt
}  // namespace umnsvp