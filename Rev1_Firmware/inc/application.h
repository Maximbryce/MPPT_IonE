#pragma once

#include "fdcan.h"
#include "hardware.h"
#include "safety.h"
#include "skylab2_boards.h"
#include "status_led.h"
#include "stm32g4xx.h"

namespace umnsvp {
namespace mppt {

enum class converter_state : uint8_t
{
    idle,
    starting_up,
    active,
    killed,
};

class Application {
   public:
    Application();

    void main();

    void disable_converter();
    void updateErrorLEDs();

    bool enable_converter();

    void sysTimerHandler();

    void fdcanRxHandler();

    void fdcanTxHandler();

    FDCAN_HandleTypeDef *getFdcanHandle();

    TIM_HandleTypeDef *getSysTimHandle();

    // Fault info if the software has hit a fault
    safety::fault_info curFault;

   private:
    // Algorithm duty step amount. This is the change in duty cycle for each run
    // of the algorithm
    static constexpr float algorithmDutyStep = 0.01;

    // This CAN pinout is for the MPPT, adjust for other boards as needed
    static constexpr uint16_t can_rx_pin = GPIO_PIN_0;
    static constexpr uint16_t can_tx_pin = GPIO_PIN_1;
    GPIO_TypeDef *const can_port = GPIOD;

    TIM_TypeDef *const sysTimerInstance = TIM7;
    TIM_TypeDef *const controlTimerInstance = TIM6;

    Hardware mppt_hardware;
    StatusLED heartBeat;
    StatusLED outputVoltageLED;
    StatusLED outputCurrentLED;  //(INDICATOR2_PORT, INDICATOR2_PIN);
    StatusLED inputVoltageLED;   //(INDICATOR3_PORT, INDICATOR3_PIN);
    StatusLED inputCurrentLED;   //(INDICATOR4_PORT, INDICATOR4_PIN);
    StatusLED tempErrorLED;      // (INDICATOR5_PORT, INDICATOR5_PIN);

    // CAN things
    can::fdcan_driver can_driver;
    skylab2::mppt_can skylab2;

    TIM_HandleTypeDef sysTimerHandle = {0};
    TIM_HandleTypeDef controlLoopTimer = {0};

    converter_state cur_state;
    skylab2::can_packet_mppt_enable control_packet = {
        .enable = true};  // Default to on

    uint8_t startup_errors = 0;

    bool precharge();
    void startSysTimer();

    void sendPowerTelemtry();

    void init();
};  // namespace mppt

}  // namespace mppt
}  // namespace umnsvp