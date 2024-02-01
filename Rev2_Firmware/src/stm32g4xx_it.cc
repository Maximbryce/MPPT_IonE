#include "stm32g4xx_it.h"

#include "application.h"
#include "comparator.h"
#include "main.h"
#include "timing.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */

extern umnsvp::mppt::Application app;

extern "C" void NMI_Handler(void) {
    __disable_irq();
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
extern "C" void HardFault_Handler(void) {
    __disable_irq();
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
extern "C" void MemManage_Handler(void) {
    __disable_irq();
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
extern "C" void BusFault_Handler(void) {
    __disable_irq();
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
extern "C" void UsageFault_Handler(void) {
    __disable_irq();
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
extern "C" void SVC_Handler(void) {
}

/**
 * @brief This function handles Debug monitor.
 */
extern "C" void DebugMon_Handler(void) {
}

/**
 * @brief This function handles Pendable request for system service.
 */
extern "C" void PendSV_Handler(void) {
}

/**
 * @brief This function handles System tick timer.
 */
extern "C" void SysTick_Handler(void) {
    HAL_IncTick();
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

// extern "C" void COMP1_2_3_IRQHandler(void) {
//     app.disable_converter();
// }

// extern "C" void COMP7_IRQHandler(void) {
//     __HAL_COMP_COMP7_EXTI_CLEAR_FLAG();
//     app.curFault.input_current_fault = true;
//     app.disable_converter();
// }

// Timer interupt for sys functions
extern "C" void TIM7_DAC_IRQHandler(void) {
    // Clear the flag
    __HAL_TIM_CLEAR_IT(umnsvp::mppt::timing::getSysTimHandle(), TIM_IT_UPDATE);
    app.sysTimerHandler();
}

// Timer interupt for ctrl functions
extern "C" void TIM6_DAC_IRQHandler(void) {
    // Clear the flag
    __HAL_TIM_CLEAR_IT(umnsvp::mppt::timing::getCtrlTimHandle(), TIM_IT_UPDATE);
    app.ctrlTimerHandler();
}

extern "C" void TIM3_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(umnsvp::mppt::timing::getHardwareTimHandle(),
                       TIM_IT_UPDATE);
    app.hardwareTimerHandler();
}

/**
 * @brief Receive interupt for the peripheral.
 *
 */
extern "C" void FDCAN1_IT0_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(app.getFdcanHandle());
}

/**
 * @brief TX interupt for the FDCan peripheral
 */
extern "C" void FDCAN1_IT1_IRQHandler(void) {
    HAL_FDCAN_IRQHandler(app.getFdcanHandle());
}

/**
 * @brief Callback function called when a message is received into FIFO 0.
 *
 * @param hfdcan
 * @param RxFifo0ITs
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan,
                               uint32_t RxFifo0ITs) {
    // Note: In the future we may need to use RxFifo0ITs depending on filter
    // configuration.
    app.fdcanRxHandler();
}

/**
 * @brief Callback function called after a message is transmitted on the CAN
 * bus.
 *
 * @param hfdcan
 * @param BufferIndexes
 */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan) {
    app.fdcanTxHandler();
}

/* USER CODE END 1 */