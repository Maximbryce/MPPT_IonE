#include "stm32g4xx_it.h"

#include "application.h"
#include "main.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */

extern umnsvp::mppt::Application app;

extern "C" void NMI_Handler(void) {
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
extern "C" void HardFault_Handler(void) {
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
extern "C" void MemManage_Handler(void) {
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
extern "C" void BusFault_Handler(void) {
    app.disable_converter();
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
extern "C" void UsageFault_Handler(void) {
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

//
///* USER CODE BEGIN 1 */
// extern DMA_HandleTypeDef batteryADC_DMA;
// extern "C" void DMA1_Channel1_IRQHandler(void) {
//     HAL_DMA_IRQHandler(app.get_dmach1_handle());
// }
//
// extern DMA_HandleTypeDef arrayVoltageADC_DMA;
// extern "C" void DMA1_Channel2_IRQHandler(void) {
//     HAL_DMA_IRQHandler(app.get_dmach2_handle());
// }
//
// extern DMA_HandleTypeDef arrayCurrentADC_DMA;
// extern "C" void DMA1_Channel3_IRQHandler(void) {
//     HAL_DMA_IRQHandler(app.get_dmach3_handle());
// }
//
// extern "C" void DMAMUX_OVR_IRQHandler(void) {
//     HAL_Delay(1);
// }

extern "C" void COMP1_2_3_IRQHandler(void) {
    if (__HAL_COMP_COMP1_EXTI_GET_FLAG()) {
        __HAL_COMP_COMP1_EXTI_CLEAR_FLAG();
        app.curFault.output_voltage_fault = true;
    } else if (__HAL_COMP_COMP2_EXTI_GET_FLAG()) {
        __HAL_COMP_COMP2_EXTI_CLEAR_FLAG();
        app.curFault.input_voltage_fault = true;
    } else if (__HAL_COMP_COMP3_EXTI_GET_FLAG()) {
        __HAL_COMP_COMP3_EXTI_CLEAR_FLAG();
        app.curFault.output_current_fault = true;
    }

    app.disable_converter();
    app.updateErrorLEDs();
}

extern "C" void COMP7_IRQHandler(void) {
    __HAL_COMP_COMP7_EXTI_CLEAR_FLAG();
    app.curFault.input_current_fault = true;
    app.disable_converter();
    app.updateErrorLEDs();
}

// Timer interupt for sys functions
extern "C" void TIM7_DAC_IRQHandler(void) {
    // Clear the flag
    __HAL_TIM_CLEAR_IT(app.getSysTimHandle(), TIM_IT_UPDATE);
    app.sysTimerHandler();
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