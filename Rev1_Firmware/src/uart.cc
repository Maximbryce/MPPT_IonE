#include "uart.h"

#include "pin_defines.h"

namespace umnsvp::mppt::uart {

UART_HandleTypeDef debugUart;
DMA_HandleTypeDef debugUartDMA;

std::array<uint16_t, sendBufferSize> TXBuffer = {};

void init(void) {
    debugUart.Instance = USART1;
    debugUart.Init.BaudRate = 115200;
    debugUart.Init.WordLength = UART_WORDLENGTH_8B;
    debugUart.Init.StopBits = UART_STOPBITS_1;
    debugUart.Init.Parity = UART_PARITY_NONE;
    debugUart.Init.Mode = UART_MODE_TX;
    debugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    debugUart.Init.OverSampling = UART_OVERSAMPLING_16;
    debugUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    debugUart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    debugUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&debugUart) != HAL_OK) {
        while (1)
            ;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&debugUart, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        while (1)
            ;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&debugUart, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        while (1)
            ;
    }
    if (HAL_UARTEx_DisableFifoMode(&debugUart) != HAL_OK) {
        while (1)
            ;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while (1)
            ;
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PE0     ------> USART1_TX
    PE1     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    debugUartDMA.Instance = DMA2_Channel1;
    debugUartDMA.Init.Request = DMA_REQUEST_USART1_TX;
    debugUartDMA.Init.Direction = DMA_MEMORY_TO_PERIPH;
    debugUartDMA.Init.PeriphInc = DMA_PINC_DISABLE;
    debugUartDMA.Init.MemInc = DMA_MINC_ENABLE;
    debugUartDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    debugUartDMA.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    debugUartDMA.Init.Mode = DMA_NORMAL;
    debugUartDMA.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&debugUartDMA) != HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&debugUart, hdmatx, debugUartDMA);
}

}  // namespace umnsvp::mppt::uart