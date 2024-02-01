#include "adc.h"

#include "pin_defines.h"

namespace umnsvp::mppt {

void ADCSensing::init() {
    init_pins();
    init_dma();
    init_adc();
    init_channels();
    calibrateADC();
}

// TODO: Check Calibration, seems a bit off
void ADCSensing::calibrateADC() {
    if (HAL_ADCEx_Calibration_Start(&batteryCurrentADCHandle,
                                    ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADCEx_Calibration_Start(&batteryVoltageADCHandle,
                                    ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADCEx_Calibration_Start(&arrayVoltageADCHandle, ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADCEx_Calibration_Start(&arrayCurrentADCHandle, ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADCEx_Calibration_Start(&miscADCHandle, ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
}

void ADCSensing::init_adc() {
    // Set the clock sources for the ADC's
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while (1)
            ;
    }

    // Enable the clocks for the ADC's
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC345_CLK_ENABLE();

    ADC_InitTypeDef adc_peripheral_init = {
        // SYSCLK is the source,
        // NOTE: This clock config is only required to be done once and can only
        // be
        // changed if no ADC is running It is universal for all of the ADC's

        .ClockPrescaler =
            ADC_CLOCK_SYNC_PCLK_DIV4,      // ADC clock runs at ~26 Mhz
        .Resolution = ADC_RESOLUTION_12B,  // 12 bit data
        .DataAlign = ADC_DATAALIGN_RIGHT,
        .GainCompensation = 0,
        .ScanConvMode = ADC_SCAN_ENABLE,
        .EOCSelection = ADC_EOC_SEQ_CONV,
        .LowPowerAutoWait = DISABLE,
        .ContinuousConvMode = DISABLE,
        .DiscontinuousConvMode = DISABLE,
        .ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO2,
        .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING,
        .SamplingMode = ADC_SAMPLING_MODE_NORMAL,
        .DMAContinuousRequests = ENABLE,
        .Overrun = ADC_OVR_DATA_OVERWRITTEN,  // ADC_OVR_DATA_PRESERVED
        .OversamplingMode = DISABLE,
    };

    // Initialize ADC 1 used for battery current sense and voltage sense
    batteryCurrentADCHandle.Init =
        adc_peripheral_init;  // copy the intialization
    batteryCurrentADCHandle.Init.NbrOfConversion = 1;
    batteryCurrentADCHandle.Instance = BATTERY_Current_ADC;
    if (HAL_ADC_Init(&batteryCurrentADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    // Initialize ADC 1 used for battery Voltage sense and voltage sense
    batteryVoltageADCHandle.Init =
        adc_peripheral_init;  // copy the intialization
    batteryVoltageADCHandle.Init.NbrOfConversion = 1;
    batteryVoltageADCHandle.Instance = BATTERY_Voltage_ADC;
    if (HAL_ADC_Init(&batteryVoltageADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    // Initialize ADC 2 used for voltage sense on array
    arrayVoltageADCHandle.Init = adc_peripheral_init;
    arrayVoltageADCHandle.Init.NbrOfConversion = 1;  // 1 Channels for this ADC
    arrayVoltageADCHandle.Instance = ARRAY_VOLTAGE_ADC;
    if (HAL_ADC_Init(&arrayVoltageADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    // Initialize ADC4 used for current sense on the array
    arrayCurrentADCHandle.Init = adc_peripheral_init;
    arrayCurrentADCHandle.Init.NbrOfConversion = 1;  // 1 Channels for this ADC
    arrayCurrentADCHandle.Instance = ARRAY_CURRENT_ADC;
    if (HAL_ADC_Init(&arrayCurrentADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    // TODO:: IS THIS AN ISSUE???? The timer for the precharge won't be started
    // bc duty is zero. So will this not read? Initialize ADC5 used for Random
    // Misc Sensing
    miscADCHandle.Init = adc_peripheral_init;
    miscADCHandle.Init.NbrOfConversion = 5;  // 1 Channels for this ADC
    miscADCHandle.Instance = MISC_ADC;
    if (HAL_ADC_Init(&miscADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
}

void ADCSensing::init_channels() {
    // define a default list of options for all of the channels
    ADC_ChannelConfTypeDef adc_channel = {
        // Sampling time based off of table 62 in datasheet and op amp output
        // characteristics
        .SamplingTime =
            ADC_SAMPLETIME_247CYCLES_5,  // ADC_SAMPLETIME_12CYCLES_5
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    battery_voltage_channel = adc_channel;
    battery_voltage_channel.Channel = BATTERY_VOLTAGE_CHANNEL;
    battery_voltage_channel.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&batteryVoltageADCHandle,
                              &battery_voltage_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    battery_current_channel = adc_channel;
    battery_current_channel.Channel = BATTERY_CURRENT_CHANNEL;
    battery_current_channel.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&batteryCurrentADCHandle,
                              &battery_current_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    array_voltage_channel = adc_channel;
    array_voltage_channel.Channel = ARRAY_VOLTAGE_CHANNEL;
    array_voltage_channel.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&arrayVoltageADCHandle, &array_voltage_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    array_current_channel = adc_channel;
    array_current_channel.Channel = ARRAY_CURRENT_CHANNEL;
    array_current_channel.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&arrayCurrentADCHandle, &array_current_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    temp_channel = adc_channel;
    temp_channel.SamplingTime =
        ADC_SAMPLETIME_640CYCLES_5;  // This sampling time is plenty fast for
                                     // the data
    temp_channel.Channel = TEMP_CHANNEL;
    temp_channel.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &temp_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    precharge_channel = adc_channel;
    precharge_channel.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    precharge_channel.Channel = PRECHARGE_CHANNEL;
    precharge_channel.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &precharge_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    array_hal_ref_channel = adc_channel;
    array_hal_ref_channel.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    array_hal_ref_channel.Channel = ARRAY_HAL_REF_CHANNEL;
    array_hal_ref_channel.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &array_hal_ref_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    battery_hal_ref_channel = adc_channel;
    battery_hal_ref_channel.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    battery_hal_ref_channel.Channel = BAT_HAL_REF_CHANNEL;
    battery_hal_ref_channel.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &battery_hal_ref_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    IC_temp = adc_channel;
    IC_temp.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    IC_temp.Channel = ADC_CHANNEL_TEMPSENSOR_ADC5;
    IC_temp.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &IC_temp) != HAL_OK) {
        while (1)
            ;
    }
}

void ADCSensing::start() {
    // Configure for 2 byte transfers
    // Static cast doesn't seem to allow a cast from the uint16_t* to the
    // uint32_t* required for function call
    if (HAL_ADC_Start_DMA(&batteryCurrentADCHandle,
                          (uint32_t *)(battery_current_measurments.data()),
                          battery_current_measurments.size())) {
        while (1)
            ;
    }
    if (HAL_DMA_Start_IT(&batteryCurrentBuffer_DMA,
                         (uint32_t)&batteryCurrentADCHandle.Instance->DR,
                         (uint32_t)battery_current_measurements_buffered.data(),
                         battery_current_measurements_buffered.size())) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(&batteryVoltageADCHandle,
                          (uint32_t *)(battery_voltage_measurements.data()),
                          battery_voltage_measurements.size())) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(&arrayVoltageADCHandle,
                          (uint32_t *)(array_voltage_measurements.data()),
                          array_voltage_measurements.size())) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(&arrayCurrentADCHandle,
                          (uint32_t *)(array_current_measurements.data()),
                          array_current_measurements.size())) {
        while (1)
            ;
    }
    if (HAL_DMA_Start_IT(&arrayCurrentBuffer_DMA,
                         (uint32_t)&arrayCurrentADCHandle.Instance->DR,
                         (uint32_t)array_current_measurements_buffered.data(),
                         array_current_measurements_buffered.size())) {
        while (1)
            ;
    }
    // Wakeup the internal temperature sensor
    ADC345_COMMON->CCR |= ADC_CCR_VSENSESEL;
    if (HAL_ADC_Start_DMA(&miscADCHandle,
                          (uint32_t *)(misc_measurements.data()),
                          misc_measurements.size())) {
        while (1)
            ;
    }
}

void ADCSensing::init_pins() {
    GPIO_InitTypeDef pin = {
        // Initialize the pins with default modes
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    pin.Pin = BATTERY_VOLTAGE_ADC_PIN;
    HAL_GPIO_Init(BATTERY_VOLTAGE_ADC_PORT, &pin);

    pin.Pin = BATTERY_CURRENT_ADC_PIN;
    HAL_GPIO_Init(BATTERY_CURRENT_ADC_PORT, &pin);

    pin.Pin = ARRAY_VOLTAGE_ADC_PIN;
    HAL_GPIO_Init(ARRAY_VOLTAGE_ADC_PORT, &pin);

    pin.Pin = ARRAY_CURRENT_ADC_PIN;
    HAL_GPIO_Init(ARRAY_CURRENT_ADC_PORT, &pin);

    pin.Pin = TEMP_ADC_PIN;
    HAL_GPIO_Init(TEMP_ADC_PORT, &pin);

    pin.Pin = PRECHARGE_SENSE_PIN;
    HAL_GPIO_Init(PRECHARGE_SENSE_PORT, &pin);

    pin.Pin = ARRAY_HAL_REF_PIN;
    HAL_GPIO_Init(ARRAY_HAL_REF_PORT, &pin);

    pin.Pin = BAT_HAL_REF_PIN;
    HAL_GPIO_Init(BAT_HAL_REF_PORT, &pin);
}

void ADCSensing::init_dma() {
    // TODO: ADC DMA requests can fail silently (page 647 f reference manual)
    // need to sample the DMA fail bit to make sure it won't do tht
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    DMA_InitTypeDef dma_init = {
        .Direction = DMA_PERIPH_TO_MEMORY,
        .PeriphInc = DMA_PINC_DISABLE,
        .MemInc = DMA_MINC_ENABLE,
        .PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
        .MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
        .Mode = DMA_CIRCULAR,
    };

    batteryCurrentADC_DMA.Init = dma_init;
    batteryCurrentADC_DMA.Instance = DMA1_Channel1;
    batteryCurrentADC_DMA.Init.Priority = DMA_PRIORITY_HIGH;
    batteryCurrentADC_DMA.Init.Request = DMA_REQUEST_ADC1;  // Battery ADC
    if (HAL_DMA_Init(&batteryCurrentADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&batteryCurrentADCHandle, DMA_Handle, batteryCurrentADC_DMA);
    // This DMA will move data from the meory block for battery measurments, to
    // a buffered one
    batteryCurrentBuffer_DMA.Init = dma_init;
    batteryCurrentBuffer_DMA.Instance = DMA1_Channel7;
    batteryCurrentBuffer_DMA.Init.Request =
        DMA_REQUEST_ADC1;  // trigger on new adc measurment from ADC
    batteryCurrentBuffer_DMA.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&batteryCurrentBuffer_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    batteryVoltageADC_DMA.Init = dma_init;
    batteryVoltageADC_DMA.Instance = DMA1_Channel2;
    batteryVoltageADC_DMA.Init.Priority = DMA_PRIORITY_HIGH;
    batteryVoltageADC_DMA.Init.Request = DMA_REQUEST_ADC3;  // Battery ADC
    if (HAL_DMA_Init(&batteryVoltageADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&batteryVoltageADCHandle, DMA_Handle, batteryVoltageADC_DMA);

    arrayVoltageADC_DMA.Init = dma_init;
    arrayVoltageADC_DMA.Instance = DMA1_Channel3;
    arrayVoltageADC_DMA.Init.Priority = DMA_PRIORITY_MEDIUM;
    arrayVoltageADC_DMA.Init.Request = DMA_REQUEST_ADC2;  // Array Voltage
    if (HAL_DMA_Init(&arrayVoltageADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&arrayVoltageADCHandle, DMA_Handle, arrayVoltageADC_DMA);

    arrayCurrentADC_DMA.Init = dma_init;
    arrayCurrentADC_DMA.Instance = DMA1_Channel4;
    arrayCurrentADC_DMA.Init.Request = DMA_REQUEST_ADC4;  // Array Current ADC
    arrayCurrentADC_DMA.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&arrayCurrentADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&arrayCurrentADCHandle, DMA_Handle, arrayCurrentADC_DMA);

    // This DMA will move data from the meory block for array measurments, to a
    // buffered one
    arrayCurrentBuffer_DMA.Init = dma_init;
    arrayCurrentBuffer_DMA.Instance = DMA1_Channel6;
    arrayCurrentBuffer_DMA.Init.Request =
        DMA_REQUEST_ADC4;  // trigger on new adc measurment from ADC
    arrayCurrentBuffer_DMA.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&arrayCurrentBuffer_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    miscADC_DMA.Init = dma_init;
    miscADC_DMA.Instance = DMA1_Channel5;
    miscADC_DMA.Init.Request = DMA_REQUEST_ADC5;  // MISC ADCs
    miscADC_DMA.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&miscADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&miscADCHandle, DMA_Handle, miscADC_DMA);
}

const std::array<uint16_t, ADC1BufferSize>
    &ADCSensing::get_battery_current_measurements() {
    return battery_current_measurments;
}
const std::array<uint16_t, ADC3BufferSize>
    &ADCSensing::get_battery_voltage_measurements() {
    return battery_voltage_measurements;
}
const std::array<uint16_t, ADC2BufferSize>
    &ADCSensing::get_array_voltage_measurements() {
    return array_voltage_measurements;
}
const std::array<uint16_t, ADC4BufferSize>
    &ADCSensing::get_array_current_measurements() {
    return array_current_measurements;
}
const std::array<uint16_t, ADC4ExtendedBufferSize>
    &ADCSensing::get_buffered_array_current_measurements() {
    return array_current_measurements_buffered;
}
const std::array<uint16_t, ADC1ExtendedBufferSize>
    &ADCSensing::get_buffered_battery_current_measurements() {
    return battery_current_measurements_buffered;
}
const std::array<uint16_t, ADC5BufferSize>
    &ADCSensing::get_misc_measurements() {
    return misc_measurements;
}

}  // namespace umnsvp::mppt
