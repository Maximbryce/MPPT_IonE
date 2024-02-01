#include "adc.h"

#include "pin_defines.h"

namespace umnsvp::mppt::hardware {

/* PRIVATE DATA DECLARATIONS */

struct adc_results_t adc_power_results[NUM_POWER_ADC_RESULTS]{
    [ARRAY_VOLTAGE_ADC_INDEX] = {.raw_value = 0,
                                 .value = 0,
                                 .conversion = HV_ADC_CONVERSION_RATIO,
                                 .offset = 0},
    [BATTERY_VOLTAGE_ADC_INDEX] = {.raw_value = 0,
                                   .value = 0,
                                   .conversion = HV_ADC_CONVERSION_RATIO,
                                   .offset = 0},
    [SLOW_ARRAY_CURRENT_ADC_INDEX] = {.raw_value = 0,
                                      .value = 0,
                                      .conversion =
                                          CURRENT_ADC_CONVERSION_RATIO,
                                      .offset = CURRENT_ADC_HAL_OFFSET},
    [FAST_ARRAY_CURRENT_ADC_INDEX] = {.raw_value = 0,
                                      .value = 0,
                                      .conversion =
                                          CURRENT_ADC_CONVERSION_RATIO,
                                      .offset = CURRENT_ADC_HAL_OFFSET},
};

struct adc_results_t adc_misc_results[NUM_MISC_ADC_RESULTS]{
    [EXTERNAL_THERMISTOR_ADC_INDEX] =
        {
            .raw_value = 0,
            .value = 0,
            .conversion = LV_ADC_CONVERSION_RATIO,
        },
    [PRECHARGE_VOLTAGE_ADC_INDEX] =
        {
            .raw_value = 0,
            .value = 0,
            .conversion = HV_ADC_CONVERSION_RATIO,
        },
    [IC_TEMP_ADC_INDEX] =
        {
            .raw_value = 0,
            .value = 0,
            .conversion = 1,
        },
};

// Limitation of DMA, needs a continguous array to send data too
std::array<uint16_t, 3> misc_measurements = {0};

/* Voltage and current sensing ADC and DMA structs */
static ADC_HandleTypeDef batteryVoltageADCHandle = {};
static DMA_HandleTypeDef batteryVoltageADC_DMA = {};

static ADC_HandleTypeDef arrayVoltageADCHandle = {};
static DMA_HandleTypeDef arrayVoltageADC_DMA = {};

static ADC_HandleTypeDef slowArrayCurrentADCHandle = {};
static DMA_HandleTypeDef slowArrayCurrentADC_DMA = {};

static ADC_HandleTypeDef fastArrayCurrentADCHandle = {};
static DMA_HandleTypeDef fastArrayCurrentADC_DMA = {};

/* Temp sensing ADC and DMA structs */
static ADC_HandleTypeDef miscADCHandle = {};
static DMA_HandleTypeDef miscADC_DMA = {};

/* PRIVATE FUNCTION DECLARATIONS */
void adc_init_pins();
void adc_init_dma();
void adc_init_peripherals();
void adc_init_channels();
void adc_calibrate();
void adc_start();

void adc_init() {
    adc_init_pins();
    adc_init_dma();
    adc_init_peripherals();
    adc_init_channels();
    adc_calibrate();
    adc_start();
}

// TODO: Check Calibration, seems a bit off
void adc_calibrate() {
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
    if (HAL_ADCEx_Calibration_Start(&fastArrayCurrentADCHandle,
                                    ADC_SINGLE_ENDED) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADCEx_Calibration_Start(&slowArrayCurrentADCHandle,
                                    ADC_SINGLE_ENDED) !=
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

void adc_init_peripherals() {
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
        // NOTE: This clock config is only required to be done once and can
        // only
        // be
        // changed if no ADC is running It is universal for all of the ADC's

        // ADC clock runs at ~53.33/10 Mhz
        .ClockPrescaler = LL_ADC_CLOCK_ASYNC_DIV10,
        .Resolution = ADC_RESOLUTION_12B,  // 12 bit data
        .DataAlign = ADC_DATAALIGN_RIGHT,
        .GainCompensation = 0,
        .ScanConvMode = ADC_SCAN_ENABLE,
        .EOCSelection = ADC_EOC_SEQ_CONV,
        .LowPowerAutoWait = DISABLE,
        .ContinuousConvMode = DISABLE,
        .DiscontinuousConvMode = DISABLE,
        .ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO,
        .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING,
        .SamplingMode = ADC_SAMPLING_MODE_NORMAL,
        .DMAContinuousRequests = ENABLE,
        .Overrun = ADC_OVR_DATA_OVERWRITTEN,
        .OversamplingMode = DISABLE,
    };

    batteryVoltageADCHandle.Init = adc_peripheral_init;
    batteryVoltageADCHandle.Init.NbrOfConversion = 1;
    batteryVoltageADCHandle.Instance = BATTERY_Voltage_ADC;
    if (HAL_ADC_Init(&batteryVoltageADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    arrayVoltageADCHandle.Init = adc_peripheral_init;
    arrayVoltageADCHandle.Init.NbrOfConversion = 1;
    arrayVoltageADCHandle.Instance = ARRAY_VOLTAGE_ADC;
    if (HAL_ADC_Init(&arrayVoltageADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    fastArrayCurrentADCHandle.Init = adc_peripheral_init;
    fastArrayCurrentADCHandle.Init.NbrOfConversion = 1;
    fastArrayCurrentADCHandle.Instance = FAST_ARRAY_CURRENT_ADC;
    if (HAL_ADC_Init(&fastArrayCurrentADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    slowArrayCurrentADCHandle.Init = adc_peripheral_init;
    slowArrayCurrentADCHandle.Init.NbrOfConversion = 1;
    slowArrayCurrentADCHandle.Instance = SLOW_ARRAY_CURRENT_ADC;
    if (HAL_ADC_Init(&slowArrayCurrentADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }

    miscADCHandle.Init = adc_peripheral_init;
    miscADCHandle.Init.NbrOfConversion = 3;
    miscADCHandle.Instance = MISC_ADC;
    if (HAL_ADC_Init(&miscADCHandle) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
}

void adc_init_channels() {
    // define a default list of options for all of the channels
    // Sampling time based off of table 62 in datasheet and op amp output
    // characteristics

    ADC_ChannelConfTypeDef battery_voltage_channel = {
        .Channel = BATTERY_VOLTAGE_CHANNEL,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef array_voltage_channel = {
        .Channel = ARRAY_VOLTAGE_CHANNEL,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef fast_array_current_channel = {
        .Channel = FAST_ARRAY_CURRENT_CHANNEL,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef slow_array_current_channel = {
        .Channel = SLOW_ARRAY_CURRENT_CHANNEL,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef temp_channel = {
        .Channel = TEMP_CHANNEL,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef precharge_channel = {
        .Channel = PRECHARGE_CHANNEL,
        .Rank = ADC_REGULAR_RANK_2,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    ADC_ChannelConfTypeDef IC_temp = {
        .Channel = ADC_CHANNEL_TEMPSENSOR_ADC5,
        .Rank = ADC_REGULAR_RANK_3,
        .SamplingTime = ADC_SAMPLETIME_247CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0,
        .OffsetSign = ADC_OFFSET_SIGN_POSITIVE,
        .OffsetSaturation = ENABLE,
    };

    if (HAL_ADC_ConfigChannel(&batteryVoltageADCHandle,
                              &battery_voltage_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&arrayVoltageADCHandle, &array_voltage_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&fastArrayCurrentADCHandle,
                              &fast_array_current_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&slowArrayCurrentADCHandle,
                              &slow_array_current_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &temp_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &precharge_channel) !=
        HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    if (HAL_ADC_ConfigChannel(&miscADCHandle, &IC_temp) != HAL_OK) {
        while (1)
            ;
    }
}

void adc_start() {
    // Configure for 2 byte transfers
    // Static cast doesn't seem to allow a cast from the uint16_t* to the
    // uint32_t* required for function call
    if (HAL_ADC_Start_DMA(
            &batteryVoltageADCHandle,
            (&adc_power_results[BATTERY_VOLTAGE_ADC_INDEX].raw_value), 1)) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(
            &arrayVoltageADCHandle,
            (&adc_power_results[ARRAY_VOLTAGE_ADC_INDEX].raw_value), 1)) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(
            &slowArrayCurrentADCHandle,
            (&adc_power_results[SLOW_ARRAY_CURRENT_ADC_INDEX].raw_value), 1)) {
        while (1)
            ;
    }
    if (HAL_ADC_Start_DMA(
            &fastArrayCurrentADCHandle,
            (&adc_power_results[FAST_ARRAY_CURRENT_ADC_INDEX].raw_value), 1)) {
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

void adc_init_pins() {
    GPIO_InitTypeDef pin = {
        // Initialize the pins with default modes
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    pin.Pin = BATTERY_VOLTAGE_ADC_PIN;
    HAL_GPIO_Init(BATTERY_VOLTAGE_ADC_PORT, &pin);

    pin.Pin = ARRAY_VOLTAGE_ADC_PIN;
    HAL_GPIO_Init(ARRAY_VOLTAGE_ADC_PORT, &pin);

    pin.Pin = FAST_ARRAY_CURRENT_ADC_PIN;
    HAL_GPIO_Init(FAST_ARRAY_CURRENT_ADC_PORT, &pin);

    pin.Pin = SLOW_ARRAY_CURRENT_ADC_PIN;
    HAL_GPIO_Init(SLOW_ARRAY_CURRENT_ADC_PORT, &pin);

    pin.Pin = TEMP_ADC_PIN;
    HAL_GPIO_Init(TEMP_ADC_PORT, &pin);

    pin.Pin = PRECHARGE_SENSE_PIN;
    HAL_GPIO_Init(PRECHARGE_SENSE_PORT, &pin);
}

void adc_init_dma() {
    // TODO: ADC DMA requests can fail silently (page 647 f reference
    // manual) need to sample the DMA fail bit to make sure it won't do tht
    __HAL_RCC_DMA1_CLK_ENABLE();

    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    DMA_InitTypeDef dma_init = {
        .Direction = DMA_PERIPH_TO_MEMORY,
        .PeriphInc = DMA_PINC_DISABLE,
        .MemInc = DMA_MINC_ENABLE,
        .PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
        .MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
        .Mode = DMA_CIRCULAR,
    };

    slowArrayCurrentADC_DMA.Init = dma_init;
    slowArrayCurrentADC_DMA.Instance = DMA1_Channel1;
    slowArrayCurrentADC_DMA.Init.Priority = DMA_PRIORITY_HIGH;
    slowArrayCurrentADC_DMA.Init.Request = DMA_REQUEST_ADC1;
    if (HAL_DMA_Init(&slowArrayCurrentADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&slowArrayCurrentADCHandle, DMA_Handle,
                  slowArrayCurrentADC_DMA);

    batteryVoltageADC_DMA.Init = dma_init;
    batteryVoltageADC_DMA.Instance = DMA1_Channel2;
    batteryVoltageADC_DMA.Init.Priority = DMA_PRIORITY_HIGH;
    batteryVoltageADC_DMA.Init.Request = DMA_REQUEST_ADC3;
    if (HAL_DMA_Init(&batteryVoltageADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&batteryVoltageADCHandle, DMA_Handle, batteryVoltageADC_DMA);

    arrayVoltageADC_DMA.Init = dma_init;
    arrayVoltageADC_DMA.Instance = DMA1_Channel3;
    arrayVoltageADC_DMA.Init.Priority = DMA_PRIORITY_MEDIUM;
    arrayVoltageADC_DMA.Init.Request = DMA_REQUEST_ADC2;
    if (HAL_DMA_Init(&arrayVoltageADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&arrayVoltageADCHandle, DMA_Handle, arrayVoltageADC_DMA);

    fastArrayCurrentADC_DMA.Init = dma_init;
    fastArrayCurrentADC_DMA.Instance = DMA1_Channel4;
    fastArrayCurrentADC_DMA.Init.Request = DMA_REQUEST_ADC4;
    fastArrayCurrentADC_DMA.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&fastArrayCurrentADC_DMA) != HAL_StatusTypeDef::HAL_OK) {
        while (1)
            ;
    }
    __HAL_LINKDMA(&fastArrayCurrentADCHandle, DMA_Handle,
                  fastArrayCurrentADC_DMA);

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

}  // namespace umnsvp::mppt::hardware
