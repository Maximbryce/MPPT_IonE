#include <array>
#pragma once
#include "stm32g4xx.h"

namespace umnsvp::mppt {

/* Battery measurements ADC define with channels */
static ADC_TypeDef* const BATTERY_Current_ADC = ADC1;
constexpr uint32_t BATTERY_CURRENT_CHANNEL = ADC_CHANNEL_1;

static ADC_TypeDef* const BATTERY_Voltage_ADC = ADC3;
constexpr uint32_t BATTERY_VOLTAGE_CHANNEL = ADC_CHANNEL_4;

/* Array Voltage ADC define with channels */
static ADC_TypeDef* const ARRAY_VOLTAGE_ADC = ADC2;
constexpr uint32_t ARRAY_VOLTAGE_CHANNEL = ADC_CHANNEL_3;

/* Array Current ADC define with channels */
static ADC_TypeDef* const ARRAY_CURRENT_ADC = ADC4;
constexpr uint32_t ARRAY_CURRENT_CHANNEL = ADC_CHANNEL_4;

/* Misc measurements ADC define with channels */
static ADC_TypeDef* const MISC_ADC = ADC5;
constexpr uint32_t BAT_HAL_REF_CHANNEL = ADC_CHANNEL_6;
constexpr uint32_t TEMP_CHANNEL = ADC_CHANNEL_14;
constexpr uint32_t PRECHARGE_CHANNEL = ADC_CHANNEL_15;
constexpr uint32_t ARRAY_HAL_REF_CHANNEL = ADC_CHANNEL_16;

/* Reference voltage for the ADC */
constexpr float adcVref = 3.3;

/* This is the size of the result from the ADC, you need to divide results from
 * the ADC by this value then mutliply by Vref to get voltage */
constexpr uint32_t ADC_RESULT_SCALAR = 4096;
constexpr uint32_t ADC_RESULT_SCALAR_BAT_CURRENT = ADC_RESULT_SCALAR * 1;
constexpr uint32_t ADC_RESULT_SCALAR_BAT_VOLTAGE = ADC_RESULT_SCALAR * 1;
constexpr uint32_t ADC_RESULT_SCALAR_ARRAY_CURRENT = ADC_RESULT_SCALAR * 1;
constexpr uint32_t ADC_RESULT_SCALAR_ARRAY_VOLTAGE = ADC_RESULT_SCALAR * 1;
constexpr uint32_t ADC_RESULT_SCALAR_MISC = ADC_RESULT_SCALAR * 1;

/* Sizes of the result buffers for the ADC's */
/* For oversampled ADC's (currently 1, 2). The buffer holds approximately 25
 * cycles of data This corresponds to approx 3.2ms to fill buffer at 50Mhz adc
 * clock */
constexpr static uint16_t ADC1BufferSize = 1;
constexpr static uint16_t ADC1ExtendedBufferSize = ADC1BufferSize * 100;
constexpr static uint16_t ADC2BufferSize = 1;
constexpr static uint16_t ADC3BufferSize = 1;
/* This array current is 1000 cycles, and sampling speed is 2Mhz, with 100
 * samples per period That means the 1000 element array is 10 cycles or 0.5ms of
 * data */
constexpr static uint16_t ADC4BufferSize = 1;
constexpr static uint16_t ADC4ExtendedBufferSize = ADC4BufferSize * 500;
constexpr static uint16_t ADC5BufferSize = 5;

class ADCSensing {
   private:
    /* These are the buffers for the various ADC outputs */
    /* These are 32 bit buffers bc technically with oversampling result could be
     * 20 bits */
    std::array<uint16_t, ADC1BufferSize> battery_voltage_measurements = {};
    std::array<uint16_t, ADC3BufferSize> battery_current_measurments = {};
    std::array<uint16_t, ADC2BufferSize> array_voltage_measurements = {};
    std::array<uint16_t, ADC4BufferSize> array_current_measurements = {};
    std::array<uint16_t, ADC5BufferSize> misc_measurements = {};

    std::array<uint16_t, ADC4ExtendedBufferSize>
        array_current_measurements_buffered = {};
    std::array<uint16_t, ADC1ExtendedBufferSize>
        battery_current_measurements_buffered = {};

    /* Voltage and current sensing ADC and DMA structs */
    ADC_HandleTypeDef batteryCurrentADCHandle = {};
    DMA_HandleTypeDef batteryCurrentADC_DMA = {};
    DMA_HandleTypeDef batteryCurrentBuffer_DMA = {};

    ADC_HandleTypeDef batteryVoltageADCHandle = {};
    DMA_HandleTypeDef batteryVoltageADC_DMA = {};

    ADC_HandleTypeDef arrayVoltageADCHandle = {};
    DMA_HandleTypeDef arrayVoltageADC_DMA = {};

    ADC_HandleTypeDef arrayCurrentADCHandle = {};
    DMA_HandleTypeDef arrayCurrentADC_DMA = {};
    DMA_HandleTypeDef arrayCurrentBuffer_DMA = {};

    /* Temp sensing ADC and DMA structs */
    ADC_HandleTypeDef miscADCHandle = {};
    DMA_HandleTypeDef miscADC_DMA = {};

    /* Structs for the various channel configs for the ADC's */
    ADC_ChannelConfTypeDef battery_voltage_channel = {};
    ADC_ChannelConfTypeDef battery_current_channel = {};
    ADC_ChannelConfTypeDef array_voltage_channel = {};
    ADC_ChannelConfTypeDef array_current_channel = {};
    ADC_ChannelConfTypeDef temp_channel = {};
    ADC_ChannelConfTypeDef precharge_channel = {};
    ADC_ChannelConfTypeDef array_hal_ref_channel = {};
    ADC_ChannelConfTypeDef battery_hal_ref_channel = {};
    ADC_ChannelConfTypeDef IC_temp = {};

    void init_pins();
    void init_dma();
    void init_adc();
    void init_channels();
    void calibrateADC();

   public:
    ADCSensing() = default;

    const std::array<uint16_t, ADC1BufferSize>&
    get_battery_current_measurements();

    const std::array<uint16_t, ADC1ExtendedBufferSize>&
    get_buffered_battery_current_measurements();

    const std::array<uint16_t, ADC3BufferSize>&
    get_battery_voltage_measurements();

    const std::array<uint16_t, ADC2BufferSize>&
    get_array_voltage_measurements();

    const std::array<uint16_t, ADC4BufferSize>&
    get_array_current_measurements();

    const std::array<uint16_t, ADC4ExtendedBufferSize>&
    get_buffered_array_current_measurements();

    const std::array<uint16_t, ADC5BufferSize>& get_misc_measurements();

    void start();
    void init();

    DMA_HandleTypeDef* get_dmach1_handle();

    DMA_HandleTypeDef* get_dmach2_handle();

    DMA_HandleTypeDef* get_dmach3_handle();
};
}  // namespace umnsvp::mppt
