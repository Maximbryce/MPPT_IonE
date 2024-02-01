#include "dac.h"

namespace umnsvp::mppt {

InternalDac::InternalDac(DAC_TypeDef *instance) : handle{instance} {
}

// Initializes the object and entire peripheral
void InternalDac::init() {
    // Init the clocks
    if (handle.Instance == DAC1) {
        __HAL_RCC_DAC1_CLK_ENABLE();
    } else if (handle.Instance == DAC2) {
        __HAL_RCC_DAC2_CLK_ENABLE();
    } else if (handle.Instance == DAC3) {
        __HAL_RCC_DAC3_CLK_ENABLE();
    } else if (handle.Instance == DAC4) {
        __HAL_RCC_DAC4_CLK_ENABLE();
    } else
        while (1)
            ;
    HAL_DAC_Init(&handle);
    DAC_ChannelConfTypeDef default_ch_options = {
        .DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC,
        .DAC_DMADoubleDataMode = DISABLE,
        .DAC_SignedFormat = DISABLE,
        .DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE,
        .DAC_Trigger = DAC_TRIGGER_NONE,
        .DAC_Trigger2 = DAC_TRIGGER_NONE,
        .DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE,
        .DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL,
        .DAC_UserTrimming = DAC_TRIMMING_FACTORY,
    };
    ch1 = default_ch_options;
    ch2 = default_ch_options;
}

bool InternalDac::init_ch1(bool internal) {
    if (!internal) {
        ch1.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    }
    HAL_DACEx_SelfCalibrate(&handle, &ch1, DAC_CHANNEL_1);
    return (HAL_DAC_ConfigChannel(&handle, &ch1, DAC_CHANNEL_1) ==
            HAL_StatusTypeDef::HAL_OK);
}

bool InternalDac::init_ch2(bool internal) {
    if (!internal) {
        ch2.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    }
    HAL_DACEx_SelfCalibrate(&handle, &ch2, DAC_CHANNEL_2);
    return (HAL_DAC_ConfigChannel(&handle, &ch2, DAC_CHANNEL_2) ==
            HAL_StatusTypeDef::HAL_OK);
}

bool InternalDac::start_ch1() {
    return (HAL_DAC_Start(&handle, DAC_CHANNEL_1) == HAL_StatusTypeDef::HAL_OK);
}

bool InternalDac::start_ch2() {
    return (HAL_DAC_Start(&handle, DAC_CHANNEL_2) == HAL_StatusTypeDef::HAL_OK);
}

void InternalDac::set_ch1_voltage(float voltage) {
    handle.Instance->DHR12R1 =
        static_cast<uint32_t>((voltage * 4096.0) / VREFF);
}

void InternalDac::set_ch2_voltage(float voltage) {
    handle.Instance->DHR12R2 =
        static_cast<uint32_t>((voltage * 4096.0) / VREFF);
}

float InternalDac::get_ch1_voltage() const {
    return (handle.Instance->DOR1 / 4096.0 * VREFF);
}

float InternalDac::get_ch2_voltage() const {
    return (handle.Instance->DOR2 / 4096.0 * VREFF);
}

}  // namespace umnsvp::mppt
