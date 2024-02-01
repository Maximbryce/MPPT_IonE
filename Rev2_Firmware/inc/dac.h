#pragma once

#include "stm32g4xx.h"

namespace umnsvp::mppt {

class InternalDac {
   private:
    DAC_HandleTypeDef handle;
    DAC_ChannelConfTypeDef ch1;
    DAC_ChannelConfTypeDef ch2;
    static constexpr float VREFF = 3.3;

   public:
    InternalDac(DAC_TypeDef *instance);

    void init();

    bool init_ch1(bool internal = true);

    bool init_ch2(bool internal = true);

    bool start_ch1();

    bool start_ch2();

    void set_ch1_voltage(float voltage);

    void set_ch2_voltage(float voltage);

    float get_ch1_voltage() const;

    float get_ch2_voltage() const;
};

}  // namespace umnsvp::mppt