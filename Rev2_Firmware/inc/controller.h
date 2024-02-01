#pragma once

#include "stm32g4xx.h"

namespace umnsvp {
namespace mppt {

class PIController {
   private:
    float wz = 0;
    float kpi = 0;
    float Ts = 0;

    float c1 = 0;
    float c2 = 0;

    float prev_error;
    float prev_output;

    float reference;

   public:
    PIController(float wz, float kpi, float Ts);
    float run(float input);
    void set_reference(float ref);
    float get_current_reference(void);
};

}  // namespace mppt
}  // namespace umnsvp
