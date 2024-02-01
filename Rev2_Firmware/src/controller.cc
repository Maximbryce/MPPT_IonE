#include "controller.h"

namespace umnsvp {
namespace mppt {

// Note: These inputs are all from a continuous time controller params. The
// discretization is done in these functions. Do not discretize before this!
// (Also wz = fz*2*pi)
PIController::PIController(float wz, float kpi, float Ts)
    : wz(wz), kpi(kpi), Ts(Ts) {
    c1 = (Ts * wz / 2) + 1;
    c2 = (Ts * wz / 2) - 1;
}

float PIController::run(float input) {
    float error = reference - input;

    float output = prev_output + kpi / wz * (c1 * error + c2 * prev_error);

    prev_output = output;
    prev_error = error;

    return output;
}

void PIController::set_reference(float ref) {
    reference = ref;
}

float PIController::get_current_reference(void) {
    return reference;
}

}  // namespace mppt
}  // namespace umnsvp