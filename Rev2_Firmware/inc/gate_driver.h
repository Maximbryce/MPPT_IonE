#pragma once

#include "stm32g4xx.h"

namespace umnsvp {
namespace mppt {

enum class driver_mode : uint8_t
{
    synchronous = 0x00,
    asynchronous = 0x01,
    off = 0x02
};

constexpr float blankingWindowPercentage = 0.1;

// The scalar for these is tds and is 160mHz/4 = 40Mhz or 25ns
enum class gate_driver_deadtime : uint8_t
{
    ns_50 = 0b0'0000010,    // Scalar of 2
    ns_125 = 0b0'0000101,   // Scalar of 5
    ns_250 = 0b0'0001010,   // Scalar of 10
    ns_500 = 0b0'0010100,   // Scalar of 20
    ns_1000 = 0b0'0101000,  // Scalar of 40
    ns_2000 = 0b0'1010000,  // Scalar of 80
};

constexpr float min_duty = 0.05;  // 1:1 conversion almost
constexpr float max_duty = 0.9;   // 1:7 conversion roughly

constexpr float FSW = 80e+3;
constexpr double DEADTIME = 20e-9;

class GateDriver {
   private:
    TIM_HandleTypeDef timer;
    float duty;
    float frequency;
    float deadtime;
    uint32_t deadtime_pulse_length;
    driver_mode cur_mode;
    void initGpios();
    void start_utility_timers();

   public:
    GateDriver(float freq, float deadtime);
    void set_duty(float duty, float aux_pulse_length);
    float get_duty() const;
    float get_freq() const;
    void clear_rearm_break_fault();
    driver_mode get_mode() const;

    void init();

    bool start();

    void stop();
};
}  // namespace mppt
}  // namespace umnsvp
