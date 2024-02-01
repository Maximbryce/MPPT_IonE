#pragma once

#include "stm32g4xx.h"

namespace umnsvp {
namespace mppt {

enum class gate_driver_frequency : uint16_t
{
    kHz_20 = 8000,
    kHz_30 = 5333,
    kHz_40 = 4000  // Supposed to be 937.5, I'm slightly high
};

enum class driver_mode : uint8_t
{
    synchronous = 0x00,
    asynchronous = 0x01,
    off = 0x02
};

constexpr float blankingWindowPercentage = 0.05;

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

class GateDriver {
   private:
    TIM_HandleTypeDef timer;
    float duty;
    gate_driver_frequency frequency;
    driver_mode cur_mode;
    uint16_t deadtime;
    void initGpios();
    void start_utility_timers();

   public:
    GateDriver(gate_driver_frequency freq);
    void disableSynchronousMode();
    void enableSynchronousMode();
    void set_duty(float duty);
    float get_duty() const;
    float get_freq() const;
    driver_mode get_mode() const;

    void init();

    bool start();

    void stop();
};
}  // namespace mppt
}  // namespace umnsvp
