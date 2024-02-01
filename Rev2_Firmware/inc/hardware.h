#pragma once
#include <array>

#include "stm32g4xx.h"

namespace umnsvp::mppt::hardware {
float average(const uint16_t* data, uint16_t size);

static constexpr float maxPrechargeDiff = 2;
static constexpr float minOutputVoltage = 20;

static constexpr double Lr = 10.2e-6;
static constexpr double Cr = 1425e-12;
static constexpr float PI_2 = 3.14159 / 2;

// Defined as: PI_2* sqrt(Lr* Cr)
static constexpr float T1 = 1.8938e-07;

static constexpr float Vout_Lr_min = 2e+6;

void hardware_init();

void start_comparators();

float get_output_voltage();
float get_filtered_input_current();
float get_lightly_filtered_input_current();
float get_input_voltage();

float get_heatsink_temp();
float get_internal_ic_temp();
float get_bus_voltage();

void convert_core_measurements();
void convert_misc_measurements();

float calc_thermistor_temp(float resistance);

// Open and close for main output relay
void output_relay_close();
void output_relay_open();

// open and close for precharge relay
void precharge_relay_close();
void precharge_relay_open();

void set_array_voltage_limit(float limit);
void set_fast_array_current_limit(float limit);
void set_battery_voltage_limit(float limit);
void set_slow_array_current_limit(float limit);

bool get_input_current_fast_comparator_output();
bool get_input_current_slow_comparator_output();
bool get_input_voltage_comparator_output();
bool get_output_voltage_comparator_output();
void clear_comparator_flags();

void set_duty(float duty_cycle);
float get_duty();
void set_debug_dac_voltage(float voltage);
void incrementDutyCycle(float increment);
void enable_gate_driver();
void disable_gate_driver();
void re_arm_pwm_break_input();

}  // namespace umnsvp::mppt::hardware