#pragma once
#include "stdint.h"

namespace umnsvp {

class State {
   public:
    State(void (*on_entry)(State* prev_state), State* (*state_function)(void),
          void (*on_exit)(State* next_state), uint32_t id);
    void (*on_entry)(State* prev_state);
    State* (*state_function)(void);
    void (*on_exit)(State* next_state);
    uint32_t id;
};

bool operator==(const State& lhs, const State& rhs);

class Machine {
   public:
    Machine(State* a_state)
        : cur_machine_state(a_state), prev_machine_state(a_state){};
    void input_sample(void);  // Function to run before every state transition
                              // to sample inputs
    void step_machine(void);
    State* get_current_state();

   private:
    State* cur_machine_state;
    State* prev_machine_state;
};

}  // namespace umnsvp