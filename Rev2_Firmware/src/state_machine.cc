#include "state_machine.h"

#include <stddef.h>
#include <stdint.h>

namespace umnsvp {

State::State(void (*on_entry)(State* prev_state), State* (*state_function)(),
             void (*on_exit)(State* next_state), uint32_t id)
    : on_entry(on_entry),
      state_function(state_function),
      on_exit(on_exit),
      id(id) {
}

bool operator==(const State& lhs, const State& rhs) {
    return lhs.id == rhs.id;
}

void Machine::step_machine() {
    if (prev_machine_state != cur_machine_state) {
        if (cur_machine_state->on_entry != NULL) {
            cur_machine_state->on_entry(prev_machine_state);
        }
    }

    State* next_state = cur_machine_state->state_function();

    if (next_state != cur_machine_state) {
        if (cur_machine_state->on_exit != NULL) {
            cur_machine_state->on_exit(next_state);
        }
    }

    prev_machine_state = cur_machine_state;
    cur_machine_state = next_state;
}

State* Machine::get_current_state() {
    return cur_machine_state;
}

}  // namespace umnsvp