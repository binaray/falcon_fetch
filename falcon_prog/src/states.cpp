#include <falcon_prog/states.h>

State::State(StateMachine *machine){ this->machine = machine; }
void State::setState( State *state){
	State *prevState = machine->current_state_;
	machine->current_state_ = state;
	delete prevState;
}
void State::stateUpdate() { ROS_INFO("Unimplemented run method"); }
void State::onInput( uint8_t input){ }
State::~State(){}

