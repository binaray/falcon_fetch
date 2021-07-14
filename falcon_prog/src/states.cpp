#include <falcon_prog/states.h>

State::State(StateMachine *machine){ this->machine = machine; }
void State::setState( State *state){
	State *prevState = machine->current_state_;
	machine->current_state_ = state;
	delete prevState;
}
void State::stateUpdate() { ROS_INFO("Unimplemented run method"); }
void State::onInput(uint8_t input){ }
State::~State(){}


StartState::StartState(StateMachine *machine) : State(machine){}
void StartState::stateUpdate() { 
	if (machine->is_beacons_init_){
		setState(new RunState(machine));
	}
}
void StartState::onInput(uint8_t input){}

RunState::RunState(StateMachine *machine) : State(machine){
	ROS_INFO("Running navigation for %d points", machine->move_goals_.size());
	machine->is_running_waypoint_ = true;	//start listening to movebase feedback
}
void RunState::stateUpdate(){
	if (machine->is_running_waypoint_){
		if (machine->goal_reached_){
			//check if feedback idle
			Position goal = machine->move_goals_.front();
			//run and dequeue
			machine->move_goals_.pop();
		}
		else{
			if (ros::Time::now() - machine->current_pos_.last_updated > machine->last_updated_timeout_){
				ROS_ERROR("Beacon signal lost. Waiting recovery...");
			}
			if (machine->is_immobile_){
				ROS_WARN("Robot is immobile");
			}
		}
	}
}
void RunState::onInput(uint8_t input){}