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


StartState::StartState(StateMachine *machine) : State(machine){ ROS_INFO("Falcon fetch program starting up"); }
void StartState::stateUpdate() { 
	ROS_INFO_THROTTLE(5, "Waiting for beacons to startup.. Left: %d", machine->stationary_beacon_count_-machine->beacons_pos_.size());
	if (machine->is_beacons_init_){
		if (machine->getOrientationEstimate()){ 
			ros::param::set("/is_recording", true);
			setState(new RunState(machine));
		}
		else ROS_ERROR_THROTTLE(5,"Unable to get orientation");
	}
}
void StartState::onInput(uint8_t input){}

RunState::RunState(StateMachine *machine) : State(machine){
	machine->readPointsFromFile();
	ROS_INFO("Running navigation for %d points", machine->move_goals_.size());
	machine->current_goal_index_ = -1;	
	if(!machine->publishNextMoveGoal()) {
		ROS_ERROR("Something went wrong: No points found..");
	}
}
void RunState::stateUpdate(){
	if (machine->goal_reached_){
		if (!machine->publishNextMoveGoal()) return setState(new EndState(machine));
	}
	else{
		if (ros::Time::now() - machine->current_pos_.last_updated > machine->last_updated_timeout_){
			ROS_ERROR_THROTTLE(10, "Beacon signal lost. Waiting recovery...");
		}
		if (machine->is_immobile_){
			ROS_WARN_THROTTLE(10, "Robot is immobile");
		}
		machine->moveTowardsGoal();
	}
}
void RunState::onInput(uint8_t input){}

EndState::EndState(StateMachine *machine) : State(machine){
	ROS_INFO("Program ended..");
}
void EndState::stateUpdate() {}
void EndState::onInput(uint8_t input){}
