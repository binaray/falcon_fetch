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


StartState::StartState(StateMachine *machine) : State(machine){
	ROS_INFO("Falcon fetch program starting up");
}
void StartState::stateUpdate() { 
	ROS_INFO_THROTTLE(5, "Waiting for beacons to startup.. Left: %d", machine->stationary_beacon_count_);
	if (machine->is_beacons_init_){
		setState(new RunState(machine));
	}
}
void StartState::onInput(uint8_t input){}

RunState::RunState(StateMachine *machine) : State(machine){
	ROS_INFO("Running navigation for %d points", machine->move_goals_.size());
	machine->current_goal_index_ = -1;	
	//machine->is_running_waypoint_ = true;	//start listening to movebase feedback
	machine->goal_reached_ = false;
}
void RunState::stateUpdate(){
	//if (machine->is_running_waypoint_){
		if (machine->goal_reached_){
			if (!machine->publishNextMoveGoal()) return setState(new EndState(machine));
		}
		else{
			if (ros::Time::now() - machine->current_pos_.last_updated > machine->last_updated_timeout_){
				ROS_ERROR_THROTTLE(1, "Beacon signal lost. Waiting recovery...");
			}
			if (machine->is_immobile_){
				ROS_WARN_THROTTLE(1, "Robot is immobile");
			}
			
			machine->moveTowardsGoal();
		}
	//}
}
void RunState::onInput(uint8_t input){}

EndState::EndState(StateMachine *machine) : State(machine){
	ROS_INFO("Program ended..");
}
void EndState::stateUpdate() {}
void EndState::onInput(uint8_t input){}
