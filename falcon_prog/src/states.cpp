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
	machine->is_running_waypoint_ = true;	//start listening to movebase feedback
	machine->goal_reached_ = false;
}
void RunState::stateUpdate(){
	if (machine->is_running_waypoint_){
		if (machine->goal_reached_){
			if (machine->move_goals_.empty())
				return setState(new EndState(machine));
			//run and dequeue
			Position goal = machine->move_goals_.front();
			machine->move_goals_.pop();
			ROS_INFO("Goal set. Remainding: %d points", machine->move_goals_.size());
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

EndState::EndState(StateMachine *machine) : State(machine){}
void EndState::stateUpdate() {}
void EndState::onInput(uint8_t input){}
