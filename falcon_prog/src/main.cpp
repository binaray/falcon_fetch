#include <falcon_prog/state_machine.h>

State *StateMachine::current_state_ = 0;

StateMachine::StateMachine(){
  bool init_result = init();
  ROS_ASSERT(init_result);
}
StateMachine::~StateMachine(){
	delete current_state_;
	delete stationary_pos_;
}

bool StateMachine::init(){
	ros::NodeHandle pn("~");
	pn.param("x_step", x_step_, float(0.1));
	pn.param("bound_padding", bound_padding_, float(0.1));
	pn.param("bound_padding", bound_padding_, float(0.1));
	pn.param("bound_padding", bound_padding_, float(0.1));
	pn.param("stationary_threshold", stationary_threshold_, float(0.1));
	
	last_updated_timeout_ = ros::Duration(5);
	immobile_timeout_ = ros::Duration(5);
	
	stationary_pos_ = 0;
	
	beacons_pos_subscriber_ = n_.subscribe<marvelmind_nav::beacon_pos_a>("beacons_pos_a", 10, &StateMachine::beaconsPosCallback, this);
	current_pos_subscriber_ = n_.subscribe<marvelmind_nav::hedge_pos>("hedge_pos", 10, &StateMachine::currentPosCallback, this);
	//waypoint_timer_ = n_.createTimer(ros::Duration(0.2), StateMachine::waypointRoutine);
	current_state_ = new StartState(this);
	return true;
}

// void StateMachine::waypointRoutine(const ros::TimerEvent& event){
	// if (is_running_waypoint_ && goal_reached_){
		// //check if feedback idle
		// Position goal = move_goals_.front();
		// //run and dequeue
		// move_goals_.pop_front();
	// }
// }

void StateMachine::update(){
	current_state_->stateUpdate();
}

void StateMachine::generateMoveGoals(){
	// First generate arbitrary rectangle from min/max points
	auto it = beacons_pos_.cbegin();
	it++;
	Position min_point = (*it).second;
	Position max_point = (*it).second;
	for (; it!=beacons_pos_.cend(); it++){
		if ((*it).second.x < min_point.x)
			min_point.x = (*it).second.x;
		if ((*it).second.y < min_point.y)
			max_point.x = (*it).second.x;
		if ((*it).second.x > max_point.x)
			max_point.x = (*it).second.x;
		if ((*it).second.y > max_point.y)
			max_point.x = (*it).second.x;
	}
	min_x_bound_ = min_point.x + bound_padding_;
	min_y_bound_ = min_point.y + bound_padding_;
	max_x_bound_ = max_point.x - bound_padding_;
	max_y_bound_ = max_point.y - bound_padding_;
	
	// Generate zig-zag path from rectangle starting from left to right from bottom to top first
	Position p;
	bool is_going_up = true;
	for (p.x=min_x_bound_; p.x<=max_x_bound_; p.x+=x_step_){
		if (is_going_up){
			p.y = min_y_bound_;
			move_goals_.push(p);
			p.y = max_y_bound_;
			move_goals_.push(p);
		}
		else{
			p.y = max_y_bound_;
			move_goals_.push(p);
			p.y = min_y_bound_;
			move_goals_.push(p);
		}
		is_going_up = !is_going_up;
	}
}

void StateMachine::beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg){
	Position data;
	data.x = msg.x_m;
	data.y = msg.y_m;
	data.last_updated = ros::Time::now();
	
	if (!is_beacons_init_){
		//beacons_pos_.insert (std::pair<int,Position>(msg.address,data));
		beacons_pos_[msg.address] = data;
		if (beacons_pos_.size()>=3){	//3 for testing; 4 for production
			generateMoveGoals();
			is_beacons_init_ = true;
		}
	}
}

void StateMachine::currentPosCallback(const marvelmind_nav::hedge_pos msg){
	current_pos_.x = msg.x_m;
	current_pos_.y = msg.y_m;
	current_pos_.last_updated = ros::Time::now();
	
	if (!stationary_pos_){
		stationary_pos_ = new Position;
		stationary_pos_->x = current_pos_.x;
		stationary_pos_->y = current_pos_.y;
		stationary_pos_->last_updated = current_pos_.last_updated;
	}
	else{
		float d = sqrt(pow((stationary_pos_->x - current_pos_.x),2)+pow((stationary_pos_->y - current_pos_.y),2));
		if (d > stationary_threshold_){
			is_immobile_ = false;
			stationary_pos_->x = current_pos_.x;
			stationary_pos_->y = current_pos_.y;
			stationary_pos_->last_updated = current_pos_.last_updated;			
		}
		else if (current_pos_.last_updated - stationary_pos_->last_updated > immobile_timeout_){
			is_immobile_ = true;
		}
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"falcon_main_node");
	StateMachine falcon;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		falcon.update();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
