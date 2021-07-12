#include <falcon_prog/state_machine.h>

State *StateMachine::current_state_ = 0;

StateMachine::StateMachine(){
  bool init_result = init();
  ROS_ASSERT(init_result);
}
StateMachine::~StateMachine(){delete current_state_;}

bool StateMachine::init(){
	beacons_pos_subscriber_ = n_.subscribe<marvelmind_nav::beacon_pos_a>("beacons_pos_a", 10, &StateMachine::beaconsPosCallback, this);
	current_pos_subscriber_ = n_.subscribe<marvelmind_nav::hedge_pos>("hedge_pos", 10, &StateMachine::currentPosCallback, this);
	return true;
}

void StateMachine::update(){
	current_state_->stateUpdate();
}

void StateMachine::beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg){
	BeaconPos data;
	data.x_ = msg.x_m;
	data.y_ = msg.y_m;
	data.freshness_ = ros::Time::now();
	
  beacons_pos_.insert (std::pair<int,BeaconPos>(msg.address,data));
	//beacons_pos_[msg.address] = data;
	/*
	it_ = beacons_pos_.find(msg.address);
	if (it != beacons_pos_.end())
		beacons_pos_.erase (it);*/
}

void StateMachine::currentPosCallback(const marvelmind_nav::hedge_pos msg){
	current_pos_.x_ = msg.x_m;
	current_pos_.y_ = msg.y_m;
	current_pos_.freshness_ = ros::Time::now();
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
