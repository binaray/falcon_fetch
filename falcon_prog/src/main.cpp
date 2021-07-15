#include <falcon_prog/state_machine.h>

#define SCALE_HEDGE 3.0

State *StateMachine::current_state_ = 0;

float quaternionToRadian(Quaternion q){
	float siny_cosp = 2 * (q.w * q.z);
    float cosy_cosp = 1 - 2 * (q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

float distanceBetweenPoints(Position a, Position b){
	return sqrt(pow((a.x - b.x),2)+pow((a.y - b.y),2));
}

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
	pn.param("stationary_beacon_count", stationary_beacon_count_, int(3));	//min beacons to start program
	pn.param("x_step", x_step_, float(0.1));
	pn.param("bound_padding", bound_padding_, float(0.1));
	pn.param("stationary_threshold", stationary_threshold_, float(0.1));
	pn.param("rotation_threshold", stationary_threshold_, float(0.05));
	pn.param("distance_threshold", distance_threshold_, float(0.02));
	pn.param("differential_movement_threshold", differential_movement_threshold_, float(0.1));
	
	last_updated_timeout_ = ros::Duration(5);
	immobile_timeout_ = ros::Duration(5);
	marker_frame_ = "my_frame";
	
	stationary_pos_ = 0;
	
	beacons_pos_subscriber_ = n_.subscribe<marvelmind_nav::beacon_pos_a>("beacons_pos_a", 10, &StateMachine::beaconsPosCallback, this);
	current_pos_subscriber_ = n_.subscribe<marvelmind_nav::hedge_imu_fusion>("hedge_imu_fusion", 10, &StateMachine::currentPosCallback, this);
	rviz_marker_publisher_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);	// Declare publisher for rviz visualization
	cmd_velocity_publisher_  = n_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	current_state_ = new StartState(this);
	return true;
}

void StateMachine::update(){
	current_state_->stateUpdate();
}

void StateMachine::showRvizMoveGoals(){
	if (rviz_marker_publisher_.getNumSubscribers() < 1) return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = marker_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CUBE;
	
	// first delete all existing markers
	marker.action = visualization_msgs::Marker::DELETEALL;
	rviz_marker_publisher_.publish(marker);
	
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05*SCALE_HEDGE;
    marker.scale.y = 0.05*SCALE_HEDGE;
    marker.scale.z = 0.02*SCALE_HEDGE;
    // Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0); //-forever- 
	
	for (int i=0; i<move_goals_.size(); i++){
		marker.id = i;	// index used as marker address
	    marker.pose.position.x = move_goals_[i].x;
		marker.pose.position.y = move_goals_[i].y;
		marker.pose.position.z = 0;
		rviz_marker_publisher_.publish(marker);
	}
}

void StateMachine::updateRvizMoveGoal(int address, int status){
	if (rviz_marker_publisher_.getNumSubscribers() < 1) return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = marker_frame_;
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05*SCALE_HEDGE;
    marker.scale.y = 0.05*SCALE_HEDGE;
    marker.scale.z = 0.02*SCALE_HEDGE;
    // Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.5f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0); //-forever- 
	
	marker.id = address;
	marker.pose.position.x = move_goals_[address].x;
	marker.pose.position.y = move_goals_[address].y;
	marker.pose.position.z = 0;
	rviz_marker_publisher_.publish(marker);
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
			move_goals_.push_back(p);
			p.y = max_y_bound_;
			move_goals_.push_back(p);
		}
		else{
			p.y = max_y_bound_;
			move_goals_.push_back(p);
			p.y = min_y_bound_;
			move_goals_.push_back(p);
		}
		is_going_up = !is_going_up;
	}
	showRvizMoveGoals();
}

bool StateMachine::publishNextMoveGoal(){
	current_goal_index_++;
	if (current_goal_index_ >= move_goals_.size()){
		ROS_INFO("All goals reached");
		return false;
	}
	goal_reached_ = false;
	ROS_INFO("Goal set. Remainding: %d points", move_goals_.size() - current_goal_index_);
	return true;
}

float StateMachine::angleDifferenceToPoint(Position p){
	p.x -= current_pos_.x;
	p.y -= current_pos_.y;
	
	float angle = atan2(p.x, p.y);
	ROS_INFO("Relative angle from current position: %f", angle);
	return current_rad_ + angle;
}

void StateMachine::moveTowardsGoal(){
	geometry_msgs::Twist cmd_vel_msg;	
	
	if (distanceBetweenPoints(current_pos_, move_goals_[current_goal_index_]) > distance_threshold_){		
		//check robot direction with goal
		float angle = angleDifferenceToPoint(move_goals_[current_goal_index_]);
		bool is_differential_movement;
		
		//rotate towards goal
		if (angle > rotation_threshold_){
			if (angle > differential_movement_threshold_){
				is_differential_movement = false;
				cmd_vel_msg.angular.z = 0.5;
			}
			else{
				is_differential_movement = true;
				cmd_vel_msg.angular.z = 0.1;
			}
		}
		else if (angle < -rotation_threshold_){
			if (angle < -differential_movement_threshold_){
				is_differential_movement = false;
				cmd_vel_msg.angular.z = -0.5;
			}
			else{
				is_differential_movement = true;
				cmd_vel_msg.angular.z = -0.1;
			}
		}
		else is_differential_movement = true;
		
		// linear movement is only present when angle difference is small enough
		if (is_differential_movement) cmd_vel_msg.linear.x = 0.5;
	}
	else{
		ROS_INFO("Goal reached.");
		goal_reached_ = true;
	}
		
	cmd_velocity_publisher_.publish(cmd_vel_msg);
}

void StateMachine::beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg){
	Position data;
	data.x = msg.x_m;
	data.y = msg.y_m;
	data.last_updated = ros::Time::now();
	
	if (!is_beacons_init_){
		//beacons_pos_.insert (std::pair<int,Position>(msg.address,data));
		beacons_pos_[msg.address] = data;
		if (beacons_pos_.size()>=stationary_beacon_count_){
			generateMoveGoals();
			is_beacons_init_ = true;
		}
	}
}

void StateMachine::currentPosCallback(const marvelmind_nav::hedge_imu_fusion msg){
	current_pos_.x = msg.x_m;
	current_pos_.y = msg.y_m;
	current_pos_.last_updated = ros::Time::now();
	
	current_orientation_.w = msg.qw;
	current_orientation_.x = msg.qx;
	current_orientation_.y = msg.qy;
	current_orientation_.z = msg.qz;
	current_rad_ = quaternionToRadian(current_orientation_);
	
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
