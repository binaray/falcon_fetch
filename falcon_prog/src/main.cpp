#include <falcon_prog/state_machine.h>

#define SCALE_HEDGE 3.0

State *StateMachine::current_state_ = 0;

float quaternionToRadian(Quaternion q){
	float siny_cosp = 2 * (q.w * q.z);
    float cosy_cosp = 1 - 2 * (q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

float distanceBetweenVectors(Position a, Position b){
	return sqrt(pow((a.x - b.x),2)+pow((a.y - b.y),2));
}

Position mulScalarToVector(Position p, float s){
	p.x*=s;
	p.y*=s;
	return p;
}

Position addConstantToVector(Position p, float c){
	p.x+=c;
	p.y+=c;
	return p;
}

Position toUnitVector(Position p){
	Position zero_p;
	float magnitude = distanceBetweenVectors(p, zero_p);
	p.x = p.x/magnitude;
	p.y = p.y/magnitude;
	return p;
}

Position getVector(Position dest, Position source){
	dest.x-=source.x;
	dest.y-=source.y;
	return dest;
}

Position addVectors(Position a, Position b){
	a.x+=b.y;
	a.y+=b.y;
	return a;
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
	pn.param("max_linear_speed", max_linear_speed_, float(0.1));
	pn.param("max_angular_speed", max_angular_speed_, float(0.1));
	pn.param("rotation_threshold", stationary_threshold_, float(0.05));
	pn.param("distance_threshold", distance_threshold_, float(0.02));
	pn.param("differential_movement_threshold", differential_movement_threshold_, float(0.1));
	pn.param("waypoint_filepath", file_path_, std::string("/waypoints.csv"));
	
	last_updated_timeout_ = ros::Duration(5);
	immobile_timeout_ = ros::Duration(5);
	marker_frame_ = "my_frame";
	
	stationary_pos_ = 0;
	
	beacons_pos_subscriber_ = n_.subscribe<marvelmind_nav::beacon_pos_a>("beacons_pos_a", 10, &StateMachine::beaconsPosCallback, this);
	current_pos_subscriber_ = n_.subscribe<marvelmind_nav::hedge_imu_fusion>("hedge_imu_fusion", 10, &StateMachine::currentPosCallback, this);
	current_yaw_subscriber_ = n_.subscribe<std_msgs::Float64>("robot_yaw", 10, &StateMachine::currentYawCallback, this);
	rviz_marker_publisher_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 100, true);	// Declare publisher for rviz visualization
	cmd_velocity_publisher_  = n_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	read_points_srv_ = n_.advertiseService("waypoints_read", &StateMachine::readPointsFromFile, this);
	delete_points_srv_ = n_.advertiseService("waypoints_delete", &StateMachine::clearPointsInFile, this);
	add_point_srv_ = n_.advertiseService("waypoint_add", &StateMachine::writeCurrentPosToFile, this);
	orientation_estimate_client_ = n_.serviceClient<std_srvs::Trigger>("find_orientation");
	current_state_ = new StartState(this);
	return true;
}

void StateMachine::update(){
	current_state_->stateUpdate();
}

void StateMachine::showRvizPos(Position p, int address, bool is_hedge){
	if (rviz_marker_publisher_.getNumSubscribers() < 1) return;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = marker_frame_;
	marker.header.stamp = ros::Time::now();
	marker.ns = "beacons";
    marker.action = visualization_msgs::Marker::ADD;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05*SCALE_HEDGE;
    marker.scale.y = 0.05*SCALE_HEDGE;
    marker.scale.z = 0.02*SCALE_HEDGE;
    // Set the color -- be sure to set alpha to something non-zero!
  if (is_hedge){
		marker.type = visualization_msgs::Marker::ARROW;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.pose.orientation.z = sin(current_rad_ * 0.5);
		marker.pose.orientation.w = cos(current_rad_ * 0.5);
		marker.lifetime = ros::Duration(5); 
  }
  else{
		marker.type = visualization_msgs::Marker::CUBE;
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.lifetime = ros::Duration(0); //-forever- 
	}
    marker.color.a = 1.0;
	
	marker.id = address;
	marker.pose.position.x = p.x;
	marker.pose.position.y = p.y;
	marker.pose.position.z = 0;
	rviz_marker_publisher_.publish(marker);
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

	/*-- naive rectangle implementation approach --
	// First generate arbitrary rectangle from min/max points
	auto it = beacons_pos_.cbegin();
	it++;
	
	Position min_point = (*it).second;
	Position max_point = (*it).second;
	
	for (; it!=beacons_pos_.cend(); it++){
		if ((*it).second.x < min_point.x)
			min_point.x = (*it).second.x;
		if ((*it).second.y < min_point.y)
			min_point.y = (*it).second.y;
		if ((*it).second.x > max_point.x)
			max_point.x = (*it).second.x;
		if ((*it).second.y > max_point.y)
			max_point.y = (*it).second.y;
	}
	ROS_INFO("Min and max positions (%f,%f) (%f,%f)",min_point.x,min_point.y,max_point.x,max_point.y);
	min_x_bound_ = min_point.x + bound_padding_;
	min_y_bound_ = min_point.y + bound_padding_;
	max_x_bound_ = max_point.x - bound_padding_;
	max_y_bound_ = max_point.y - bound_padding_;
	ROS_INFO("Min and max bounds (%f,%f) (%f,%f)",min_x_bound_,min_y_bound_,max_x_bound_,max_y_bound_);
	
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
	*/
	
	/*-- dynamic rectangle approach --*
	//Construct up horizontal and vertical move vector from bottom-leftmost point
	Position left_most_p = (*it).second;
	Position bottom_most_p = (*it).second;
	
	for (; it!=beacons_pos_.cend(); it++){
		if ((*it).second.x < min_point.x){
			left_most_p.x = (*it).second.x;
			left_most_p.y = (*it).second.y;
		}
		if ((*it).second.y < min_point.y){
			bottom_most_p.x = (*it).second.x;
			bottom_most_p.y = (*it).second.y;
		}
	}
	Position min_point = (bottom_most_p.y<left_most_p.x) ? bottom_most_p : left_most_p;
	
	// x_v(ector) and y_v(ector) are assigned arbitrarily first to shortest and next shortest distance
	// --Note--
	// --x and y here refer to horizontal and vertical movement components here; not the coordinate system of the robot tf
	// --------
	Position x_v, y_v;
	Position max_point;	// opposite point from min_point to determine operation bounds
	float min_d = 0;
	float max_d = 0;
	for (it = beacons_pos_.cbegin(); it!=beacons_pos_.cend(); it++){
		float d = distanceBetweenVectors((*it).second, min_point);
		if (d!=0){
			if (min_d==0 || d<min_d){
				//store previous shortest vector to y_v and new shortest to x_v
				y_v = x_v;
				x_v.x = (*it).second.x-min_point.x;
				x_v.y = (*it).second.y-min_point.y;
			}
			else if (d>max_d) {
				max_d = d;
				max_point = (*it).second;
			}
		}
	}
	// then determine x/y vectors
	if (x_v.x < y_v.x){
		Position temp = x_v;
		x_v = y_v;
		y_v = temp;
	}
	//compute unit vectors
	x_v = toUnitVector(x_v);
	y_v = toUnitVector(y_v);
	
	//transform xy vectors to be perpendicular
	float x_dot_y = x_v.x*y_v.x + x_v.y*y_v.y;
	float angle = acosh(x_dot_y);
	
	/
	ROS_INFO("Min and max positions (%f,%f) (%f,%f)",min_point.x,min_point.y,max_point.x,max_point.y);
	min_x_bound_ = min_point.x + bound_padding_ * x_v.x;
	min_y_bound_ = min_point.y + bound_padding_ * x_v.y;
	max_x_bound_ = max_point.x - bound_padding_ * y_v.x;
	max_y_bound_ = max_point.y - bound_padding_ * y_v.y;
	ROS_INFO("Min and max bounds (%f,%f) (%f,%f)",min_x_bound_,min_y_bound_,max_x_bound_,max_y_bound_);
	
	// Generate zig-zag path from rectangle starting from left to right from bottom to top first
	bool is_going_up = true;
	for (min_point.x=min_x_bound_; min_point.x<=max_x_bound_; min_point.x+=x_step_){
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
	for (int i = 0; i<move_goals_.size(); i++){
		ROS_INFO("Goal[%d]: (%f,%f)",i,move_goals_[i].x,move_goals_[i].y);
	}
	//*/
	
	
	/*-- cross configuration --*/
	auto it = beacons_pos_.cbegin();
	Position left_most_p = (*it).second;
	Position bottom_most_p = (*it).second;
	Position right_most_p = (*it).second;
	Position top_most_p = (*it).second;
		
	it++;
	
	for (; it!=beacons_pos_.cend(); it++){
		ROS_INFO("Comparing x with %f %f",(*it).second.x,left_most_p.x);
		if ((*it).second.x < left_most_p.x){
			ROS_INFO("Updated %f -> %f",(*it).second.x,left_most_p.x);
			left_most_p = (*it).second;
		}
		if ((*it).second.y < bottom_most_p.y){
			bottom_most_p = (*it).second;
		}
		if ((*it).second.x > right_most_p.x){
			right_most_p = (*it).second;
		}
		if ((*it).second.y > top_most_p.y){
			top_most_p = (*it).second;
		}
	}
	
	ROS_INFO("BM(%f,%f) TM(%f,%f) RM(%f,%f) LM(%f,%f)",
	bottom_most_p.x,bottom_most_p.y,
	top_most_p.x,top_most_p.y, 
	right_most_p.x,right_most_p.y,
	left_most_p.x,left_most_p.y);
	
	// Translate points to movebounds
	left_most_p.x += bound_padding_;
	bottom_most_p.y += bound_padding_;
	right_most_p.x -= bound_padding_;
	top_most_p.y -= bound_padding_;
	
	ROS_INFO("Bounded points--\nBM(%f,%f) TM(%f,%f) RM(%f,%f) LM(%f,%f)",
	bottom_most_p.x,bottom_most_p.y,
	top_most_p.x,top_most_p.y, 
	right_most_p.x,right_most_p.y,
	left_most_p.x,left_most_p.y);
	
	/*
	Position y_v = getVector(top_most_p, bottom_most_p);
	Position x_v = getVector(right_most_p, left_most_p);
	ROS_INFO("x_v(%f,%f) y_v(%f,%f)",
	x_v.x,x_v.y,
	y_v.x,y_v.y);
	
	Position half_x_v = x_v;	//to compute start point from bottom_most_p
	half_x_v.x = -(x_v.x)/2;
	half_x_v.y = -(x_v.y)/2;
	ROS_INFO("half_x_v(%f,%f)",
	half_x_v.x,half_x_v.y);
	
	//compute unit vectors & compute x step
	//x_v = mulScalarToVector(toUnitVector(x_v), x_step_);
	//ROS_INFO("x_step(%f,%f)",
	//x_v.x,x_v.y);
	x_v.x=x_step_;
	x_v.y=0;*/
	
	Position p;
	bool is_going_up = true;
	for (p.x=left_most_p.x; p.x<=right_most_p.x; p.x+=x_step_){
		if (is_going_up){
			p.y = bottom_most_p.y;
			move_goals_.push_back(p);
			p.y = top_most_p.y;
			move_goals_.push_back(p);
		}
		else{
			p.y = top_most_p.y;
			move_goals_.push_back(p);
			p.y = bottom_most_p.y;
			move_goals_.push_back(p);
		}
		is_going_up = !is_going_up;
	}
	
	
	/*/ Generate zig-zag path from rectangle starting from left to right from bottom to top first
	Position p = addVectors(bottom_most_p, half_x_v);	//bottom left start point
	int steps = distanceBetweenVectors(left_most_p,right_most_p)/x_step_;
	bool is_going_up = true;
	for (int i=0; i<steps; i++){
		if (is_going_up){
			p = addVectors(p, mulScalarToVector(x_v, i));
			move_goals_.push_back(p);
			p = addVectors(p, y_v);
			move_goals_.push_back(p);
		}
		else{
			p = addVectors(p, mulScalarToVector(x_v, i));
			move_goals_.push_back(p);
			p = getVector(p, y_v);	//p - y_v
			move_goals_.push_back(p);
		}
		is_going_up = !is_going_up;
	}*/
	showRvizMoveGoals();
}

bool StateMachine::getOrientationEstimate(){
	ros::service::waitForService("find_orientation", -1);
  if(ros::service::exists("find_orientation", true)){
    std_srvs::Trigger srv;
    orientation_estimate_client_.call(srv);
    return srv.response.success;
  }
  return false;
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

bool StateMachine::writeCurrentPosToFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	std::ofstream myfile(file_path_);
	for(int i=0; i < move_goals_.size(); i++){
		myfile<<move_goals_[i].x <<","<<move_goals_[i].y<<"\n";
	}
	myfile.close();
	return true;
}

bool StateMachine::clearPointsInFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	move_goals_.clear();
	std::ofstream myfile(file_path_);
	myfile.open(file_path_, std::ofstream::out | std::ofstream::trunc);
	myfile.close();
	return true;
}

bool StateMachine::readPointsFromFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	move_goals_.clear();
	std::ifstream myfile;
	myfile.open(file_path_);
	std::string line;
	while(std::getline(myfile,line)){
		Position p;
		std::string token;
		std::stringstream ss(line);
		std::getline(ss,token,',');
		p.x=std::stod(token);
		std::getline(ss,token,',');
		p.y=std::stod(token);
		move_goals_.push_back(p);
	}
	myfile.close();
	showRvizMoveGoals();
	return true;
}

float StateMachine::angleDifferenceToPoint(Position p){
	p.x -= current_pos_.x;
	p.y -= current_pos_.y;
	
	float angle = abs(atan2(p.y, p.x));
	if (p.x<0) {
		angle = (p.y<0) ? angle-M_PI : M_PI-angle;
	}
	else{
		if (p.y<0) angle = -angle;
	}
	ROS_INFO("Relative angle from current position: %f", angle);
	float result = angle - current_rad_;
	if (result>M_PI){
		result = -angle + current_rad_; 
	}
	ROS_INFO("Relative angle from current orientation: %f", result);	
	return result;
}

void StateMachine::moveTowardsGoal(){
	geometry_msgs::Twist cmd_vel_msg;	
	
	if (distanceBetweenVectors(current_pos_, move_goals_[current_goal_index_]) > distance_threshold_){		
		//check robot direction with goal
		float angle = angleDifferenceToPoint(move_goals_[current_goal_index_]);
		bool is_differential_movement;
		
		//rotate towards goal
		if (angle > rotation_threshold_){
			if (angle > differential_movement_threshold_){
				is_differential_movement = false;
				cmd_vel_msg.angular.z = max_linear_speed_;
			}
			else{
				is_differential_movement = true;
				cmd_vel_msg.angular.z = max_linear_speed_;
			}
		}
		else if (angle < -rotation_threshold_){
			if (angle < -differential_movement_threshold_){
				is_differential_movement = false;
				cmd_vel_msg.angular.z = max_angular_speed_;
			}
			else{
				is_differential_movement = true;
				cmd_vel_msg.angular.z = -max_angular_speed_;
			}
		}
		else is_differential_movement = true;
		
		// linear movement is only present when angle difference is small enough
		if (is_differential_movement) cmd_vel_msg.linear.x = 0.5;
		ROS_INFO_THROTTLE(1, "Moving to [%d] (%f,%f) linear: %f ang: %f",current_goal_index_,move_goals_[current_goal_index_].x,move_goals_[current_goal_index_].y,cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
	}
	else{
		ROS_INFO("Goal reached.");
		updateRvizMoveGoal(current_goal_index_, 1);
		goal_reached_ = true;
	}
		
	//cmd_velocity_publisher_.publish(cmd_vel_msg);
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
	showRvizPos(data, msg.address, false);
}

void StateMachine::currentPosCallback(const marvelmind_nav::hedge_imu_fusion msg){
	current_pos_.x = msg.x_m;
	current_pos_.y = msg.y_m;
	current_pos_.last_updated = ros::Time::now();
	/*
	current_orientation_.w = msg.qw;
	current_orientation_.x = msg.qx;
	current_orientation_.y = msg.qy;
	current_orientation_.z = msg.qz;
	current_rad_ = quaternionToRadian(current_orientation_);
	*/
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
	showRvizPos(current_pos_, -1, true);
}

void StateMachine::currentYawCallback(const std_msgs::Float64 msg){
	current_rad_ = msg.data;
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
