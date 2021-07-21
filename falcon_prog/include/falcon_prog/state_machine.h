#ifndef StateMachine_h
#define StateMachine_h

#include "ros/ros.h"
#include <queue>
#include <math.h> 
#include <falcon_prog/states.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <visualization_msgs/Marker.h>
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"
#include <iostream>
#include <fstream>
#include <sstream>

#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
#include "marvelmind_nav/hedge_telemetry.h"
#include "marvelmind_nav/hedge_quality.h"
#include "marvelmind_nav/marvelmind_waypoint.h"

struct Position{
	float x=0.0;
	float y=0.0;
	ros::Time last_updated;
};

struct Quaternion{
	float x=0.0;
	float y=0.0;
	float z=0.0;
	float w=1.0;
};

class State;

class StateMachine{
	friend class State;
	public:
		StateMachine();
		~StateMachine();
		void update();
		bool publishNextMoveGoal();
		float angleDifferenceToPoint(Position p);
		void moveTowardsGoal();
		bool getOrientationEstimate();
		
		//runtime variables
		bool is_beacons_init_ = false;
		std::map<int,Position> beacons_pos_;
		Position current_pos_;
		Quaternion current_orientation_;
		double current_rad_;
		Position *stationary_pos_;	//stores earliest stationary point to check for immobility
		
		float min_x_bound_;	//inner rectangle bound inside beacon area (operational area for robot)
		float min_y_bound_;
		float max_x_bound_;
		float max_y_bound_;
		std::vector<Position> move_goals_;
		int current_goal_index_ = -1;	//index to current goal in move_goals_- also used as rviz address reference
		
		bool is_running_waypoint_ = false;
		bool goal_reached_ = false;
		bool is_immobile_ = false;
		
		//startup constants
		int stationary_beacon_count_;
		float x_step_;
		float bound_padding_;
		float max_linear_speed_;
		float max_angular_speed_;
		float stationary_threshold_;
		float rotation_threshold_;
		float distance_threshold_;
		float differential_movement_threshold_;
		ros::Duration last_updated_timeout_;
		ros::Duration immobile_timeout_;
		std::string file_path_;
		
	private:		
		static State *current_state_;
		std::string marker_frame_;
		
		ros::NodeHandle n_;
		ros::Subscriber beacons_pos_subscriber_;
		ros::Subscriber current_pos_subscriber_;
		ros::Subscriber current_yaw_subscriber_;
		ros::Publisher cmd_velocity_publisher_;
		ros::Publisher rviz_marker_publisher_;
		ros::ServiceServer add_point_srv_;
		ros::ServiceServer delete_points_srv_;
		ros::ServiceServer read_points_srv_;
		ros::ServiceClient orientation_estimate_client_;

		void beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg);
		void currentPosCallback(const marvelmind_nav::hedge_imu_fusion msg);
		void currentYawCallback(const std_msgs::Float64 msg);
		
		bool init();
		void generateMoveGoals();
		void showRvizMoveGoals();
		void updateRvizMoveGoal(int address, int status);
		void showRvizPos(Position p, int address, bool is_hedge);
		
		bool writeCurrentPosToFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool clearPointsInFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool readPointsFromFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif
