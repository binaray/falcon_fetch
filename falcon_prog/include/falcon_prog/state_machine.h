#ifndef StateMachine_h
#define StateMachine_h

#include "ros/ros.h"
#include <queue>
#include <math.h> 
#include <falcon_prog/states.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
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
	float x;
	float y;
	ros::Time last_updated;
};

class State;

class StateMachine{
	friend class State;
	public:
		StateMachine();
		~StateMachine();
		void update();
		void updateRvizMoveGoal(int address, int status);
		
		//runtime variables
		bool is_beacons_init_ = false;
		std::map<int,Position> beacons_pos_;
		Position current_pos_;
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
		float stationary_threshold_;
		ros::Duration last_updated_timeout_;
		ros::Duration immobile_timeout_;
		
	private:		
		static State *current_state_;
		std::string marker_frame_;
		
		ros::NodeHandle n_;
		ros::Subscriber beacons_pos_subscriber_;
		ros::Subscriber current_pos_subscriber_;
		ros::Publisher rviz_marker_publisher_;

		void beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg);
		void currentPosCallback(const marvelmind_nav::hedge_pos msg);
		
		bool init();
		void generateMoveGoals();
		void showRvizMoveGoals();
};

#endif
