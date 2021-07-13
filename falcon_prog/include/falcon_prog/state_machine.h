#ifndef StateMachine_h
#define StateMachine_h

#include "ros/ros.h"
#include <falcon_prog/states.h>
#include "std_msgs/String.h"
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
		
		//runtime variables
		bool is_beacons_init_ = false;
		std::map<int,Position> beacons_pos_;
		Position current_pos_;
		float min_x_bound_;
		float min_y_bound_;
		float max_x_bound_;
		float max_y_bound_;
		std::queue<Position> move_goals_;
		
		bool is_moving_ = false;
		bool is_running_waypoint_ = false;
		
		//startup constants
		float x_step_;
		float bound_padding_;
		ros::Duration last_updated_timeout_;
		ros::Duration immobile_timeout_;
		
		Position current_goal_;
		Position prev_goal_;
		
	private:		
		static State *current_state_;
		
		ros::NodeHandle n_;
		ros::Subscriber beacons_pos_subscriber_;
		ros::Subscriber current_pos_subscriber_;
		ros::Timer waypoint_timer_;
		void beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg);
		void currentPosCallback(const marvelmind_nav::hedge_pos msg);
		
		bool init();
		void generateMoveGoals();
		void waypointRoutine(const ros::TimerEvent& event)
};

#endif
