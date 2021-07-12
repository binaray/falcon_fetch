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

struct BeaconPos{
	float x_;
	float y_;
	ros::Time freshness_;
};

class State;

class StateMachine{
	friend class State;
	public:
		StateMachine();
		~StateMachine();
		void update();
	private:
		ros::NodeHandle n_;
		static State *current_state_;
		std::map<int,BeaconPos> beacons_pos_;
		std::map<int,BeaconPos>::iterator it_;
		BeaconPos current_pos_;
		
		bool init();
		
		ros::Subscriber beacons_pos_subscriber_;
		ros::Subscriber current_pos_subscriber_;		
		void beaconsPosCallback(const marvelmind_nav::beacon_pos_a msg);
		void currentPosCallback(const marvelmind_nav::hedge_pos msg);
};

#endif
