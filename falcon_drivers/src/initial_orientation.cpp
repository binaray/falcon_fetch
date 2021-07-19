#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Trigger.h"

#include <math.h> 

namespace InitialOrientation{
	ros::ServiceServer initial_orientation_srv_;
	ros::Subscriber hedge_subscriber_;
	ros::Subscriber imu_subscriber_;
	ros::Publisher cmd_vel_publisher_;
	float initial_x_ = 0;
	float initial_y_ = 0;
	float current_x_ = 1;
	float current_y_ = 1;
	float orientation_w_;
	float orientation_z_;
	float final_angle_;
	ros::Time start_time_;
	
	bool findOrientation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	//subscribe to imu and hedge. move robot at 0.10m/s for 1 s. find the angle relative to x axis 
		ROS_INFO("Called find orientation srv. Initializing...");
		//initial_x_ = current_x_;
		//initial_y_ = current_y_;
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_msg.linear.x = 0.1;
		cmd_vel_publisher_.publish(cmd_vel_msg);
		start_time_ = ros::Time::now();
		while(ros::Time::now() - start_time_ < ros::Duration(1)){
			ROS_INFO_THROTTLE(1,"Waiting for robot to initialise with orientation");
		}
		cmd_vel_msg.linear.x = 0;
		cmd_vel_publisher_.publish(cmd_vel_msg);
		
		final_angle_ = atan((current_x_ - initial_x_)/(current_y_ - initial_y_))/M_PI*180;
		ROS_INFO("Final directional angle: %5f", final_angle_);
		res.success = true;
		res.message = final_angle_;
		return true;
	}

	void hedgeCallback(const marvelmind_nav::hedge_imu_fusion msg){
		current_x_ = msg.x_m;
		current_y_ = msg.y_m;
	}
	
	void imuCallback(const sensor_msgs::Imu msg){
		orientation_w_ = msg.orientation.w;
		orientation_z_ = msg.orientation.z;
	}

}

int main(int argc, char **argv){
	ros::init(argc,argv,"falcon_initial_orientation_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ROS_INFO("Initializing falcon initial orientation node!");

	InitialOrientation::hedge_subscriber_ = nh.subscribe<marvelmind_nav::hedge_imu_fusion>("hedge_imu_fusion", 10, InitialOrientation::hedgeCallback);
	InitialOrientation::imu_subscriber_ = nh.subscribe<sensor_msgs::Imu>("bno055_imu/data", 10, InitialOrientation::imuCallback);
	InitialOrientation::cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	InitialOrientation::initial_orientation_srv_ = nh.advertiseService("find_orientation", InitialOrientation::findOrientation);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
