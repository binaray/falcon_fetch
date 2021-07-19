#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"

#include <math.h> 

namespace InitialOrientation{
	ros::ServiceServer initial_orientation_srv_;
	ros::Subscriber hedge_subscriber_;
	ros::Subscriber imu_subscriber_;
	ros::Publisher cmd_vel_publisher_;
	ros::Publisher robot_yaw_publisher_;
	std_msgs::Float64 yaw_msg_;
	float initial_x_;
	float initial_y_;
	float current_x_;
	float current_y_;
	float orientation_w_;
	float orientation_z_;
	float basic_angle_;
	float final_angle_;
	float offset_angle_;
	ros::Time start_time_;
	bool is_orientation_ok_ = false;
	double roll_, pitch_, yaw_;
  tf::Quaternion quat_;
	
	bool findOrientation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	//subscribe to imu and hedge. move robot at 0.10m/s for 1 s. find the angle relative to x axis 
		ROS_INFO("Called find orientation srv. Initializing...");
		initial_x_ = current_x_;
		initial_y_ = current_y_;
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_msg.linear.x = 0.1;
		cmd_vel_publisher_.publish(cmd_vel_msg);
		start_time_ = ros::Time::now();
		while(ros::Time::now() - start_time_ < ros::Duration(1)){
			ros::spinOnce();
			ROS_INFO_THROTTLE(1,"Waiting for robot to initialise with orientation");
		}
		cmd_vel_msg.linear.x = 0;
		cmd_vel_publisher_.publish(cmd_vel_msg);
		// make sure robot comes to a stop, process after 1s
		ROS_INFO("Waiting for robot to come to a stop");
		ros::Rate sleep_rate(1);
		sleep_rate.sleep();
		basic_angle_ = atan((current_x_ - initial_x_)/(current_y_ - initial_y_))/M_PI*180;
		if(current_x_>initial_x_){
			if(current_y_>initial_y_) final_angle_ = basic_angle_;
			else final_angle_ = -basic_angle_;
		}
		else{
			if(current_y_>initial_y_) final_angle_ = 180-basic_angle_;
			else final_angle_ = -180+basic_angle_;
		}
		ROS_INFO("Directional angle wrt x-axis: %5f", final_angle_);
		offset_angle_ = final_angle_ - yaw_;
		res.success = true;
		res.message = std::to_string(final_angle_)+","+std::to_string(offset_angle_);
		is_orientation_ok_ = true;
		return true;
	}
	
	void publishRobotYaw(){
		if(is_orientation_ok_){
			// need to take into account of the global(to beacon) and relative(to robot) orientation
			yaw_msg_.data = fmod((yaw_/M_PI*180 + offset_angle_),360);
			if(yaw_msg_.data > 180) yaw_msg_.data = yaw_msg_.data - 360;
			if(yaw_msg_.data < -180) yaw_msg_.data = yaw_msg_.data+360;
			robot_yaw_publisher_.publish(yaw_msg_);
		}
	}

	void hedgeCallback(const marvelmind_nav::hedge_imu_fusion msg){
		current_x_ = msg.x_m;
		current_y_ = msg.y_m;
	}
	
	void imuCallback(const sensor_msgs::ImuConstPtr& msg){
		//orientation_w_ = msg->orientation.w;
		//orientation_z_ = msg->orientation.z;
  	tf::quaternionMsgToTF(msg->orientation, quat_);
  	tf::Matrix3x3(quat_).getRPY(roll_, pitch_, yaw_);
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
	InitialOrientation::robot_yaw_publisher_ = nh.advertise<std_msgs::Float64>("robot_yaw", 10);
	InitialOrientation::initial_orientation_srv_ = nh.advertiseService("find_orientation", InitialOrientation::findOrientation);

	while(ros::ok()){
		InitialOrientation::publishRobotYaw();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
