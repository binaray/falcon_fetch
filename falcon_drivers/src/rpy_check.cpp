#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "marvelmind_nav/hedge_imu_fusion.h"

ros::Publisher quat_pub;

void imuCb(const sensor_msgs::ImuConstPtr& msg){
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("R: %5f, P: %5f, Y: %5f \nAngular speeds - X: %5f, Y: %5f, Z: %5f \nLinear accels - X: %5f, Y: %5f, Z: %5f",
	roll, pitch, yaw, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

void convertToQuat(float r, float p, float y){
  tf::Quaternion quat;
  quat = tf::createQuaternionFromRPY(r,p,y);
  sensor_msgs::Imu msg;
  msg.orientation.x = quat[0];
  msg.orientation.y = quat[1];
  msg.orientation.z = quat[2];
  msg.orientation.w = quat[3];
  quat_pub.publish(msg);
}

void imuRawCb(const marvelmind_nav::hedge_imu_fusion msg){
  tf::Quaternion quat_uwb;
  quat_uwb[0] = msg.qx;
  quat_uwb[1] = msg.qy;
  quat_uwb[2] = msg.qz;
  quat_uwb[3] = msg.qw;
  double roll_uwb, pitch_uwb, yaw_uwb;
  tf::Matrix3x3(quat_uwb).getRPY(roll_uwb, pitch_uwb, yaw_uwb);
  ROS_INFO("R: %5f, P: %5f, Y: %5f", roll_uwb, pitch_uwb, yaw_uwb);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rpy_check");
  ros::NodeHandle n;
  std::string imu_topic;
  if(argc>1) imu_topic = argv[1];
  else imu_topic = "imu";
  ROS_INFO("RPY CHECK SUBSCRIBING TO %s", imu_topic.c_str());
	ros::Subscriber imu_sub = n.subscribe(imu_topic, 10, &imuCb);
  ros::Subscriber imu_raw_sub = n.subscribe("hedge_imu_fusion", 10, &imuRawCb);
  quat_pub = n.advertise<sensor_msgs::Imu>("check_quat", 1000);
  ros::spin();
  return 0;
}
