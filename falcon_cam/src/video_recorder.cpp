/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include "marvelmind_nav/hedge_pos_ang.h"
#include "std_msgs/Float64.h"

#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;


namespace VideoSaver{
	int capture_interval_;
	std::string timezone_;
	ros::Timer save_video_timer_;
	std::string file_name_;
	std::string base_file_path_;
	std::string file_dir_;
	std::string current_path_;
	std::string current_date_path_;
	int current_date_;
	int old_date_;
	std::time_t current_time_;
	std::string helper_str_;
	int file_count_ = 0;
	boost::format g_format_;
	boost::format g_format_2_;
	std::string camera_log_file_name_;
	std::ofstream file_;
	ros::Time cam_start_time_;
	ros::Time cam_current_time_;
	ros::Duration cam_timestamp_;
	int cam_hr_timestamp_;
	int cam_min_timestamp_;
	float cam_sec_timestamp_;
	
	ros::Subscriber current_pos_subscriber_;
	ros::Subscriber current_yaw_subscriber_;
	float current_x_;
	float current_y_;
	float current_yaw_;
	
	bool is_recording_ = false;
	

  void saveVideoCallback(const ros::TimerEvent& event){
		file_count_ ++;
		try {
			file_name_ = (g_format_).str();
			camera_log_file_name_ = (g_format_2_).str();

		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}
		try {
			file_name_ = (g_format_ % file_count_).str();
			camera_log_file_name_ = (g_format_2_ % file_count_).str();
		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}
		try { 
			file_name_ = (g_format_ % file_count_ % "avi").str();
			camera_log_file_name_ = (g_format_2_ % file_count_ % "csv").str();
		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}
		//file_name_ = (g_format_ % file_count_ % "avi").str();
		//ROS_INFO("Saving video as %s", file_name_.c_str());
  }

	std::string doubleDigitStr(int number){
		std::stringstream ss;
		ss << std::setw(2) << std::setfill('0') << (number);
		std::string s = ss.str();
		return s;
	}

	void recordTimestamp(){
		cam_current_time_ = ros::Time::now();
		cam_timestamp_ = cam_current_time_ - cam_start_time_;
		file_.open(camera_log_file_name_, std::fstream::app);
		file_ <<cam_timestamp_<< "," << current_x_ << "," << current_y_ << "," << current_yaw_ << "\n";
		/*
		cam_timestamp_ = (cam_current_time_ - cam_start_time_).toSec();
		cam_hr_timestamp_ = cam_timestamp_/3600;
		cam_min_timestamp_ = (cam_timestamp_ - cam_hr_timestamp_*3600)/60;
		cam_sec_timestamp_ = cam_timestamp_ - cam_hr_timestamp_*3600 - cam_min_timestamp_*60;
		file_.open(camera_log_file_name_, std::fstream::app);
		if(cam_sec_timestamp_<10) file_ << "[" +cam_hr_timestamp_ + ":" + doubleDigitStr(cam_min_timestamp_) + ":0" + cam_sec_timestamp_<< "]" << " \n";
		else file_ << "[" +cam_hr_timestamp_ + ":" + doubleDigitStr(cam_min_timestamp_) + ":0" + cam_sec_timestamp_<< "]" << " \n";
		*/
		file_.close();
	}

  void createFolder(std::string file_path){
		// folder name will be yyyymmdd
		ROS_INFO("Timezone: %s", timezone_.c_str());
		char* char_arr;
		char_arr = &timezone_[0];
		putenv(char_arr);
		current_time_ = std::time(nullptr);
		tm *time_ = std::localtime(&current_time_);
		current_date_path_ = std::to_string(time_->tm_year+1900);
		helper_str_ = doubleDigitStr(time_->tm_mon+1);
		current_date_path_ = current_date_path_.append(helper_str_);
		helper_str_ = doubleDigitStr(time_->tm_mday);
		current_date_path_ = current_date_path_.append(helper_str_);
		current_date_ = std::stoi(current_date_path_);
		ROS_INFO("Current date: %s", current_date_path_.c_str());

		// check if folder exists; else create new folder
		file_dir_ = base_file_path_.append("/");
		current_path_ = file_dir_;
		file_dir_ = file_dir_.append(current_date_path_);
		boost::filesystem::create_directory(file_dir_);
		ROS_INFO("Full video file directory: %s", file_dir_.c_str());
		std::string helper_str_dir = file_dir_;
		file_name_ = (file_dir_.append("/")).append(file_name_);
		g_format_.parse(file_name_);

		// create new file to record timestamps for each frame
		camera_log_file_name_ = (helper_str_dir.append("/")).append("camera%04d_log.csv");
		g_format_2_.parse(camera_log_file_name_);

		try {
			file_name_ = (g_format_).str();
			camera_log_file_name_ = (g_format_).str();

		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}
		try {
			file_name_ = (g_format_ % file_count_).str();
			camera_log_file_name_ = (g_format_2_ % file_count_).str();
		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}
		try { 
			file_name_ = (g_format_ % file_count_ % "avi").str();
			camera_log_file_name_ = (g_format_2_ % file_count_ % "csv").str();
		} catch (...) { 
			g_format_.clear(); 
			g_format_2_.clear();
		}

		ROS_INFO("Camera log file path: %s", camera_log_file_name_.c_str());
		// if file exists, delete
		file_.open(camera_log_file_name_, std::ios::out | std::ios::trunc);
		file_ << "Start time: " << asctime(time_);
		file_.close();

		// delete folder after 7 days
		old_date_ = current_date_ - 7;
		file_dir_ = current_path_.append(std::to_string(old_date_));
		ROS_WARN("Deleting 7 days old worth of videos in %s", file_dir_.c_str());
		boost::filesystem::remove_all(file_dir_);
  }
  
  void currentPosCallback(const marvelmind_nav::hedge_pos_ang msg){
		current_x_ = msg.x_m;
		current_y_ = msg.y_m;
	}
	
  void currentYawCallback(const std_msgs::Float64 msg){
		current_yaw_ = msg.data;
	}

}


void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    if (!outputVideo.isOpened()) {
		ROS_INFO("Camera log file path: %s", VideoSaver::camera_log_file_name_.c_str());
		//std::ofstream file_(camera_log_file_name_);
        cv::Size size(image_msg->width, image_msg->height);
				ROS_INFO("Saving current video as %s", VideoSaver::file_name_.c_str());
				filename = VideoSaver::file_name_;
        outputVideo.open(filename, 
#if CV_MAJOR_VERSION >= 3
                cv::VideoWriter::fourcc(codec.c_str()[0],
#else
                CV_FOURCC(codec.c_str()[0],
#endif
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]), 
                fps,
                size,
                true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }

        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

    }

    if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1.0 / fps))
    {
      // Skip to get video with correct fps
      return;
    }

		if(VideoSaver::is_recording_){
		  try
		  {
		    cv_bridge::CvtColorForDisplayOptions options;
		    options.do_dynamic_scaling = use_dynamic_range;
		    options.min_image_value = min_depth_range;
		    options.max_image_value = max_depth_range;
		    options.colormap = colormap;
		    const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
		    if (!image.empty()) {
		      outputVideo << image;
		// save timestamp of frames
			VideoSaver::recordTimestamp();
		      //ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
		      g_count++;
		      g_last_wrote_time = image_msg->header.stamp;
		      ros::spinOnce();
		    } else {
		        ROS_WARN("Frame skipped, no data!");
		    }
				if(filename!=VideoSaver::file_name_){
					outputVideo.release();
					return;
				}
		  } catch(cv_bridge::Exception)
		  {
		      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
		      return;
		  }
	  }
	  else{
	  	ROS_INFO_THROTTLE(5, "Waiting for recording trigger to record video.");
			// start time stamp
			VideoSaver::cam_start_time_ = ros::Time::now();
		}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

	nh.param("timezone",VideoSaver::timezone_,std::string("TZ=Asia/Singapore"));

    local_nh.param("filename", VideoSaver::file_name_, std::string("camera%04i.avi"));
		local_nh.param("base_file_path", VideoSaver::base_file_path_, std::string("/home/falconrpi"));
    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);
    local_nh.param("capture_interval", VideoSaver::capture_interval_, 600);
		VideoSaver::save_video_timer_ = nh.createTimer(ros::Duration(VideoSaver::capture_interval_), VideoSaver::saveVideoCallback);
/*
		VideoSaver::g_format_.parse(VideoSaver::file_name_);

		try {
      VideoSaver::file_name_ = (VideoSaver::g_format_).str();
    } catch (...) { VideoSaver::g_format_.clear(); }
    try {
      VideoSaver::file_name_ = (VideoSaver::g_format_ % VideoSaver::file_count_).str();
    } catch (...) { VideoSaver::g_format_.clear(); }
    try { 
      VideoSaver::file_name_ = (VideoSaver::g_format_ % VideoSaver::file_count_ % "avi").str();
    } catch (...) { VideoSaver::g_format_.clear(); }
*/

    if (stamped_filename) {
      std::size_t found = VideoSaver::file_name_.find_last_of("/\\");
      std::string path = VideoSaver::file_name_.substr(0, found + 1);
      std::string basename = VideoSaver::file_name_.substr(found + 1);
      std::stringstream ss;
      ss << ros::Time::now().toNSec() << basename;
      VideoSaver::file_name_ = path + ss.str();
      ROS_INFO("Video recording to %s", VideoSaver::file_name_.c_str());
			VideoSaver::g_format_.parse(VideoSaver::file_name_);
    }

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

		VideoSaver::createFolder(VideoSaver::base_file_path_);

    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName("image");
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);
    
    VideoSaver::current_pos_subscriber_ = nh.subscribe<marvelmind_nav::hedge_pos_ang>("hedge_pos_ang", 10, VideoSaver::currentPosCallback);
    VideoSaver::current_yaw_subscriber_ = nh.subscribe<std_msgs::Float64>("robot_yaw", 10, VideoSaver::currentYawCallback);
    ros::param::get("/is_recording", VideoSaver::is_recording_);
    
    ros::Rate loop_rate(10);
    
    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    while(ros::ok()){
    	ros::spinOnce();
    	loop_rate.sleep();
	    ros::param::get("/is_recording", VideoSaver::is_recording_);
  	}
    //std::cout << "\nVideo saved as " << VideoSaver::file_name_ << std::endl;
}
