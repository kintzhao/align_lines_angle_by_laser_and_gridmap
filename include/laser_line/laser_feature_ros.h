#ifndef _LASER_FEATURE_ROS_H_
#define _LASER_FEATURE_ROS_H_

#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "laser_line/line_feature.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "laser_line/Line.h"
#include "laser_line/Lines.h"

#include "sys/time.h"

namespace line_feature
{

	class LaserFeatureROS
	{
	public:
		LaserFeatureROS();
		~LaserFeatureROS();

		laser_line::Lines extractLine(const sensor_msgs::LaserScan &scan_msg);
		laser_line::Lines extractLine(const sensor_msgs::LaserScan &scan_msg, const geometry_msgs::Pose2D& laser_pose);
		void setSubSensor(bool flag) { is_sub_sensors_ = flag; }
		void setShowMarker(bool flag) { is_show_markers_ = flag; }
	private:
		// memeber function
		void publishLines(const std::vector<gline> &m_gline, const std::string frame_id="map");
		// 发布直线分割消息
		void publishMarkerMsg(const std::vector<gline> &, visualization_msgs::Marker &marker_msg);
		// 开始函数，包括读取文件（先验地图等信息）
		// load params
		void load_params();
		// 角度参量
		void compute_bearing(const sensor_msgs::LaserScan &);
		// 激光线程函数（ros节点回调函数），采集激光并进行处理，处理频率以采集频率为准
		void scanCB(const sensor_msgs::LaserScan::ConstPtr &);
		void transformLinesToMapFrame(const std::vector<gline> &glines_in, std::vector<gline> &glines_out);
		std::vector<gline> transformLinesToMapFrame(const std::vector<gline>& glines_in, const geometry_msgs::Pose2D& laser_pose);
		void startgame();
	public:
		laser_line::Lines lines_msg_;

	private:
		// 参数信息laser
		bool com_bearing_flag;

		double m_startAng;
		double m_AngInc;
		LineFeature line_feature_;

		std::string frame_id_;
		std::string scan_topic_;

		std::vector<gline> m_gline;
		std::vector<line> m_line;
		// ROS
		ros::NodeHandle nh_;
		ros::NodeHandle nh_local_;
		ros::Subscriber scan_subscriber_;
		ros::Publisher line_publisher_;
		ros::Publisher marker_publisher_, laser_lines_publisher_, matchered_scan_publisher_;

		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener tfl_;
    	bool is_sub_sensors_ = false;
		bool is_show_markers_ = true;
		bool is_test_local_ = true;				
	};

}
#endif
