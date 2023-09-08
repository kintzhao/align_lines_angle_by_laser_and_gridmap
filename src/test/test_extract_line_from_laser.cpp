#include <ros/ros.h>
#include "laser_line/laser_feature_ros.h"

using namespace std;
 
int main(int argc,char** argv)
{
	ROS_DEBUG("Starting laserline node.");

	ros::init(argc, argv, "laser_line");
	line_feature::LaserFeatureROS line_feature_ros;

	ros::Rate r(10);
	while (ros::ok())
	{

		r.sleep();
		ros::spinOnce();
	}

 
 
	return 0;
}
