#include <ros/ros.h>
#include "laser_line/laser_feature_ros.h"

using namespace std;
 
int main(int argc,char** argv)
{
	ROS_DEBUG("Starting laserline node.");

	ros::init(argc, argv, "laserline");
	line_feature::LaserFeatureROS line_feature_ros();
 
	return 0;
}
