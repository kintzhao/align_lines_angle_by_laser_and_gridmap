#include <ros/ros.h>
#include "laser_line/extract_line_from_map.h"

using namespace std;
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_line_from_map");

    ExtractLineFromMap extract_line_from_map;
	ros::Rate r(2);
	while (ros::ok())
	{
		extract_line_from_map.loop();
		r.sleep();
		ros::spinOnce();
	}

    return 0;
}
