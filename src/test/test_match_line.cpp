#include <ros/ros.h>
#include "laser_line/match_line.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_matcher_node");
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "/tmp/log_line_matcher_node_");
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;

    LineMatcher matcher;

    ros::spin();
    return 0;
}