#include <ros/ros.h>
#include <laser_line/Lines.h>
#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Core>

#include <visualization_msgs/MarkerArray.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 
struct Line {
    Eigen::Vector2d p1, p2;
};
 
class LineMatcher
{
public:
    LineMatcher();

private:    
    void laserLinesCallback(const laser_line::Lines::ConstPtr& msg);
    void mapLinesCallback(const laser_line::Lines::ConstPtr& msg);

    std::vector<Line> selectLongestLines(const std::vector<Line>& lines, int n);
    int matchLines(const std::vector<Line>& selected_lines, const std::vector<Line>& map_lines, std::vector<std::pair<Line, Line>>& matched_lines, std::vector<double>& matched_score);
    double calculateAngle(const Line&  l);
    double normalLineAngle(const double  angle);
    void processOptimization();

    visualization_msgs::Marker lineToMarker(const Line &line, int id, const std::string &ns, const std::array<float, 4> &color, const std::string &frame_id);
    void publishLinesAsMarkers(const ros::Publisher& marker_pub, const std::vector<std::pair<Line, Line>> & matched_lines);

private:
    ros::Publisher matched_lines_marker_pub_;
    ros::Subscriber laser_lines_sub;
    ros::Subscriber map_lines_sub;
    ros::NodeHandle nh;

    std::vector<Line> laser_lines;
    std::vector<Line> map_lines;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_bro_;    
};
