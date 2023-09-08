 
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "laser_line/Lines.h"
#include "laser_line/Line.h"
 
class ExtractLineFromMap
{
public:
    ExtractLineFromMap() ;
	void loop();
    laser_line::Lines extractLine(const nav_msgs::OccupancyGrid& map_msg);
    void setSubSensor(bool flag) { is_sub_sensors_ = flag; }
    void setShowMarker(bool flag) { is_show_markers_ = flag; }
    
private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
	void publishLinesMarker(const laser_line::Lines& lines_msg);

public:
	laser_line::Lines lines_msg_;	
private:
    ros::Subscriber map_sub;
    ros::Publisher marker_pub, lines_pub_;
    ros::NodeHandle nh;

	bool is_show_markers_ = true;
    bool is_sub_sensors_ = false;    
};
