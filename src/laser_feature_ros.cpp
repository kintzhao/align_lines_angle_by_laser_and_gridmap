#include "laserline/laser_feature_ros.h"
#include "sys/time.h"
#include "glog/logging.h"
#include "laser_line/Line.h"
#include "laser_line/Lines.h"
 

//FILE *fpd = fopen("data.txt","w+");
namespace line_feature
{

LaserFeatureROS::LaserFeatureROS(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
	nh_(nh),
	nh_local_(nh_local),
	com_bearing_flag(false),
	tfl_(tf_buffer_)
{
	load_params();
	scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LaserFeatureROS::scanCB, this);

	if(show_lines_)
	{
		marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/laser_lines_marker", 1);
	}
	laser_lines_publisher_ = nh_.advertise<laser_line::Lines>("/laser_lines", 1);	
	matchered_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("/matchered_scan", 1);
	ros::spin();
}

LaserFeatureROS::~LaserFeatureROS()
{

}

void LaserFeatureROS::compute_bearing(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
	double angle_increment_,angle_start_;
	angle_increment_ = scan_msg->angle_increment;
	angle_start_ = scan_msg->angle_min;
	
	line_feature_.set_angle_increment(angle_increment_);
	line_feature_.set_angle_start(angle_start_);
	
	std::vector<double> bearings, cos_bearings, sin_bearings;
	std::vector<unsigned int> index;
	unsigned int i = 0;
	for (double b = scan_msg->angle_min; b <= scan_msg->angle_max; b += scan_msg->angle_increment)
	{
		bearings.push_back(b);
		cos_bearings.push_back(cos(b));
    	sin_bearings.push_back(sin(b));
    	index.push_back(i);
    	i++;
	}

	line_feature_.setCosSinData(bearings, cos_bearings, sin_bearings, index);
	ROS_DEBUG("Data has been cached.");
}

void LaserFeatureROS::scanCB(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
	ros::Time statrt_time = ros::Time::now();
	LOG(INFO)<<"scanCB";
	if(!com_bearing_flag)
	{
		compute_bearing(scan_msg);
		com_bearing_flag = true;
	}
	std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
	line_feature_.setRangeData(scan_ranges_doubles);

	startgame();
	LOG(INFO)<<"startgame cost:"<<( ros::Time::now() - statrt_time).toSec();

	sensor_msgs::LaserScan matched_scan = *scan_msg;
	matched_scan.header.stamp = ros::Time::now();
	matched_scan.header.frame_id = "matched_laser";
	matchered_scan_publisher_.publish(matched_scan);
}

void LaserFeatureROS::publishMarkerMsg(const std::vector<gline> &m_gline,visualization_msgs::Marker &marker_msg)
{
	marker_msg.ns = "line_extraction";
	marker_msg.id = 0;
	marker_msg.type = visualization_msgs::Marker::LINE_LIST;
	marker_msg.scale.x = 0.05;
	marker_msg.color.r = 0.0;
	marker_msg.color.g = 0.0;
	marker_msg.color.b = 0.5;
	marker_msg.color.a = 1.0;
    marker_msg.pose.orientation.w = 1.0;

	for (std::vector<gline>::const_iterator cit = m_gline.begin(); cit != m_gline.end(); ++cit)
	{
		geometry_msgs::Point p_start;
		p_start.x = cit->x1;
	    p_start.y = cit->y1;
	    p_start.z = 0;
	    marker_msg.points.push_back(p_start);
	    geometry_msgs::Point p_end;
	    p_end.x = cit->x2;
	    p_end.y = cit->y2;
	    p_end.z = 0;
	    marker_msg.points.push_back(p_end);
	}
	marker_msg.header.frame_id = "map";//"laser";
	marker_msg.header.stamp = ros::Time::now();
}

void LaserFeatureROS::publishLines(const std::vector<gline> &m_gline)
{
  	if (show_lines_)
  	{
  		visualization_msgs::Marker marker_msg;
    	publishMarkerMsg(m_gline, marker_msg);
  		marker_publisher_.publish(marker_msg);
 	}
	
	laser_line::Lines lines_msg;
	for (std::vector<gline>::const_iterator cit = m_gline.begin(); cit != m_gline.end(); ++cit)
	{
		laser_line::Line line_msg;
		line_msg.header.stamp = ros::Time::now();
		line_msg.header.frame_id = "map";//"laser";
 
		line_msg.p1.x = cit->x1;
	    line_msg.p1.y = cit->y1;
	    line_msg.p1.z = 0;
 
	    line_msg.p2.x = cit->x2;
	    line_msg.p2.y = cit->y2;
	    line_msg.p2.z = 0;
	    lines_msg.lines.push_back(line_msg);
	}

	lines_msg.header.stamp = ros::Time::now();
	lines_msg.header.frame_id = "map";//"laser";
	laser_lines_publisher_.publish(lines_msg);
}

void LaserFeatureROS::transformLinesToMapFrame(const std::vector<gline>& glines_in, std::vector<gline>& glines_out)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        // Get the transform from the laser frame to the map frame.
        transform = tf_buffer_.lookupTransform("map", "laser", ros::Time(0), ros::Duration(1.0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    for(const auto& line : glines_in)
    {
        gline transformed_line;

        // Transform start point
        geometry_msgs::PointStamped start_point_in, start_point_out;
        start_point_in.header.frame_id = "laser";
        start_point_in.point.x = line.x1;
        start_point_in.point.y = line.y1;

        tf2::doTransform(start_point_in, start_point_out, transform);
        transformed_line.x1 = start_point_out.point.x;
        transformed_line.y1 = start_point_out.point.y;

        // Transform end point
        geometry_msgs::PointStamped end_point_in, end_point_out;
        end_point_in.header.frame_id = "laser";
        end_point_in.point.x = line.x2;
        end_point_in.point.y = line.y2;

        tf2::doTransform(end_point_in, end_point_out, transform);
        transformed_line.x2 = end_point_out.point.x;
        transformed_line.y2 = end_point_out.point.y;

        // Append the transformed line to the output vector
        glines_out.push_back(transformed_line);
    }
}


//主函数
void LaserFeatureROS::startgame()
{
	std::vector<line> lines;
	std::vector<gline> glines;
  	line_feature_.extractLines(lines,glines);

	std::vector<gline> map_lines;
	transformLinesToMapFrame(glines, map_lines);
	publishLines(map_lines);
}

// Load ROS parameters
void LaserFeatureROS::load_params()
{
	ROS_DEBUG("*************************************");
	ROS_DEBUG("PARAMETERS:");
  
	std::string frame_id, scan_topic;
	bool show_lines;

	nh_local_.param<std::string>("frame_id", frame_id, "laser");
	frame_id_ = frame_id;
	ROS_DEBUG("frame_id: %s", frame_id_.c_str());

	nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
	scan_topic_ = scan_topic;
	ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

	nh_local_.param<bool>("show_lines", show_lines, true);

	show_lines_ = show_lines;
	ROS_DEBUG("show_lines: %s", show_lines ? "true" : "false");

	// Parameters used by the line extraction algorithm

	int min_line_points,seed_line_points;
	double least_thresh,min_line_length,predict_distance;

	nh_local_.param<double>("least_thresh", least_thresh, 0.04);
	line_feature_.set_least_threshold(least_thresh);
	ROS_DEBUG("least_thresh: %lf", least_thresh);

	nh_local_.param<double>("min_line_length", min_line_length, 0.5);
	line_feature_.set_min_line_length(min_line_length);
	ROS_DEBUG("min_line_length: %lf", min_line_length);
  
	nh_local_.param<double>("predict_distance", predict_distance, 0.1);
	line_feature_.set_predict_distance(predict_distance);
	ROS_DEBUG("predict_distance: %lf", predict_distance);

	nh_local_.param<int>("seed_line_points", seed_line_points, 6);
	line_feature_.set_seed_line_points(seed_line_points);
	ROS_DEBUG("seed_line_points: %d", seed_line_points);

  	nh_local_.param<int>("min_line_points", min_line_points, 12);
  	line_feature_.set_min_line_points(min_line_points);
  	ROS_DEBUG("min_line_points: %d", min_line_points);

  	ROS_DEBUG("*************************************");
}

}
