extern "C"
{
    #include "lsd.h"
}

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "laser_line/Lines.h"
#include "laser_line/Line.h"

#include "glog/logging.h"


class MapAnalyzer
{
private:
    ros::Subscriber map_sub;
    ros::Publisher marker_pub, lines_pub_;
    ros::NodeHandle nh;

	laser_line::Lines lines_msg_;
public:
    MapAnalyzer() : nh("~")  // Using private NodeHandle
    {
        map_sub = nh.subscribe("/map", 10, &MapAnalyzer::mapCallback, this);
        marker_pub = nh.advertise<visualization_msgs::Marker>("/map_lines_marker", 10);
        lines_pub_ = nh.advertise<laser_line::Lines>("/map_lines", 10);		
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {
		// Handle the map data here  For now, just print out the map's metadata
		ROS_INFO("Received a map with width: %d, height: %d", map_msg->info.width, map_msg->info.height);
		LOG(INFO)<<"Received a map with width: "<<map_msg->info.width<<" height:"<<map_msg->info.height;

		image_double image = new_image_double(map_msg->info.width, map_msg->info.height);
		int xsize = image->xsize;
		int ysize = image->ysize;
		int y,x;
		for (y = 0;y < ysize;y++)
		{
			for (x = 0;x < xsize;x++)
			{
				image->data[y * xsize + x] = map_msg->data[y * xsize + x];
			}
		}
		ntuple_list detected_lines = lsd(image);
		free_image_double(image);
		LOG(INFO)<<"Finish lsd: "<<detected_lines->size;

		//pixel to meter
		{
			int nLines = detected_lines->size;	
			int dim = detected_lines->dim;
			for (int i = 0; i < nLines; i++)
			{
				float a1 = detected_lines->values[i*dim+0];
				float a2 = detected_lines->values[i*dim+1];
				float a3 = detected_lines->values[i*dim+2];
				float a4 = detected_lines->values[i*dim+3];
				if (sqrt( (a3-a1)*(a3-a1)+ (a4-a2)*(a4-a2) ) >= 12)
				{
					laser_line::Line line_msg;
					line_msg.header.frame_id = "map";
					line_msg.header.stamp = ros::Time::now();
					line_msg.p1.x = a1*map_msg->info.resolution + map_msg->info.origin.position.x;
					line_msg.p1.y = a2*map_msg->info.resolution + map_msg->info.origin.position.y;

					line_msg.p2.x = a3*map_msg->info.resolution + map_msg->info.origin.position.x;
					line_msg.p2.y = a4*map_msg->info.resolution + map_msg->info.origin.position.y;

					lines_msg_.lines.push_back(line_msg);					
				}	
			}
			lines_msg_.header.stamp = ros::Time::now();
			lines_msg_.header.frame_id = "map";
		}

		{
			lines_pub_.publish(lines_msg_);	
			LOG(INFO)<<"Finish lines_pub_";
			publishLinesMarker(lines_msg_);
			LOG(INFO)<<"Finish publishLinesMarker";			
		}
    }

	void  publishLinesMarker(const laser_line::Lines& lines_msg)
	{
		bool show_lines = true;
		if (show_lines)
		{
			visualization_msgs::Marker lines;
			lines.header.frame_id = "map";
			lines.header.stamp = ros::Time::now();
			lines.ns = "lines";
			lines.action = visualization_msgs::Marker::ADD;
			lines.pose.orientation.w = 1.0;
			lines.id = 0;
			lines.type = visualization_msgs::Marker::LINE_LIST;
        	lines.pose.orientation.w = 1.0;
			// LINE_LIST markers use only the x component of scale, for the line width
			lines.scale.x = 0.025;
			for (int i = 0; i < lines_msg.lines.size(); i++)
			{
				lines.points.push_back(lines_msg.lines[i].p1);
				lines.points.push_back(lines_msg.lines[i].p2);
			}

			// Set the color -- be sure to set alpha to something non-zero!
			lines.color.r = 1.0;
			lines.color.a = 1.0;

			marker_pub.publish(lines);
			//LOG(INFO)<<"Finish marker_pub ";
		}
	}	

	void loop()
	{
		lines_pub_.publish(lines_msg_);	
		//LOG(INFO)<<"Finish lines_pub_";
		publishLinesMarker(lines_msg_);
		//LOG(INFO)<<"Finish publishLinesMarker";			
	}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_line_from_map");

    MapAnalyzer analyzer;
	ros::Rate r(2);
	while (ros::ok())
	{
		analyzer.loop();
		r.sleep();
		ros::spinOnce();
	}
	

    //ros::spin();
    return 0;
}
