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

#define BACKWARD_HAS_DW 1
#include "backward.hpp"

namespace backward{
    backward::SignalHandling sh;
}

struct Line {
    Eigen::Vector2d p1, p2;
};

//点到直线的距离
template <typename T>
T pointToLineDistance(const Eigen::Matrix<T, 2, 1> &p, const Line &line) {
    Eigen::Matrix<T, 2, 1> v = line.p2.template cast<T>() - line.p1.template cast<T>();
    Eigen::Matrix<T, 2, 1> w = p - line.p1.template cast<T>();

    T c1 = w.dot(v);
    // if (c1 <= T(0)) return (p - line.p1.template cast<T>()).norm();

    T c2 = v.dot(v);
    // if (c2 <= c1) return (p - line.p2.template cast<T>()).norm();

    T b = c1 / c2;
    Eigen::Matrix<T, 2, 1> projection = line.p1.template cast<T>() + b * v;
    return (p - projection).norm();
}


// // 点到线段的距离
template <typename T>
T pointToSegmentDistance(const Eigen::Matrix<T, 2, 1> &p, const Line &line) {
    Eigen::Matrix<T, 2, 1> v = line.p2.template cast<T>() - line.p1.template cast<T>();
    Eigen::Matrix<T, 2, 1> w = p - line.p1.template cast<T>();

    T c1 = w.dot(v);
    if (c1 <= T(0)) return T(-1);//(p - line.p1.template cast<T>()).norm();

    T c2 = v.dot(v);
    if (c2 <= c1) return T(-1);//(p - line.p2.template cast<T>()).norm();

    T b = c1 / c2;
    Eigen::Matrix<T, 2, 1> projection = line.p1.template cast<T>() + b * v;
    return (p - projection).norm();
}

struct AngleDifferenceCostFunction {
    AngleDifferenceCostFunction(double angle1, double angle2, double w) 
        : angle1_(angle1), angle2_(angle2), w_(w) {}

    template <typename T>
    bool operator()(const T* const angle_offset, T* residual) const {
        T adjusted_angle = angle1_ + *angle_offset;
        T diff = adjusted_angle - T(angle2_);
        if (diff > T(M_PI_2)) {
            diff -= M_PI;
        } else if (diff < T(-M_PI_2)) {
            diff += M_PI;
        }
        if(diff < T(0.0))
            diff *= T(-1.0);
        residual[0] = diff*T(w_);//adjusted_angle - T(angle2_);
        return true;
    }

private:
    double angle1_, angle2_, w_;
};
 
class LineMatcher
{
private:
    ros::Publisher matched_lines_marker_pub_;
    ros::Subscriber laser_lines_sub;
    ros::Subscriber map_lines_sub;
    ros::NodeHandle nh;

    std::vector<Line> laser_lines;
    std::vector<Line> map_lines;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_bro_;
public:
    LineMatcher() : nh("~") 
    {
        laser_lines_sub = nh.subscribe("/laser_lines", 10, &LineMatcher::laserLinesCallback, this);
        map_lines_sub = nh.subscribe("/map_lines", 10, &LineMatcher::mapLinesCallback, this);

        matched_lines_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/matched_lines", 10);
    }

    void laserLinesCallback(const laser_line::Lines::ConstPtr& msg)
    {
        laser_lines.clear();
        for(size_t i = 0; i < msg->lines.size(); i++)
        {
            Line l;
            l.p1 = Eigen::Vector2d(msg->lines[i].p1.x, msg->lines[i].p1.y);
            l.p2 = Eigen::Vector2d(msg->lines[i].p2.x, msg->lines[i].p2.y);
            laser_lines.push_back(l);
        }

        if(!map_lines.empty())
            processOptimization();
    }

    void mapLinesCallback(const laser_line::Lines::ConstPtr& msg)
    {
        map_lines.clear();
        for(size_t i = 0; i < msg->lines.size(); i++)
        {
            Line l;
            l.p1 = Eigen::Vector2d(msg->lines[i].p1.x, msg->lines[i].p1.y);
            l.p2 = Eigen::Vector2d(msg->lines[i].p2.x, msg->lines[i].p2.y);
            map_lines.push_back(l);
        }
    }

    std::vector<Line> selectLongestLines(const std::vector<Line>& lines, int n)
    {
        // Assuming 'lines' is populated
        std::vector<std::pair<double, Line>> length_and_lines;
        for(const auto& line : lines) 
        {
            double length = (line.p1 - line.p2).norm();
            length_and_lines.push_back({length, line});
        }

        std::sort(length_and_lines.begin(), length_and_lines.end(), 
            [](const std::pair<double, Line>& a, const std::pair<double, Line>& b) {
                return a.first > b.first;  // Sort in descending order
            });

        std::vector<Line> longest_lines;
        for(int i = 0; i < n && i < length_and_lines.size(); i++) 
        {
            longest_lines.push_back(length_and_lines[i].second);
        }
        return longest_lines;
    }

    int matchLines(const std::vector<Line>& selected_lines, const std::vector<Line>& map_lines, std::vector<std::pair<Line, Line>>& matched_lines, std::vector<double>& matched_score)
    {
        for(int i = 0; i < selected_lines.size(); i++) 
        {
            const Line& check_line = selected_lines[i];
            double dx = check_line.p2.x() - check_line.p1.x();
            double dy = check_line.p2.y() - check_line.p1.y();
            double check_length = std::sqrt(dx*dx + dy*dy);
            double check_angle = normalLineAngle(std::atan2(dy, dx));
            LOG(INFO)<<"selected_lines_"<<i<<" len:"<<check_length;

            double step = 0.1;
            int count = std::ceil(check_length/step);

            Eigen::Vector2d check_line_next;
            std::vector<std::pair<double, Line>> cost_and_lines;
            for(const auto& line : map_lines) 
            {
                double lx = line.p2.x() - line.p1.x();
                double ly = line.p2.y() - line.p1.y();
                double line_angle = normalLineAngle(std::atan2(ly,lx ));
                double l_length = std::sqrt(lx*lx + ly*ly);
                double angle_diff = check_angle - line_angle;
                //angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
                angle_diff = normalLineAngle(angle_diff);
                //if(std::abs(angle_diff) > 1.57 || l_length < 0.4)
                if(l_length < 0.3 || std::abs(angle_diff) > 60*M_PI/180.0)                
                    continue;
                
                double dis_cost = 0.0;
                int hit_count = 0;
                for(int c=0; c<count; c++)
                {
                    check_line_next(0) = check_line.p1.x() + step*c*dx/check_length;
                    check_line_next(1) = check_line.p1.y() + step*c*dy/check_length;  
                    double d = pointToSegmentDistance<double>(check_line_next, line);
                    if(d < 0 || d > 1.25)
                        continue;
                    
                    hit_count++;
                    dis_cost += d;
                }
                
                dis_cost = dis_cost/hit_count;
                if(hit_count > 0.4* count && hit_count > 4 && dis_cost < 1.0)
                {
                    cost_and_lines.push_back({dis_cost, line});                    
                }
            }
            
            if(!cost_and_lines.empty())
            {
                std::sort(cost_and_lines.begin(), cost_and_lines.end(), 
                    [](const std::pair<double, Line>& a, const std::pair<double, Line>& b) {
                        return a.first > b.first;  // Sort in descending order
                    });

                LOG(INFO)<<" matched_lines: i"<<i<<" check_length:"<<check_length<<"  aver_dis_cost:"<<cost_and_lines.back().first;
                matched_lines.push_back(std::make_pair(check_line, cost_and_lines.back().second)); 
                matched_score.push_back(cost_and_lines.back().first);
            }
        }
        return matched_lines.size();
    }

    double calculateAngle(const Line&  l)
    {
        double radian = std::atan2(l.p2.y() - l.p1.y(), l.p2.x() - l.p1.x());
        //LOG(INFO)<<"raw_radian:"<<radian;

        if (radian > M_PI_2) {
            radian -= M_PI;
        } else if (radian < -M_PI_2) {
            radian += M_PI;
        }
        //LOG(INFO)<<"new_radian:"<<radian;
        return radian;
    }

    double normalLineAngle(const double  angle)
    {
        double radian = angle;

        if (radian > M_PI_2) {
            radian -= M_PI;
        } else if (radian < -M_PI_2) {
            radian += M_PI;
        }
        
        return radian;
    }

    void processOptimization()
    {
        auto longest_lines = selectLongestLines(laser_lines, 3);
        LOG(INFO)<<"longest_lines:"<<longest_lines.size();
        std::vector<std::pair<Line, Line>> matched_lines;
        std::vector<double> matched_score;        
        int match_size = matchLines(longest_lines, map_lines, matched_lines, matched_score);        
        LOG(INFO)<<"->matched_lines:"<<matched_lines.size();

        publishLinesAsMarkers(matched_lines_marker_pub_,matched_lines);  

        //double transformation[3] = {0.0, 0.0, 0.0};  // no rotation, no translation
        double angle_offset = 0.0;

        ceres::Problem problem;
        float sum_cost = std::accumulate(matched_score.begin(), matched_score.end(), 0.0);
        std::vector<double> weights;   
        for(int i=0 ; i< matched_score.size(); i++)
        {
            weights.push_back(1- matched_score[i]/sum_cost + 0.68);
        }

        for(int i=0 ; i< matched_lines.size(); i++)
        {
            LOG(INFO)<<"  i:"<<i<<"laser_line p1:" << matched_lines[i].first.p1.x()<<" "<<matched_lines[i].first.p1.y()<<" p2:"<<matched_lines[i].first.p2.x()<<" "<<matched_lines[i].first.p2.y();            
            LOG(INFO)<<"  i:"<<i<<"map_line p1:" << matched_lines[i].second.p1.x()<<" "<<matched_lines[i].second.p1.y()<<" p2:"<<matched_lines[i].second.p2.x()<<" "<<matched_lines[i].second.p2.y();            

            double angle1 = calculateAngle(matched_lines[i].first);
            double angle2 =  calculateAngle(matched_lines[i].second);
            double angle_diff = normalLineAngle(angle2 - angle1);
            LOG(INFO)<<"  i:"<<i<<" scan_angle1:"<<angle1<<" map_angle2:"<<angle2<<" diff:"<<angle_diff<<" angle:"<<angle_diff*180.0/M_PI<< std::endl;   
            if(std::abs(angle_diff) < 60*M_PI/180.0 )
            {
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<AngleDifferenceCostFunction, 1, 1>(
                        new AngleDifferenceCostFunction(angle1, angle2, weights[i])),
                    nullptr,
                    &angle_offset);
                LOG(INFO)<<"  AddResidualBlock: add";              
            }
            else
            {
                LOG(WARNING)<<"  AddResidualBlock: Skip";  
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;//true;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        //std::cout << summary.FullReport() << "\n";
        // std::cout << "Estimated transformation : " << "Rotation (in radians): " << transformation[0] 
        //         << ", Translation: [" << transformation[1] << ", " << transformation[2] << "]"  << std::endl;
        LOG(INFO) << "===>>Estimated transformation, angle_offset: " <<angle_offset <<" angle:"<<angle_offset*180.0/M_PI<< std::endl;   


        //recheck_matched_line
        double diff_angle = 0.0;
        for(int i=0 ; i< matched_lines.size(); i++)
        {
            double angle1 = calculateAngle(matched_lines[i].first) + angle_offset;
            double angle2 =  calculateAngle(matched_lines[i].second) ;
            double diff = normalLineAngle(angle2 - angle1);
            diff_angle += std::abs(diff);
            LOG(INFO)<<" Recheck_diff_angele: i: "<<i<<" diff:"<<diff;   
        }
        LOG(INFO)<<" Recheck_diff_angele: diff_angle:"<<diff_angle;   
        LOG(WARNING)<<"============================================================="; 
        if(std::abs(diff_angle) > 10*M_PI/180.0)
        {
            LOG(WARNING)<<"==>> Matched failed:"<<diff_angle; 

            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(0, 0, 0.0); 
            geometry_msgs::Quaternion scan_quat = tf2::toMsg(tf2_quat);

            geometry_msgs::TransformStamped scan_trans;
            scan_trans.header.stamp = ros::Time::now();
            scan_trans.header.frame_id = "laser";
            scan_trans.child_frame_id = "matched_laser";

            scan_trans.transform.translation.x = 0.0;
            scan_trans.transform.translation.y = 0.0;
            scan_trans.transform.translation.z = 0.0;
            scan_trans.transform.rotation = scan_quat;
            tf_bro_.sendTransform(scan_trans);            
        }
        else
        {
            LOG(INFO)<<"==>> Matched Success:"<<diff_angle; 
            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(0, 0, angle_offset);  // Roll and Pitch are 0, Yaw is angle_offset
            geometry_msgs::Quaternion scan_quat = tf2::toMsg(tf2_quat);

            geometry_msgs::TransformStamped scan_trans;
            scan_trans.header.stamp = ros::Time::now();
            scan_trans.header.frame_id = "laser";
            scan_trans.child_frame_id = "matched_laser";

            scan_trans.transform.translation.x = 0.0;
            scan_trans.transform.translation.y = 0.0;
            scan_trans.transform.translation.z = 0.0;
            scan_trans.transform.rotation = scan_quat;
            tf_bro_.sendTransform(scan_trans);
        }
        LOG(WARNING)<<"------------------------------------------------------";         
    }


    visualization_msgs::Marker lineToMarker(const Line &line, int id, const std::string &ns, const std::array<float, 4> &color, const std::string &frame_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id; // or any other frame you are using
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::Point p1, p2;
        p1.x = line.p1.x();
        p1.y = line.p1.y();
        p1.z = id*0.2; // assuming 2D

        p2.x = line.p2.x();
        p2.y = line.p2.y();
        p2.z = id*0.2; // assuming 2D

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.scale.x = 0.05; // width of the line
        marker.color.r = color[0]*(0.5+id*0.5);
        marker.color.g = color[1]*(0.5+id*0.5);
        marker.color.b = color[2];
        marker.color.a = color[3]; // transparency

        marker.lifetime = ros::Duration(0.5); // Duration marker is alive. Change as needed.

        return marker;
    }

    void publishLinesAsMarkers(const ros::Publisher& marker_pub, const std::vector<std::pair<Line, Line>> & matched_lines)
    {
        visualization_msgs::MarkerArray markers;
        for (size_t i = 0; i < matched_lines.size(); i++) {
            markers.markers.push_back(lineToMarker(matched_lines[i].first, i, "matched_laser_lines", {1.0, 1.0, 0.0, 1.0}, "map"));
            markers.markers.push_back(lineToMarker(matched_lines[i].second, i, "matched_map_lines", {1.0, 1.0, 0.0, 1.0}, "map"));
        }
        marker_pub.publish(markers);
    }
};

int main(int argc, char** argv)
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
