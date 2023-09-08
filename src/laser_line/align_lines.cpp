#include "laser_line/align_lines.h"
 
// 点到直线的距离
template <typename T>
T pointToLineDistance(const Eigen::Matrix<T, 2, 1> &p, const Line &line)
{
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
T pointToSegmentDistance(const Eigen::Matrix<T, 2, 1> &p, const Line &line)
{
    Eigen::Matrix<T, 2, 1> v = line.p2.template cast<T>() - line.p1.template cast<T>();
    Eigen::Matrix<T, 2, 1> w = p - line.p1.template cast<T>();

    T c1 = w.dot(v);
    if (c1 <= T(0))
        return T(-1); //(p - line.p1.template cast<T>()).norm();

    T c2 = v.dot(v);
    if (c2 <= c1)
        return T(-1); //(p - line.p2.template cast<T>()).norm();

    T b = c1 / c2;
    Eigen::Matrix<T, 2, 1> projection = line.p1.template cast<T>() + b * v;
    return (p - projection).norm();
}

struct AngleDifferenceCostFunction
{
    AngleDifferenceCostFunction(double angle1, double angle2, double w)
        : angle1_(angle1), angle2_(angle2), w_(w) {}

    template <typename T>
    bool operator()(const T *const angle_offset, T *residual) const
    {
        T adjusted_angle = angle1_ + *angle_offset;
        T diff = adjusted_angle - T(angle2_);
        if (diff > T(M_PI_2))
        {
            diff -= M_PI;
        }
        else if (diff < T(-M_PI_2))
        {
            diff += M_PI;
        }
        if (diff < T(0.0))
            diff *= T(-1.0);
        residual[0] = diff * T(w_); // adjusted_angle - T(angle2_);
        return true;
    }

private:
    double angle1_, angle2_, w_;
};

AlignLine::AlignLine() : nh("~"), is_sub_sensors_(true), is_show_markers_(true)
{
    extract_line_from_map_.setSubSensor(is_sub_sensors_);
    extract_line_from_map_.setShowMarker(is_show_markers_);    
    extract_line_from_scan_.setSubSensor(is_sub_sensors_);
    extract_line_from_scan_.setShowMarker(is_show_markers_);  

    if(is_sub_sensors_)
    {
        laser_lines_sub = nh.subscribe("/laser_lines", 10, &AlignLine::laserLinesCallback, this);
        map_lines_sub = nh.subscribe("/map_lines", 10, &AlignLine::mapLinesCallback, this);        
    }

    if(is_show_markers_)
        matched_lines_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/matched_lines", 10);
}

void AlignLine::setSubSensor(bool flag)
{
    is_sub_sensors_ = flag;
    extract_line_from_map_.setSubSensor(is_sub_sensors_);
    extract_line_from_scan_.setSubSensor(is_sub_sensors_);
    LOG(INFO)<<"setSubSensor:"<<(int)flag;   
}

void AlignLine::setShowMarker(bool flag)
{
    is_show_markers_ = flag;
    extract_line_from_map_.setShowMarker(is_show_markers_);    
    extract_line_from_scan_.setShowMarker(is_show_markers_);    
    LOG(INFO)<<"setShowMarker:"<<(int)flag;      
}
 
void AlignLine::extractLineFromMap(const nav_msgs::OccupancyGrid& map_msg)
{
    laser_line::Lines lines_msg = extract_line_from_map_.extractLine(map_msg);
    map_lines_.clear();
    for (size_t i = 0; i < lines_msg.lines.size(); i++)
    {
        Line l;
        l.p1 = Eigen::Vector2d(lines_msg.lines[i].p1.x, lines_msg.lines[i].p1.y);
        l.p2 = Eigen::Vector2d(lines_msg.lines[i].p2.x, lines_msg.lines[i].p2.y);
        map_lines_.push_back(l);
    }
}

void AlignLine::extractLineFromScan(const sensor_msgs::LaserScan& scan_msg)
{
    laser_line::Lines lines_msg = extract_line_from_scan_.extractLine(scan_msg);
    laser_lines_.clear();
    for (size_t i = 0; i < lines_msg.lines.size(); i++)
    {
        Line l;
        l.p1 = Eigen::Vector2d(lines_msg.lines[i].p1.x, lines_msg.lines[i].p1.y);
        l.p2 = Eigen::Vector2d(lines_msg.lines[i].p2.x, lines_msg.lines[i].p2.y);
        laser_lines_.push_back(l);
    }    
}

void AlignLine::extractLineFromScan(const sensor_msgs::LaserScan& scan_msg, const geometry_msgs::Pose2D& laser_pose)
{
    laser_line::Lines lines_msg = extract_line_from_scan_.extractLine(scan_msg, laser_pose);
    laser_lines_.clear();
    for (size_t i = 0; i < lines_msg.lines.size(); i++)
    {
        Line l;
        l.p1 = Eigen::Vector2d(lines_msg.lines[i].p1.x, lines_msg.lines[i].p1.y);
        l.p2 = Eigen::Vector2d(lines_msg.lines[i].p2.x, lines_msg.lines[i].p2.y);
        laser_lines_.push_back(l);
    }    
}

bool AlignLine::alignProcess()
{
    bool align = false;
    if (!map_lines_.empty())
        align = processOptimization();

    return  align;  
}

double AlignLine::getAlignAngle()
{
    return align_angle_;
}

void AlignLine::laserLinesCallback(const laser_line::Lines::ConstPtr &msg)
{
    laser_lines_.clear();
    for (size_t i = 0; i < msg->lines.size(); i++)
    {
        Line l;
        l.p1 = Eigen::Vector2d(msg->lines[i].p1.x, msg->lines[i].p1.y);
        l.p2 = Eigen::Vector2d(msg->lines[i].p2.x, msg->lines[i].p2.y);
        laser_lines_.push_back(l);
    }

    if (!map_lines_.empty())
        processOptimization();
}

void AlignLine::mapLinesCallback(const laser_line::Lines::ConstPtr &msg)
{
    map_lines_.clear();
    for (size_t i = 0; i < msg->lines.size(); i++)
    {
        Line l;
        l.p1 = Eigen::Vector2d(msg->lines[i].p1.x, msg->lines[i].p1.y);
        l.p2 = Eigen::Vector2d(msg->lines[i].p2.x, msg->lines[i].p2.y);
        map_lines_.push_back(l);
    }
}

std::vector<Line> AlignLine::selectLongestLines(const std::vector<Line> &lines, int n)
{
    // Assuming 'lines' is populated
    std::vector<std::pair<double, Line>> length_and_lines;
    for (const auto &line : lines)
    {
        double length = (line.p1 - line.p2).norm();
        length_and_lines.push_back({length, line});
    }

    std::sort(length_and_lines.begin(), length_and_lines.end(),
              [](const std::pair<double, Line> &a, const std::pair<double, Line> &b)
              {
                  return a.first > b.first; // Sort in descending order
              });

    std::vector<Line> longest_lines;
    for (int i = 0; i < n && i < length_and_lines.size(); i++)
    {
        longest_lines.push_back(length_and_lines[i].second);
    }
    return longest_lines;
}

int AlignLine::matchLines(const std::vector<Line> &selected_lines, const std::vector<Line> &map_lines,
                            std::vector<std::pair<Line, Line>> &matched_lines, std::vector<double> &matched_score)
{
    for (int i = 0; i < selected_lines.size(); i++)
    {
        const Line &check_line = selected_lines[i];
        double dx = check_line.p2.x() - check_line.p1.x();
        double dy = check_line.p2.y() - check_line.p1.y();
        double check_length = std::sqrt(dx * dx + dy * dy);
        double check_angle = normalLineAngle(std::atan2(dy, dx));
        LOG(INFO) << "selected_lines_" << i << " len:" << check_length;

        double step = 0.1;
        int count = std::ceil(check_length / step);

        Eigen::Vector2d check_line_next;
        std::vector<std::pair<double, Line>> cost_and_lines;
        for (const auto &line : map_lines)
        {
            double lx = line.p2.x() - line.p1.x();
            double ly = line.p2.y() - line.p1.y();
            double line_angle = normalLineAngle(std::atan2(ly, lx));
            double l_length = std::sqrt(lx * lx + ly * ly);
            double angle_diff = check_angle - line_angle;
            // angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
            angle_diff = normalLineAngle(angle_diff);
            // if(std::abs(angle_diff) > 1.57 || l_length < 0.4)
            if (l_length < 0.3 || std::abs(angle_diff) > 60 * M_PI / 180.0)
                continue;

            double dis_cost = 0.0;
            int hit_count = 0;
            for (int c = 0; c < count; c++)
            {
                check_line_next(0) = check_line.p1.x() + step * c * dx / check_length;
                check_line_next(1) = check_line.p1.y() + step * c * dy / check_length;
                double d = pointToSegmentDistance<double>(check_line_next, line);
                if (d < 0 || d > 1.25)
                    continue;

                hit_count++;
                dis_cost += d;
            }

            dis_cost = dis_cost / hit_count;
            if (hit_count > 0.4 * count && hit_count > 4 && dis_cost < 1.0)
            {
                cost_and_lines.push_back({dis_cost, line});
            }
        }

        if (!cost_and_lines.empty())
        {
            std::sort(cost_and_lines.begin(), cost_and_lines.end(),
                      [](const std::pair<double, Line> &a, const std::pair<double, Line> &b)
                      {
                          return a.first > b.first; // Sort in descending order
                      });

            LOG(INFO) << " matched_lines: i" << i << " check_length:" << check_length << "  aver_dis_cost:" << cost_and_lines.back().first;
            matched_lines.push_back(std::make_pair(check_line, cost_and_lines.back().second));
            matched_score.push_back(cost_and_lines.back().first);
        }
    }
    return matched_lines.size();
}

double AlignLine::calculateAngle(const Line &l)
{
    double radian = std::atan2(l.p2.y() - l.p1.y(), l.p2.x() - l.p1.x());
    // LOG(INFO)<<"raw_radian:"<<radian;

    if (radian > M_PI_2)
    {
        radian -= M_PI;
    }
    else if (radian < -M_PI_2)
    {
        radian += M_PI;
    }
    // LOG(INFO)<<"new_radian:"<<radian;
    return radian;
}

double AlignLine::normalLineAngle(const double angle)
{
    double radian = angle;

    if (radian > M_PI_2)
    {
        radian -= M_PI;
    }
    else if (radian < -M_PI_2)
    {
        radian += M_PI;
    }

    return radian;
}

bool AlignLine::processOptimization()
{
    auto longest_lines = selectLongestLines(laser_lines_, 3);
    LOG(INFO) << "Select longest_lines:" << longest_lines.size();

    std::vector<std::pair<Line, Line>> matched_lines;
    std::vector<double> matched_score;
    int match_size = matchLines(longest_lines, map_lines_, matched_lines, matched_score);
    LOG(INFO) << "->matched_lines:" << matched_lines.size();
    if(matched_lines_marker_pub_)
        publishLinesAsMarkers(matched_lines_marker_pub_, matched_lines);

    double angle_offset = 0.0;
    float sum_cost = std::accumulate(matched_score.begin(), matched_score.end(), 0.0);
    std::vector<double> weights;
    for (int i = 0; i < matched_score.size(); i++)
    {
        weights.push_back(1 - matched_score[i] / sum_cost + 0.68);
    }

    ceres::Problem problem;
    for (int i = 0; i < matched_lines.size(); i++)
    {
        LOG(INFO) << "  i:" << i << "laser_line p1:" << matched_lines[i].first.p1.x() << " " << matched_lines[i].first.p1.y() << " p2:" << matched_lines[i].first.p2.x() << " " << matched_lines[i].first.p2.y();
        LOG(INFO) << "  i:" << i << "map_line p1:" << matched_lines[i].second.p1.x() << " " << matched_lines[i].second.p1.y() << " p2:" << matched_lines[i].second.p2.x() << " " << matched_lines[i].second.p2.y();

        double angle1 = calculateAngle(matched_lines[i].first);
        double angle2 = calculateAngle(matched_lines[i].second);
        double angle_diff = normalLineAngle(angle2 - angle1);
        LOG(INFO) << "  i:" << i << " scan_angle1:" << angle1 << " map_angle2:" << angle2 << " diff:" << angle_diff << " angle:" << angle_diff * 180.0 / M_PI << std::endl;
        if (std::abs(angle_diff) < 60 * M_PI / 180.0)
        {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<AngleDifferenceCostFunction, 1, 1>(
                    new AngleDifferenceCostFunction(angle1, angle2, weights[i])),
                nullptr,
                &angle_offset);
            LOG(INFO) << "  AddResidualBlock: add";
        }
        else
        {
            LOG(WARNING) << "  AddResidualBlock: Skip";
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false; // true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << "===>>Estimated transformation, angle_offset: " << angle_offset << " angle:" << angle_offset * 180.0 / M_PI << std::endl;

    bool has_aligned = true;
    // recheck_matched_line
    double rechecked_diff_angle = 0.0;
    for (int i = 0; i < matched_lines.size(); i++)
    {
        double angle1 = calculateAngle(matched_lines[i].first) + angle_offset;
        double angle2 = calculateAngle(matched_lines[i].second);
        double diff = normalLineAngle(angle2 - angle1);
        rechecked_diff_angle += std::abs(diff);
    }

    LOG(INFO) << " Recheck_diff_angele: rechecked_diff_angle:" << rechecked_diff_angle;
    LOG(WARNING) << "=============================================================";
    if (matched_lines.size() > 0 && std::abs(rechecked_diff_angle) < 10 * M_PI / 180.0)
    {
        LOG(INFO) << "==>> Matched Success:" << rechecked_diff_angle;
        align_angle_ = angle_offset;
        has_aligned = true;                
    }
    else
    {
        LOG(WARNING) << "==>> Matched failed:" << rechecked_diff_angle;
        has_aligned = false;
        align_angle_ = 0.0;
    }
    LOG(WARNING) << "------------------------------------------------------";

    return  has_aligned;
}

visualization_msgs::Marker AlignLine::lineToMarker(const Line &line, int id, const std::string &ns, const std::array<float, 4> &color, const std::string &frame_id)
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
    p1.z = id * 0.2; // assuming 2D

    p2.x = line.p2.x();
    p2.y = line.p2.y();
    p2.z = id * 0.2; // assuming 2D

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    marker.scale.x = 0.05; // width of the line
    marker.color.r = color[0] * (0.5 + id * 0.5);
    marker.color.g = color[1] * (0.5 + id * 0.5);
    marker.color.b = color[2];
    marker.color.a = color[3]; // transparency

    marker.lifetime = ros::Duration(0.5); // Duration marker is alive. Change as needed.

    return marker;
}

void AlignLine::publishLinesAsMarkers(const ros::Publisher &marker_pub, const std::vector<std::pair<Line, Line>> &matched_lines)
{
    visualization_msgs::MarkerArray markers;
    for (size_t i = 0; i < matched_lines.size(); i++)
    {
        markers.markers.push_back(lineToMarker(matched_lines[i].first, i, "matched_laser_lines", {1.0, 1.0, 0.0, 1.0}, "map"));
        markers.markers.push_back(lineToMarker(matched_lines[i].second, i, "matched_map_lines", {1.0, 1.0, 0.0, 1.0}, "map"));
    }
    marker_pub.publish(markers);
}
 