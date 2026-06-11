#ifndef SLAM_HPP
#define SLAM_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/empty.hpp>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cctype>

namespace husky_mapping {
struct Particle {
    double x, y, theta;
    double weight;
};

class SLAM : public rclcpp::Node {
public:
    SLAM();
    virtual ~SLAM();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void initMap();
    Particle optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void publishMap(const rclcpp::Time& stamp);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    void saveMap();

    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;

    Particle odom_pose_;          
    Particle current_slam_pose_; 
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<int> map_counts_; 
    
    bool odom_initialized_ = false;
    bool map_initialized_ = false;

    const double map_resolution_ = 0.01; // cm per pixel
    const int map_width_ = 1000;          
    const int map_height_ = 1000;         
    const double map_origin_x_ = -5.0;  // Center the (0,0) world point
    const double map_origin_y_ = -5.0;
    const std::string filename_ = "husky_map";
};

}
#endif //SLAM_HPP
