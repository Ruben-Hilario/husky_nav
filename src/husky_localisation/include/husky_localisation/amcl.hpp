#Ifndef AMCL_HPP
#define AMCL_HPP
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

namespace husky_amcl {
    struct Particle {
    double x, y, theta;
    double weight;
};

class AMCL: public rclcpp::Node {
public:
    AMCL();
    ~AMCL() = default;
private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void initializeParticlesGlobal();
    void resampleParticles();
    void estimateRobotPose();
    Particle optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    void markPoseOnMap(const Particle& pose, int pixel_half_size = 15);

    void publishParticles(const rclcpp::Time& stamp);
    void publishEstimatedPose(const rclcpp::Time& stamp);
    void publishMapToOdomTransform(const rclcpp::Time& stamp);
    
    nav_msgs::msg::OccupancyGrid load_map_from_file(const std::string& yaml_path);
    void publishMap();
    void publishMap(const nav_msgs::msg::OccupancyGrid& map);
    void computeDistanceField();

    void markPoseOnMap(nav_msgs::msg::OccupancyGrid& grid, const Particle& pose, int pixel_half_size = 15);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mt19937 gen_{std::random_device{}()};

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    nav_msgs::msg::OccupancyGrid map_;
    std::vector<Particle> particles_;
    Particle odom_pose_;
    Particle last_odom_pose_;
    Particle estimated_pose_;

    bool odom_initialized_ = false;
    bool map_initialized_ = false;
    bool particles_initialized_ = false;
    std::vector<float> dist_field_;
    
    const size_t num_particles_ = 5000; 
    const double linear_noise_ = 0.05; 
    const double angular_noise_ = 0.02;
    double distance_since_resample = 0.0;
    double angle_since_resample = 0.0;
    const double RESAMPLE_DIST_THRESHOLD = 0.15; // 15 cm
    const double RESAMPLE_ANG_THRESHOLD = 0.1;  // ~11 degrees
};
}
#endif //AMCL_HPP