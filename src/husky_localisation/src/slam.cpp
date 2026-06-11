#include "husky_localisation/slam.hpp"

namespace husky_mapping {

SLAM::SLAM() : Node("custom_slam_node") {
    auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&SLAM::odomCallback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&SLAM::scanCallback, this, std::placeholders::_1));

    // map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    initMap();
    RCLCPP_INFO(this->get_logger(), "SLAM Node started");
}

SLAM::~SLAM() {
    RCLCPP_INFO(this->get_logger(), "Node destroying. Saving map");
    saveMap();
}

void SLAM::initMap() {
    map_.header.frame_id = "odom";
    map_.info.resolution = map_resolution_;
    map_.info.width = map_width_;
    map_.info.height = map_height_;
    map_.info.origin.position.x = map_origin_x_;
    map_.info.origin.position.y = map_origin_y_;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    map_.data.assign(map_width_ * map_height_, -1);
    map_counts_.assign(map_width_ * map_height_, 0);
    map_initialized_ = true;
}

void SLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;

    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;
    odom_pose_.theta = 2.0 * std::atan2(q_z, q_w);

    if (!odom_initialized_) {
        current_slam_pose_ = odom_pose_;
        odom_initialized_ = true;
    }
}

void SLAM::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!odom_initialized_ || !map_initialized_) return;

    // around dead reckoning pose to kill the drift/shaking
    Particle corrected_pose = optimizePoseByScanMatching(current_slam_pose_, msg);
    current_slam_pose_ = corrected_pose;
    updateMapOccupancy(corrected_pose, msg);
    publishMap(msg->header.stamp);
    publishMapToOdomTransform(msg->header.stamp);
}

Particle SLAM::optimizePoseByScanMatching(const Particle& predicted_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    Particle best_pose = predicted_pose;
    double best_score = -1.0;

    // Search window configuration (Tweak these to alter lookups)
    const double pos_step = 0.02; // 2 cm search resolution
    const double ang_step = 0.01; // ~0.5 degrees step
    const int search_range = 2;   // Evaluates combinations around base pose
    
    for (int x_idx = -search_range; x_idx <= search_range; ++x_idx) {
        for (int y_idx = -search_range; y_idx <= search_range; ++y_idx) {
            for (int t_idx = -search_range; t_idx <= search_range; ++t_idx) {
                
                Particle candidate;
                candidate.x = predicted_pose.x + (x_idx * pos_step);
                candidate.y = predicted_pose.y + (y_idx * pos_step);
                candidate.theta = predicted_pose.theta + (t_idx * ang_step);

                double score = 0.0;
                for (size_t i = 0; i < scan->ranges.size(); i += 5) { // Subsample to optimize speed
                    double r = scan->ranges[i];
                    if (r < scan->range_min || r > scan->range_max) continue;

                    double angle = scan->angle_min + i * scan->angle_increment;
                    double wx = candidate.x + r * std::cos(candidate.theta + angle);
                    double wy = candidate.y + r * std::sin(candidate.theta + angle);

                    int mx, my;
                    if (worldToMap(wx, wy, mx, my)) {
                        int index = my * map_width_ + mx;
                        if (map_.data[index] > 50) { // Hit a known cell!
                            score += 1.0;
                        }
                    }
                }

                if (score > best_score) {
                    best_score = score;
                    best_pose = candidate;
                }
            }
        }
    }
    
    // Fall back smoothly to internal prediction if tracking has no anchor overlap yet
    return (best_score > 2) ? best_pose : predicted_pose;
}

// void SLAM::updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
//     for (size_t i = 0; i < scan->ranges.size(); ++i) {
//         double r = scan->ranges[i];
//         if (r < scan->range_min || r > scan->range_max) continue;

//         double angle = scan->angle_min + i * scan->angle_increment;
//         // Project local observation safely into our absolute reference map coordinates
//         double wx = corrected_pose.x + r * std::cos(corrected_pose.theta + angle);
//         double wy = corrected_pose.y + r * std::sin(corrected_pose.theta + angle);

//         int mx, my;
//         if (worldToMap(wx, wy, mx, my)) {
//             int index = my * map_width_ + mx;
//             map_counts_[index]++;
//             // Artisanal occupancy filtering: if a cell gets hits, flag it permanently as occupied
//             if (map_counts_[index] >= 1) {
//                 map_.data[index] = 100; // 100 = Occupied
//             }
//         }
//     }
// }

void SLAM::updateMapOccupancy(const Particle& corrected_pose, const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
    int mx0, my0;
    if (!worldToMap(corrected_pose.x, corrected_pose.y, mx0, my0)) {
        return;
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        bool is_max_range = false;
        if (r > scan->range_max) {
            r = scan->range_max;
            is_max_range = true;
        } else if (r < scan->range_min) {
            continue; 
        }

        double angle = scan->angle_min + i * scan->angle_increment;
        double wx = corrected_pose.x + r * std::cos(corrected_pose.theta + angle);
        double wy = corrected_pose.y + r * std::sin(corrected_pose.theta + angle);

        int mx1, my1;
        if (!worldToMap(wx, wy, mx1, my1)) continue;

        // 2. Bresenham's 
        int dx = std::abs(mx1 - mx0);
        int dy = std::abs(my1 - my0);
        int sx = (mx0 < mx1) ? 1 : -1;
        int sy = (my0 < my1) ? 1 : -1;
        int err = dx - dy;

        int cx = mx0;
        int cy = my0;

        while (true) {
            if (cx == mx1 && cy == my1) {
                break;
            }

            int index = cy * map_width_ + cx;
            if (map_.data[index] == -1) {
                map_.data[index] = 0;
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                cx += sx;
            }
            if (e2 < dx) {
                err += dx;
                cy += sy;
            }
        }
        if (!is_max_range) {
            int target_index = my1 * map_width_ + mx1;
            map_counts_[target_index]++;
            
            if (map_counts_[target_index] >= 1) {
                map_.data[target_index] = 100;
            }
        }
    }
}

bool SLAM::worldToMap(double wx, double wy, int& mx, int& my) const {
    if (wx < map_origin_x_ || wy < map_origin_y_) return false;
    mx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    my = static_cast<int>((wy - map_origin_y_) / map_resolution_);
    return (mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_);
}

void SLAM::mapToWorld(int mx, int my, double& wx, double& wy) const {
    wx = map_origin_x_ + (mx + 0.5) * map_resolution_;
    wy = map_origin_y_ + (my + 0.5) * map_resolution_;
}

void SLAM::publishMap(const rclcpp::Time& stamp) {
    map_.header.stamp = stamp;
    map_pub_->publish(map_);
}

void SLAM::publishMapToOdomTransform(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = current_slam_pose_.x - odom_pose_.x;
    tf.transform.translation.y = current_slam_pose_.y - odom_pose_.y;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_slam_pose_.theta - odom_pose_.theta);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
}

void SLAM::saveMap() {
    std::string pgm_filename = filename_ + ".pgm";
    std::string yaml_filename = filename_ + ".yaml";

    // 1. SAVE THE PGM IMAGE
    std::ofstream pgm_f(pgm_filename, std::ios::out | std::ios::binary);
    if (!pgm_f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file for saving!");
        return;
    }
    pgm_f << "P5\n" << map_width_ << " " << map_height_ << "\n255\n";
    for (int y = map_height_ - 1; y >= 0; --y) {
        for (int x = 0; x < map_width_; ++x) {
            int idx = y * map_width_ + x;
            int cell = map_.data[idx];

            unsigned char pixel_val;
            if (cell == -1) {
                pixel_val = 205;
            } else if (cell == 100) {
                pixel_val = 0;
            } else {
                pixel_val = 254;
            }
            pgm_f.put(pixel_val);
        }
    }
    pgm_f.close();
    std::ofstream yaml_f(yaml_filename, std::ios::out);
    if (!yaml_f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for saving!");
        return;
    }

    yaml_f << "image: " << pgm_filename << "\n";
    yaml_f << "resolution: " << map_resolution_ << "\n";
    yaml_f << "origin: [" << map_origin_x_ << ", " << map_origin_y_ << ", 0.0, 0.0, 0.0, 0.0]\n";
    yaml_f << "negate: 0\n";
    yaml_f << "occupied_thresh: 0.65\n";
    yaml_f << "free_thresh: 0.196\n";

    yaml_f.close();
    RCLCPP_INFO(this->get_logger(), "Map saved successfully to %s and %s", pgm_filename.c_str(), yaml_filename.c_str());
}
}