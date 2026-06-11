#include "husky_localisation/amcl.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<husky_amcl::AMCL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
