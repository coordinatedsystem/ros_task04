#include <rclcpp/rclcpp.hpp>
#include "hik_camera/hik_camera_driver.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("hik_camera");
    
    HikCameraDriver driver(node);

    if (!driver.initialize()) {
        RCLCPP_WARN(node->get_logger(), "Camera initialization failed at startup. "
                    "Driver will attempt to reconnect automatically.");
    } else {
        RCLCPP_INFO(node->get_logger(), "Camera initialized at startup.");
        if (!driver.startCapture()) {
            RCLCPP_WARN(node->get_logger(), "Failed to start capture at startup. "
                        "Will retry in reconnect loop.");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Hikvision camera node running... "
                "Waiting for camera connection.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
