/*** 
 * @Author: linin00
 * @Date: 2022-11-24 07:19:10
 * @LastEditTime: 2022-11-24 07:44:03
 * @LastEditors: linin00
 * @Description: 
 * @FilePath: /ros2/ros2/src/camera/src/listener.cpp
 * @
 */
#include "realsense2_camera_msgs/msg/metadata.hpp"
#include "rclcpp/rclcpp.hpp"
static void handler(realsense2_camera_msgs::msg::Metadata::SharedPtr msg) {
  printf("[YDLIDAR INFO]: I get a msg: %s\n", msg->json_data.c_str());
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handler");
  auto suber = node->create_subscription<realsense2_camera_msgs::msg::Metadata> (
    "/camera/color/metadata",
    rclcpp::SensorDataQoS(),
    handler
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}