/*** 
 * @Author: linin00
 * @Date: 2022-11-24 08:42:54
 * @LastEditTime: 2022-11-24 09:26:11
 * @LastEditors: linin00
 * @Description: 
 * @FilePath: /ros2/ros2/src/camera/src/depth_image.cpp
 * @
 */
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
static void handler(sensor_msgs::msg::Image::SharedPtr msg) {
  printf("[CAMERA INFO]: I get a depth image:\n height:%d, width:%d\n", msg.get()->height, msg.get()->width);
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handler");
  auto suber = node->create_subscription<sensor_msgs::msg::Image> (
    "/camera/depth/image_rect_raw",
    rclcpp::SensorDataQoS(),
    handler
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
