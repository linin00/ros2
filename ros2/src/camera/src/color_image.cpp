/*** 
 * @Author: linin00
 * @Date: 2022-11-24 08:30:42
 * @LastEditTime: 2022-11-24 08:34:03
 * @LastEditors: linin00
 * @Description: 
 * @FilePath: /ros2/ros2/src/camera/src/color_image.cpp
 * @
 */
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
static void handler(sensor_msgs::msg::Image::SharedPtr msg) {
  printf("[YDLIDAR INFO]: I get a msg:\n height:%d, width:%d\n", msg.get()->height, msg.get()->width);
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handler");
  auto suber = node->create_subscription<sensor_msgs::msg::Image> (
    "/camera/color/image_raw",
    rclcpp::SensorDataQoS(),
    handler
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
