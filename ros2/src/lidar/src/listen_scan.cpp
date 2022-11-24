/*** 
 * @Author: linin00
 * @Date: 2022-11-24 09:11:20
 * @LastEditTime: 2022-11-24 09:21:22
 * @LastEditors: linin00
 * @Description: 
 * @FilePath: /ros2/ros2/src/lidar/src/listen_scan.cpp
 * @
 */
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
static void handler(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  printf("[YDLIDAR INFO]: I get a scan:\n angle_min:%f, angle_max:%f, size of ranges: %lu\nsize of intensities: %lu\n", 
  msg.get()->angle_min, 
  msg.get()->angle_max,
  msg.get()->ranges.size(),
  msg.get()->intensities.size());
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handler");
  auto suber = node->create_subscription<sensor_msgs::msg::LaserScan> (
    "/scan",
    rclcpp::SensorDataQoS(),
    handler
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
