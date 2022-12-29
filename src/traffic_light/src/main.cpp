/*** 
 * @Author: linin00
 * @Date: 2022-12-29 08:54:04
 * @LastEditTime: 2022-12-29 09:50:29
 * @LastEditors: linin00
 * @Description: 
 * @FilePath: /ros2/src/traffic_light/src/main.cpp
 * @
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <stdio.h>
#include <string>
class TrafficLightPublisher : public rclcpp::Node {
public:
  TrafficLightPublisher() : Node("TrafficLightNode") {
		this->traffic_pub = this->create_publisher<std_msgs::msg::String>("/traffic", 20);
	};
	void pub(char* msg_c) {
		std_msgs::msg::String msg;
		if (!strcmp(msg_c, "r")) {
			// msg.set__data(msg_c);
			msg.set__data("red");
		} else if (!strcmp(msg_c, "g")) {
			msg.set__data("green");
		} else {
			printf("Unknown: %s\n", msg_c);
			return;
		}
		printf("publish:%s\n", msg.data.c_str());
		this->traffic_pub->publish(msg);
	}
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traffic_pub;
};

int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<TrafficLightPublisher>());
	TrafficLightPublisher test;
	while(true) {
		char input[100];
		scanf("%s", input);
		if (!strcmp(input, "q")) break;
		test.pub(input);
	}
	rclcpp::shutdown();
	return 0;
}
