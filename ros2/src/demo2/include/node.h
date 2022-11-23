#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include <chrono>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node {
public:
    TalkerNode(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "%s initialized!", name.c_str());
        this->talk = this->create_publisher<std_msgs::msg::String>("test", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TalkerNode::timer_callback, this));
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr talk;
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "test");
        std_msgs::msg::String msg;
        msg.data = "msg";
        this->talk->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

class ListenerNode : public rclcpp::Node {
public:
    ListenerNode(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "%s initialized!", name.c_str());
        this->listen = this->create_subscription<std_msgs::msg::String>("test", 10, std::bind(&ListenerNode::topic_callback, this, _1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listen;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Recieved：'%s'", msg->data.c_str());
    }
};