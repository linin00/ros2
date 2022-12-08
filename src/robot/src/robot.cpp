#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;
class robot {
private:
  std::string device_name = "can0"; 
  std::unique_ptr<ScoutRobot> scout;
  ProtocolDetector detector;
  bool is_scout_mini = false;
public:
  robot() {
    if (detector.Connect(device_name)) {
      auto proto = detector.DetectProtocolVersion(5);
      if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        scout = std::unique_ptr<ScoutRobot>(
            new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
      } else if (proto == ProtocolVersion::AGX_V2) {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        scout = std::unique_ptr<ScoutRobot>(
            new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
      } else {
        std::cout << "Detected protocol: UNKONWN" << std::endl;
        exit(1);
      }
    } else {
      exit(1);
    }
    if (scout == nullptr)
      std::cout << "Failed to create robot object" << std::endl;

    scout->Connect(device_name);

    if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
      scout->EnableCommandedMode();
    }
  }
  void SetMotionCommand(double linear, double angular) {
    scout->SetMotionCommand(linear, angular);
  }
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value, AgxLightMode r_mode, uint8_t r_value) {
    scout->SetLightCommand(f_mode, f_value, r_mode, r_value);
  }
  void PrintStatus() {
    auto state = scout->GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "control mode: "
              << static_cast<int>(state.system_state.control_mode)
              << " , vehicle state: "
              << static_cast<int>(state.system_state.vehicle_state)
              << " , error code: " << std::hex << state.system_state.error_code
              << std::dec
              << ", battery voltage: " << state.system_state.battery_voltage
              << std::endl;
    std::cout << "velocity (linear, angular): "
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << std::endl;
    std::cout << "core state age (ms): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                AgxMsgRefClock::now() - state.time_stamp)
                .count()
              << std::endl;

    auto actuator = scout->GetActuatorState();
    if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
      for (int i = 0; i < 4; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
          actuator.actuator_state[i].motor_id,
          actuator.actuator_state[i].current,
          actuator.actuator_state[i].rpm,
          actuator.actuator_state[i].driver_temp,
          actuator.actuator_state[i].motor_temp);
      }
      std::cout << "actuator state age (ms): "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                  AgxMsgRefClock::now() - actuator.time_stamp)
                  .count()
                << std::endl;
    } else {
      for (int i = 0; i < 4; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
          actuator.actuator_hs_state[i].motor_id,
          actuator.actuator_hs_state[i].current,
          actuator.actuator_hs_state[i].rpm,
          actuator.actuator_ls_state[i].driver_temp,
          actuator.actuator_ls_state[i].motor_temp);
      }
      std::cout << "actuator state age (ms): "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                    AgxMsgRefClock::now() - actuator.time_stamp)
                    .count()
                << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
  }
};



static robot bot;

static void handler(geometry_msgs::msg::Twist::SharedPtr msg) {
  bot.SetMotionCommand(msg.get()->linear.x, msg.get()->angular.z);
  bot.PrintStatus();
}

int main(int argc, char ** argv) {
  // std::cout << "Light: const off" << std::endl;
  // bot.SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
  // sleep(3);
  // std::cout << "Light: const on" << std::endl;
  // bot.SetLightCommand(CONST_ON, 0, CONST_ON, 0);
  // sleep(3);
  // std::cout << "Light: breath" << std::endl;
  // bot.SetLightCommand(BREATH, 0, BREATH, 0);
  // sleep(3);
  // std::cout << "Light: custom 30-40" << std::endl;
  // bot.SetLightCommand(CUSTOM, 30, CUSTOM, 40);
  // sleep(3);
  // bot.SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robot");
  auto suber = node->create_subscription<geometry_msgs::msg::Twist> (
    "/cmd_vel",
    rclcpp::SensorDataQoS(),
    handler
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}