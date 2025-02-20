#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class ArmSendCmdNode : public rclcpp::Node
{
public:
  ArmSendCmdNode();
  virtual ~ArmSendCmdNode();

private:
  int control_ms_;
  
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  sensor_msgs::msg::JointState joint_state_cmd_ang_;
  double joint_cmd_angs_[2];

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  
  void controlCallback();
};

#endif  // READ_WRITE_NODE_HPP_
