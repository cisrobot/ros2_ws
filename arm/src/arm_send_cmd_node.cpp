#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include "arm_send_cmd_node.hpp"


#define FAIL_MEASURE 0
#define USE_EGO_MEASURE 1
#define USE_OPP_MEASURE 2
#define USE_MERGE_MEASURE 3


using std::placeholders::_1;
using namespace std::chrono_literals;

std::string node_name = "arm_send_cmd_node";

ArmSendCmdNode::ArmSendCmdNode(): Node(node_name)
{
  std::string joint_arm_cmd_topic;
  this->declare_parameter("control_ms", rclcpp::ParameterValue(5));
  this->declare_parameter("joint_arm_cmd_topic", rclcpp::ParameterValue("joint_arm_cmd_topic"));
  this->declare_parameter("joint_cmd_pan", rclcpp::ParameterValue(1.57));
  this->declare_parameter("joint_cmd_tilt", rclcpp::ParameterValue(1.57));


  this->get_parameter("control_ms", control_ms_);
  this->get_parameter("joint_arm_cmd_topic", joint_arm_cmd_topic);
  this->get_parameter("joint_cmd_pan", joint_cmd_angs_[0]);
  this->get_parameter("joint_cmd_tilt", joint_cmd_angs_[1]);
  
  rclcpp::QoS custom_qos_profile(1);
  custom_qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  joint_state_cmd_ang_.name = {"joint_cmd_pan", "joint_cmd_tilt"};

  joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_arm_cmd_topic, custom_qos_profile);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(control_ms_),
    std::bind(&ArmSendCmdNode::controlCallback, this));

}

void ArmSendCmdNode::controlCallback()
{
  joint_state_cmd_ang_.position = {joint_cmd_angs_[0], joint_cmd_angs_[1]};
  joint_cmd_pub_->publish(joint_state_cmd_ang_);  
}



ArmSendCmdNode::~ArmSendCmdNode()
{

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto arm_send_ang_node = std::make_shared<ArmSendCmdNode>();
  rclcpp::spin(arm_send_ang_node);
  rclcpp::shutdown();
  return 0;
}

