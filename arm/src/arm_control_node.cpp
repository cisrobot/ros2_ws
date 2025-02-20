#include <cstdio>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "arm_control_node.hpp"

#define DEBUG_PRINT 0
// #define DEBUG_PRINT 1

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_HOMING_OFFSET 20
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define ADDR_PROFILE_VELOCITY 112
#define ADDR_MAX_POSITION_LIMIT 48
#define ADDR_MIN_POSITION_LIMIT 52

#define JOINT_1_ID 1
#define JOINT_2_ID 2

#define OP_POS_CONTROL_MODE 3
#define OP_EXT_POS_CONTROL_MODE 4

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 115200
#define RAD_TO_PULSE 651.918  // 4096 (pulse per rot) / 2pi (rad)
#define PULSE_TO_RAD 0.00153  // 2pi (rad) / 4096 (pulse per rot)
#define MIN_OVER_RAD 9.5496// 60 sec / 2pi
#define VELOCITY_REVO_RESOLUTION 0.229 // 0.229 (rev/min) (given from https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ )
#define MAX_ROT_MIN 31 // (rev/min) (given from https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ )


template <typename T>
const T& clamp(const T& value, const T& minValue, const T& maxValue) {
    return (value < minValue) ? minValue : ((maxValue < value) ? maxValue : value);
}


using std::placeholders::_1;
using namespace std::chrono_literals;

std::string node_name = "arm_control_node";

ArmControlNode::ArmControlNode(): Node(node_name)
{
  this->declare_parameter("joint_pub_period_ms", rclcpp::ParameterValue(10));
  this->declare_parameter("joint_pan_enable_torque", rclcpp::ParameterValue(false));
  this->declare_parameter("joint_tilt_enable_torque", rclcpp::ParameterValue(false));
  this->declare_parameter("joint_pan_init_ang", rclcpp::ParameterValue(1.57));
  this->declare_parameter("joint_tilt_init_ang", rclcpp::ParameterValue(1.57));
  this->declare_parameter("dynamixel_port", rclcpp::ParameterValue("/dev/ttyUSB0"));
  this->declare_parameter("min_control_gain", rclcpp::ParameterValue(0.1));
  this->declare_parameter("max_control_gain", rclcpp::ParameterValue(0.5));
  this->declare_parameter("max_ang_err", rclcpp::ParameterValue(0.39)); // 30 deg * 960(H) / 1280(W)
  this->declare_parameter("lim_pan_left_ang", rclcpp::ParameterValue(0.78));
  this->declare_parameter("lim_pan_right_ang", rclcpp::ParameterValue(2.35));
  this->declare_parameter("lim_tilt_up_ang", rclcpp::ParameterValue(0.78));
  this->declare_parameter("lim_tilt_down_ang", rclcpp::ParameterValue(2.35));
  this->declare_parameter("joint_cmd_topic", rclcpp::ParameterValue("/joint_arm_cmd"));

  this->get_parameter("joint_pub_period_ms", joint_pub_period_ms_);
  this->get_parameter("joint_pan_enable_torque", enable_torques_[0]);
  this->get_parameter("joint_tilt_enable_torque", enable_torques_[1]);
  this->get_parameter("joint_pan_init_ang", joint_init_angs_[0]);
  this->get_parameter("joint_tilt_init_ang", joint_init_angs_[1]);
  this->get_parameter("dynamixel_port", dynamixel_port_);
  this->get_parameter("min_control_gain", min_control_gain_);
  this->get_parameter("max_control_gain", max_control_gain_);
  this->get_parameter("max_ang_err", max_ang_err_);
  this->get_parameter("lim_pan_left_ang", lim_angs_[0]);
  this->get_parameter("lim_pan_right_ang", lim_angs_[1]);
  this->get_parameter("lim_tilt_up_ang", lim_angs_[2]);
  this->get_parameter("lim_tilt_down_ang", lim_angs_[3]);
  this->get_parameter("joint_cmd_topic", joint_cmd_topic_);

  init_ang_thres_ = 0.1;
  init_position_ = true;

  // gain_scale_ = (max_control_gain_ - min_control_gain_) / pow(max_ang_err_, 2);
  gain_scale_ = 0.5 * (max_control_gain_ - min_control_gain_);
  err_scale_ = M_PI / max_ang_err_;

  RCLCPP_WARN(this->get_logger(), "Enable torque (joint_pan:%d, joint_tilt:%d)", enable_torques_[0], enable_torques_[1]);


  dxl_error_ = 0;
  dxl_comm_result_ = COMM_TX_FAIL;

  const char *c_device_name = dynamixel_port_.c_str();
  portHandler_ = dynamixel::PortHandler::getPortHandler(c_device_name);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  dxl_comm_result_ = portHandler_->openPort();
  if (dxl_comm_result_ == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port. %s", dynamixel_port_.c_str());
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result_ = portHandler_->setBaudRate(BAUDRATE);
  if (dxl_comm_result_ == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate %d", BAUDRATE);
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate %d", BAUDRATE);
  }

  joint_ids_[0] = JOINT_1_ID;
  joint_ids_[1] = JOINT_2_ID;
  joint_curr_angs_[0] = 0;
  joint_curr_angs_[1] = 0;
  joint_target_angs_[0] = joint_init_angs_[0];
  joint_target_angs_[1] = joint_init_angs_[1];

  joint_ang_thres_ = 0.02;

  setupDynamixel();

  RCLCPP_INFO(this->get_logger(), "Run Dynamicxel Pos Control node");

  // QoS 설정: 손실이 발생해도 되고 최대한 빠른 속도로 전송
  // rclcpp::QoS custom_qos_profile(1);  // Set history depth to 1
  // custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  // custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // rclcpp::QoS custom_qos_profile(1);
  // custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  // custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  rclcpp::QoS custom_qos_profile(1);
  custom_qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("/joint_arm_states", 10);

  joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_cmd_topic_,
    custom_qos_profile,
    std::bind(&ArmControlNode::jointTargetAngCallback, this, _1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(joint_pub_period_ms_),
    std::bind(&ArmControlNode::jointPubTimerCallback, this));

  strt_time_ = this->get_clock()->now();
}


void ArmControlNode::jointPubTimerCallback()
{
  for (int i = 0; i < 2; i++) {
    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
      portHandler_,
      joint_ids_[i],
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&curr_ang_pulses_[i]),
      &dxl_error_
    );
    if (dxl_comm_result_ == COMM_SUCCESS) {
      joint_curr_angs_[i] = pulseToRad(curr_ang_pulses_[i]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive angle. err=%d", dxl_comm_result_);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "ID(%d) rad: %.2f(%d) / ID(%d) rad: %.2f(%d)",
  //                 joint_ids_[0], joint_curr_angs_[0], curr_ang_pulses[0],
  //                 joint_ids_[1], joint_curr_angs_[1], curr_ang_pulses[1]);
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = this->now();
  joint_state.name = {"joint_pan", "joint_tilt"};  // Example joint names
  joint_state.position = {joint_curr_angs_[0], joint_curr_angs_[1]};  // Example joint positions
  joint_state_pub_->publish(joint_state);
  jointSetAngs();
}

void ArmControlNode::jointTargetAngCallback(const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  if (init_position_) {
    return;
  }
  joint_target_angs_[0] = _msg->position[0];
  joint_target_angs_[1] = _msg->position[1];
}


void ArmControlNode::jointSetAngs()
{
  // Could be replaced by pid control. but it is p control with varying p gain by the error range
  double joint_err_angs[2] = {0, 0};
  int init_ang_reached_cnt = 0;
  for (int i = 0; i < 2; i++) {
    joint_err_angs[i] = joint_target_angs_[i] - joint_curr_angs_[i];
    joint_err_angs[i] = clamp(joint_err_angs[i], -max_ang_err_, max_ang_err_);

    double control_gain = min_control_gain_;
    if (init_position_) {
      if (fabs(joint_err_angs[i]) < init_ang_thres_) {
        init_ang_reached_cnt++;
      }
    } else {
      control_gain = gain_scale_ * (1.0 - cos(err_scale_ * joint_err_angs[i])) + min_control_gain_;
    }
    double joint_cmd_ang = control_gain * joint_err_angs[i];
    
    // joint_cmd_ang = clamp(joint_curr_angs_[i] + joint_cmd_ang, lim_angs_[i*2], lim_angs_[i*2 + 1]);
    // int rad2pulse = radToPulse(joint_cmd_ang);
    // int goal_position = curr_ang_pulses_[i] + rad2pulse;

    double goal_ang = clamp(joint_curr_angs_[i] + joint_cmd_ang, lim_angs_[i*2], lim_angs_[i*2 + 1]);
    int goal_position = radToPulse(goal_ang);
    controlAng(goal_position, joint_ids_[i]);
  }
  if (init_position_ && init_ang_reached_cnt == 2) {
    RCLCPP_WARN(this->get_logger(), "Arrive init pose ang = %.2f / %.2f", joint_curr_angs_[0], joint_curr_angs_[1]);
    init_position_ = false;
  }
}


void ArmControlNode::controlAng(const int &goal_position, const int &joint_id) {
  dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    portHandler_,
    (uint8_t) joint_id,
    ADDR_GOAL_POSITION,
    goal_position,
    &dxl_error_
  );

  if (dxl_comm_result_ != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result_));
  } else if (dxl_error_ != 0) {
    RCLCPP_ERROR(this->get_logger(), "[%d] %s", joint_id, packetHandler_->getRxPacketError(dxl_error_));
  }
}


void ArmControlNode::setupDynamixel()
{
  uint8_t op_modes[2] = {OP_POS_CONTROL_MODE, OP_POS_CONTROL_MODE};
  for (int i = 0; i < 2; i++) {
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_,
      (uint8_t) joint_ids_[i],
      ADDR_OPERATING_MODE,
      op_modes[i],
      &dxl_error_
    );

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set Position Control Mode.");
    }

    // Enable Torque of DYNAMIXEL
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_,
      (uint8_t) joint_ids_[i],
      ADDR_TORQUE_ENABLE,
      enable_torques_[i],
      &dxl_error_
    );

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
    }
  }
}



int ArmControlNode::radToPulse(const double &target_ang)
{
  int goal_position = (int)(target_ang * RAD_TO_PULSE);
  return goal_position;
}

double ArmControlNode::pulseToRad(const int &inp_pulse)
{
  double out_rad = inp_pulse * PULSE_TO_RAD;
  return out_rad;
}

ArmControlNode::~ArmControlNode()
{
  packetHandler_->write1ByteTxRx(
    portHandler_,
    (uint8_t) BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error_
  );
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto arm_control_node = std::make_shared<ArmControlNode>();
  rclcpp::spin(arm_control_node);
  rclcpp::shutdown();

  return 0;
}

