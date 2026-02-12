/**
 * Hardware Bridge Node (C++ Version)
 * 
 * AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY
 * 
 * C++ implementation for deterministic timing and real-time performance.
 */

#include "robot_hardware_bridge/bridge_node.hpp"
#include <cstdlib>
#include <chrono>

using namespace std::chrono_literals;

namespace robot_hardware_bridge
{

// Static member definition
const std::vector<std::string> HardwareBridgeNode::JOINT_NAMES = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
};

// ============================================================================
// SimulationBackend Implementation
// ============================================================================

SimulationBackend::SimulationBackend(rclcpp::Node* node, const std::string& mode)
  : node_(node), mode_(mode)
{
  current_positions_.resize(NUM_JOINTS, 0.0);
  target_positions_.resize(NUM_JOINTS, 0.0);
}

void SimulationBackend::start()
{
  RCLCPP_INFO(node_->get_logger(), "Simulation backend started in %s mode", mode_.c_str());
}

void SimulationBackend::stop()
{
  RCLCPP_INFO(node_->get_logger(), "Simulation backend stopped");
}

void SimulationBackend::send_trajectory(const trajectory_msgs::msg::JointTrajectory& msg)
{
  if (msg.points.empty()) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Set target to final point
  const auto& final_point = msg.points.back();
  for (size_t i = 0; i < NUM_JOINTS && i < final_point.positions.size(); ++i) {
    target_positions_[i] = final_point.positions[i];
  }
  
  executing_ = true;
  
  // Create pending ack
  pending_ack_ = std::make_shared<robot_msgs::msg::TrajectoryAck>();
  pending_ack_->trajectory_id = msg.header.stamp;
  pending_ack_->status = robot_msgs::msg::TrajectoryAck::STATUS_ACCEPTED;
  
  RCLCPP_INFO(node_->get_logger(), "Trajectory accepted, moving to target");
}

sensor_msgs::msg::JointState::SharedPtr SimulationBackend::get_joint_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Interpolate towards target
  if (executing_) {
    bool reached = true;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      double diff = target_positions_[i] - current_positions_[i];
      if (std::abs(diff) > 0.001) {
        current_positions_[i] += diff * INTERPOLATION_RATE;
        reached = false;
      } else {
        current_positions_[i] = target_positions_[i];
      }
    }
    if (reached) {
      executing_ = false;
      RCLCPP_INFO(node_->get_logger(), "Target position reached");
    }
  }
  
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp = node_->now();
  msg->name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  msg->position = current_positions_;
  msg->velocity.resize(NUM_JOINTS, 0.0);
  msg->effort.resize(NUM_JOINTS, 0.0);
  
  return msg;
}

robot_msgs::msg::ExecutionState::SharedPtr SimulationBackend::get_execution_state()
{
  auto msg = std::make_shared<robot_msgs::msg::ExecutionState>();
  msg->header.stamp = node_->now();
  msg->is_executing = executing_.load();
  msg->progress = executing_ ? 0.5 : 1.0;
  return msg;
}

robot_msgs::msg::TrajectoryAck::SharedPtr SimulationBackend::get_trajectory_ack()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto ack = pending_ack_;
  pending_ack_.reset();
  return ack;
}

// ============================================================================
// HardwareBridgeNode Implementation
// ============================================================================

HardwareBridgeNode::HardwareBridgeNode()
  : Node("hardware_bridge_cpp")
{
  // Declare parameters
  this->declare_parameter("robot_mode", "SIM");
  this->declare_parameter("controller_type", "FAKE");
  this->declare_parameter("controller_ip", "192.168.1.100");
  this->declare_parameter("controller_port", 5000);
  this->declare_parameter("controller_proto", "TCP");
  
  // Get parameters
  robot_mode_ = this->get_parameter("robot_mode").as_string();
  controller_type_ = this->get_parameter("controller_type").as_string();
  controller_ip_ = this->get_parameter("controller_ip").as_string();
  controller_port_ = this->get_parameter("controller_port").as_int();
  controller_proto_ = this->get_parameter("controller_proto").as_string();
  
  // Override from environment
  if (const char* env = std::getenv("ROBOT_MODE")) {
    robot_mode_ = env;
  }
  if (const char* env = std::getenv("CONTROLLER_TYPE")) {
    controller_type_ = env;
  }
  
  RCLCPP_INFO(this->get_logger(), "Hardware Bridge (C++) starting in %s mode", robot_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Controller type: %s", controller_type_.c_str());
  
  // Initialize backend (simulation only for now)
  backend_ = std::make_unique<SimulationBackend>(this, controller_type_);
  
  // QoS profiles
  auto trajectory_qos = rclcpp::QoS(10).reliable();
  auto joint_state_qos = rclcpp::QoS(1).best_effort();
  
  // Publishers
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", joint_state_qos);
  
  trajectory_ack_pub_ = this->create_publisher<robot_msgs::msg::TrajectoryAck>(
    "/trajectory_ack", trajectory_qos);
  
  execution_state_pub_ = this->create_publisher<robot_msgs::msg::ExecutionState>(
    "/execution_state", trajectory_qos);
  
  // Subscribers
  trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/planned_trajectory",
    trajectory_qos,
    std::bind(&HardwareBridgeNode::trajectory_callback, this, std::placeholders::_1));
  
  // Start backend
  backend_->start();
  
  // State publishing timer (100 Hz for real-time performance)
  state_timer_ = this->create_wall_timer(
    10ms,  // 100 Hz
    std::bind(&HardwareBridgeNode::publish_state_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Hardware Bridge (C++) initialized");
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Hardware Bridge (C++)");
  if (backend_) {
    backend_->stop();
  }
}

void HardwareBridgeNode::trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());
  
  // Schema validation
  if (!validate_schema(msg)) {
    return;
  }
  
  // Relay to backend
  RCLCPP_DEBUG(this->get_logger(), "Schema validation passed, relaying to backend");
  backend_->send_trajectory(*msg);
}

bool HardwareBridgeNode::validate_schema(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  // Validate joint count
  if (msg->joint_names.size() != JOINT_NAMES.size()) {
    RCLCPP_ERROR(this->get_logger(), 
      "Schema error: Expected %zu joints, got %zu",
      JOINT_NAMES.size(), msg->joint_names.size());
    publish_rejection(msg, "SCHEMA_ERROR_JOINT_COUNT");
    return false;
  }
  
  // Validate joint names
  for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
    if (msg->joint_names[i] != JOINT_NAMES[i]) {
      RCLCPP_ERROR(this->get_logger(),
        "Schema error: Expected joint %s, got %s",
        JOINT_NAMES[i].c_str(), msg->joint_names[i].c_str());
      publish_rejection(msg, "SCHEMA_ERROR_JOINT_NAME");
      return false;
    }
  }
  
  // Validate points not empty
  if (msg->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Schema error: Empty points array");
    publish_rejection(msg, "SCHEMA_ERROR_EMPTY_POINTS");
    return false;
  }
  
  // Validate position array lengths
  for (size_t i = 0; i < msg->points.size(); ++i) {
    if (msg->points[i].positions.size() != JOINT_NAMES.size()) {
      RCLCPP_ERROR(this->get_logger(),
        "Schema error: Point %zu has %zu positions, expected %zu",
        i, msg->points[i].positions.size(), JOINT_NAMES.size());
      publish_rejection(msg, "SCHEMA_ERROR_POSITIONS_LENGTH");
      return false;
    }
  }
  
  return true;
}

void HardwareBridgeNode::publish_rejection(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg,
  const std::string& reason)
{
  auto ack = robot_msgs::msg::TrajectoryAck();
  ack.trajectory_id = msg->header.stamp;
  ack.status = robot_msgs::msg::TrajectoryAck::STATUS_REJECTED;
  ack.reject_reason = reason;
  trajectory_ack_pub_->publish(ack);
}

void HardwareBridgeNode::publish_state_callback()
{
  // Publish joint state
  auto joint_state = backend_->get_joint_state();
  if (joint_state) {
    joint_state_pub_->publish(*joint_state);
  }
  
  // Publish execution state
  auto exec_state = backend_->get_execution_state();
  if (exec_state) {
    execution_state_pub_->publish(*exec_state);
  }
  
  // Publish any pending trajectory acks
  auto ack = backend_->get_trajectory_ack();
  if (ack) {
    trajectory_ack_pub_->publish(*ack);
  }
}

}  // namespace robot_hardware_bridge

// Main entry point
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_hardware_bridge::HardwareBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
