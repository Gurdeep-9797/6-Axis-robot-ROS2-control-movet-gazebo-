/**
 * Hardware Bridge Node (C++ Version)
 * 
 * AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY
 * 
 * This node acts as a protocol gateway between ROS 2 and the RT Controller.
 * It performs SCHEMA validation only; all semantic validation is done by Controller.
 * 
 * C++ implementation for deterministic timing and real-time performance.
 */

#ifndef ROBOT_HARDWARE_BRIDGE__BRIDGE_NODE_HPP_
#define ROBOT_HARDWARE_BRIDGE__BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <robot_msgs/msg/trajectory_ack.hpp>
#include <robot_msgs/msg/execution_state.hpp>

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>

namespace robot_hardware_bridge
{

/**
 * Backend interface for hardware abstraction
 */
class IBackend
{
public:
  virtual ~IBackend() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void send_trajectory(const trajectory_msgs::msg::JointTrajectory& msg) = 0;
  virtual sensor_msgs::msg::JointState::SharedPtr get_joint_state() = 0;
  virtual robot_msgs::msg::ExecutionState::SharedPtr get_execution_state() = 0;
  virtual robot_msgs::msg::TrajectoryAck::SharedPtr get_trajectory_ack() = 0;
};

/**
 * Simulation backend for testing without hardware
 */
class SimulationBackend : public IBackend
{
public:
  explicit SimulationBackend(rclcpp::Node* node, const std::string& mode);
  void start() override;
  void stop() override;
  void send_trajectory(const trajectory_msgs::msg::JointTrajectory& msg) override;
  sensor_msgs::msg::JointState::SharedPtr get_joint_state() override;
  robot_msgs::msg::ExecutionState::SharedPtr get_execution_state() override;
  robot_msgs::msg::TrajectoryAck::SharedPtr get_trajectory_ack() override;

private:
  rclcpp::Node* node_;
  std::string mode_;
  std::vector<double> current_positions_;
  std::vector<double> target_positions_;
  std::mutex state_mutex_;
  std::atomic<bool> executing_{false};
  robot_msgs::msg::TrajectoryAck::SharedPtr pending_ack_;
  
  static constexpr size_t NUM_JOINTS = 6;
  static constexpr double INTERPOLATION_RATE = 0.1;  // 10% per cycle
};

/**
 * Hardware Bridge Node - RELAY ONLY
 * 
 * This node relays commands between ROS and the Controller.
 * It has NO authority over motor control or safety.
 */
class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode();
  ~HardwareBridgeNode() override;

private:
  // Expected joint names (must match URDF)
  static const std::vector<std::string> JOINT_NAMES;
  
  // Parameters
  std::string robot_mode_;
  std::string controller_type_;
  std::string controller_ip_;
  int controller_port_;
  std::string controller_proto_;
  
  // Backend
  std::unique_ptr<IBackend> backend_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<robot_msgs::msg::TrajectoryAck>::SharedPtr trajectory_ack_pub_;
  rclcpp::Publisher<robot_msgs::msg::ExecutionState>::SharedPtr execution_state_pub_;
  
  // Subscribers
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  
  // Timer for state publishing (100 Hz)
  rclcpp::TimerBase::SharedPtr state_timer_;
  
  // Callbacks
  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void publish_state_callback();
  
  // Helpers
  void publish_rejection(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg,
    const std::string& reason);
  bool validate_schema(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
};

}  // namespace robot_hardware_bridge

#endif  // ROBOT_HARDWARE_BRIDGE__BRIDGE_NODE_HPP_
