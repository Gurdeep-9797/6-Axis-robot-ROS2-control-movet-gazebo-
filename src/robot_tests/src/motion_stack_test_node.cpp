#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MotionStackTestNode : public rclcpp::Node {
public:
    MotionStackTestNode() : Node("motion_stack_test_node") {
        RCLCPP_INFO(get_logger(), "=== MOTION STACK TEST NODE ===");
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                last_joint_state_ = msg;
            });
        
        ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    }
    
    int run_tests() {
        int failures = 0;
        
        // Wait for services
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // TEST 1: IK Service Exists
        RCLCPP_INFO(get_logger(), "TEST 1: IK Service Exists");
        if (!ik_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "FAIL: /compute_ik service not available");
            failures++;
        } else {
            RCLCPP_INFO(get_logger(), "PASS: /compute_ik service exists");
        }
        
        // TEST 2: Frame Existence
        RCLCPP_INFO(get_logger(), "TEST 2: Frame Existence");
        try {
            auto transform = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero, std::chrono::seconds(5));
            RCLCPP_INFO(get_logger(), "PASS: base_link -> tool0 transform exists");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "FAIL: Frame lookup failed: %s", ex.what());
            failures++;
        }
        
        // TEST 3: Joint State Publisher
        RCLCPP_INFO(get_logger(), "TEST 3: Joint State Authority");
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (last_joint_state_ && last_joint_state_->name.size() == 6) {
            RCLCPP_INFO(get_logger(), "PASS: Receiving 6-DOF joint states");
        } else {
            RCLCPP_ERROR(get_logger(), "FAIL: Not receiving valid joint states");
            failures++;
        }
        
        // TEST 4: IK Reachable Pose
        RCLCPP_INFO(get_logger(), "TEST 4: IK Reachable Pose");
        {
            auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
            request->ik_request.group_name = "robot_arm";
            request->ik_request.pose_stamped.header.frame_id = "base_link";
            request->ik_request.pose_stamped.pose.position.x = 0.3;
            request->ik_request.pose_stamped.pose.position.y = 0.0;
            request->ik_request.pose_stamped.pose.position.z = 0.4;
            request->ik_request.pose_stamped.pose.orientation.w = 1.0;
            
            auto future = ik_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                if (response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "PASS: IK solved reachable pose");
                } else {
                    RCLCPP_ERROR(get_logger(), "FAIL: IK failed on reachable pose (code: %d)", response->error_code.val);
                    failures++;
                }
            } else {
                RCLCPP_ERROR(get_logger(), "FAIL: IK service call timed out");
                failures++;
            }
        }
        
        // TEST 5: IK Unreachable Pose (Should Fail)
        RCLCPP_INFO(get_logger(), "TEST 5: IK Unreachable Pose");
        {
            auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
            request->ik_request.group_name = "robot_arm";
            request->ik_request.pose_stamped.header.frame_id = "base_link";
            request->ik_request.pose_stamped.pose.position.x = 5.0;
            request->ik_request.pose_stamped.pose.position.y = 5.0;
            request->ik_request.pose_stamped.pose.position.z = 5.0;
            request->ik_request.pose_stamped.pose.orientation.w = 1.0;
            
            auto future = ik_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_from_this(), future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "PASS: IK correctly rejected unreachable pose");
                } else {
                    RCLCPP_ERROR(get_logger(), "FAIL: IK accepted unreachable pose (should fail)");
                    failures++;
                }
            }
        }
        
        // Summary
        RCLCPP_INFO(get_logger(), "=== TEST SUMMARY ===");
        if (failures == 0) {
            RCLCPP_INFO(get_logger(), "ALL TESTS PASSED");
        } else {
            RCLCPP_ERROR(get_logger(), "%d TEST(S) FAILED", failures);
        }
        
        return failures;
    }
    
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionStackTestNode>();
    int result = node->run_tests();
    rclcpp::shutdown();
    return result;
}
