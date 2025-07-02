#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <vector>
#include <string>
#include "final_work/action_client_node.hpp"

using namespace std::chrono_literals;

URTrajectoryClient::URTrajectoryClient() : Node("ur_trajectory_client")
{
    // Initialize action client for joint trajectory controller
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this,
        // Standard controller name (verify with 'ros2 action list' when robot is running)
        "/joint_trajectory_controller/follow_joint_trajectory"
    );

    // Tolerances
    goal_tolerances_.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        goal_tolerances_[i].name = joint_names[i];
        goal_tolerances_[i].position = 0.01;  
        goal_tolerances_[i].velocity = 0.01;  
        goal_tolerances_[i].acceleration = 0.01; 
    }

}
bool URTrajectoryClient::send_joint_positions(const std::vector<double>& positions) 
{

    // Wait for action server to become available
    if (!client_->wait_for_action_server(2s)) {
        RCLCPP_ERROR(get_logger(), "Action server unavailable after 2s timeout");
        return false;
    }

    // Prepare goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    goal_msg.trajectory.points.push_back(point);

    goal_msg.goal_tolerance = goal_tolerances_;


    RCLCPP_INFO(get_logger(), "Sending single-point trajectory with %zu joints", positions.size());

    // Send action goal and handle response
    auto goal_future = client_->async_send_goal(goal_msg);
    
    // Wait for goal acceptance
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to transmit goal to server");
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal rejected by action server");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted. Waiting for execution result...");

    // Get final execution result
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to retrieve result");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Trajectory execution completed successfully");
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Execution failed with error code: %d", 
                    static_cast<int>(result.code));
        return false;
    }
}
        
bool URTrajectoryClient::send_linear_trajectory(const std::vector<std::vector<double>>& joint_positions_list) 
{
    // Verify action server availability
    if (!client_->wait_for_action_server(2s)) {
        RCLCPP_ERROR(get_logger(), "Action server unavailable after 2s timeout");
        return false;
    }

    // Prepare multi-point trajectory message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;
    
    // Configure timing parameters
    double time_step = time_from_start / joint_positions_list.size();

    RCLCPP_INFO(get_logger(), "Building multi-point trajectory with %zu waypoints", 
                joint_positions_list.size());

    // Construct trajectory points
    double current_time = 0.0;
    for (const auto& positions : joint_positions_list) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        current_time += time_step;
        point.time_from_start = rclcpp::Duration::from_seconds(current_time);
        goal_msg.trajectory.points.push_back(point);
    }
    goal_msg.goal_tolerance = goal_tolerances_;

    // Send trajectory and handle response
    auto goal_future = client_->async_send_goal(goal_msg);
    
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to transmit trajectory to server");
        return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Trajectory rejected by action server");
        return false;
    }

    // Retrieve execution result
    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to retrieve execution result");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Multi-point trajectory executed successfully");
        return true;
    } else {
        RCLCPP_ERROR(get_logger(), "Multi-point execution failed with code: %d", 
                    static_cast<int>(result.code));
        return false;
    }
}