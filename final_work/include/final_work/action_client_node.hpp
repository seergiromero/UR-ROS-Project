#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include "control_msgs/action/follow_joint_trajectory.hpp"

// Control msgs with the action follow joint trajectory
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
// Used to send positions to the controller
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class URTrajectoryClient : public rclcpp::Node {
public:
    /**
     * @brief Constructor: Initializes the action client for sending joint trajectory goals.
     */
    URTrajectoryClient();

    //=== Action Sending Functions ===//

    /**
     * @brief Send joint positions to the robot controller.
     * 
     * @param positions  The joint positions to send as a vector of doubles.
     * @return true if the goal was sent successfully, false otherwise.
     */
    bool send_joint_positions(const std::vector<double>& positions);

    /**
     * @brief Send a series of joint positions (linear trajectory) to the robot controller.
     * 
     * @param joint_positions_list  List of joint positions, each set representing a trajectory point.
     * @return true if the trajectory was sent successfully, false otherwise.
     */
    bool send_linear_trajectory(const std::vector<std::vector<double>>& joint_positions_list);

private:
    //=== Action Client ===//

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_; ///< The action client for sending FollowJointTrajectory goals.

    //=== Joint Names ===//

    std::vector<std::string> joint_names = {
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    }; ///< List of joint names for the UR robot.

    //=== Configuration ===//

    double time_from_start = 3; ///< Time from start for trajectory (in seconds).

    //=== Tolerances ===//

    std::vector<control_msgs::msg::JointTolerance> goal_tolerances_; ///< Joint tolerances for the trajectory.

};

