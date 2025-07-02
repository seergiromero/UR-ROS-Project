#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "final_work/srv/set_aruco_id.hpp"

using SetArucoId = final_work::srv::SetArucoId;

/**
 * @class GraspBroadcaster
 * @brief Node that broadcasts pre-grasp and grasp TF frames for specified ArUco IDs.
 *
 * Provides two services:
 * - add_grasp_frame: compute and broadcast transforms for a given ArUco marker ID.
 * - remove_grasp_frame: stop broadcasting (dynamic frames expire automatically).
 */
class GraspBroadcaster : public rclcpp::Node {
public:
  /**
   * @brief Constructor for GraspBroadcaster node.
   *
   * Declares parameters, initializes TF listener and broadcaster, and
   * creates the add/remove service servers.
   */
  GraspBroadcaster();

private:
  /**
   * @brief Service callback to add and broadcast pre-grasp and grasp frames.
   *
   * Looks up the transform from base_link to the specified ArUco frame, then
   * creates two TF frames:
   * - pre_grasp_<id>: offset in Z by pre_z_offset_.
   * - grasp_<id>: offset in Z by pick_z_offset_.
   *
   * @param request   Contains the ArUco ID to process.
   * @param response  Returns success and message status.
   */
  void addCallback(
    const std::shared_ptr<SetArucoId::Request> request,
    std::shared_ptr<SetArucoId::Response> response);

  /**
   * @brief Service callback to remove grasp frames.
   *
   * With dynamic broadcasting, frames will expire when not resent.
   * This callback simply returns success, clients should stop relying on the frame.
   *
   * @param request   Contains the ArUco ID to stop broadcasting (unused).
   * @param response  Returns success and message status.
   */
  void removeCallback(
    const std::shared_ptr<SetArucoId::Request> request,
    std::shared_ptr<SetArucoId::Response> response);

  /// TF2 buffer to store transforms
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /// TF2 listener to receive transforms on the TF topic
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// Dynamic broadcaster to send TF frames on demand
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  /// Service server to add and broadcast grasp frames
  rclcpp::Service<SetArucoId>::SharedPtr add_service_;

  /// Service server to remove grasp frames (dynamic frames expire automatically)
  rclcpp::Service<SetArucoId>::SharedPtr remove_service_;

  /// Vertical offset for pre-grasp frame (meters above marker)
  double pre_z_offset_ = 0.25;

  /// Vertical offset for grasp frame (meters above marker)
  double pick_z_offset_ = 0.14;
};
