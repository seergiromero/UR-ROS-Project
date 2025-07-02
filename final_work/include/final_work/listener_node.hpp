#pragma once

#include "final_work/msg/piece_pose.hpp"
#include "final_work/msg/piece_pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <unordered_map>
#include <string>

/**
 * @class ChessListener
 * @brief Subscribes to ArUco detections and writes piece poses to a YAML file.
 *
 * Listens to the /aruco_marker_publisher/markers topic, maps marker IDs to chess piece names,
 * looks up transforms in a configurable reference frame, and writes a YAML configuration
 * with each piece's pose. The YAML is saved under <package_share>/config/chess_configuration.yaml.
 */
class ChessListener : public rclcpp::Node {
public:
  /**
   * @brief Construct a new ChessListener node.
   *
   * Initializes the TF2 buffer and listener, populates the ArUco-to-piece map,
   * creates the subscription to marker detections, and logs initialization info.
   */
  ChessListener();

private:
  /**
   * @brief Callback for incoming ArUco MarkerArray messages.
   *
   * Clears any previously stored piece poses, iterates over detected markers,
   * looks up the transform from reference_frame_ to each marker's frame,
   * stores the resulting pose in piece_poses_, and finally emits a YAML file
   * containing all piece poses under <package_share>/config/chess_configuration.yaml.
   *
   * @param msg Shared pointer to the received MarkerArray message.
   */
  void aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg);

  // Publisher for PiecePoseArray messages
  rclcpp::Publisher<final_work::msg::PiecePoseArray>::SharedPtr piece_pose_array_pub_;

  /// TF2 buffer for transform lookups.
  tf2_ros::Buffer tf_buffer_;

  /// TF2 listener that populates the tf_buffer_.
  tf2_ros::TransformListener tf_listener_;

  /// Subscription to the ArUco MarkerArray topic.
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_sub_;

  /// Map from piece name (string) to its current pose.
  std::unordered_map<std::string, geometry_msgs::msg::Pose> piece_poses_;

  /// Map from ArUco marker ID (int) to chess piece name (string).
  std::unordered_map<int, std::string> aruco_to_piece_;

  /// Reference frame used for transform lookups (e.g., "world").
  std::string reference_frame_;
};
