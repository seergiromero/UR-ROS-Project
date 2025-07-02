#ifndef FINAL_WORK__KILL_PIECE_NODE_HPP_
#define FINAL_WORK__KILL_PIECE_NODE_HPP_

#include "final_work/move_piece_node.hpp"
#include <string>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace final_work {

class KillPieceNode : public rclcpp::Node
{
public:
  KillPieceNode();

  /// Execute a kill move: remove killed piece to kill zone, then move killer piece to target square
  void killPiece(const std::string & killer_piece, const std::string & killed_piece, const std::string & target_square);

private:
  std::shared_ptr<MovePieceNode> move_piece_node_;
  
  /// Fixed kill position
  std::string kill_position_;
  
  /// YAML node to store board positions (same as MovePieceNode)
  YAML::Node board_positions_;
  
  /// Load board configuration from YAML file
  void loadBoardConfig();
  
  /// Get board pose with proper coordinate transformation (same as MovePieceNode)
  std::optional<geometry_msgs::msg::Pose> getBoardPose(const std::string & square);
  
  /// Get vertical orientation for chess pieces (same as MovePieceNode)
  geometry_msgs::msg::Quaternion getVerticalOrientation() const;
  
  /// TF2 buffer and listener for coordinate transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace final_work

#endif // FINAL_WORK__KILL_PIECE_NODE_HPP_
