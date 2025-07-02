#ifndef FINAL_WORK_CASTLING_ACTION_HPP
#define FINAL_WORK_CASTLING_ACTION_HPP

#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "final_work/action/castling.hpp"
#include "final_work/move_piece_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using Castling = final_work::action::Castling;
using GoalHandleCastling = rclcpp_action::ServerGoalHandle<Castling>;

class CastlingActionServer : public rclcpp::Node
{
public:
  CastlingActionServer();

private:
  rclcpp_action::Server<Castling>::SharedPtr action_server_;
  std::shared_ptr<final_work::MovePieceNode> move_piece_node_;
  
  // YAML configuration for board positions
  YAML::Node board_positions_;

  // Load board configuration from YAML file
  void loadBoardConfig();

  // Get board position from YAML (similar to MovePieceNode::getBoardPose)
  std::optional<geometry_msgs::msg::Pose> getBoardPose(const std::string& square);

  // Get piece names for castling
  std::pair<std::string, std::string> getCastlingPieces(const std::string& castling_type);

  // Get source and destination squares for castling
  struct CastlingMoves {
    std::string king_from, king_to, rook_from, rook_to;
  };

  CastlingMoves getCastlingMoves(const std::string& castling_type);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Castling::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCastling> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleCastling> goal_handle);

  bool movePiece(const std::string& piece_name, const std::string& from_square, 
                 const std::string& to_square, 
                 std::shared_ptr<GoalHandleCastling> goal_handle,
                 std::shared_ptr<Castling::Feedback> feedback,
                 const std::string& stage_name);

  void execute(const std::shared_ptr<GoalHandleCastling> goal_handle);
};

#endif // FINAL_WORK_CASTLING_ACTION_HPP
