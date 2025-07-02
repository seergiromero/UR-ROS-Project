#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "final_work/action/kill_piece.hpp"
#include "final_work/kill_piece_node.hpp"

using KillPiece = final_work::action::KillPiece;
using GoalHandleKillPiece = rclcpp_action::ServerGoalHandle<KillPiece>;

class KillPieceActionServer : public rclcpp::Node
{
public:
  KillPieceActionServer()
  : Node("kill_piece_action_server")
  {
    // Unique identifier
    std::string node_name = "kill_piece_action_server_" + std::to_string(getpid());
    
    // Kill logic instance
    kill_piece_node_ = std::make_shared<final_work::KillPieceNode>();

    // Create action server
    action_server_ = rclcpp_action::create_server<KillPiece>(
      this,
      "kill_piece",
      std::bind(&KillPieceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&KillPieceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&KillPieceActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "KillPiece Action Server started");
  }

private:
  rclcpp_action::Server<KillPiece>::SharedPtr action_server_;
  std::shared_ptr<final_work::KillPieceNode> kill_piece_node_;

  // Callback for new goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const KillPiece::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal: killer_piece='%s', killed_piece='%s', target_square='%s'", 
      goal->killer_piece.c_str(), goal->killed_piece.c_str(), goal->target_square.c_str());
    // Accept and execute
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Callback for cancellation
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleKillPiece> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Goal cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // When goal is accepted, launch thread execution
  void handle_accepted(
    const std::shared_ptr<GoalHandleKillPiece> goal_handle)
  {
    std::thread(
      std::bind(&KillPieceActionServer::execute, this, goal_handle)
    ).detach();
  }

  void execute(const std::shared_ptr<GoalHandleKillPiece> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<KillPiece::Result>();
    auto feedback = std::make_shared<KillPiece::Feedback>();

    try {
    
        feedback->stage = "removing_killed_piece";
        feedback->progress = 0;
        goal_handle->publish_feedback(feedback);

        // Feedback publication
        feedback->progress = 25;
        goal_handle->publish_feedback(feedback);

        feedback->stage = "moving_killer_piece";
        feedback->progress = 50;
        goal_handle->publish_feedback(feedback);

        // Esxecute kill operation
        kill_piece_node_->killPiece(goal->killer_piece, goal->killed_piece, goal->target_square);

        feedback->stage = "completed";
        feedback->progress = 100;
        goal_handle->publish_feedback(feedback);

        // Success
        result->success = true;
        result->message = "Kill operation completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Kill operation failed: %s", e.what());
        result->success = false;
        result->message = e.what();
        goal_handle->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<KillPieceActionServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
