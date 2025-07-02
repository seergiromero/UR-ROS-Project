#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "final_work/action/move_piece.hpp"
#include "final_work/move_piece_node.hpp"

using MovePiece = final_work::action::MovePiece;
using GoalHandleMovePiece = rclcpp_action::ServerGoalHandle<MovePiece>;

class MovePieceActionServer : public rclcpp::Node
{
public:
  MovePieceActionServer()
  : Node("move_piece_action_server")
  {
    // Instance of movement's logic
    move_piece_node_ = std::make_shared<final_work::MovePieceNode>();

    // Create action server
    action_server_ = rclcpp_action::create_server<MovePiece>(
      this,
      "move_piece",
      std::bind(&MovePieceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MovePieceActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MovePieceActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "MovePiece Action Server started");
  }

private:
  rclcpp_action::Server<MovePiece>::SharedPtr action_server_;
  std::shared_ptr<final_work::MovePieceNode> move_piece_node_;

  // Callback for new goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MovePiece::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal: piece='%s', square='%s'", 
      goal->piece_name.c_str(), goal->square.c_str());
    // Accept and execute
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Callback for cancelation
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMovePiece> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(get_logger(), "Goal cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // When goal is accepted, launch thread execution
  void handle_accepted(
    const std::shared_ptr<GoalHandleMovePiece> goal_handle)
  {
    std::thread(
      std::bind(&MovePieceActionServer::execute, this, goal_handle)
    ).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMovePiece> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MovePiece::Result>();
    auto feedback = std::make_shared<MovePiece::Feedback>();
    
    // Simulation offsets
    double offset_y = 0.02;
    double offset_x = -0.01;

    try {
        // Step 1: Add grasp frame
        feedback->stage = "adding_grasp_frame";
        goal_handle->publish_feedback(feedback);
        int aruco_id = move_piece_node_->getArucoId(goal->piece_name);
        move_piece_node_->callAddGraspFrame(aruco_id);

        // Step 2: Open gripper
        feedback->stage = "opening_gripper";
        goal_handle->publish_feedback(feedback);
        if (!move_piece_node_->openGripper()) {
            throw std::runtime_error("Failed to open gripper");
        }

        // Step 3: Move to pre-pose
        feedback->stage = "moving_to_pre_pick";
        goal_handle->publish_feedback(feedback);
        auto src_pre_pose = move_piece_node_->getPrePiecePose(goal->piece_name);
        src_pre_pose.value().position.y += offset_y;
        src_pre_pose.value().position.x += offset_x;
        if (!src_pre_pose) throw std::runtime_error("Pre-pick pose not found");
        if (!move_piece_node_->moveToPose(src_pre_pose.value(), 0.0)) {
            throw std::runtime_error("Failed to move to pre-pick pose");
        }

        // Step 4: Descend and grip
        feedback->stage = "picking_piece";
        goal_handle->publish_feedback(feedback);
        auto src_pose = move_piece_node_->getPiecePose(goal->piece_name);
        src_pose.value().position.y += offset_y;
        src_pose.value().position.x += offset_x;
        if (!src_pose) throw std::runtime_error("Pick pose not found");
        if (!move_piece_node_->pick(src_pre_pose.value(), src_pose.value())) {
            throw std::runtime_error("Pick operation failed");
        }

        // Step 5: Delete grasp frame
        feedback->stage = "removing_grasp_frame";
        goal_handle->publish_feedback(feedback);
        move_piece_node_->callRemoveGraspFrame(aruco_id);

        // Step 6: Move to pre-place
        feedback->stage = "moving_to_pre_place";
        goal_handle->publish_feedback(feedback);
        auto dst_pose = move_piece_node_->getBoardPose(goal->square);
        dst_pose.value().position.y += offset_y;
        dst_pose.value().position.x += offset_x;
        if (!dst_pose) throw std::runtime_error("Board pose not found");
        if (!move_piece_node_->moveToPose(dst_pose.value(), move_piece_node_->pre_place_offset_)) {
            throw std::runtime_error("Failed to move to pre-place pose");
        }

        // Step 7: Place piece
        feedback->stage = "placing_piece";
        goal_handle->publish_feedback(feedback);
        double dyn_offset = move_piece_node_->getPlaceOffset(goal->piece_name);
        if (!move_piece_node_->place(dst_pose.value(), dyn_offset)) {
            throw std::runtime_error("Place operation failed");
        }

        // Step 8: Moving home
        feedback->stage = "going_home";
        goal_handle->publish_feedback(feedback);
        if (!move_piece_node_->moveHome()) {
            throw std::runtime_error("Home operation failed");
        }

        // Success
        result->success = true;
        result->message = "Move completed successfully";
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Move failed: %s", e.what());
        result->success = false;
        result->message = e.what();
        goal_handle->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<MovePieceActionServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
