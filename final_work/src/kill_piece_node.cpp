#include "final_work/kill_piece_node.hpp"
#include <stdexcept>

using namespace std::chrono_literals;

namespace final_work {

KillPieceNode::KillPieceNode() 
  : Node("kill_piece_node"), kill_position_("kill1")
{
  // Initialize the MovePieceNode
  move_piece_node_ = std::make_shared<MovePieceNode>();
  
  // Initialize TF2 buffer and listener (same as MovePieceNode)
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Load board configuration (same as MovePieceNode)
  loadBoardConfig();
  
  RCLCPP_INFO(get_logger(), "KillPieceNode initialized with kill position: %s", kill_position_.c_str());
}

void KillPieceNode::loadBoardConfig()
{
    try {
        auto pkg_path = ament_index_cpp::get_package_share_directory("final_work");
        YAML::Node config = YAML::LoadFile(pkg_path + "/config/chess_board_positions.yaml");
        
        // Load directly to root node (same as MovePieceNode)
        board_positions_ = config; 
        
        RCLCPP_INFO(get_logger(), "Board configuration loaded successfully");

    } catch (const YAML::Exception & e) {
        RCLCPP_FATAL(get_logger(), "Failed to load board config: %s", e.what());
        rclcpp::shutdown();
    }
}

std::optional<geometry_msgs::msg::Pose> KillPieceNode::getBoardPose(const std::string & square)
{
    try {
        YAML::Node pos = board_positions_[square];

        // Build PoseStamped in world frame (same as MovePieceNode)
        geometry_msgs::msg::PoseStamped world_pose;
        world_pose.header.frame_id = "world";
        world_pose.header.stamp = this->get_clock()->now();
        world_pose.pose.position.x = pos["x"].as<double>();
        world_pose.pose.position.y = pos["y"].as<double>();
        world_pose.pose.position.z = pos["z"].as<double>();
        world_pose.pose.orientation = getVerticalOrientation();

        // Transform pose into robot base frame (same as MovePieceNode)
        geometry_msgs::msg::PoseStamped base_pose;
        int attempts = 0;
        while (rclcpp::ok() && attempts < 5) {
            try {
                base_pose = tf_buffer_->transform(world_pose, "base", 500ms);
                return base_pose.pose;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(get_logger(), "Attempt %d transforming %s: %s",
                            attempts+1, square.c_str(), ex.what());
                rclcpp::sleep_for(500ms);
                attempts++;
            }
        }

        RCLCPP_ERROR(get_logger(), "Failed to transform square %s to base frame", square.c_str());
        return std::nullopt;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid square '%s': %s", square.c_str(), e.what());
        return std::nullopt;
    }
}

geometry_msgs::msg::Quaternion KillPieceNode::getVerticalOrientation() const
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, M_PI/4.0);
    return tf2::toMsg(q);
}

void KillPieceNode::killPiece(const std::string & killer_piece, const std::string & killed_piece, const std::string & target_square)
{
  RCLCPP_INFO(get_logger(), "Starting kill operation: %s kills %s, moving to %s", 
              killer_piece.c_str(), killed_piece.c_str(), target_square.c_str());

  double offset_y = 0.01;
  double offset_x = -0.02;

  try {
    // ========== FIRST OPERATION: REMOVE KILLED PIECE ==========
    RCLCPP_INFO(get_logger(), "Phase 1: Removing killed piece '%s'", killed_piece.c_str());
    
    // Step 1: Add grasp frame for killed piece
    int killed_aruco_id = move_piece_node_->getArucoId(killed_piece);
    move_piece_node_->callAddGraspFrame(killed_aruco_id);

    // Step 2: Open gripper
    if (!move_piece_node_->openGripper()) {
      throw std::runtime_error("Failed to open gripper for killed piece");
    }

    // Step 3: Move to killed piece's pre-pose
    auto killed_pre_pose = move_piece_node_->getPrePiecePose(killed_piece);
    killed_pre_pose.value().position.y += offset_y;
    killed_pre_pose.value().position.x += offset_x;
    if (!killed_pre_pose) throw std::runtime_error("Killed piece pre-pick pose not found");
    if (!move_piece_node_->moveToPose(killed_pre_pose.value(), 0.0)) {
      throw std::runtime_error("Failed to move to killed piece pre-pick pose");
    }

    // Step 4: Descend and grab killed piece
    auto killed_pose = move_piece_node_->getPiecePose(killed_piece);
    killed_pose.value().position.y += offset_y;
    killed_pose.value().position.x += offset_x;
    if (!killed_pose) throw std::runtime_error("Killed piece pick pose not found");
    if (!move_piece_node_->pick(killed_pre_pose.value(), killed_pose.value())) {
      throw std::runtime_error("Pick operation failed for killed piece");
    }

    // Step 5: Delete killed_piece's grasp frame
    move_piece_node_->callRemoveGraspFrame(killed_aruco_id);

    // Step 6: Move to pre-place of kill position
    // Use getBoardPose with proper coordinate transformation
    auto kill_pose = getBoardPose(kill_position_);
    kill_pose.value().position.y += offset_y;
    kill_pose.value().position.x += offset_x;
    if (!kill_pose) {
      RCLCPP_WARN(get_logger(), "getBoardPose failed for '%s', trying MovePieceNode method", kill_position_.c_str());
      kill_pose = move_piece_node_->getBoardPose(kill_position_);
    }
    
    if (!kill_pose) throw std::runtime_error("Kill position pose not found: " + kill_position_);
    if (!move_piece_node_->moveToPose(kill_pose.value(), move_piece_node_->pre_place_offset_)) {
      throw std::runtime_error("Failed to move to kill position pre-place pose");
    }

    // Step 7: Put killed piece in kill position
    double killed_dyn_offset = move_piece_node_->getPlaceOffset(killed_piece);
    if (!move_piece_node_->place(kill_pose.value(), killed_dyn_offset)) {
      throw std::runtime_error("Place operation failed for killed piece at kill position");
    }

    RCLCPP_INFO(get_logger(), "Phase 1 completed: Killed piece moved to %s", kill_position_.c_str());

    // ========== SECOND OPERATION: MOVE KILLER PIECE ==========
    RCLCPP_INFO(get_logger(), "Phase 2: Moving killer piece '%s' to target '%s'", 
                killer_piece.c_str(), target_square.c_str());

    // Step 8: Add grasp frame for killer piece
    int killer_aruco_id = move_piece_node_->getArucoId(killer_piece);
    move_piece_node_->callAddGraspFrame(killer_aruco_id);

    // Step 9: Open gripper
    if (!move_piece_node_->openGripper()) {
      throw std::runtime_error("Failed to open gripper for killer piece");
    }

    // Step 10: Move to pre-pose of killer piece
    auto killer_pre_pose = move_piece_node_->getPrePiecePose(killer_piece);
    killer_pre_pose.value().position.y += offset_y;
    killer_pre_pose.value().position.x += offset_x;
    if (!killer_pre_pose) throw std::runtime_error("Killer piece pre-pick pose not found");
    if (!move_piece_node_->moveToPose(killer_pre_pose.value(), 0.0)) {
      throw std::runtime_error("Failed to move to killer piece pre-pick pose");
    }

    // Step 11: Descend and grab killer piece
    auto killer_pose = move_piece_node_->getPiecePose(killer_piece);
    killer_pose.value().position.y += offset_y;
    killer_pose.value().position.x += offset_x;
    if (!killer_pose) throw std::runtime_error("Killer piece pick pose not found");
    if (!move_piece_node_->pick(killer_pre_pose.value(), killer_pose.value())) {
      throw std::runtime_error("Pick operation failed for killer piece");
    }

    // Step 12: Remove grasp frame of the killer piece
    move_piece_node_->callRemoveGraspFrame(killer_aruco_id);

    // Step 13: Move to pre-place of the target square
    // Use getBoardPose with proper coordinate transformation
    auto target_pose = getBoardPose(target_square);
    target_pose.value().position.y += offset_y;
    target_pose.value().position.x += offset_x;
    if (!target_pose) {
      RCLCPP_WARN(get_logger(), "getBoardPose failed for '%s', trying MovePieceNode method", target_square.c_str());
      target_pose = move_piece_node_->getBoardPose(target_square);
    }
    
    if (!target_pose) throw std::runtime_error("Target square pose not found: " + target_square);
    if (!move_piece_node_->moveToPose(target_pose.value(), move_piece_node_->pre_place_offset_)) {
      throw std::runtime_error("Failed to move to target square pre-place pose");
    }

    // Step 14: Put killer piece in target square
    double killer_dyn_offset = move_piece_node_->getPlaceOffset(killer_piece);
    if (!move_piece_node_->place(target_pose.value(), killer_dyn_offset)) {
      throw std::runtime_error("Place operation failed for killer piece at target square");
    }

    RCLCPP_INFO(get_logger(), "Phase 2 completed: Killer piece moved to target square");
    RCLCPP_INFO(get_logger(), "Kill operation completed successfully");
    RCLCPP_INFO(get_logger(), "Moving home");

    if (!move_piece_node_->moveHome()) {
      throw std::runtime_error("Home operation failed");
    }

    
    
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Kill operation failed: %s", e.what());
    throw; // Re-throw to let the action server handle it
  }
}

} // namespace final_work
