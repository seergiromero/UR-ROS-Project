#include "final_work/castling_node.hpp"

CastlingActionServer::CastlingActionServer()
: Node("castling_action")
{
  // Load board configuration from YAML
  loadBoardConfig();

  // Instanciar la lógica de movimiento
  move_piece_node_ = std::make_shared<final_work::MovePieceNode>();

  // Crear servidor de acción
  action_server_ = rclcpp_action::create_server<Castling>(
    this,
    "castling",
    std::bind(&CastlingActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&CastlingActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&CastlingActionServer::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(get_logger(), "Castling Action Server started");
}

void CastlingActionServer::loadBoardConfig()
{
    try {
        auto pkg_path = ament_index_cpp::get_package_share_directory("final_work");
        YAML::Node config = YAML::LoadFile(pkg_path + "/config/chess_board_positions.yaml");
        
        // Acceder directamente al nodo raíz (sin niveles adicionales)
        board_positions_ = config; 

        RCLCPP_INFO(get_logger(), "Board configuration loaded successfully");

    } catch (const YAML::Exception & e) {
        RCLCPP_FATAL(get_logger(), "Failed to load board config: %s", e.what());
        rclcpp::shutdown();
    }
}

std::optional<geometry_msgs::msg::Pose> CastlingActionServer::getBoardPose(const std::string& square)
{
    try {
        YAML::Node pos = board_positions_[square];

        // Build Pose directly (we'll let MovePieceNode handle TF transformations)
        geometry_msgs::msg::Pose pose;
        pose.position.x = pos["x"].as<double>();
        pose.position.y = pos["y"].as<double>(); 
        pose.position.z = pos["z"].as<double>();
        
        // Use the same vertical orientation as MovePieceNode
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI/4.0);
        pose.orientation = tf2::toMsg(q);

        return pose;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid square '%s': %s", square.c_str(), e.what());
        return std::nullopt;
    }
}

// Get piece names for castling
std::pair<std::string, std::string> CastlingActionServer::getCastlingPieces(const std::string& castling_type)
{
  // Return king and rook piece names
  if (castling_type == "kingside" || castling_type == "short") {
    return {"kingB", "rookB2"};
  }
  else {
    return {"kingB", "rookB1"};
  }
}

// Get source and destination squares for castling
CastlingActionServer::CastlingMoves CastlingActionServer::getCastlingMoves(const std::string& castling_type)
{
  if (castling_type == "kingside" || castling_type == "short") {
    return {"E8", "G8", "H8", "F8"};
  }
  else { // queenside/long
    return {"E8", "C8", "A8", "D8"};
  }
}

rclcpp_action::GoalResponse CastlingActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Castling::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received castling goal: type='%s'", 
    goal->castling_type.c_str());

  // Validate castling type
  if (goal->castling_type != "kingside" && goal->castling_type != "short" &&
      goal->castling_type != "queenside" && goal->castling_type != "long") {
    RCLCPP_ERROR(get_logger(), "Invalid castling type: %s", goal->castling_type.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CastlingActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleCastling> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(get_logger(), "Castling goal cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CastlingActionServer::handle_accepted(
  const std::shared_ptr<GoalHandleCastling> goal_handle)
{
  std::thread(
    std::bind(&CastlingActionServer::execute, this, goal_handle)
  ).detach();
}

bool CastlingActionServer::movePiece(const std::string& piece_name, const std::string& from_square, 
               const std::string& to_square, 
               std::shared_ptr<GoalHandleCastling> goal_handle,
               std::shared_ptr<Castling::Feedback> feedback,
               const std::string& stage_name)
{
  double offset_y = 0.01;
  double offset_x = -0.02;

  try {
    // Update feedback
    feedback->stage = stage_name;
    goal_handle->publish_feedback(feedback);

    // Get ArUco ID and add grasp frame
    int aruco_id = move_piece_node_->getArucoId(piece_name);
    move_piece_node_->callAddGraspFrame(aruco_id);

    // Open gripper
    if (!move_piece_node_->openGripper()) {
      throw std::runtime_error("Failed to open gripper for " + piece_name);
    }

    // Move to pre-pick pose
    auto pre_pose = move_piece_node_->getPrePiecePose(piece_name);
    pre_pose.value().position.x += offset_x;
    pre_pose.value().position.y += offset_y;
    if (!pre_pose) throw std::runtime_error("Pre-pick pose not found for " + piece_name);
    if (!move_piece_node_->moveToPose(pre_pose.value(), 0.0)) {
      throw std::runtime_error("Failed to move to pre-pick pose for " + piece_name);
    }

    // Pick the piece
    auto pick_pose = move_piece_node_->getPiecePose(piece_name);
    pick_pose.value().position.x += offset_x;
    pick_pose.value().position.y += offset_y;
    if (!pick_pose) throw std::runtime_error("Pick pose not found for " + piece_name);
    if (!move_piece_node_->pick(pre_pose.value(), pick_pose.value())) {
      throw std::runtime_error("Pick operation failed for " + piece_name);
    }

    // Remove grasp frame
    move_piece_node_->callRemoveGraspFrame(aruco_id);

    // Get destination pose using our YAML-based function
    auto dst_pose = getBoardPose(to_square);
    dst_pose.value().position.x += offset_x;
    dst_pose.value().position.y += offset_y;
    if (!dst_pose) throw std::runtime_error("Board pose not found for " + to_square);
    
    // Use MovePieceNode's getBoardPose for proper TF transformation if needed
    auto transformed_dst_pose = move_piece_node_->getBoardPose(to_square);
    if (!transformed_dst_pose) throw std::runtime_error("Failed to transform board pose for " + to_square);
    
    if (!move_piece_node_->moveToPose(transformed_dst_pose.value(), move_piece_node_->pre_place_offset_)) {
      throw std::runtime_error("Failed to move to pre-place pose for " + to_square);
    }

    // Place the piece
    double place_offset = move_piece_node_->getPlaceOffset(piece_name);
    if (!move_piece_node_->place(transformed_dst_pose.value(), place_offset)) {
      throw std::runtime_error("Place operation failed for " + piece_name + " at " + to_square);
    }

    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to move %s from %s to %s: %s", 
      piece_name.c_str(), from_square.c_str(), to_square.c_str(), e.what());
    return false;
  }
}

void CastlingActionServer::execute(const std::shared_ptr<GoalHandleCastling> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Castling::Result>();
  auto feedback = std::make_shared<Castling::Feedback>();

  try {
    // Get castling moves and pieces
    auto moves = getCastlingMoves(goal->castling_type);
    auto pieces = getCastlingPieces(goal->castling_type);

    RCLCPP_INFO(get_logger(), "Executing %s castling", goal->castling_type.c_str());
    RCLCPP_INFO(get_logger(), "King: %s -> %s, Rook: %s -> %s", 
      moves.king_from.c_str(), moves.king_to.c_str(),
      moves.rook_from.c_str(), moves.rook_to.c_str());

    // Validate that all required squares exist in the YAML configuration
    if (!getBoardPose(moves.king_from) || !getBoardPose(moves.king_to) ||
        !getBoardPose(moves.rook_from) || !getBoardPose(moves.rook_to)) {
      throw std::runtime_error("One or more castling squares not found in board configuration");
    }

    // Step 1: Move the King
    feedback->stage = "moving_king";
    feedback->current_piece = pieces.first;
    goal_handle->publish_feedback(feedback);
    
    if (!movePiece(pieces.first, moves.king_from, moves.king_to, 
                   goal_handle, feedback, "moving_king")) {
      throw std::runtime_error("Failed to move king");
    }

    // Step 2: Move the Rook
    feedback->stage = "moving_rook";
    feedback->current_piece = pieces.second;
    goal_handle->publish_feedback(feedback);
    
    if (!movePiece(pieces.second, moves.rook_from, moves.rook_to, 
                   goal_handle, feedback, "moving_rook")) {
      throw std::runtime_error("Failed to move rook");
    }

    if (!move_piece_node_->moveHome()) {
      throw std::runtime_error("Failed to move home");
    }

    // Success
    result->success = true;
    result->message = goal->castling_type + " castling completed successfully";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Castling goal succeeded");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Castling failed: %s", e.what());
    result->success = false;
    result->message = e.what();
    goal_handle->abort(result);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<CastlingActionServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
