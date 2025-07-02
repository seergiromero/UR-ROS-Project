#include "final_work/move_piece_node.hpp"
#include "final_work/action_client_node.hpp"

#include <filesystem>

using namespace std::chrono_literals;
namespace final_work {

MovePieceNode::MovePieceNode()
: Node("move_piece_node")
{
  initializeClients();
  loadBoardConfig();
  RCLCPP_INFO(get_logger(), "MovePieceNode initialized.");
}

void MovePieceNode::initializeClients()
{
  // Inverse Kinematics service client
  ik_client_ = create_client<kinenikros2::srv::InverseKinematics>("inverse_kinematics");

  // UR trajectory/action client
  trajectory_client_ = std::make_shared<URTrajectoryClient>();

  // Gripper service clients
  gripper_open_client_ = create_client<robotiq_85_gripper_server::srv::GripperOpen>("GripperOpen");
  gripper_close_client_= create_client<robotiq_85_gripper_server::srv::GripperClose>("GripperClose");
  gripper_order_client_= create_client<robotiq_85_gripper_server::srv::GripperOrder>("/GripperOrder");

  // TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create and delete Aruco grasp frames
  add_client_ = this->create_client<final_work::srv::SetArucoId>("add_grasp_frame");
  remove_client_ = this->create_client<final_work::srv::SetArucoId>("remove_grasp_frame");
}

void MovePieceNode::loadBoardConfig()
{
    // Get the information from the yaml file that contains tha position of each square
    try {
        auto pkg_path = ament_index_cpp::get_package_share_directory("final_work");
        YAML::Node config = YAML::LoadFile(pkg_path + "/config/chess_board_positions.yaml");
        
        board_positions_ = config; 

    } catch (const YAML::Exception & e) {
        RCLCPP_FATAL(get_logger(), "Failed to load board config: %s", e.what());
        rclcpp::shutdown();
    }
}

//=== Getter Functions ===//

std::optional<geometry_msgs::msg::Pose> MovePieceNode::getPrePiecePose(const std::string & piece_name)
{
  return lookupPiecePose(piece_name, "pre_grasp_");
}

std::optional<geometry_msgs::msg::Pose> MovePieceNode::getPiecePose(const std::string & piece_name)
{
  return lookupPiecePose(piece_name, "grasp_");
}

int MovePieceNode::getArucoId(const std::string & piece_name)
{   
    // Check which aruco belongs to the piece name
    int aruco_id = -1;
    for (const auto & p : aruco_to_piece_) {
        if (p.second == piece_name) {
            aruco_id = p.first;
            return aruco_id;
        }
    }
    return -1;
}

double MovePieceNode::getPlaceOffset(const std::string & piece_name)
{

  // Depending on the piece_name see what offset needs
  double dyn_offset = pre_place_offset_;  
  for (auto & [type, offset] : piece_place_height_) {
    if (piece_name.size() >= type.size() &&
        piece_name.compare(0, type.size(), type) == 0)
    {
      dyn_offset = offset;
      break;
    }
  }
  return dyn_offset;
}

std::optional<geometry_msgs::msg::Pose> MovePieceNode::getBoardPose(const std::string & square)
{
    try {
        YAML::Node pos = board_positions_[square];

        // Build PoseStamped in world frame
        geometry_msgs::msg::PoseStamped world_pose;
        world_pose.header.frame_id = "world";
        world_pose.header.stamp = this->get_clock()->now();
        world_pose.pose.position.x = pos["x"].as<double>();
        world_pose.pose.position.y = pos["y"].as<double>();
        world_pose.pose.position.z = pos["z"].as<double>();
        world_pose.pose.orientation = getVerticalOrientation();

        // Transform pose into robot base frame
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

geometry_msgs::msg::Quaternion MovePieceNode::getVerticalOrientation() const
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, M_PI/4.0);
    return tf2::toMsg(q);
}

//=== Action Client Callbacks ===//

void MovePieceNode::callAddGraspFrame(int aruco_id)
{
    // Calls the service that by giving the aruco_id it creates a frame for the pick and prepick
    auto request = std::make_shared<final_work::srv::SetArucoId::Request>();
    request->aruco_id = aruco_id;
    auto future = add_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        throw std::runtime_error("Service call to add_grasp_frame failed");
    }
    if (!future.get()->success) {
        throw std::runtime_error(future.get()->message);
    }
}

void MovePieceNode::callRemoveGraspFrame(int aruco_id)
{
    // Service to remove the frames created to pick a piece
    auto request = std::make_shared<final_work::srv::SetArucoId::Request>();
    request->aruco_id = aruco_id;
    auto future = remove_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        throw std::runtime_error("Service call to remove_grasp_frame failed");
    }
    if (!future.get()->success) {
        throw std::runtime_error(future.get()->message);
    }
}

bool MovePieceNode::callIK(const geometry_msgs::msg::Pose & target, std::vector<double> & joints)
{
    // Request arguments
    auto req = std::make_shared<kinenikros2::srv::InverseKinematics::Request>();
    req->type = "ur3e";
    req->pose = target;

    if (!ik_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(get_logger(), "IK service unavailable");
        return false;
    }
    auto fut = ik_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), fut
        ) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "IK request failed");
        return false;
    }

    auto res = fut.get();
    if (!res->status) {
        RCLCPP_ERROR(get_logger(), "IK solution invalid");
        return false;
    }
    // Changes the input varible with the correct joints pose
    joints.assign(res->ik_solution[0].ik.begin(), res->ik_solution[0].ik.end());
    return true;
}

std::optional<geometry_msgs::msg::Pose> MovePieceNode::lookupPiecePose(const std::string & piece_name,const std::string & frame_prefix)
{
    // Get ID from piece name
    int aruco_id = MovePieceNode::getArucoId(piece_name); 

    // Frame name
    std::string frame = frame_prefix + std::to_string(aruco_id);

    // Try TF
    int attempts = 0;
    while (rclcpp::ok() && attempts < 5) {
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(
                "base", frame, tf2::TimePointZero, 500ms);

            geometry_msgs::msg::Pose pose;
            
            // Configure robot pos
            pose.orientation = getVerticalOrientation();
            pose.position.x  = tf_stamped.transform.translation.x;
            pose.position.y  = tf_stamped.transform.translation.y;
            pose.position.z  = tf_stamped.transform.translation.z;

            return pose;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(),
                        "Attempt %d for %s failed: %s",
                        attempts+1, frame.c_str(), ex.what());
            rclcpp::sleep_for(500ms);
            ++attempts;
        }
    }

    RCLCPP_ERROR(get_logger(),
                 "Failed to lookup transform for %s after %d attempts",
                 frame.c_str(), attempts);
    return std::nullopt;
}

//=== Movement Functions ===//

bool MovePieceNode::moveToPose(const geometry_msgs::msg::Pose & pose, double z_lift)
{
    geometry_msgs::msg::Pose lifted = pose;
    lifted.position.z += z_lift; // Offset of the gripper


    std::vector<double> joints;
    if (!callIK(lifted, joints)) {
        return false;
    }
    return trajectory_client_->send_joint_positions(joints);
}

bool MovePieceNode::pick(const geometry_msgs::msg::Pose & pre_pose_, const geometry_msgs::msg::Pose & pose_)
{

    geometry_msgs::msg::Pose pre_pose = pre_pose_;
    geometry_msgs::msg::Pose pose = pose_;


    if (!moveToPose(pre_pose, 0.0)) {
        return false;
    }
    // Descend to pick depth
    if (!moveLinear(pre_pose, pose, 3)) {
        return false;
    }
    // Close gripper on piece
    if (!closeGripper()) {
        return false;
    }
    // Short pause to ensure grip
    rclcpp::sleep_for(500ms);
    // Retract to pre-pick height
    return moveLinear(pose, pre_pose, 3);
}

bool MovePieceNode::place(const geometry_msgs::msg::Pose & pose, double offset)
{
    
    geometry_msgs::msg::Pose target = pose;
    geometry_msgs::msg::Pose pre_target = pose;
    target.position.z += offset;
    pre_target.position.z += pre_place_offset_;

    // Descend to placement depth
    if (!moveLinear(pre_target, target, 3)) {
        return false;
    }
    // Release piece
    if (!openGripper()) {
        return false;
    }
    // Retract to pre-place height
    return moveLinear(target, pre_target, 3);
}

//=== Gripper Functions ===//

bool MovePieceNode::openGripper()
{
    auto req = std::make_shared<robotiq_85_gripper_server::srv::GripperOpen::Request>();
    req->order = true;
    auto fut = gripper_open_client_->async_send_request(req);
    return rclcpp::spin_until_future_complete(
               this->get_node_base_interface(), fut
           ) == rclcpp::FutureReturnCode::SUCCESS;
}

bool MovePieceNode::closeGripper()
{
    RCLCPP_INFO(get_logger(), "Closing gripper with force control...");
    auto req = std::make_shared<robotiq_85_gripper_server::srv::GripperOrder::Request>();
    req->position = 0.58;  // target position
    req->speed    = 0.5;  // moderate speed
    req->force    = 0.2;  // controlled force

    auto fut = gripper_order_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), fut) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Gripper close request failed");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Gripper closed");
    return true;
}

//=== Linear Motion Functions ===//

std::vector<geometry_msgs::msg::Pose> MovePieceNode::interpolateLinearMotion(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end,
    int steps)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(steps);

    for (int i = 0; i < steps; ++i) {
        double ratio = static_cast<double>(i) / (steps - 1);
        geometry_msgs::msg::Pose pose;
        
        // Interpolate position
        pose.position.x = start.position.x + ratio * (end.position.x - start.position.x);
        pose.position.y = start.position.y + ratio * (end.position.y - start.position.y);
        pose.position.z = start.position.z + ratio * (end.position.z - start.position.z);
        
        // Interpolate orientation
        tf2::Quaternion start_quat, end_quat;
        tf2::fromMsg(start.orientation, start_quat);
        tf2::fromMsg(end.orientation, end_quat);
        tf2::Quaternion interp_quat = tf2::slerp(start_quat, end_quat, ratio);
        pose.orientation = tf2::toMsg(interp_quat);
        
        poses.push_back(pose);
    }

    return poses;
}

bool MovePieceNode::moveLinear(const geometry_msgs::msg::Pose& start_pose,
                              const geometry_msgs::msg::Pose& end_pose,
                              int steps)
{

    // Generate intermidiate poses in a path
    auto poses = interpolateLinearMotion(start_pose, end_pose, steps);
    
    // Calculate IK for each pose
    std::vector<std::vector<double>> all_joint_positions;
    for (const auto& pose : poses) {
        std::vector<double> joints;
        if (!callIK(pose, joints)) {
            RCLCPP_ERROR(get_logger(), "IK failed during linear motion planning.");
            return false;
        }
        all_joint_positions.push_back(joints);
    }
    
    // Send complete trajectory
    return trajectory_client_->send_linear_trajectory(all_joint_positions);
}

bool MovePieceNode::moveHome(){

    // Home joints position
    std::vector<double> joints = {
        0.000, 
        -1.57, 
        0.000, 
        -1.57, 
        0.000, 
        0.000
    }; 

    return trajectory_client_->send_joint_positions(joints);


}
//=== Move Piece Function ===//

void MovePieceNode::movePiece(const std::string & piece_name, const std::string & square)
{

    int aruco_id = MovePieceNode::getArucoId(piece_name);

    // Create the frames
    auto request_add = std::make_shared<final_work::srv::SetArucoId::Request>();
    request_add->aruco_id = aruco_id;
    auto future_add = add_client_->async_send_request(request_add);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_add) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "add_grasp_frame: %s",
                  future_add.get()->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error calling add_grasp_frame");
      return;
    }

    // Lookup source and destination poses
    auto src_pre_pose = getPrePiecePose(piece_name);
    auto src_pose = getPiecePose(piece_name);
    auto dst_pose = getBoardPose(square);

    // Abort if either pose lookup failed
    if (!src_pre_pose || !dst_pose) {
        RCLCPP_ERROR(get_logger(), "Pose lookup failed. Aborting move.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Moving %s to %s", piece_name.c_str(), square.c_str());

    // Open gripper before approaching piece
    if (!openGripper()) {
        RCLCPP_ERROR(get_logger(), "Failed to open gripper.");
        return;
    }

    // Execute pick operation
    if (!pick(src_pre_pose.value(), src_pose.value())) {
        RCLCPP_ERROR(get_logger(), "Pick operation failed.");
        return;
    }

    
    // Move above target square
    if (!moveToPose(dst_pose.value(), pre_place_offset_)) {
        RCLCPP_ERROR(get_logger(), "Failed to move to pre-place pose.");
        return;
    }

    // calculas el offset dinámico:
    double dyn_offset = getPlaceOffset(piece_name);

    // Execute place operation
    if (!place(dst_pose.value(), dyn_offset)) {
        RCLCPP_ERROR(get_logger(), "Place operation failed.");
        return;
    }

    if(!moveHome()){
        RCLCPP_ERROR(get_logger(), "Move home failed");
        return;
    }

    RCLCPP_INFO(get_logger(), "Move completed successfully");

    // Remove frames
    auto request_rem = std::make_shared<final_work::srv::SetArucoId::Request>();
    request_rem->aruco_id = aruco_id;
    auto future_rem = remove_client_->async_send_request(request_rem);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_rem) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "remove_grasp_frame: %s",
                  future_rem.get()->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error calling remove_grasp_frame");
    }
}


} // namespace final_work
