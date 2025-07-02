#include "final_work/custom_grasps_tf2_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

GraspBroadcaster::GraspBroadcaster()
: Node("grasp_broadcaster")
{

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Dynamic broadcaster to send transforms only when requested
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create service server to add and broadcast grasp frames
  add_service_ = this->create_service<SetArucoId>(
    "add_grasp_frame",
    std::bind(&GraspBroadcaster::addCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create service server to remove grasp frames (dynamic frames will expire)
  remove_service_ = this->create_service<SetArucoId>(
    "remove_grasp_frame",
    std::bind(&GraspBroadcaster::removeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  RCLCPP_INFO(this->get_logger(), "GraspBroadcaster node initialized");
}

void GraspBroadcaster::addCallback(
  const std::shared_ptr<SetArucoId::Request> request,
  std::shared_ptr<SetArucoId::Response> response)
{
  int id = request->aruco_id;
  std::string marker_frame = "aruco_" + std::to_string(id);

  // Attempt to lookup the transform from base to the marker frame
  geometry_msgs::msg::TransformStamped base_to_marker;
  try {
    base_to_marker = tf_buffer_->lookupTransform(
      "base",  
      marker_frame,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not find transform for %s: %s", marker_frame.c_str(), ex.what());
    response->success = false;
    response->message = "Transform not available.";
    return;
  }

  // Create pre-grasp frame by applying pre_z_offset_
  geometry_msgs::msg::TransformStamped pre;
  pre.header.stamp = this->get_clock()->now();
  pre.header.frame_id = base_to_marker.header.frame_id;
  pre.child_frame_id = "pre_grasp_" + std::to_string(id);
  pre.transform = base_to_marker.transform;
  pre.transform.translation.z += pre_z_offset_;

  // Broadcast the pre-grasp frame once
  broadcaster_->sendTransform(pre);

  // Create grasp frame by applying pick_z_offset_
  geometry_msgs::msg::TransformStamped grasp;
  grasp.header.stamp = this->get_clock()->now();
  grasp.header.frame_id = base_to_marker.header.frame_id;
  grasp.child_frame_id = "grasp_" + std::to_string(id);
  grasp.transform = base_to_marker.transform;
  grasp.transform.translation.z += pick_z_offset_;

  // Broadcast the grasp frame once
  broadcaster_->sendTransform(grasp);

  response->success = true;
  response->message = "Broadcasted pre_grasp_" + std::to_string(id) +
                      " and grasp_" + std::to_string(id);
}

void GraspBroadcaster::removeCallback(
  const std::shared_ptr<SetArucoId::Request>,
  std::shared_ptr<SetArucoId::Response> response)
{
  // Dynamic TF frames expire when not resent
  response->success = true;
  response->message = "Dynamic frames will expire naturally.";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
