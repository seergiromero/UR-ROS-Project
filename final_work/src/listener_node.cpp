#include "final_work/listener_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

ChessListener::ChessListener()
: Node("listener_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // Map of ArUco ID to chess piece name
    aruco_to_piece_ = {
        {201, "pawnB1"}, {202, "pawnB2"}, {203, "pawnB3"}, {204, "pawnB4"},
        {205, "pawnB5"}, {206, "pawnB6"}, {207, "pawnB7"}, {208, "pawnB8"},
        {209, "rookB1"}, {210, "rookB2"}, {211, "knightB1"}, {212, "knightB2"},
        {213, "bishopB1"}, {214, "bishopB2"}, {215, "queenB"}, {216, "kingB"},
        {301, "pawnW1"}, {302, "pawnW2"}, {303, "pawnW3"}, {304, "pawnW4"},
        {305, "pawnW5"}, {306, "pawnW6"}, {307, "pawnW7"}, {308, "pawnW8"},
        {309, "rookW1"}, {310, "rookW2"}, {311, "knightW1"}, {312, "knightW2"},
        {313, "bishopW1"}, {314, "bishopW2"}, {315, "queenW"}, {316, "kingW"}
    };
    

    // Use "world" as reference frame for all transforms
    reference_frame_ = "world";

    // Subscribe to the ArUco marker array topic
    aruco_sub_ = create_subscription<aruco_msgs::msg::MarkerArray>(
        "/aruco_marker_publisher/markers", 10,
        std::bind(&ChessListener::aruco_callback, this, std::placeholders::_1)
    );

    piece_pose_array_pub_ = this->create_publisher<final_work::msg::PiecePoseArray>("piece_pose_array_topic", 10);

    RCLCPP_INFO(get_logger(), "Chess listener initialized with %ld pieces", aruco_to_piece_.size());
    RCLCPP_INFO(get_logger(), "Using reference frame: %s", reference_frame_.c_str());
}

void ChessListener::aruco_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Clear any previously stored poses
    piece_poses_.clear();
    final_work::msg::PiecePoseArray pose_array_msg;

    for (const auto & marker : msg->markers) {
        // Find corresponding chess piece name for this ArUco ID
        auto it = aruco_to_piece_.find(marker.id);
        if (it == aruco_to_piece_.end()) {
            continue;
        }
        std::string piece_name = it->second;

        // Lookup transform from reference_frame_ to the marker frame
        std::string marker_frame = "aruco_" + std::to_string(marker.id);
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                reference_frame_, marker_frame, tf2::TimePointZero
            );
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(), "Transform lookup failed: %s", ex.what());
            continue;
        }

        // Store the pose of this piece in the map
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;
        pose.orientation = transform_stamped.transform.rotation;
        piece_poses_[piece_name] = pose;

        // Create a msg PiecePose with name and pose
        final_work::msg::PiecePose piece_msg;
        piece_msg.name=piece_name;
        piece_msg.pose=pose;

        //Add to the Array list
        pose_array_msg.poses.push_back(piece_msg);
    }

    // Publish the message in the topic
    piece_pose_array_pub_->publish(pose_array_msg);

    RCLCPP_DEBUG(get_logger(), "Published PiecePoseArray with %zu poses", pose_array_msg.poses.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChessListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
