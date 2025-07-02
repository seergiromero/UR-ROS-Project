#include "final_work/piece_ubication_node.hpp"
#include "final_work/msg/piece_pose_array.hpp"
#include "final_work/msg/piece_pose.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

PieceUbicationNode::PieceUbicationNode()
: Node("piece_ubication_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Load the configuration of the board
  std::string package_path = ament_index_cpp::get_package_share_directory("final_work");
  std::string config_path = package_path + "/config/chess_board_positions.yaml";

  YAML::Node board_yaml = YAML::LoadFile(config_path);
  for (auto it = board_yaml.begin(); it != board_yaml.end(); ++it) {
      std::string square = it->first.as<std::string>();
      geometry_msgs::msg::Point pt;
      pt.x = it->second["x"].as<double>();
      pt.y = it->second["y"].as<double>();
      pt.z = it->second["z"].as<double>();
      board_positions_[square] = pt;
  }

  // Suscribe to the node PiecePoseArray
  piece_pose_sub_ = create_subscription<final_work::msg::PiecePoseArray>(
    "piece_pose_array_topic", 10,
    std::bind(&PieceUbicationNode::piecePoseCallback, this, std::placeholders::_1)
  );

  //Publisher to the node PieceUbicationArray
  piece_ubication_pub_ = this->create_publisher<final_work::msg::PieceUbicationArray>(
      "piece_ubication_array_topic", 10
  );

  RCLCPP_INFO(this->get_logger(), "PieceUbicationNode initialized.");
}

//Publish in the node node piece_ubication_node
void PieceUbicationNode::piecePoseCallback(const final_work::msg::PiecePoseArray::SharedPtr msg)
{
    final_work::msg::PieceUbicationArray ubication_msg;

    for (const auto & piece : msg->poses) {
        const std::string& name = piece.name;
        const geometry_msgs::msg::Point& pos = piece.pose.position;
        std::string square = findClosestSquare(pos);

        ubication_msg.names.push_back(name);
        ubication_msg.squares.push_back(square);

        RCLCPP_DEBUG(get_logger(), "%s -> %s", name.c_str(), square.c_str());
    }

    piece_ubication_pub_->publish(ubication_msg);
}

// Find the closest square to each piece
std::string PieceUbicationNode::findClosestSquare(const geometry_msgs::msg::Point & piece_position)
{
  double min_distance = std::numeric_limits<double>::max();
  std::string closest_square = "";

  // For each piece, calculate the Euclidean distance to each square
  for (const auto & [square_name, square_position] : board_positions_) {
      double dx = piece_position.x - square_position.x;
      double dy = piece_position.y - square_position.y;
      double dz = piece_position.z - square_position.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      //Store the closest square to the piece.
      if (distance < min_distance) {
          min_distance = distance;
          closest_square = square_name;
      }
  }

  return closest_square;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PieceUbicationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
