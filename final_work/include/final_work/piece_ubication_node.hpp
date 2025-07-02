#ifndef FINAL_WORK_PIECE_UBICATION_NODE_HPP
#define FINAL_WORK_PIECE_UBICATION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <map>
#include <string>
#include "final_work/msg/piece_pose_array.hpp"
#include "final_work/msg/piece_ubication_array.hpp"


class PieceUbicationNode : public rclcpp::Node
{
public:
  PieceUbicationNode();

private:
  void loadPiecePosition();
  void piecePoseCallback(const final_work::msg::PiecePoseArray::SharedPtr msg);
  std::string findClosestSquare(const geometry_msgs::msg::Point & piece_position);

  rclcpp::Subscription<final_work::msg::PiecePoseArray>::SharedPtr piece_pose_sub_;
  rclcpp::Publisher<final_work::msg::PieceUbicationArray>::SharedPtr piece_ubication_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, geometry_msgs::msg::Point> board_positions_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // FINAL_WORK_PIECE_UBICATION_NODE_HPP
