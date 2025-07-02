#ifndef KINENIK_SRV_SERVER_H
#define KINENIK_SRV_SERVER_H

#include <sstream>      // std::stringstream

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "kinenikros2/srv/inverse_kinematics.hpp"

// Kinenic library
#include <kinenikros2/kinenik_ur.h>

class MyServiceNode : public rclcpp::Node
{
private:
  rclcpp::Service<kinenikros2::srv::InverseKinematics>::SharedPtr service_;
  bool ComposeFullName(const std::shared_ptr<kinenikros2::srv::InverseKinematics::Request> request,
        std::shared_ptr<kinenikros2::srv::InverseKinematics::Response> response);

public:
  MyServiceNode(std::string passedNodeName="VOID")
    : Node(passedNodeName)
  {
    RCLCPP_INFO(this->get_logger(), "[kinenikros2]: Ready to provide Inverse Kinematics.");
    // like the subscriber class node it's needed the boost::bind to acces the member method 
    // with 2 placeholders to pass request and response to the callback
    service_ = this->create_service<kinenikros2::srv::InverseKinematics>("inverse_kinematics", 
      std::bind(&MyServiceNode::ComposeFullName, this, std::placeholders::_1, std::placeholders::_2 ));
  }
  
};

#endif // KINENIK_SRV_SERVER_H