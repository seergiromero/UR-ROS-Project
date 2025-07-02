/* *************************************************************************\
  Copyright 2022-2023 Institute of Industrial and Control Engineering (IOC)
                Universitat Politecnica de Catalunya
                BarcelonaTech
* Software License Agreement (BSD License)
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Institute of Industrial and Control Engineering,
*     (IOC), Universitat Politecnica de Catalunya or BarcelonaTech nor
*     the names of its contributors may be used to endorse or promote
*     products derived from this software without specific prior
*     written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Leopold Palomo-Avellaneda
  Desc:   aruco broadcaster ROS2
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

// Messages:
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <aruco_msgs/msg/marker.hpp>
#include <aruco_msgs/msg/marker_array.hpp>

// Services:
#include "aruco_broadcaster/srv/get_marker_tf.hpp"

// Transforms:
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>

//This is the class for only the modelization RVIZ and the real robot WidowX XM430 with its own constructor and methods
class ArucoBroadcaster : public rclcpp::Node
{
public:
  // Constructor:
  ArucoBroadcaster() : Node("aruco_broadcaster"){

    RCLCPP_INFO(this->get_logger(),"\033[1;32m---->\033[0m Aruco_broadcaster is running.");

    // Initialize parameters:
    this->declare_parameter("markerList", rclcpp::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("camera_frame", rclcpp::PARAMETER_STRING);
    this->declare_parameter("aruco_frame", rclcpp::PARAMETER_STRING);
    this->init_params();

    this->sub_camera_aruco_tf_ = this->create_subscription<aruco_msgs::msg::MarkerArray>("/aruco_marker_publisher/markers", 10, std::bind(&ArucoBroadcaster::aruco_callback, this, std::placeholders::_1));
    this->server_get_marker_tf_ = this->create_service<aruco_broadcaster::srv::GetMarkerTf>("get_marker_Tf", std::bind(&ArucoBroadcaster::get_marker_tf, this,std::placeholders::_1, std::placeholders::_2));
    
    // Initialize the tfListener:
    this->tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);

    // Initialize the tfBroadcaster:
    this->tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:

  //Subscribers:
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_camera_aruco_tf_;
  
  // Servers:
  rclcpp::Service<aruco_broadcaster::srv::GetMarkerTf>::SharedPtr server_get_marker_tf_;

  // Transforms:
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  // Parameters:
  rclcpp::Parameter markerList_param, camera_frame_param, aruco_frame_param;

  // Variables:
  std::vector<int64_t> markerList;
  std::string camera_frame, aruco_frame;
  bool publish_all;

  // Callbacks:
  void aruco_callback(const aruco_msgs::msg::MarkerArray & marker_info);
  void get_marker_tf(const std::shared_ptr<aruco_broadcaster::srv::GetMarkerTf::Request> request, std::shared_ptr<aruco_broadcaster::srv::GetMarkerTf::Response> response);

  //Methods:
  void init_params();
};

// Callbacks code:
void ArucoBroadcaster::aruco_callback(const aruco_msgs::msg::MarkerArray & marker_info){
  
  geometry_msgs::msg::TransformStamped camera_H_marker_msg;

  for(uint i = 0; i < marker_info.markers.size();i++){
    // Set the transformation:
    camera_H_marker_msg.header.stamp = this->get_clock()->now();
    camera_H_marker_msg.header.frame_id = this->camera_frame;
    camera_H_marker_msg.child_frame_id = this->aruco_frame + "_" + std::to_string(marker_info.markers.at(i).id);

    camera_H_marker_msg.transform.translation.x = marker_info.markers.at(i).pose.pose.position.x;
    camera_H_marker_msg.transform.translation.y = marker_info.markers.at(i).pose.pose.position.y;
    camera_H_marker_msg.transform.translation.z = marker_info.markers.at(i).pose.pose.position.z;
    
    camera_H_marker_msg.transform.rotation.x = marker_info.markers.at(i).pose.pose.orientation.x;
    camera_H_marker_msg.transform.rotation.y = marker_info.markers.at(i).pose.pose.orientation.y;
    camera_H_marker_msg.transform.rotation.z = marker_info.markers.at(i).pose.pose.orientation.z;
    camera_H_marker_msg.transform.rotation.w = marker_info.markers.at(i).pose.pose.orientation.w;

    // Send the transformation:
    if(this->publish_all){
      this->tfBroadcaster_->sendTransform(camera_H_marker_msg);
    } else if(std::find(markerList.begin(), markerList.end(), marker_info.markers.at(i).id) != markerList.end()){
      this->tfBroadcaster_->sendTransform(camera_H_marker_msg);
    }
  }
}

void ArucoBroadcaster::get_marker_tf(const std::shared_ptr<aruco_broadcaster::srv::GetMarkerTf::Request> request, std::shared_ptr<aruco_broadcaster::srv::GetMarkerTf::Response> response){
  try{
    response->ret = this->tfBuffer_->lookupTransform(request->parent, request->marker_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }
}

void ArucoBroadcaster::init_params(){
  //Checking the params about the list:

  if(this->has_parameter("markerList")){
    this->markerList_param = this->get_parameter("markerList");
    this->markerList = this->markerList_param.as_integer_array();
    // To solve the ROS2 inability to use empty list/arrays as parameter values in YAML files:
    if(this->markerList.at(0) == 0){
      this->markerList.erase(this->markerList.begin());
    }
    if(this->markerList.empty()){
      RCLCPP_WARN_STREAM(this->get_logger(), "Aruco broadcaster will publish all the arucos found.");
      this->publish_all = true;
    }else{
      RCLCPP_INFO_STREAM(this->get_logger(), "markerList has size of: " << markerList.size());
      this->publish_all = false;
    }
  }else{
    RCLCPP_WARN_STREAM(this->get_logger(), "No param named markerList.");
    this->publish_all = true;
  }

  if(this->has_parameter("camera_frame")){
    this->camera_frame_param = this->get_parameter("camera_frame");
    this->camera_frame = this->camera_frame_param.as_string();
    if(this->camera_frame.empty()){
      RCLCPP_FATAL_STREAM(this->get_logger(), "Parameter camera_frame has not reference. The program will exit!!!");
      rclcpp::shutdown();
    }else{
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter camera_frame named: " + camera_frame);
    }
  }else{
    RCLCPP_FATAL_STREAM(this->get_logger(), "No param named camera_frame. The program will exit!!!");
    rclcpp::shutdown();

  }

    if(this->has_parameter("aruco_frame")){
    this->aruco_frame_param = this->get_parameter("aruco_frame");
    this->aruco_frame = this->aruco_frame_param.as_string();
    if(this->aruco_frame.empty()){
      RCLCPP_FATAL_STREAM(this->get_logger(), "Parameter aruco_frame has not reference. The program will exit!!!");
      rclcpp::shutdown();
    }else{
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter aruco_frame named: " + aruco_frame);
    }
  }else{
    RCLCPP_FATAL_STREAM(this->get_logger(), "No param named aruco_frame. The program will exit!!!");
    rclcpp::shutdown();

  }
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  // Creates a shared pointer to an instance of the ArucoBroadcaster class:
  auto my_ArucoBroadcaster = std::make_shared<ArucoBroadcaster>();
  rclcpp::spin(my_ArucoBroadcaster);
  rclcpp::shutdown();
  return 0;
}