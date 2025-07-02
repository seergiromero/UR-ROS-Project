#include "ur_chesslab/chesslab_setup2_demo.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"


URChesslabDemo::URChesslabDemo() : Node("chesslab_setup2_demo"), clock(RCL_ROS_TIME), publish_markers_only_(false)
{
  // == Load parameters ==
  this->declare_parameter<bool>("publish_markers_only", false);
  this->get_parameter("publish_markers_only", this->publish_markers_only_);
  this->declare_parameter<bool>("publish_camera_markers", false);
  this->get_parameter("publish_camera_markers", this->publish_camera_markers_);
  this->chesslab.setPublishCameraMarkers(this->publish_camera_markers_); 
  this->chesslab.resetScene(); 

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // == Load markers from own providers ==
  this->marker_array.markers.clear();
  std::vector<Marker> chesslab_markers = this->chesslab.getMarkerVector();
  this->marker_array.markers.insert(this->marker_array.markers.end(), chesslab_markers.begin(), chesslab_markers.end());

  // == Publishers/Subscribers ==
  this->markers_pub = this->create_publisher<MarkerArray>("markers_chesslab_demo", 10);

  if (!this->publish_markers_only_)
  {

    // == Set JointState ==
    this->joint_state_arm.header.frame_id="base";
    this->joint_state_arm.name.resize(7);
    this->joint_state_arm.name[0] ="shoulder_pan_joint";
    this->joint_state_arm.name[1] ="shoulder_lift_joint";
    this->joint_state_arm.name[2] ="elbow_joint";
    this->joint_state_arm.name[3] ="wrist_1_joint";
    this->joint_state_arm.name[4] ="wrist_2_joint";
    this->joint_state_arm.name[5] ="wrist_3_joint";
    this->joint_state_arm.name[6] ="robotiq_85_left_knuckle_joint";
    this->joint_state_arm.position.resize(7);
    this->joint_state_arm.position[0] = 0.0;
    this->joint_state_arm.position[1] = -3.14159/2.0;
    this->joint_state_arm.position[2] = 0.0;
    this->joint_state_arm.position[3] = -3.14159/2.0;
    this->joint_state_arm.position[4] = 0.0;
    this->joint_state_arm.position[5] = 0.0;
    this->joint_state_arm.position[6] = 0.0;

    // == Publishers/Subscribers ==
    //this->joint_state_arm_pub = this->create_publisher<JointState>("joint_states", 10);
    this->joint_state_arm_pub = this->create_publisher<JointState>("ur_joint_states", 10);

    // == Service Servers ==
    // Own server: Reset scene
    this->serviceResetScene = this->create_service<srvEmpty>(
      "/chesslab_setup2_demo/reset_scene", std::bind(&URChesslabDemo::resetSceneCallback, this, 
                        std::placeholders::_1, std::placeholders::_2));

    // Own server: Set Robot COnfiguration
    this->serviceRobConf = this->create_service<srvSetRobConf>(
      "/chesslab_setup2_demo/set_robot_config", std::bind(&URChesslabDemo::robConfCallback, this, 
                        std::placeholders::_1, std::placeholders::_2));

    // Own server: Set Object Pose
    this->serviceObjPose = this->create_service<srvSetObjPose>(
      "/chesslab_setup2_demo/set_object_pose", std::bind(&URChesslabDemo::objPoseCallback, this, 
                        std::placeholders::_1, std::placeholders::_2));

    // Own server: Attach Obj
    this->serviceAttachObj = this->create_service<srvAttachObj>(
      "/chesslab_setup2_demo/attach_obj", std::bind(&URChesslabDemo::attachObjCallback, this, 
                        std::placeholders::_1, std::placeholders::_2));
    // Own server: Attach Obj
    this->serviceDetachObj = this->create_service<srvDetachObj>(
      "/chesslab_setup2_demo/detach_obj", std::bind(&URChesslabDemo::detachObjCallback, this, 
                        std::placeholders::_1, std::placeholders::_2));
  }

  // == Declare timerCallback ==
  this->timer = this->create_wall_timer(50ms, [this](){ this->timerCallback(); }
  );
}

void URChesslabDemo::timerCallback()
{
  // Get updated markers
  this->marker_array.markers.clear();
  std::vector<Marker> chesslab_markers = this->chesslab.getMarkerVector();
  this->marker_array.markers.insert(this->marker_array.markers.end(), chesslab_markers.begin(), chesslab_markers.end());
  
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: Publishing %lu markers.", this->marker_array.markers.size());
  for (uint i = 0; i < this->marker_array.markers.size(); i++)
    this->marker_array.markers[i].header.stamp = this->get_clock()->now();
  this->markers_pub->publish(this->marker_array);
  
  if (!this->publish_markers_only_)
  {
    this->joint_state_arm.header.stamp = this->get_clock()->now();
    this->joint_state_arm_pub->publish(this->joint_state_arm);
  }
}

//! Service that sets the initial set-up
bool URChesslabDemo::resetSceneCallback(
  const std::shared_ptr<srvEmpty::Request>  &req,
  const std::shared_ptr<srvEmpty::Response> &res)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: Received ResetScene request. Setting the initial scene.");

  // Unused
  (void)req;
  (void)res;

  //Reset the pieces locations - this must be done after dettaching
  this->chesslab.resetScene();

  //Reset the robot in rviz
  for(int idx=0; idx<6; idx++)
    this->joint_state_arm.position[idx] = 0.0;

  return true;
}


//! Service that sets the robot config (joint states)
bool URChesslabDemo::robConfCallback(const std::shared_ptr<srvSetRobConf::Request>  &req,
                                     const std::shared_ptr<srvSetRobConf::Response> &res)
{
  // Check
  if (req->conf.size()!=6) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: Received wrong number of Joint States (set_robot_conf).");
    return false;
  }
  
  // Unused
  (void)res;

  //set joint states
  this->joint_state_arm.position[0] = req->conf[0];
  this->joint_state_arm.position[1] = req->conf[1];
  this->joint_state_arm.position[2] = req->conf[2];
  this->joint_state_arm.position[3] = req->conf[3];
  this->joint_state_arm.position[4] = req->conf[4];
  this->joint_state_arm.position[5] = req->conf[5];

  // Publish current info
  timerCallback();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: New robot configuration set to: (%f, %f, %f, %f, %f, %f, %f).", 
    req->conf[0],req->conf[1],req->conf[2],req->conf[3],req->conf[4],req->conf[5],req->conf[6]);

  return true;
}

//! Service that sets the pose of an object in rviz (w.r.t. chess_frame)
bool URChesslabDemo::objPoseCallback(const std::shared_ptr<srvSetObjPose::Request>  &req,
                                     const std::shared_ptr<srvSetObjPose::Response> &res)
{
  // Unused
  (void)res;

  // Check in the marker array
  bool found = this->chesslab.updateMarker(req->objid, 
    req->p.position.x, req->p.position.y, req->p.position.z, 
    req->p.orientation.x, req->p.orientation.y, req->p.orientation.z, req->p.orientation.w, "chess_frame");

  if (!found)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: Object ID not found.");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[chesslab_setup2_demo]: Object with ID %d set to pose = (%f, %f, %f, %f, %f, %f, %f)", 
    req->objid, req->p.position.x, req->p.position.y, req->p.position.z, 
    req->p.orientation.x, req->p.orientation.y, req->p.orientation.z, req->p.orientation.w );

  // Publish current info
  timerCallback();
  return true;
}

//! Service that attachs an obstacle to the gripper
bool URChesslabDemo::attachObjCallback(const std::shared_ptr<srvAttachObj::Request>  &req,
                                     const std::shared_ptr<srvAttachObj::Response> &res)
{
  RCLCPP_INFO(get_logger(), "Attach obstacle with aruco mark %d to gripper", req->objarucoid);
  // Unused
  (void)res;

  //Sets the transform that attaches the object to the robot gripper in rviz
  //it sets the frame of team_A__gripper_right_pad (or  team_B__gripper_right_pad) as the reference frame for the object
  //then computes the from this reference frame to the object reference frame

  //current transform from chessboard to object
  int i = chesslab.getMarkerIdMap().find(req->objarucoid)->second;
  tf2::Transform chessboard2obj;
  auto objpos=chesslab.getMarkerVector()[i].pose.position;
  tf2::Vector3 v(objpos.x, objpos.y, objpos.z);
  auto objori=chesslab.getMarkerVector()[i].pose.orientation;
  tf2::Quaternion q(objori.x, objori.y, objori.z, objori.w);
  chessboard2obj.setOrigin(v);
  chessboard2obj.setRotation(q);

  //transform from chessboard to robot link tool0
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
      transformStamped = tf_buffer_->lookupTransform("chess_frame", "tool0", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex){
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chessboard2tool0;
  //tf2::convert(transformStamped, chessboard2tool0);
  auto objpos2=transformStamped.transform.translation;
  tf2::Vector3 v2(objpos2.x, objpos2.y, objpos2.z);
  auto objori2=transformStamped.transform.rotation;
  tf2::Quaternion q2(objori2.x, objori2.y, objori2.z, objori2.w);
  chessboard2tool0.setOrigin(v2);
  chessboard2tool0.setRotation(q2);

  //transform from robot gripper pad to object
  tf2::Transform tool2obj = chessboard2tool0.inverse() * chessboard2obj;

  //set the transform from right pad frame to oject
  this->chesslab.updateMarker(chesslab.getMarkerVector()[i].id, 
      tool2obj.getOrigin().getX(),
      tool2obj.getOrigin().getY(),
      tool2obj.getOrigin().getZ(),
      tool2obj.getRotation().getX(),
      tool2obj.getRotation().getY(),
      tool2obj.getRotation().getZ(),
      tool2obj.getRotation().getW(),
      "tool0");
  return true;
}


//! Service that detachs the obstacle from the gripper
bool URChesslabDemo::detachObjCallback(const std::shared_ptr<srvDetachObj::Request>  &req,
                                     const std::shared_ptr<srvDetachObj::Response> &res)
{
  RCLCPP_INFO(get_logger(), "Detach obstacle with aruco mark %d from gripper", req->objarucoid);
  // Unused
  (void)res;

  //Sets the transform that attaches the object to the robot gripper in rviz
  //it sets the frame of team_A__gripper_right_pad (or  team_B__gripper_right_pad) as the reference frame for the object
  //then computes the from this reference frame to the object reference frame

  //current transform from tool to object
  int i = chesslab.getMarkerIdMap().find(req->objarucoid)->second;
  tf2::Transform tool2obj;
  auto objpos=chesslab.getMarkerVector()[i].pose.position;
  tf2::Vector3 v(objpos.x, objpos.y, objpos.z);
  auto objori=chesslab.getMarkerVector()[i].pose.orientation;
  tf2::Quaternion q(objori.x, objori.y, objori.z, objori.w);
  tool2obj.setOrigin(v);
  tool2obj.setRotation(q);

  //transform from chessboard to robot link tool0
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
      transformStamped = tf_buffer_->lookupTransform("chess_frame", "tool0", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex){
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chessboard2tool0;
  //tf2::convert(transformStamped, chessboard2tool0);
  auto objpos2=transformStamped.transform.translation;
  tf2::Vector3 v2(objpos2.x, objpos2.y, objpos2.z);
  auto objori2=transformStamped.transform.rotation;
  tf2::Quaternion q2(objori2.x, objori2.y, objori2.z, objori2.w);
  chessboard2tool0.setOrigin(v2);
  chessboard2tool0.setRotation(q2);

  //transform from chessboard to object
  tf2::Transform chessboard2obj = chessboard2tool0 * tool2obj;

  //set the transform from chess_frame to oject
  this->chesslab.updateMarker(chesslab.getMarkerVector()[i].id, 
      chessboard2obj.getOrigin().getX(),
      chessboard2obj.getOrigin().getY(),
      chessboard2obj.getOrigin().getZ(),
      chessboard2obj.getRotation().getX(),
      chessboard2obj.getRotation().getY(),
      chessboard2obj.getRotation().getZ(),
      chessboard2obj.getRotation().getW(),
      "chess_frame");
  return true;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<URChesslabDemo>());
  rclcpp::shutdown();

  return 0;
}
