#ifndef UR_CHESSLAB_DEMO
#define UR_CHESSLAB_DEMO

#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "ur_chesslab/chesslab.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "chesslab_setup2_interfaces/srv/set_rob_conf.hpp"
#include "chesslab_setup2_interfaces/srv/set_obj_pose.hpp"
#include "chesslab_setup2_interfaces/srv/attach_obj.hpp"
#include "chesslab_setup2_interfaces/srv/detach_obj.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

using JointState = sensor_msgs::msg::JointState;
using Pose = geometry_msgs::msg::Pose;
using srvEmpty = std_srvs::srv::Empty;
using srvSetRobConf = chesslab_setup2_interfaces::srv::SetRobConf;
using srvSetObjPose = chesslab_setup2_interfaces::srv::SetObjPose;
using srvAttachObj = chesslab_setup2_interfaces::srv::AttachObj;
using srvDetachObj = chesslab_setup2_interfaces::srv::DetachObj;

class URChesslabDemo : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Clock clock;
        void timerCallback();
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

        bool publish_markers_only_;
        bool publish_camera_markers_;

        // Maker providers
        Chesslab chesslab;

        // Publishing objects
        MarkerArray marker_array;
        JointState joint_state_arm;

        // Publishers / Subscribers
        rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub;
        rclcpp::Publisher<JointState>::SharedPtr joint_state_arm_pub;

        // Services Servers
        rclcpp::Service<srvEmpty>::SharedPtr serviceResetScene;
        bool resetSceneCallback(const std::shared_ptr<srvEmpty::Request>  &req,
                                const std::shared_ptr<srvEmpty::Response> &res);

        rclcpp::Service<srvSetRobConf>::SharedPtr serviceRobConf;
        bool robConfCallback(const std::shared_ptr<srvSetRobConf::Request>  &req,
                             const std::shared_ptr<srvSetRobConf::Response> &res);

        rclcpp::Service<srvSetObjPose>::SharedPtr serviceObjPose;
        bool objPoseCallback(const std::shared_ptr<srvSetObjPose::Request>  &req,
                        const std::shared_ptr<srvSetObjPose::Response> &res);

        rclcpp::Service<srvAttachObj>::SharedPtr serviceAttachObj;
        bool attachObjCallback(const std::shared_ptr<srvAttachObj::Request>  &req,
                        const std::shared_ptr<srvAttachObj::Response> &res);

        rclcpp::Service<srvDetachObj>::SharedPtr serviceDetachObj;
        bool detachObjCallback(const std::shared_ptr<srvDetachObj::Request>  &req,
                        const std::shared_ptr<srvDetachObj::Response> &res);

    public:
        URChesslabDemo();
};

#endif // UR_CHESSLAB_DEMO
