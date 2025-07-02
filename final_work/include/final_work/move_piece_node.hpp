#ifndef FINAL_WORK_MOVE_PIECE_NODE_HPP_
#define FINAL_WORK_MOVE_PIECE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "kinenikros2/srv/inverse_kinematics.hpp"
#include "final_work/action_client_node.hpp"
#include "final_work/srv/set_aruco_id.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "robotiq_85_gripper_server/srv/gripper_open.hpp"
#include "robotiq_85_gripper_server/srv/gripper_close.hpp"
#include "robotiq_85_gripper_server/srv/gripper_order.hpp"

namespace final_work {

/**
 * @class MovePieceNode
 * @brief Node to detect chess pieces via ArUco TF, compute IK, and
 *        execute pick & place on UR robot with Robotiq gripper.
 */
class MovePieceNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor: sets up service/action clients and loads config.
     */
    MovePieceNode();

    /**
     * @brief pick a piece and place it at target square.
     *
     * @param piece_name   Name of the piece to move (e.g., "kingB").
     * @param square       Destination square in chess notation (e.g., "D2").
     */
    void movePiece(const std::string & piece_name, const std::string & square);

    /**
     * @brief Descend to pick depth, close gripper, and retract.
     *
     * @param pose  Target grasp pose.
     * @return true on success, false otherwise.
     */
    bool pick(const geometry_msgs::msg::Pose & pre_pose, const geometry_msgs::msg::Pose & pose);

    /**
     * @brief Descend to placement depth, open gripper, and retract.
     *
     * @param pose  Target pose for place (center of square).
     * @return true on success, false otherwise.
     */
    bool place(const geometry_msgs::msg::Pose & pose, double offset);

    //=== Internal Services ===//

    /**
     * @brief Add grasp frame to the system for a given ArUco ID.
     *
     * @param aruco_id  ArUco marker ID for the piece to be grasped.
     */
    void callAddGraspFrame(int aruco_id);

    /**
     * @brief Remove grasp frame from the system for a given ArUco ID.
     *
     * @param aruco_id  ArUco marker ID for the piece to be released.
     */
    void callRemoveGraspFrame(int aruco_id);

    //=== Client Initialization ===//

    /**
     * @brief Initialize ROS service/action clients and TF buffer.
     */
    void initializeClients();

    /**
     * @brief Load chessboard square positions from YAML into memory.
     */
    void loadBoardConfig();

    //=== Getter Functions ===//

    /**
     * @brief Lookup piece pose using ArUco marker.
     *
     * @param piece_name  Name of the chess piece (e.g., "pawnB1").
     * @return Optional Pose in robot base frame, or std::nullopt on failure.
     */
    std::optional<geometry_msgs::msg::Pose> getPiecePose(const std::string & piece_name);

    /**
     * @brief Get board square pose from loaded configuration.
     *
     * @param square  Chess square identifier (e.g., "E4").
     * @return Optional Pose in base frame, or std::nullopt on error.
     */
    std::optional<geometry_msgs::msg::Pose> getBoardPose(const std::string & square);

    /**
     * @brief Returns a quaternion representing a downward-facing orientation.
     *        Roll 180° flips tool, Yaw 45° aligns tool orientation.
     * 
     * @return Quaternion for vertical orientation.
     */
    geometry_msgs::msg::Quaternion getVerticalOrientation() const;

    //=== Inverse Kinematics ===//

    /**
     * @brief Call the inverse kinematics service to compute joint angles for a target Pose.
     *
     * @param target  Desired end-effector pose in base frame.
     * @param joints  Output vector of joint positions upon success.
     * @return true if IK succeeded, false otherwise.
     */
    bool callIK(const geometry_msgs::msg::Pose & target, std::vector<double> & joints);

    //=== Robot Motion ===//

    /**
     * @brief Move robot to a pose with a vertical lift offset (pre/post approach).
     *
     * @param pose    Base pose to approach.
     * @param z_lift  Vertical offset (positive lifts up, negative descends).
     * @return true on successful motion, false otherwise.
     */
    bool moveToPose(const geometry_msgs::msg::Pose & pose, double z_lift);


    /**
     * @brief Move robot to the home pose.
     *
     * @return true on successful motion, false otherwise.
     */

    bool moveHome();

    //=== Gripper Functions ===//

    /**
     * @brief Open the Robotiq gripper.
     * @return true on success, false otherwise.
     */
    bool openGripper();

    /**
     * @brief Send a GripperOrder to close the Robotiq gripper with controlled force.
     * @return true on success, false on failure.
     */
    bool closeGripper();

    //=== Helper Functions ===//

    /**
     * @brief Apply camera correction to the pose.
     * 
     * @param pose  Input pose to be corrected.
     * @return Corrected pose.
     */
    geometry_msgs::msg::Pose applyCameraCorrection(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Get pre-grasp pose for a piece using ArUco marker.
     *
     * @param piece_name  Name of the chess piece (e.g., "pawnB1").
     * @return Optional Pose for pre-grasp position.
     */
    std::optional<geometry_msgs::msg::Pose> getPrePiecePose(const std::string & piece_name);

    /**
     * @brief Lookup piece pose using a specific frame.
     *
     * @param piece_name   Name of the piece.
     * @param frame_prefix Prefix for the frame (e.g., "base_").
     * @return Optional Pose in specified frame.
     */
    std::optional<geometry_msgs::msg::Pose> lookupPiecePose(const std::string & piece_name, const std::string & frame_prefix);

    /**
     * @brief Get placement offset for a piece.
     *
     * @param piece_name   Name of the piece (e.g., "kingB").
     * @return Placement offset in meters.
     */
    double getPlaceOffset(const std::string & piece_name);

    /**
     * @brief Get ArUco ID for a piece.
     *
     * @param piece_name   Name of the piece (e.g., "kingB").
     * @return ArUco ID.
     */
    int getArucoId(const std::string & piece_name);

    //=== Motion Control ===//

    /**
     * @brief Interpolate linear motion between two poses.
     *
     * @param start  Starting pose.
     * @param end    End pose.
     * @param steps  Number of interpolation steps.
     * @return A vector of poses for the linear motion.
     */
    std::vector<geometry_msgs::msg::Pose> interpolateLinearMotion(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end, int steps);

    /**
     * @brief Move the robot along a linear trajectory between two poses.
     *
     * @param start_pose   Starting pose.
     * @param end_pose     End pose.
     * @param steps        Number of steps for interpolation.
     * @return true if motion completed, false otherwise.
     */
    bool moveLinear(const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& end_pose, int steps);

    //=== Clients & Interfaces ===//
    rclcpp::Client<kinenikros2::srv::InverseKinematics>::SharedPtr ik_client_;
    std::shared_ptr<URTrajectoryClient> trajectory_client_;
    rclcpp::Client<robotiq_85_gripper_server::srv::GripperOpen>::SharedPtr  gripper_open_client_;
    rclcpp::Client<robotiq_85_gripper_server::srv::GripperClose>::SharedPtr gripper_close_client_;
    rclcpp::Client<robotiq_85_gripper_server::srv::GripperOrder>::SharedPtr gripper_order_client_;

    //=== TF2 Interfaces ===//
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //=== Configuration & State ===//
    std::unordered_map<int, std::string> aruco_to_piece_= {
        {201, "pawnB1"}, {202, "pawnB2"}, {203, "pawnB3"}, {204, "pawnB4"},
        {205, "pawnB5"}, {206, "pawnB6"}, {207, "pawnB7"}, {208, "pawnB8"},
        {209, "rookB1"}, {210, "rookB2"}, {211, "knightB1"}, {212, "knightB2"},
        {213, "bishopB1"}, {214, "bishopB2"}, {215, "queenB"}, {216, "kingB"},
        {301, "pawnW1"}, {302, "pawnW2"}, {303, "pawnW3"}, {304, "pawnW4"},
        {305, "pawnW5"}, {306, "pawnW6"}, {307, "pawnW7"}, {308, "pawnW8"},
        {309, "rookW1"}, {310, "rookW2"}, {311, "knightW1"}, {312, "knightW2"},
        {313, "bishopW1"}, {314, "bishopW2"}, {315, "queenW"}, {316, "kingW"},
    };  ///< Map ArUco ID to piece name
    std::string board_positions_path_;                    ///< Path to board YAML file
    YAML::Node board_positions_;                          ///< Loaded board positions
    std::unordered_map<std::string,double> piece_place_height_= {
        {"king", 0.25},
        {"queen", 0.25},
        {"rook", 0.23},
        {"bishop", 0.23},
        {"knight", 0.23},
        {"pawn", 0.21}
    };

    rclcpp::Client<final_work::srv::SetArucoId>::SharedPtr add_client_;
    rclcpp::Client<final_work::srv::SetArucoId>::SharedPtr remove_client_;

    //=== Motion Offsets (meters) ===//
    const double pre_pick_offset_   = 0.07;  ///< Lift above piece before descent
    const double pre_place_offset_  = 0.35;  ///< Lift above square before descent
    const double pick_depth_offset_ = 0.1;   ///< Descent depth into piece
    const double pre_place_offsetV2_ = 0.25; ///< Not used
};

} // namespace final_work

#endif // FINAL_WORK_MOVE_PIECE_NODE_HPP_
