#include "ur_chesslab/chesslab.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <sstream>
#include <type_traits>

Chesslab::Chesslab()
{
    // Predefined sizes
    this->chessboard_height = 0.04;
    this->short_height = 0.04; //pawns
    this->middle_height = 0.06; //tower, horses and knights
    this->tall_height = 0.08; //queen, king 

    this->publish_camera_markers = false;

    resetScene();
}   

void Chesslab::setPublishCameraMarkers(const bool value)
{
    this->publish_camera_markers = value;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Chesslab]: Setting publish_camera_markers to %s",  this->publish_camera_markers ? "true" : "false");
}

void Chesslab::resetScene(void)
{
    // Set markers
    this->marker_vec.clear();

    //chessboard
    //(correction in x direction of the chess frame - equivalent to the correction in flat_chessboard.sdf file in y direction)
    this->chessboard = createMarker("chess_frame", "chess_pieces", 0, 
        "package://chesslab_setup2/models/flat_chessboard/meshes/flat_chessboard.dae", true,
        0.0, 0.0, 0.0, 0.0, 0.0, 3.1415927);
    this->marker_vec.push_back(this->chessboard);

    //Id made coincident with aruco marker id
    //Black pawns: 201 to 208
    //Black towers: 209, 210
    //Black horses: 211, 212
    //Black kniwhts: 213, 214
    //Black queen and kings: 215, 216

    //marker of kingB placed w.r.t chess frame
    this->kingB = createMarker("chess_frame", "chess_pieces_ns", 216,
        "package://chesslab_setup2/models/kingB/meshes/kingB.dae", true,
        -0.175, 0.025, this->tall_height/2, 0.0, 0.0, 0.0);
    this->marker_vec.push_back(this->kingB);

    //Id made coincident with aruco marker id
    //White pawns: 301 to 308
    //White towers: 309, 310
    //White horses: 311, 312
    //White knights: 313, 314
    //White queen and kings: 315, 316

    //marker of kingW (piece id 316) could be placed with the x and y coordinates of the tf aruco_316 and 0.04 m below.
    //this->kingW = createMarker("aruco_316", "chess_pieces_ns", 316,
    //    "package://chesslab_setup2/models/kingW/meshes/kingW.dae", true,
    //    0.0, 0.0, -0.04, 0.0, 0.0, 0.0, 1.0,
    //    1.0, 1.0, 1.0,
    //    0.0, 0.0, 0.0, 1.0);
    //this->marker_vec.push_back(this->kingW);

    this->kingW = createMarker("chess_frame", "chess_pieces_ns", 316,
        "package://chesslab_setup2/models/kingW/meshes/kingW.dae", true,
        0.175, 0.025, this->tall_height/2, 0.0, 0.0, 0.0);
    this->marker_vec.push_back(this->kingW);

    //Camera
    if (this->publish_camera_markers)
    {
        //marker with coordinates made coincident with the camera_color_optical_frame
        this->camera = createMarker("camera_color_optical_frame", "camera_ns", 998, 
            "package://chesslab_setup2/models/realsense_camera/meshes/d435.dae", true,
            0.0, 0.0, 0.0, 0.0, 0.0, 3.1415927);
        this->marker_vec.push_back(this->camera);

        // Camera support
        this->camera_support = createMarker("chess_frame", "camera_ns", 999,
            "package://chesslab_setup2/models/camera_support/meshes/camera_support.dae", true,
            0.0, -0.27, 0.005, 0.0, 0.0, 0.0);
        this->marker_vec.push_back(this->camera_support);
    }
    //fill map
    for(long unsigned int i=0; i<this->marker_vec.size();i++){
        aruco2rviz_map.insert(std::pair<unsigned int,unsigned int>(this->marker_vec[i].id, i));
    }
}

Marker Chesslab::createMarker(std::string _frame_id, std::string _ns, int _id, 
    std::string _mesh, bool _embedded_materials,
    double _x, double _y, double _z, double _rx, double _ry, double _rz, 
    double _scale_x, double _scale_y, double _scale_z, 
    double _color_r, double _color_g, double _color_b, double _color_a)
{  
    Marker marker;
    marker.header.frame_id = _frame_id;
    marker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    marker.ns = _ns;
    marker.id = _id;

    marker.type = Marker::MESH_RESOURCE;
    marker.action = Marker::ADD;
    marker.mesh_resource = _mesh;
    marker.mesh_use_embedded_materials = _embedded_materials; //if true the r,g,b values are overriden

    marker.pose.position.x = _x;
    marker.pose.position.y = _y;
    marker.pose.position.z = _z;


    tf2::Quaternion q;
    q.setRPY(_rx, _ry, _rz);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = _scale_x;
    marker.scale.y = _scale_y;
    marker.scale.z = _scale_z;

    marker.color.r = _color_r;
    marker.color.g = _color_g;
    marker.color.b = _color_b;
    marker.color.a = _color_a;

    //marker.lifetime = rclcpp::Duration(1s);
    marker.frame_locked = true;

    return marker;
}

bool Chesslab::updateMarker(int _id, double _x, double _y, double _z, double _qx, double _qy, double _qz, double _qw, std::string _frame_id)
{
    bool found=false;

    //int i = aruco2rviz_map.find(_id)->second;
    for(size_t i=0; i<this->marker_vec.size(); i++) {
        if (this->marker_vec[i].id == _id)
        {
            this->marker_vec[i].pose.position.x = _x;
            this->marker_vec[i].pose.position.y = _y;
            this->marker_vec[i].pose.position.z = _z;

            this->marker_vec[i].pose.orientation.x = _qx;
            this->marker_vec[i].pose.orientation.y = _qy;
            this->marker_vec[i].pose.orientation.z = _qz;
            this->marker_vec[i].pose.orientation.w = _qw;
            this->marker_vec[i].header.frame_id = _frame_id;
            this->marker_vec[i].header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

            found = true;
            break;
        }
    }

    return found;
}


std::vector<Marker> Chesslab::getMarkerVector(void)
{
    return this->marker_vec;
}


std::map<unsigned int, unsigned int> Chesslab::getMarkerIdMap(void)
{
    return this->aruco2rviz_map;
}