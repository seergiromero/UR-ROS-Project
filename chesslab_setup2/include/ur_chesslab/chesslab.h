#ifndef CHESSLAB
#define CHESSLAB

#include <cstdio>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

// Chessboard setup class
// Pieces will be located w.r.t /chess_frame located at the center of the board on the top surface
class Chesslab 
{
    private:

        // Predefined sizes
        double chessboard_height;
        double short_height;
        double middle_height;
        double tall_height;
    
        bool publish_camera_markers;

        // Markers
        Marker camera;
        Marker camera_support;
        Marker chessboard;
        Marker kingW;
        Marker kingB;
        Marker pawn_W1;
        std::vector<Marker> marker_vec;
        std::map<unsigned int, unsigned int> aruco2rviz_map;

        // Fill maker function
        Marker createMarker(std::string _frame_id, std::string _ns, int _id,
                            std::string _mesh, bool _embedded_materials,
                            double _x, double _y, double _z, double _rx, double _ry, double _rz,  
                            double _scale_x=1.0, double _scale_y=1.0, double _scale_z=1.0, 
                            double _color_r=0.0, double _color_g=0.0, double _color_b=0.0, double _color_a=1.0);

    public:
        Chesslab();
        void resetScene(void);
        std::vector<Marker> getMarkerVector();
        std::map<unsigned int, unsigned int> getMarkerIdMap();
        bool updateMarker(int _id, double _x, double _y, double _z, 
                          double _qx, double _qy, double _qz, double _qw, std::string _frame_id);
        void setPublishCameraMarkers(const bool value);
};

#endif // CHESSLAB
