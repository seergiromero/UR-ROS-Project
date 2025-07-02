/* *************************************************************************\
  Copyright 2020 Institute of Industrial and Control Engineering (IOC)
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
  Desc:   cameraPosFinder class to find a camera position wrt a fixed frame
*/


#ifndef CAMERAPOSFINDER_HPP
#define CAMERAPOSFINDER_HPP
// C++
#include <vector>
#include <cstdint>

// custom class
#include "transformpool.hpp"

// Eigen stuff
#include <Eigen/Geometry>

// ROS stuff
#include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/Int64.hpp>
// #include <std_srvs/srv/set_bool.hpp>
#include <aruco_msgs/msg/marker.hpp>
#include <aruco_msgs/msg/marker_array.hpp>

// #include <tablesens/srv/configure_from_service.hpp>


class CameraPosFinder : public rclcpp::Node {

        struct markerPosRef{
                Eigen::Affine3d fixRef;
                transformPool poolT;
        };
public:
        // Constructor:
        CameraPosFinder();

        // Methods:
        void findCamera(unsigned int iterations);

private:

        //Subscribers:
        rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr obtainMarkers;
       
        // Map:
        std::map<uint,markerPosRef> markersFixRefs;

        // Counters:
        uint64_t iteration, maxIter;

        // Capturing:
        bool capturing;
        void calculate_Pose(const aruco_msgs::msg::MarkerArray &msg);

        // State:
        bool configured;

        bool configureFromParams(); // prepare the class
        bool captureMarkersStart(uint64_t elem);
        void calculateFinalCameraPosition();

        // void callback_number(const std_msgs::Int64& msg);

        // bool callback_reset_counter(std_srvs::SetBool::Request &req,
        //                                 std_srvs::SetBool::Response &res);
};

#endif // CAMERAPOSFINDER_HPP
