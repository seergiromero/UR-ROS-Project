tableSens
=========

This package contains a ROS2 node that using a list of ar_track_alvar markers and its relative position with respect to (w.r.t.) a fixed frame determines where is located the camera frame w.r.t. your defined fixed frame . The node has some functionality to configure, start capturing and obtain the proposed position of the camera. When it's configured, it subscribes to the _ar_pose_marker_ topic and capture a fixed number of frames (set by the user) and after calculate the camera position.

How to build it
---------------

The software package is a standard ROS2 package that could be build using colcon/cmake tools. Just download the package from https://gitioc.upc.edu/labs/tablesens.git via git.

- Create your colcon_ws as always.
- Clone the repo in the src dir.

```
git clone https://gitioc.upc.edu/labs/tablesens.git -b ros2

```

- Build it with _colcon build_ in the colcon_ws dir.


How to use it?
--------------

First of all you need to print some [ar_track_alvar markers](http://wiki.ros.org/ar_track_alvar). Verify the size of your print page because many printers reduce the final size. In a yaml file, like the example contained in the conf directory, put the markers (number) that you have fix in your reference setup (markerFixList) and its position w.r.t your defined fixed frame. Also, write the camera frame (_camera_frame_) where the markers are referred (optional).

The launch directory contains a launch file as example. It simply load the parameters defined in the yaml file and run the table calibrator node where finally it will print the proposed position of the camera. 

On your launch file, add a a _static_transform_publisher_ transform in your launch file as:

```
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_broadcaster" args=" posx posy posz qx qy qx qw /your_fixed_frame /your_camera_frame" />
```

and then your camera frame are quite well situated in your setup w.r.t. your fixed frame. 

Pieces of code
--------------

This package has some functionality that can be cherry pick for other projects. One of them is the class *transformpool*, that stores a collection of Eigen::Affine3d transforms and calculate its average in position and rotation.

To calculate the position average it uses the common method of the [Arithmetic Mean](https://en.wikipedia.org/wiki/Arithmetic_mean). For the rotation part, firstly the same previous method was used with the quaternion of the rotation as a vector. This is in the code commented, but a more improved method has been implemented using Eigen Vectors as is proposed [here](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872_2007014421.pdf).


Some notes to use it
--------------------

This code has been tested in limited conditions and with only one camera. It's not fully tested and needs a lot of improve. Some TODO topics to be improved:

* Fully implement its functionality as services so you can incorporate a marker in a robot arm (fixed in the tcp, or in some part of the arm, but a known position), move the arm to be seen by the camera, knowing the position of the marker by the Direct Kinematics and then call the algorithm to obtain where is the camera w.r.t. the robot base frame. This functionality is not fully implemented.

* Currently in the transformpool, the transforms _T_ are considered with the same weidth when the average is calculated. In the function to calculate the final position using all the markers pools the distante (average of all the measured transforms) of the marker to the camera are used to ponderate. This function uses the Arithmetic mean for all the calculus. Still pending to incorporate the Eigen Vector algorithm. 

* More experiments are needed and different setups. However, the pixel size is a critical parameter for the final precision.

