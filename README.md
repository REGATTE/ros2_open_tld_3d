# ros2_open_tld_3d

## Introduction

This is a ROS2 C++ implementation of OpenTLD that was originally published in MATLAB by Zdenek Kalal. OpenTLD is used for tracking objects in video streams. What makes this algorithm outstanding is that it does not make use of any training data. This implementation is based solely on open source libraries, meaning that you do not need any commercial products to compile or run it.

This version as PCL integration. Using a 3D camera, you can track a point in depth AND in position in the 2D image

Here is the list and description of all ROS parameters used in OpenTLD :

+ *`queue size`* : int, default 100.
Set the queue size for the subscribers.
+ *`Tracking3`* : bool, default true.
Determine if OpenTLD needs only to do 2D tracking of the object
(false) or if a full 3D Tracking using the Point Cloud to send informa-
tion.

+ *``Graphical interface``* : bool, default true.
Set the use of a graphical interface.

+ *`ShowTrajectory`* : bool, default false.
If true, the trajectory of the object in the 2D image will be shown and recorded.

+ *`Trajectory length`* : int, default 50.
If ShowTrajectory is set to true : length of the trajectory shown, in number of positions.

+ *`TimeBetweenFrames`* : double, default 0.1.
Maximum time delay in between now and the frame used by openTLD to have a good latency.

+ *`LoadModel`* : bool, default false.
Set to true, OpenTLD will try to load the model situated at the path provided by the next parameter, Path2Model.

+ *`Path2Model`* : string, default ” /model”.
A path to a previously recorded model to be used by OpenTLD if LoadModel is set to True

+ *`SavingFile`* : string, default ” /model”.
A path where to save the model currently used by OpenTLD, if requested by the user.

Here is a keyboard shortcuts : 

+ *`l`* : toggle learning.

+ *`a`* : toggle alternating mode (if true, detector is switched off when tracker is available).

+ *`e`* : export Model.

+ *`i`* : import Model.

+ *`c`* : clear model, reinitialize the tracking.

+ *`r`* : ask the user for a new object in the current frame.

+ *`n`* : ask the user for a image from the current frame to add in the learning.

# ROS2 3D bouding box object detection from pointclouds
This is a package for a baseline 3D bounding box detection and tracking using pointcloud data. 

It's based on this package https://github.com/praveen-palanisamy/multiple-object-tracking-lidar

### Design and algorithm

1. Detect from clustering
    Unsupervised euclidean cluster extraction
2. Track
    tracking (object ID & data association) with an ensemble of Kalman Filters
3. Classify static and dynamic object

### Installation
`colcon build`

### Usage

`ros2 run ros2_open_tld_3d pcl_object_detection --ros-args --param-file config/config.yaml`

### Topics

Subscriptions:
- Name: `/filtered_clouds`
Type: `sensor_msgs::msg::PointCloud2`

Publishers:

- Name: `viz`     Type:`visualization_msgs::msg::MarkerArray`

- Name: `cluster`
Type: `sensor_msgs::msg::PointCloud2`

- Name: `obj_id`
Type: `std_msgs::msg::Int32MultiArray`