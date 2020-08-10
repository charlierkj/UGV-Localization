# ugv_localization

Description: customized ROS package for localization of unmanned ground vehicle, by fusing with Extended Kalman Filter from external robot_localization package. 

## Dependencies
System: ROS-Kinetic on Ubuntu 16.04. 

External ROS packages: Most of them can be installed by
```sudo apt-get install ros-kinetic-{package_name}```

*  mapviz, mapviz_plugins, tile_map
*  message_filters 
*  robot_localization
*  tf2_ros

Customized ROS packages:

*  imu_3dm_gx4
*  rampage_msgs 
*  ublox

## Download and Build

Please clone the repo into your local machine and checkout the robot_localization branch.
Then move the ugv_localization folder under /src of your local workspace.
Run the following comman to build the package 
```
catkin build ugv_localization
```

## Run the nodes

### 1. Set up the mapviz server for visualizing google map

1.1. Create the MapProxy configuration folder.
```
mkdir ~/mapproxy
```
1.2. Start MapProxy server
```
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

### 2. Start localization under your workspace
```
source devel/setup.bash
roslaunch ugv_localization global.launch data:={real/bagfile} 
```

Node: you have to replace ```{real/bagfile}``` with either ```real``` (if you want to run the localizer on robot in real-time) or the filename (without extension) of pre-recorded rosbag (if you want to run the localizer with replaying a bagfile under /bag folder).
