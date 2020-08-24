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

**Note: This step is optional and can be skipped if you want to use mapbox instead of google map for visualization in mapviz (which is the current default setting for mapviz).**

1.1. Create the MapProxy configuration folder.
```
mkdir ~/mapproxy
```
1.2. Start MapProxy server
```
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```
1.3. Switch the Source of "tile_map" in mapviz to "google map".  

### 2. Start localization under your workspace
```
source devel/setup.bash
roslaunch ugv_localization global.launch data:={real/bagfile} 
```

Node: you have to replace ```{real/bagfile}``` with either ```real``` (if you want to run the localizer on robot in real-time) or the filename (without extension) of pre-recorded rosbag (if you want to run the localizer with replaying a bagfile under /bag folder).

### 3. Parameter files and other utilities

The message processing related parameters are in "/params/topic_process.yaml". (This is used in "topic_process_node", which integrates all GPS and wheel encoder message conversions into one single node.)

The ekf filter-related parameters are in "/params/global/ekf_odom_imu_gps.yaml".

The clicking points and trajectory generation related parameters are in "/params/reftra_gen.yaml" (the associated launch file is "/launch/gen_traj.launch"). To note, when the parameter "save_traj_file" is set to "None", the generated trajectory as well as its plot will not be saved; if it is set to "save_traj_file : traj_abc" for example, the generated trajectory message will be saved to "/traj/traj_abc.yaml", and its plot will be saved to "/traj/traj_abc.png"
(Note: for starting and stopping clicking points, you need to double-click on mapviz, however, you should not double click too fast because mapviz will respond slow.) 

**Generate trajectory from pre-recorded bag file:**
If you want to generate a trajectory from a pre-recorded bag file, you can simply run the "traj_opt.py" node without needing to use any launch files. For example, if you run like

```
rosrun ugv_localization traj_opt.py _start_time:=1592522650 _end_time:=1592522743 _wps_every_secs=2 _bagfile:=test_01 _topic:=/odometry _save_traj:=traj_abc _vel:=2 _accel:=2
```
(Node: These three parameters above are optional: save_traj, vel, accel)

It will generate a trajectory with velocity of 2 m/s and acceleration of 2 m/s^2, using the "/odometry" topic (message type: Odometry) in bag file "/bag/test_01.bag" between 1592522650 and 1592522743 (ROS time), and save it to "/traj/traj_abc.yaml". The plot of trajectory will be saved to "/traj/traj_abc.png". (To note, the waypoints are sampled every 2 seconds here, which is specified with the parameter "wps_every_secs".)

**Refine current trajectory with InteractiveMarker in Rviz:**
If you want to refine the generated trajectory by moving clicked waypoints, you should follow these steps:

1). Make sure the paramter "enable_refine" is set to True in /params/reftra_gen.yaml

2). After clicking points in mapviz, you will see a series of blue cubic markers in Rviz, located at where you clicked.

3). Move each of the cubic markers on the X-Y plane by draging the arrows around the marker.

4). After moving each marker (waypoints) to your desired locations in Rviz, you can left-click on whichever marker to re-generate trajectory. (The trajectory message will be re-published to the same topic, and its yaml file as well as its plot will be saved to the same path if you specified one.)



