# URDF Snippets for ROS+Gazebo Simulation

## Overview

This repository contains a ROS package to simplify the robot URDF model createion process for Gazebo simulation and RViz visulization using XACRO snippets.

## Introduction

Unified Robot Description Format (URDF) uses XML tags for represnting robots. XACRO (Xml+mACRO) helps in creating parametrized URDF files. The URDF files can be used by ROS to display robot models inside RViz, furthermore, it can be used to create robot models to be used by Gazebo. This package contains a collection of useful XACRO snippets created to further simpliy the URDF creation. For example to create a robot model it is as simple as the following code [An example code file can be found within the urdf directory of the package](./urdf/test_snippets.urdf.xacro).

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="Test_Robot">

    <!-- Include the URDF snippets for visual/inertia/links/joints tags -->
    <xacro:include filename="./snippets.urdf.xacro"/>

    <!-- Create links: w:x-axis, l:y-axis, h:z-axis -->
    <link name="base_link"/>
    <xacro:shape_link name="chassis" type="box" m="1" w="0.4" l="0.2" h="0.05" rgba="0 0 1 ${alpha}" color="Blue"/>
    <xacro:shape_link name="left_wheel" type="cylinder" m="0.5" r="0.1" h="0.05" rgba="0 1 0 1" color="Green"/>
    <xacro:shape_link name="right_wheel" type="cylinder" m="0.5" r="0.1" h="0.05" rgba="0 1 0 1" color="Green"/>
    <xacro:shape_link name="front_castor" type="sphere" m="0.1" r="0.05" rgba="1 0 0 1" color="Red"/>
    <!-- Create Joints: -->
    <xacro:shape_joint type="fixed" parent="base_link" child="chassis" xyz="0.1 0 0"/>
    <xacro:shape_joint type="continuous" parent="chassis" child="left_wheel" axis="0 0 -1" xyz="-0.1 0.125 0" rpy="1.57 0 0"/>
    <xacro:shape_joint type="continuous" parent="chassis" child="right_wheel" axis="0 0 -1" xyz="-0.1 -0.125 0" rpy="1.57 0 0"/>
    <xacro:shape_joint type="fixed" parent="chassis" child="front_castor" xyz="0.15 0 -0.05"/>
    <xacro:castor_friction name="front_castor" mu1="0" mu2="0" kp="1e9" kd="0" fdir="0 0 0"/>
    <!-- Add Sensors -->
    <xacro:robot_drive_diff chassis="base_link" left="joint_chassis_left_wheel" right="joint_chassis_right_wheel" l="0.25" d="0.1" cmdtopic="cmd_vel" odomtopic="odom"/>
    <xacro:sensor_imu_ros name="imu" parent="chassis" topic="/imu" rate="20" noise="0" xyz="0 0 0.05" visulize="false"/>
    <xacro:sensor_rgb_camera name="front_camera" parent="imu" width="640" height="480" fps="10" xyz="0.15 0 0.15"/>
    <xacro:sensor_lidar_3d name="velodyne_lidar" parent="front_camera" topic="/velodyne_points" rate="20" noise="0" minRange="0.1" maxRange="100" samples="360" xyz="0 0 0.15" visulize="false"/>

</robot>
```

**Explanation**
The above code create a simple two wheel differential drive mobile robot with a 3D LiDAR, a RGB camera and an IMU. The user can simply include the snippets file and use the sample macros to create various links, joints and attach sensors to the robot.

# Usage

The package can be downloaded into your workspace source folder (e.g. ~/catkin/src) as follows:

```
git clone https://github.com/dringakn/gazebo_simulation.git
or using SSH
git clone git@github.com:dringakn/gazebo_simulation.git
```

The package related dependencies can be installed from your workspace folder (e.g. ~/catkin_ws/) using following command

```
rosdep install --from-paths src --ignore-src -y -r
catkin build
or
catkin_make
source devel/setup.bash
```

The following example launch file can be used to test the generated URDF file.

```
roslaunch gazebeo_simulation test_urdf.launch
```
