# URDF Snippets for ROS+Gazebo Simulation

![Simulation](./.media/urdf_snippets_package.gif)

## Overview

This repository contains a ROS package to simplify the robot URDF model creation process using XACRO snippets for the purpose of Gazebo simulation and robot model visulization inside ROS RViz.

## Introduction

Unified Robot Description Format ([URDF](http://wiki.ros.org/urdf)) uses XML tags to model various robot elements such as links, joint and sensors. [XACRO](http://wiki.ros.org/xacro)(Xml+mACRO) macros simplifies the robot model creation process by parametrizing the robot model files. The xacro file can be translated into the urdf file by using the following command.

```
xacro filename.urdf.xacro -o filename.urdf
```

The URDF files are used by ROS to display robot models inside RViz, furthermore, it can be used to create robot models for Gazebo simulation.

In order to visulize the robot model in Rviz, the robot URDF file is loaded as a parameter into the ros parameter server through the launch file as follows:

```
    <arg name="model" default="$(dirname)/../urdf/test_snippets.urdf.xacro"/>
    <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />
```

This package contains a collection of useful XACRO snippets to simpliy the robot model creation process for simulattion and visulization purpose.

For example in order to create a simple two wheel differential drive mobile robot equipped with a 3D LiDAR, a RGB color camera and an IMU sensor following xacro file can the used:

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

[The example code file can be found within the urdf directory of the package](./urdf/test_snippets.urdf.xacro).

**Explanation**
The above code creates the URDF model for a differential drive robot named `Test_Robot` in a very compact manner. The robot has **eight links** `base_link, chassis, left_wheel, right_wheel, front_castor, imu, front_camera, and velodyne_lidar`, **seven joints** `joint_base_link_chassis, joint_chassis_left_wheel, joint_chassis_right_wheel, joint_chassis_front_castor, joint_chassis_imu, joint_imu_front_camera, and joint_front_camera_velodyne_lidar`, **three sensors** `imu, front_camera, velodyne_lidar` and **a controller** `robot_diff_drive`.

The `base_link` is a dummy link used as a reference for all other parts of the robot. The other four links are self explanatory. Three other links are created implicitly to represent three sensors. Similarly three joints are implicitly created to join these sensors to the robot. Each link has dynamic properties such as mass and inertia associated with it. The inertia tensor is automatically calculated for each link using shape type, size and mass properties.

**Available Snippets**
The user can simply include the [snippets file](./urdf/snippets.urdf.xacro) to create robot links, joints and attach sensors to the robot.

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

http://wiki.ros.org/gazebo_ros

http://wiki.ros.org/robot_state_publisher
http://wiki.ros.org/joint_state_publisher
http://wiki.ros.org/octomap
http://wiki.ros.org/teleop_twist_joy
http://wiki.ros.org/teleop_twist_keyboard
