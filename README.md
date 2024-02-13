# URDF Snippets for ROS+Gazebo Simulation

![Simulation](./.media/urdf_snippets_package.gif)

## Overview

This repository contains a ROS (**melodic**) package to simplify the robot URDF model creation process using XACRO snippets for the purpose of Gazebo simulation and robot model visulization inside ROS RViz.

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

The `front_castor` link's dynamic properties are set to be frictionless using the `castor_friction` macro.

## Available Snippets

The user can simply include the [snippets file](./urdf/snippets.urdf.xacro) inside the robot model file in order to create links, joints and sensors for the robot. If the macro parameters are skipped then the default values are being used. Here is a brief summary of various snippets:

**shape_link**
This macros is used to create basic geometry shapes as link. The geometry shape is specified as `type` property of the macro with following possible values, `box, cylinder, sphere`. It creates the visual, inertial, and collision tags of the specified link

```
    <xacro:macro name="shape_link" params="name:=MyRobot type:=box m:=1 w:=1 l:=1 h:=1 r:=1 rgba:='0 0 0 1' xyz='0 0 0' rpy='0 0 0' color:=Red mesh:=''">

```

**shape_joint**
This macro is used to attach two links with the specified `type` of the joint. The joint can be any one of the following `fixed, floating, revolute, continuous, prismatic`. Axis specifies the axis of motion or rotation of the specified joint. In order to place the joint at the specified pose, `xyz` and `rpy` values can be used. The values are specified in metric units i.e. `m` and `rad`.

```
    <xacro:macro name="shape_joint" params="type:='fixed' parent:='p' child:='c' axis='0 0 0' xyz='0 0 0' rpy='0 0 0'">
```

**sensor_rgb_camera**
This macro is used to create a link for the color camera and attaches it to specified parent link.

```
    <xacro:macro name="sensor_rgb_camera" params=" name:=cam parent:=chassis width:=640 height:=480 fps:=10 visulize:=true near:=0.01 far:=100 hfov:=60 xyz:='0 0 0'">
```

**sensor_stereo_camera**
This macro is used to create a link for the stereo camera and attaches it to specified parent link.

```
    <xacro:macro name="sensor_stereo_camera" params=" name:=stereo parent:=chassis width:=640 height:=480 fps:=10 visulize:=true near:=0.01 far:=300 hfov:=60 xyz:='0 0 0' baseline:=0.07">
```

**sensor_depth_camera**
This macro is used to create a link for the depth camera (such as kinect/realsense) and attaches it to specified parent link.

```
    <xacro:macro name="sensor_depth_camera" params=" name:=stereo parent:=chassis width:=640 height:=480 fps:=10 visulize:=true near:=0.01 far:=300 hfov:=60 xyz:='0 0 0' baseline:=0.1">
```

**sensor_video**
This macro is used to create a link for the video stream to be displayed inside gazebo and attaches it to specified parent link.

```
    <xacro:macro name="sensor_video" params="link:=chassis width:=160 height:=120 topic:=image">
```

**sensor_bumper**
This macro is used to create a link for the bumpber sensor and attaches it to specified parent link.

```
    <xacro:macro name="sensor_bumper" params="name:=bumper rate:=20 frame:=world">
```

**sensor_imu_gazebo**
This macro is used to create a link for an IMU sensor and attaches it to specified parent link. The acceleration output don't contains the gravitational acceleration.

```
    <xacro:macro name="sensor_imu_gazebo" params="parent:=chassis name:=base_link topic:=imu rate:=20 noise:=0">
```

**sensor_imu_ros**
This macro is used to create a link for an IMU sensor and attaches it to specified parent link. The acceleration output contains the gravitational acceleration.

```
    <xacro:macro name="sensor_imu_ros" params="parent:=chassis name:=base_link topic:=imu rate:=20 noise:=0 xyz:='0 0 0' visulize:=true">
```

**sensor_laser_2d**
This macro is used to create a link for a 2D laser ranger sensor, such as SICK/Hokuyo, and attaches it to specified parent link.

```
    <xacro:macro name="sensor_laser_2d" params="parent:=chassis name:=laser topic:=/scan rate:=20 noise:=0 minRange:=0.1 maxRange:=100 samples:=360 xyz:='0 0 0' visulize:=true">
```

**sensor_lidar_3d**
This macro is used to create a link for a 3D LIDAR range sensor, such as Velodyne, and attaches it to specified parent link.

```
    <xacro:macro name="sensor_lidar_3d" params="parent:=chassis name:=base_link topic:=/velodyne_points rate:=20 noise:=0 minRange:=0.1 maxRange:=100 samples:=360 xyz:='0 0 0' visulize:=true">
```

**castor_friction**
This macro is used to set the dynamic properties of the specified link as frictionless.

```
    <xacro:macro name="castor_friction" params="name:=castor_wheel mu1:=0 mu2:=0 kp:=1e9 kd:=0 fdir:='0 0 0'">
```

**robot_drive_diff**
This macro is used to attach a motion controller to the specified link.

```
    <xacro:macro name="robot_drive_diff" params="chassis:=base_link left:=joint_chassis_left_wheel right:=joint_chassis_right_wheel l:=0.25 d:=0.1 cmdtopic:=cmd_vel odomtopic:=odom">
```

**robot_drive_skid_steer**
This macro is used to attach a motion controller to the specified link.

```
    <xacro:macro name="robot_drive_skid_steer" params="chassis:=base_link front_left:=joint_chassis_front_left_wheel front_right:=joint_chassis_front_right_wheel rear_left:=joint_chassis_rear_left_wheel rear_right:=joint_chassis_rear_right_wheel l:=0.25 d:=0.1 cmdtopic:=cmd_vel odomtopic:=odom">
```

**robot_drive_plane**
This macro is used to attach a motion controller to the specified link.

```
    <xacro:macro name="robot_drive_plane" params="chassis:=base_link odomtopic:=odom cmdtopic:=cmd_vel rate:=20">
```

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

For teleoperation of the robot, install following package
```
sudo apt install ros-<distro>-teleop-twist-keyboard
e.g.
sudo apt install ros-noetic-teleop-twist-keyboard
```

The following example launch file can be used to test the generated URDF file.

```
roslaunch gazebeo_simulation test_urdf.launch
```

The example launch file convert the XACRO robot model to URDF and load it to the ROS parameter server. It executes the RViz using the [rviz.launch](./launch/rviz.launch) file. The transform information between various links of the robot is published by the [robot state publisher](http://wiki.ros.org/robot_state_publisher) node. The node reads the robot model available at the parameter server and perodically publishes the transform messages. All the robot joints information is published by the [robot joint state publisher](http://wiki.ros.org/joint_state_publisher) node. This node publishes fake joint information. In case of real robot, the joint information is provided by the robot hardware.

The gazebo simulation is launched using [gazebo_ros](http://wiki.ros.org/gazebo_ros) package. This package loads a [sample world file](./world/turtlebot_playground.world) available in the world sub directory of the package. Afterwards it spawns the robot model using the URDF file.

Finally, to send navigation commands to the robot, teleoperation through keyboard, [keyboard.launch](./launch/keyboard.launch) file is spawned which uses [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package.
_optionally_ in order to command the robot using a joystick, [joy.launch](./launch/joy.launch) launch file is provided. It is based on the [teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy) package.

**Extra**
In order to use the publish point cloud by the velodyne LIDAR, [octomap.launch](./launch/octomap.launch) file can be used which is based on the [package](http://wiki.ros.org/octomap).

