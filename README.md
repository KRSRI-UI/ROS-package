## Komponen Robot

- 3 buah Baterai LiPo LPBpower 2200 mAH 3S 25C
- Raspberry Pi 4B RAM 8Gb
- Pi Camera V2.1
- 6 buah Dynamixel AX-12A
- 12 buah Dynamixel AX-18A
- MPU 9250/6500
- RPLidar A1M1
- Servo PWM MG996R
- Mini DC 12V Brushless Pump AD20P – 1230A Hmax 300CM Qmax 240 L / H
- Robotis U2D2
- Relay 3.3V
- Buck Converter XY-3606
- Rocker Switch DPST
- 3 buah Kapasitor 220uF 16V

# ROS Hexapod Stack


## 1. Documentation

This is my implementation of a hexapod functioning in the ROS framework. Agnostic to either a 3dof or 4dof hexapod. It is still very much a work in progress and I am still actively developing it. 

Thanks to Shubhankar Das there are two gaits offered, the original sinusoidal tripod gait and a new ripple gait.

* Author: Kevin M. Ochs
* Contributor: Shubhankar Das
* Contributor: Renée Love
* Contributor: Konstantinos Chatzilygeroudis
* Contributor: Kurt Eckhardt
* Contributor: Romain Reignier

## 2. Expected Hardware for mapping

* Primesense Sensor, Asus Xtion or Intel Realsense
* IMU (Current master branch uses a Phidgets 3/3/3 Spatial in launch files.)

## 3. Dependencies

```
sudo apt install git
sudo apt install ros-noetic-sound-play
sudo apt install ros-noetic-openni2-launch
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-rtabmap
sudo apt install ros-noetic-rtabmap-ros
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-navfn
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-diagnostic-updater
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-depthimage-to-laserscan
sudo apt install ros-noetic-imu-filter-madgwick
sudo apt install ros-noetic-ros-controllers
sudo apt install ros-noetic-ros-control
sudo apt install ros-noetic-spacenav-node
sudo apt install libusb-1.0-0-dev
sudo apt install libsdl-dev

sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-gazebo-ros-control
```

**_Joystick_**


For pairing a PS3 controller you can either install BlueZ5 or follow the below link.

https://help.ubuntu.com/community/Sixaxis

## 4. Nodes

**_hexapod_controller_**

This is the main node of the stack. It handles all control, gait, IK and servo communications with the legs. Minimal latency was required to keep the gait smooth and synced with odometry hence the reason they are all combined in this one node.

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist) Velocity command. 
     body_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the orientation of the body.
     head_scalar (geometry_msgs::AccelStamped) Scalar to modifiy the pan and tilt of the optional turret.
     state (std_msgs::Bool) Bool array to record state of the hexapod. Standing up, sitting etc.
     imu/data (sensor_msgs::Imu) Used in optional auto body leveling on non level ground.
     
*Published Topics*

    sounds (hexapod_msgs::Sounds) Custom message to send sound cues to the optional sound package.
    joint_states (sensor_msgs::JointState) Joint states for rviz and such.
    odometry/calculated (nav_msgs::Odometry) Calculated odometry from the gait system in the package.
    twist (geometry_msgs::TwistWithCovarianceStamped) Twist message syncronized with the gait system. 
     

**_hexapod_bringup_**

This package has all the launch files. From simple locomotion only to full mapping and localization examples. 

**_hexapod_description_**

This package has all the param files. You will start with one of the param config files to describe your hexapod. It also has params for different telop controllers. The xacro and meshes also reside in this package.


**Example Launch Command**

*Keyboard Teleop*
```
roslaunch hexapod_bringup hexapod_simple.launch
```
## 5. Install

```
git clone https://github.com/KRSRI-UI/ROS-package.git
```

For Raspberry Pi2 please add these compiler optimizations after first build.
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7
```

For ODROID XU3 please add these compiler optimizations after first build.
```
[workspace]/build/CMakeCache.txt
Change: CMAKE_CXX_FLAGS:STRING=-O3 -pipe -march=armv7-a -mcpu=cortex-a9 -mfloat-abi=hard
```

## Videos 
------
_Click on picture for redirect to YouTube video._


Rviz recording of 3D mapping using RTABmap.

[![ScreenShot](http://img.youtube.com/vi/-3Ejgy1nFOg/0.jpg)](https://www.youtube.com/watch?v=-3Ejgy1nFOg)

https://www.youtube.com/watch?v=-3Ejgy1nFOg



Small video of Golem research platform and IMU testing.

[![ScreenShot](http://img.youtube.com/vi/IP-1HebkZnU/0.jpg)](https://www.youtube.com/watch?v=IP-1HebkZnU)

https://www.youtube.com/watch?v=IP-1HebkZnU



Renée Love's odometry test video using the phantomX.

[![ScreenShot](http://img.youtube.com/vi/VYBAM0MrvWI/0.jpg)](https://www.youtube.com/watch?v=VYBAM0MrvWI)

https://www.youtube.com/watch?v=VYBAM0MrvWI



## Pictures

Rviz screenshot of point cloud and laserscan active.

![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/depthwithlaser.jpg)

2D room mapping in Rviz.

![ScreenShot](http://forums.trossenrobotics.com/gallery/files/8/6/6/6/2d_slam.jpg)

Renée Love's adaptation of the Hexapod stack for Trossen's  [PhantomX](http://www.trossenrobotics.com/phantomx-ax-hexapod.aspx).

![ScreenShot](http://forums.trossenrobotics.com/gallery/files/1/2/6/6/9/screenshot_from_2015-04-22_20_23_15.png)


## 6. How To Use:

### Turtlebot Launch:
```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
```
export TURTLEBOT3_MODEL=waffle
roslaunch rtabmap_demos demo_turtlebot3_navigation.launch
```

### Reignblaze-Gazebo Launch:
```
# roslaunch hexapod_navigation launchfile_sample.launch
roslaunch hexapod_navigation depthimage_to_laserscan.launch
```

### Open-VINS ROS1 Install:
```
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev
sudo apt-get install python3-catkin-tools python3-osrf-pycommon

cd src
git clone https://github.com/rpng/open_vins/
cd ..
catkin build
catkin_make

cd /home/aldy/datasets/euroc_mav/
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag
```

### Open-VINS Launch:
```
cd catkin_ws_ov/
source devel/setup.bash
roslaunch ov_msckf subscribe.launch config:=euroc_mav dobag:=true
```
```
rviz
```
