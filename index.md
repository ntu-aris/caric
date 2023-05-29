---
layout: default
---


# Contents

* [Introduction](#introduction)
* [Installation](#installation)
  * [Install the dependencies](#install-the-dependencies)
  * [Install the CARIC packages](#install-the-caric-packages)
  * [Test the baseline inspection method](#test-the-baseline-cooperative-inspection-method)
* [Benchmark](#benchmarking)
  * [Example scenarios](#example-scenarios)
  * [The image capture quality metric](#the-image-capture-quality-metric)
  * [How the points are counted](#how-the-points-are-counted)
* [Develop your CARI Scheme](#developing-your-cari-scheme)
  * [Estimation data](#estimation-data)
  * [UAV control interface](#uav-control-interface)
  * [Communication between the drones](#communication-between-the-drones)


# Introduction

CARIC (short for **C**ooperative **A**erial **R**obots **I**nspection **C**hallenge) is a software stack based on Gazebo and [RotorS](https://github.com/ethz-asl/rotors_simulator). The objective of CARIC is twofold. First, it aims to faithfully simulate multi-UAV systems operating in typical real-world inspection missions. Second, based on this tool, different cooperative inspection schemes can be benchmarked based on typical inspection scenarios.

In a typical inspection mission, the main goal is to capture images on the surface of some structures with the best quality possible. However, exploration is also a secondary objective that needs to be addressed to identify the structure and its surface. Oftentimes, bounding box(es) can be set around the target of interest to narrow the area of exploration. Given the same basic information such as the bounding boxes, mission time, sensor specifications and UAV dynamic model, different cooperative control schemes can be compared with each other by some inspection metric.

The software is motivated by the belief that a multi-UAV system can reduce inspection time to a fraction of that by a single UAV. Hence, the job can be done more efficiently, acheive better quality and shorten the possible disruption to other operations at the venue.
To accomplish this, novel cooperative strategies to coordinate the trajectories of the UAVs in an optimal manner is a crucial component. Moreover, the optimization problem is subjected to other constraints such as non-line-of-sight (LOS) communication loss, collision avoidance, and operation time. These constraints are closely reflected in CARIC (Figure 1).

<!-- <p align="center"> -->
<div style="text-align:center">
  <img src="docs/mbs_trimmed_spedup.gif" alt="facade_inspection" width="80%"/>
  <figcaption>Figure 1. CARIC software package can simulate UAV dynamics, physical collisions, camera-FOV-based instance detection, and LOS-only communications</figcaption>
</div>
<!-- </p> -->

# Installation

The system is principally developed and tested on the following system configuration:

* NVIDIA GPU-enabled computers (GTX 2080, 3070, 4080)
* Ubuntu 20.04 / 16.04
* ROS Noetic / Melodic
* Gazebo 11
* Python 3.8

The following instructions assume that CARIC is going to be installed on an Ubuntu 20.04 system with ROS Noetic. We will also provide remarks on installing CARIC on Ubuntu 16.04 with ROS Melodic.

## Install the dependencies

First please run the following commands to install some neccessary dependencies:

```bash
# Update the system
sudo apt-get update && sudo apt upgrade ;

# Install some tools and dependencies
sudo apt-get install python3-wstool python3-catkin-tools python3-empy \
                     protobuf-compiler libgoogle-glog-dev \
                     ros-$ROS_DISTRO-control-toolbox \
                     ros-$ROS_DISTRO-octomap-msgs \
                     ros-$ROS_DISTRO-octomap-ros \
                     ros-$ROS_DISTRO-mavros \
                     ros-$ROS_DISTRO-mavros-msgs \
                     ros-$ROS_DISTRO-rviz-visual-tools \
                     ros-$ROS_DISTRO-gazebo-plugins;

# Install gaze 11 (default for Ubuntu 20.04)
sudo apt-get install ros-noetic-gazebo* ;
```
_NOTE_:
* On Ubuntu 16.04, replace `python3` in the package names above with `python`.
* On Ubuntu 16.04, user may need to remove the default Gazebo 9 and install Gazebo 11. Otherwise Gazebo may crash due to conflict between the GPU-based lidar simulator and the raytracing operations in our custom-built `gazebo_ppcom_plugin.cpp`.
* Make sure protobuf version is 3.6.1 by running the following command:
```bash
protoc --version
```
If protoc version is other than 3.6.1, try to remove protoc, and then reinstall with `sudo apt install protobuf-compiler`.
There can be multiple versions of the protobuf installed in the system. You can find the locations of the version used by the command `whereis protoc`.

## Install the CARIC packages
Once the dependencis have been installed, please create a new workspace for CARIC, clone the necessary packages into it, and compile:

```bash
# Create the workspace
mkdir -p ~/ws_caric/src
cd ~/ws_caric/src
wstool init

# Download the packages:

# Manager node for the mission
git clone https://github.com/ntu-aris/caric_mission

# Simulate UAV dynamics and other physical proccesses
git clone https://github.com/ntu-aris/rotors_simulator

# GPU-enabled lidar simulator, modified from: https://github.com/lmark1/velodyne_simulator
git clone https://github.com/ntu-aris/velodyne_simulator

# To generate an trajectory based on fixed setpoints. Only for demo
git clone https://github.com/ntu-aris/traj_gennav

# Converting the trajectory setpoint to rotor speeds
git clone https://github.com/ntu-aris/tcc

# Build the workspace
cd ~/ws_caric/
catkin build
```
The compilation may report errors due to missing depencies or some packages in CARIC are not yet registered to the ros package list. This can be resolved by installing the missing dependencies (via `sudo apt isntall <package>` or `sudo apt install ros-$ROS_DISTRO-<ros_package_name>)`, then/or try `catkin build` again as the compiled packages are added to dependency.

## Test the baseline cooperative inspection method

We propose a cooperative inspection method on top of our simulator in the software stack. Please run it with this command:

```bash
source ~/ws_caric/devel/setup.bash
roscd caric_mission/scripts
bash launch_all.sh
```

# Benchmarking

## Example scenarios
Inspired by our previous projects, we include the following scenarios in the software stack:

* Building inspection
* Aircraft inspection
* Crane inspection

Each scenario has a specialized environment, obstacles, and UAV team configuration setup.

## The image capture quality metric
The metric is based on capture quality of interest points on the object surface, with the motion blur, angle, and distance taken into account.
(to be elaborated)

## How the points are counted
Only interest points that have been reported to GCS will be counted.

# Developing your CARI scheme

## Estimation data

CARIC is intended for investigating cooperative control schemes, hence perception proccesses such as sensor fusion, SLAM, map merging, etc... are assumed available and accurate (for now). Users can obtain the accurate odometry from `/[node_id]/ground_truth/odometry` (inclusing pose, velocity and acceleration), and pointcloud from `/[node_id]/cloud_inW` (`[node_id]` is the name of the robot, for example `firefly1`). These topics should be sufficient for control, mapping, and obstacle avoidance purposes. Moreover, a robot can also receive the neigbours' odometry in `/[node_id]/nbr_odom_cloud`, as well as the neigbours' latest lidar key frame `/[node_id]/nbr_kf_cloud`.However these topics only have the messages from neighours that have LOS to the robot.

## UAV control interface

Whatever control strategy is developed, the control signal should be eventually converted to standard multi-rotor command. Specifically the UAVs are controlled using the standard ROS message `trajectory_msgs/MultiDOFJointTrajectory`. The controller subscribes to the command trajectory topic `/firefly/command/trajectory`. Below are sample codes used to publish a trajectory command in `traj_gennav_node.cpp`:

```cpp
trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

geometry_msgs::Transform transform_msg;
geometry_msgs::Twist accel_msg, vel_msg;

transform_msg.translation.x = _pos_reprojected(0);
transform_msg.translation.y = _pos_reprojected(1);
transform_msg.translation.z = _pos_reprojected(2);
transform_msg.rotation.x = 0;
transform_msg.rotation.y = 0;
transform_msg.rotation.z = sinf(_yaw*0.5);
transform_msg.rotation.w = cosf(_yaw*0.5);

trajpt_msg.transforms.push_back(transform_msg);

vel_msg.linear.x = _vel(0);
vel_msg.linear.y = _vel(1);
vel_msg.linear.z = _vel(2);

accel_msg.linear.x = _acc(0);
accel_msg.linear.x = _acc(1);
accel_msg.linear.x = _acc(2);

trajpt_msg.velocities.push_back(vel_msg);
trajpt_msg.accelerations.push_back(accel_msg);
trajset_msg.points.push_back(trajpt_msg);

trajset_msg.header.stamp = ros::Time::now();
trajectory_pub.publish(trajset_msg); 
```
Note that the controller is triggered once when the trajectory command is published (you need to continuously publish trajectory command to generate drone movements).

## Communication between the drones

Each robot is given a unique ID in a so-called ppcom network, for e.g. gcs, firefly1, firely2. These IDs can be specified in the [description file](https://github.com/ntu-aris/rotors_simulator/blob/a976102c9465bd2a04afcabb18014f5c019b3f4f/rotors_description/ppcom_network/caric_ppcom_network.txt).

In real-world conditions, communications between the nodes can be interrupted by obstacles that block the LOS between them. To subject a topic to this effect, users can do the following:
* Launch the node `ppcom_router` under `caric_mission`:
```bash
rosrun caric_mission ppcom_router.py # This can also be called in a launch file
```
* Advertise the topic as normal, for e.g. (example is in python but the equivalent can be done in C++):
```bash
msg_pub = rospy.Publisher('/ping_message', std_msgs.msg.String, queue_size=1)
```
* Call the service `/create_ppcom_topic`, specifying the source node ID, the target nodes' IDs, the topic, the package, and the message definition in that package:
```bash
# Create a service proxy
create_ppcom_topic = rospy.ServiceProxy('create_ppcom_topic', CreatePPComTopic)
# Register the topic with ppcom router
response = create_ppcom_topic('firefly1', ['all'], '/ping_message', 'std_msgs', 'String') # 'all' means all ppcom nodes can receive message from this topic.
print(f"Response {response}") # Error will be appended to the response.
```
* For each target specified, a new topic with the original name appended with the target node ID will be created. Target nodes can subscribe to this topic and it will only receive the data from the source node when there is LOS. For example the node `firefly2` can subscribe to `/ping_message/firefly2` whoses message are relayed from `/ping_message`.

* Multiple nodes can advertise the same topic. Messages published to the shared topic will only be relayed to the intended targets as specified in the service call by the source node.

* To better understand the LOS-only communication feature, you can launch the simulation by `run_mbs.launch`, then launch the scripts `ppcom_router.py`, `ppcom_firefly1_talker.py`, `ppcom_firefly2_talker.py`, `ppcom_firefly3_talker.py` in different terminals. Hence you can observe the messages sent and received by each node via the `ppcom_router` like in the following video. Notice how the messages are dropped when the corresponding entry in the LOS matrix turns to 0 (the `firefly1 --> firefly3` LOS status is indicated by the entry at 2nd row, 4th column).

<div align="center">
  <iframe width="800" height="450" src="https://www.youtube.com/embed/_DtOr0rsJMQ"
          title="YouTube video player" frameborder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
          allowfullscreen></iframe>
</div>