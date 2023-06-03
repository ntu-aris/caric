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
The metric is based on capture quality of interest points on the object surface, with the line of sight, motion blur, and resolution taken into account.
The final judging criteria is the total number of interest points that have been fully captured and communitated back to the ground station.
For a point to be fully captured, it has to satisfy the following criteria:

* Line of sight and fielf of view: the interest point has to fall in the field of view of the camera, and the camera has direct line of sight to the interest point (not obstructed by any other objects). The camera horizontal and vertical fields of view are defined by the parameters `HorizFOV` and `VertFOV` in the file `caric_ppcom_network.txt`. Note that the camera orientation can be controlled as described in the section [Camera Gimbal Control](#camera-gimbal-control).

* Motion blur: motion blur is resulted from moving object during the camera exposure duration defined by the parameter `ExposureTime`. The motion blur metric, defined as the number of pixels that an interest point moves across during the exposure, is computed as: 

$$
\text{horizontal\_blur} = \dfrac{|u_1-u_0|}{\text{pixel\_width}},\\
u_0 = \text{focal\_length}*\dfrac{x_0}{z_0},\\
u_1 = \text{focal\_length}*\dfrac{x_1}{z_1},\\
[x_1,y_1,z_1]^\top = [x_0,y_0,z_0]^\top + \mathbf{v}*\text{exposure\_time}.
$$

Here, $[x_0,y_0,z_0]^\top$ is the position of the interest point at the time of capture, and $[x_1,y_1,z_1]^\top$ is the updated position considering the velocity of the interest point in the camera frame $\mathbf{v}$ obtained at the time of the capture. The figure below illustrates the horizontal motion blur by showing the horizontal (X-Z) plane of the camera frame.
<!-- <p align="center"> -->
<div style="text-align:center">
  <img src="docs/motionblur1.png" alt="resolution1" width="40%"/>
  <figcaption>Illustration of horizontal resolution computation</figcaption>
</div>
<!-- </p> -->

The vertical blur can be computed similarly by replacing $x_0$ and $x_1$ with $y_0$ and $y_1$ in the above computation of $u_0$ and $u_1$. For an interest point to be considered captured, the movement of the interest point has to be smaller than 1 pixel (so that the image is sharp), i.e.,

$$
\text{horizontal\_blur} < 1,\\
\text{vertical\_blur} < 1.\\
$$

* Image resolution: the resolution of the image is expressed in mm/pixel, representing the size of the real-world object captured in one pixel of the image. To achieve a satisfactory resolution, the computed horizontal and vertical resolution have to be smaller than a desired mm/pixel value. Given the position of an interest point in the camera frame and its normal (perpendicular to its surface), the horizontal and vertical resolution can be obtained by displacing the interest point by $\pm 0.5$ mm along the line intersecting the interest surface and the horizontal/vertical plane in the camera coordinate system, and then finding the corresponding length of the object in the image. The image below illustrates this process, where the length of the object in the image is expressed as $|u_1-u_2|$ .

<!-- <p align="center"> -->
<div style="text-align:center">
  <img src="docs/resolution1.png" alt="resolution1" width="40%"/>
  <figcaption>Illustration of horizontal resolution computation</figcaption>
</div>
<!-- </p> -->

The horizontal resolution is computed as $\text{horizontal\_resolution}=\frac{pixel\_width}{|u_1-u_2|}$. Similarly, $\text{vertical\_resolution}=\frac{pixel\_width}{|v_1-v_2|}$, $v_1$ and $v_2$ are the $y$-coordinates of the points in the image plane obtained by displacing the interest point along the line intersecting the interest surface and the vertical plane.
For a point to be considered captured, the resolutions have to satisfy
$$
\text{horizontal\_resolution} < \text{desired\_mm\_per\_pixel},\\
\text{vertical\_resolution} < \text{desired\_mm\_per\_pixel}.
$$

## How the points are counted
Only interest points that have been reported to GCS will be counted.

# Developing your CARI scheme

## Estimation data

CARIC is intended for investigating cooperative control schemes, hence perception proccesses such as sensor fusion, SLAM, map merging, etc... are assumed available and accurate (for now). Users can obtain the accurate odometry from `/[node_id]/ground_truth/odometry` (inclusing pose, velocity and acceleration), and pointcloud from `/[node_id]/cloud_inW` (`[node_id]` is the name of the robot, for example `firefly1`). These topics should be sufficient for control, mapping, and obstacle avoidance purposes. Moreover, a robot can also receive the neigbours' odometry in `/[node_id]/nbr_odom_cloud`, as well as the neigbours' latest lidar key frame `/[node_id]/nbr_kf_cloud`.However these topics only have the messages from neighours that have LOS to the robot.

## UAV control interface

Whatever control strategy is developed, the control signal should be eventually converted to standard multi-rotor command. Specifically the UAVs are controlled using the standard ROS message `trajectory_msgs/MultiDOFJointTrajectory`. The controller subscribes to the command trajectory topic `/[node_id]/command/trajectory`. Below are sample codes used to publish a trajectory command in `traj_gennav_node.cpp`, given 3d target states in the global(world) frame `target_pos`, `target_vel`, `target_acc` and a target yaw `target_yaw`:

```cpp
trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

geometry_msgs::Transform transform_msg;
geometry_msgs::Twist accel_msg, vel_msg;

transform_msg.translation.x = target_pos(0);
transform_msg.translation.y = target_pos(1);
transform_msg.translation.z = target_pos(2);
transform_msg.rotation.x = 0;
transform_msg.rotation.y = 0;
transform_msg.rotation.z = sinf(target_yaw*0.5);
transform_msg.rotation.w = cosf(target_yaw*0.5);

trajpt_msg.transforms.push_back(transform_msg);

vel_msg.linear.x = target_vel(0);
vel_msg.linear.y = target_vel(1);
vel_msg.linear.z = target_vel(2);

accel_msg.linear.x = target_acc(0);
accel_msg.linear.x = target_acc(1);
accel_msg.linear.x = target_acc(2);

trajpt_msg.velocities.push_back(vel_msg);
trajpt_msg.accelerations.push_back(accel_msg);
trajset_msg.points.push_back(trajpt_msg);

trajset_msg.header.stamp = ros::Time::now();
trajectory_pub.publish(trajset_msg); //trajectory_pub has to be defined as a ros::Publisher
```
There are multiple ways you can control the robots:

`Full-state control`: consider you have computed the future trajectory of a robot with timestamped target position, velocity, acceleration and yaw. You may publish the target states at the desired timestamp using the above example code.

`Position-based control`: you may also send non-zero target positions and yaw with zero velocity and acceleration, the robot will reach the target and hover there. Note that if the target position is far from the robot's target position, aggresive movement of the robot is expected.

`Velocity/acceleration-based control`: when setting target positions to zeros and setting non-zero velocities or accelerations, the robot will try to move with the desired velocity/acceleration. The actual velocity/acceleration may not follow the desired states exactly due to the realistic low level controller. Hence, the users are suggested to take into account the state feedback when generating the control inputs.
### Camera Gimbal Control
The camera is assummed to be installed on a camera stabilizer (gimbal) located at [`CamPositionX`, `CamPositionY`, `CamPositionZ`] in the body frame of the drone. To be realistic, we allow the users to control the gimbal pitch and yaw angle while keeping the gimbal roll at zero. The gimbal control interface is the topic `/[node_id]/command/gimbal` of type `geometry_msgs/Twist`. An example is shown below:
```cpp
  geometry_msgs::Twist gimbal_msg;
  gimbal_msg.linear.x = -1.0; //setting linear.x to -1.0 enables velocity control mode.
  gimbal_msg.linear.y = 0.0;  //if linear.x set to 1.0, linear,y and linear.z are the 
  gimbal_msg.linear.z = 0.0;  //target pitch and yaw angle, respectively.
  gimbal_msg.angular.x = 0.0; 
  gimbal_msg.angular.y = target_pitch_rate; //in velocity control mode, this is the target pitch velocity
  gimbal_msg.angular.z = target_yaw_rate; //in velocity control mode, this is the target yaw velocity
  gimbal_cmd_pub_.publish(gimbal_msg);
```
As explained in the comments in the sample code, the interface allows angle-based or rate-based control. When `gimbal_msg.linear.x` is set to 1.0, the fields `gimbal_msg.linear.y` and `gimbal_msg.linear.z` indicates the command pitch and yaw angle, respectively. The pitch and yaw angles are controlled independently: given a target pitch/yaw angle, the gimbal will move with the maximum pitch/yaw rate defined by the parameter `gimbal_rate_max` until reaching the target. In velocity control mode, the gimbal pitch/yaw rates can be set to any value in the range [-`gimbal_rate_max`,+`gimbal_rate_max`]. The gimbal pitch and yaw only operate in the ranges [-`gimbal_pitch_max`,+`gimbal_pitch_max`] and [-`gimbal_yaw_max`,+`gimbal_yaw_max`], respectively.

Here, the gimbal roll, pitch and yaw angles are defined as the euler angles (Z-Y-X rotation sequence) describing the gimbal orientation with respect to a virtual frame, whose X-axis is always parallel to the X-axis of the drone body frame, and X-Y plane is always parallel to the X-Y plane in the world frame (due to roll being stabilized). If we define the camera frame with its X-axis perpendicular to the image plane, and Z-axis pointing upward in the image plane, then, the euler angle of the camera with respect to the world frame can be obtained as
```cpp
  camera_Yaw_in_world_frame = drone_yaw_in_world_frame + gimbal_yaw;
  camera_Pitch_in_world_frame = gimbal_pitch;
  camera_roll_in_world_frame = 0.0;
```

The gimbal euler angle and angular rates are published through the topic `/[node_id]/gimbal` of type `geometry_msgs/TwistStamped`. The fields `twist.linear` indicates the euler angle while the fields `twist.angular` indicates the angular rates (a bit of deviation from the original meaning of the message type).

### Camera Trigger
Two camera trigger modes are allowed. If the parameter `manual_trigger` is set to false, the robot will automatically trigger camera capture at a fixed time interval defined by the parameter `trigger_interval`. If the parameter `manual_trigger` is set to true, the user may send camera trigger command by publishing to a topic `/[node_id]/command/camera_trigger` of type `rotors_comm/BoolStamped`:

```cpp
rotors_comm::BoolStamped msg;
msg.header.stamp = ros::Time::now();
msg.data = true;
trigger_pub.publish(msg);  //trigger_pub has to be defined as a ros::Publisher
```
Note that in manual trigger mode, the time stamps of two consecutive trigger commands should still be separated by an interval larger than the parameter `trigger_interval`, otherwise, the second trigger command will be ignored. The benefit of using manual trigger is that the users may send the trigger command at the exact time that results in the best capture quality.

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

