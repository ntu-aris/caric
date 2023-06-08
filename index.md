---
layout: default
---

# 1. Table of contents

- [1. Table of contents](#1-table-of-contents)
- [2. Introduction](#2-introduction)
- [3. Installation](#3-installation)
  - [3.1. Install the dependencies](#31-install-the-dependencies)
    - [3.1.1. Ubuntu 20.04 + ROS Noetic](#311-ubuntu-2004--ros-noetic)
    - [3.1.2. Ubuntu 18.04 + ROS Melodic](#312-ubuntu-1804--ros-melodic)
    - [3.1.3. Protobuf version](#313-protobuf-version)
  - [3.2. Install the CARIC packages](#32-install-the-caric-packages)
  - [3.3. Run the flight test](#33-run-the-flight-test)
- [4. The benchmark's design](#4-the-benchmarks-design)
  - [4.1. The UAV fleet](#41-the-uav-fleet)
  - [4.2. Inspection scenarios](#42-inspection-scenarios)
  - [4.3. The image capture quality metric](#43-the-image-capture-quality-metric)
    - [4.3.1. LOS and FOV](#431-los-and-fov)
    - [4.3.2. Motion blur](#432-motion-blur)
    - [4.3.3. Image resolution](#433-image-resolution)
  - [4.4. Evaluation](#44-evaluation)
- [5. Developing your CARI scheme](#5-developing-your-cari-scheme)
  - [5.1. Ground rules](#51-ground-rules)
  - [5.2. Estimation data](#52-estimation-data)
  - [5.3. UAV control interface](#53-uav-control-interface)
    - [5.3.1. Camera gimbal control](#531-camera-gimbal-control)
    - [5.3.2. Camera trigger](#532-camera-trigger)
  - [5.4. Simulated networked communication](#54-simulated-networked-communication)


# 2. Introduction

CARIC (short for **C**ooperative **A**erial **R**obots **I**nspection **C**hallenge) is a software stack based on [Gazebo](https://packages.ubuntu.com/source/jammy/gazebo), [RotorS](https://github.com/ethz-asl/rotors_simulator) and other open-source packages. The objective of CARIC is twofold. First, it aims to faithfully simulate multi-UAV systems operating in typical real-world inspection missions. Second, based on this tool, different cooperative inspection schemes can be benchmarked based on typical inspection scenarios.

In a typical inspection mission, the main goal is to capture images on the surface of some structures with the best quality possible. However, exploration is also a secondary objective that needs to be addressed to identify the structure and its surface. Oftentimes, bounding box(es) can be set around the target of interest to narrow the area of exploration. Given the same basic information such as the bounding boxes, mission time, sensor specifications and UAV dynamic model, different cooperative control schemes can be compared with each other by some inspection metric.

The software is motivated by the belief that a multi-UAV system, especially a heterogenous one, can accomplish the task more efficiently, acheive better quality and shorten the possible disruption to other operations at the venue.
To accomplish this, novel cooperative strategies to coordinate the trajectories of the UAVs in an optimal manner is a crucial component. Moreover, the optimization problem is subjected to other constraints such as communication loss, collision avoidance, and operation time. These constraints are closely reflected in CARIC (see the figure below).

<div style="text-align:center">
  <img src="docs/mbs_trimmed_spedup.gif" alt="facade_inspection" width="80%"/>
  <figcaption>CARIC software package can simulate UAV dynamics, physical collisions, camera-FOV-based instance detection, and line-of-sight-only communications</figcaption>
</div>

# 3. Installation

The system is principally developed and tested on the following system configuration:

* NVIDIA GPU-enabled computers (GTX 2080, 3070, 4080)
* Ubuntu 20.04 / 18.04
* ROS Noetic / Melodic
* Gazebo 11
* Python 3.8

##  3.1. Install the dependencies
###  3.1.1. Ubuntu 20.04 + ROS Noetic
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

# Install gazebo 11 (default for Ubuntu 20.04)
sudo apt-get install ros-noetic-gazebo* ;
```
Check the protobuf version as described in [Protobuf Version](#protobuf-version).
###  3.1.2. Ubuntu 18.04 + ROS Melodic
First please run the following commands to install some neccessary dependencies:

```bash
# Update the system
sudo apt-get update && sudo apt upgrade ;

# Install some tools and dependencies
sudo apt-get install python-wstool python-catkin-tools python-empy \
                     protobuf-compiler libgoogle-glog-dev \
                     ros-$ROS_DISTRO-control-toolbox \
                     ros-$ROS_DISTRO-octomap-msgs \
                     ros-$ROS_DISTRO-octomap-ros \
                     ros-$ROS_DISTRO-mavros \
                     ros-$ROS_DISTRO-mavros-msgs \
                     ros-$ROS_DISTRO-rviz-visual-tools \
                     ros-$ROS_DISTRO-gazebo-plugins;

# Install gazebo 11 (default in melodic is gazebo 9)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install ros-melodic-gazebo11-ros-control \
                     ros-melodic-gazebo11-ros \
                     ros-melodic-gazebo11-msgs \
                     ros-melodic-gazebo11-plugins \
                     ros-melodic-gazebo11-ros-pkgs \
                     ros-melodic-gazebo11-dev;
```
_NOTE_:
* On Ubuntu 18.04, Gazebo 11 is needed, otherwise Gazebo may crash due to conflict between the GPU-based lidar simulator and the raytracing operations in our custom-built `gazebo_ppcom_plugin.cpp`.

Check the protobuf version as described in [Protobuf Version](#protobuf-version).
###  3.1.3. Protobuf version
* We have tested protobuf 3.0.0 and 3.6.1 with our packages. Protobuf version can be checked by running the following command:
```bash
protoc --version
```
If protoc version needs to be updated, try to remove protoc, and then reinstall with `sudo apt install protobuf-compiler`.
There can be multiple versions of the protobuf installed in the system. You can find the locations of the version used by the command `whereis protoc`.

##  3.2. Install the CARIC packages
Once the dependencis have been installed, please create a new workspace for CARIC, clone the necessary packages into it, and compile:

```bash
# Create the workspace
mkdir -p ~/ws_caric/src
cd ~/ws_caric/src

# Download the packages:

# Manager node for the mission
git clone https://github.com/ntu-aris/caric_mission

# Simulate UAV dynamics and other physical proccesses
git clone https://github.com/ntu-aris/rotors_simulator

# GPU-enabled lidar simulator, modified from: https://github.com/lmark1/velodyne_simulator
git clone https://github.com/ntu-aris/velodyne_simulator

# Converting the trajectory setpoint to rotor speeds
git clone https://github.com/ntu-aris/tcc

# To generate an trajectory based on fixed setpoints. Only used for demo, to be replaced by your controller
git clone https://github.com/ntu-aris/traj_gennav

# Build the workspace
cd ~/ws_caric/
catkin build
```
The compilation may report errors due to missing depencies or some packages in CARIC are not yet registered to the ros package list. This can be resolved by installing the missing dependencies (via `sudo apt install <package>` or `sudo apt install ros-$ROS_DISTRO-<ros_package_name>)`), then/or try `catkin build` again as the compiled packages are added to dependency.

##  3.3. Run the flight test

To make sure the code compiles and runs smoothly, please launch the example flight test with some pre-defined fixed trajectories:

```bash
source ~/ws_caric/devel/setup.bash
roscd caric_mission/scripts
bash launch_mbs.sh
```

You should see 5 UAVs take off, follow a fixed trajectory, and fall down when time is out.

# 4. The benchmark's design

##  4.1. The UAV fleet
A 5-UAV team is designed for the challenge, two of the _explorer_ class (nicknamed `jurong` and `raffles`), and three of the _photographer_ class (`changi`, `sentosa`, and `nanyang`), plus one GCS (Ground Control Station). Each unit has an intended role in the mission.
<div style="text-align:center">
  <img src="docs/fleet.jpeg" alt="fleet" width="40%"/>
  <figcaption>An illustration of one GCS, one explorer, and two photographers in Gazbo environment.</figcaption>
</div>

* The explorer drone carries a rotating lidar apparatus and a gimballed camera.
* The photographer is smaller and only carries gimballed camera.
* The GCS is where the images will be sent back and processed by the drones.

Note that the explorer is twice the size and weight of the photographer. Thanks to the bigger size, it can carry the lidar and quickly map the environment, at the cost of slower speed. In contrast, the photographers have higher speed, thus they can quickly cover the surfaces that have been mapped by the explorer to obtain images of higher score. The GCS's role is to compare the images taken by the drones. For each interest point, the GCS can select the image with the best quality to assign the score to it.

##  4.2. Inspection scenarios
The following scenarios are included in the challenge:

* Building inspection: The environment features a 60m tall building model that consists of three main vertical towers with a single void deck connecting the tops. The full 5-UAV fleet is deployed in this environment.

* Aircraft inspection: The environment features an airplane placed at the entrance of a hangar. The interest points are only located on the airplane. One explorer and two photographers are deployed in this environment. The airplane model is about 20m tall and 70m long.
  
* Crane inspection: The environment consists of two cranes that are 60 and 80 meters tall, typical in construction sites, plus one 50m tall gantry crane, typical of seaport environments. The full 5-UAV fleet is deployed in this environment.

##  4.3. The image capture quality metric
The metric is based on capture quality of interest points on the object surface.
<!-- The final judging criteria is the total number of interest points that have been fully captured and communitated back to the ground station.
For a point to be fully captured, it -->
A capture has to satisfy the following criteria:

###  4.3.1. LOS and FOV
The interest point has to fall in the field of view (FOV) of the camera, and the camera has direct line of sight (LOS) to the interest point (not obstructed by any other objects). The camera horizontal FOV and vertical FOV are defined by the parameters `HorzFOV` and `VertFOV` in the file [caric_ppcom_network.txt](https://github.com/ntu-aris/rotors_simulator/blob/master/rotors_description/ppcom_network/caric_ppcom_network.txt). Note that the camera orientation can be controlled as described in the section [Camera gimbal control](#531-camera-gimbal-control).

###  4.3.2. Motion blur
Motion blur is resulted from moving object during the camera exposure duration defined by the parameter `ExposureTime`. The motion blur metric, defined as the number of pixels that an interest point moves across during the exposure, is computed as: 

$$
\text{horizontal_blur} = \dfrac{|u_1-u_0|}{\text{pixel_width}},\\
u_0 = \text{focal_length}*\dfrac{x_0}{z_0},\\
u_1 = \text{focal_length}*\dfrac{x_1}{z_1},\\
[x_1,y_1,z_1]^\top = [x_0,y_0,z_0]^\top + \mathbf{v}*\text{exposure_time}.
$$

Here, $[x_0,y_0,z_0]^\top$ is the position of the interest point at the time of capture, and $[x_1,y_1,z_1]^\top$ is the updated position considering the velocity of the interest point in the camera frame $\mathbf{v}$ obtained at the time of the capture (see our derivation for this velocity at the following [link](docs/CARIC_motion_blur.pdf)). The figure below illustrates the horizontal motion blur by showing the horizontal (X-Z) plane of the camera frame.

<div style="text-align:center">
  <img src="docs/motionblur1.png" alt="resolution1" width="40%"/>
  <figcaption>Figure 2. Illustration of horizontal resolution computation</figcaption>
</div>

The vertical blur can be computed similarly by replacing $x_0$ and $x_1$ with $y_0$ and $y_1$ in the above computation of $u_0$ and $u_1$. For an interest point to be considered captured, the movement of the interest point has to be smaller than 1 pixel (so that the image is sharp), i.e.,

$$
\text{horizontal_blur} < 1,\\
\text{vertical_blur} < 1.\\
$$

###  4.3.3. Image resolution
The resolution of the image is expressed in mm/pixel, representing the size of the real-world object captured in one pixel of the image. To achieve a satisfactory resolution, the computed horizontal and vertical resolution have to be smaller than a desired mm/pixel value. Given the position of an interest point in the camera frame and its normal (perpendicular to its surface), the horizontal and vertical resolution can be obtained by displacing the interest point by $\pm 0.5$ mm along the line intersecting the interest surface and the horizontal/vertical plane in the camera coordinate system, and then finding the corresponding length of the object in the image. The image below illustrates this process, where the length of the object in the image is expressed as $\left|u_1-u_2\right|$.

<div style="text-align:center">
  <img src="docs/resolution1.png" alt="resolution1" width="40%"/>
  <figcaption>Figure 3. Illustration of horizontal resolution computation</figcaption>
</div>

The horizontal resolution is computed as $\text{horizontal_resolution}=\frac{\text{pixel_width}}{|u_1-u_2|}$. Similarly, $\text{vertical_resolution}=\frac{\text{pixel_width}}{|v_1-v_2|}$, $v_1$ and $v_2$ are the $y$-coordinates of the points in the image plane obtained by displacing the interest point along the line intersecting the interest surface and the vertical plane.
For a point to be considered captured, the resolutions have to satisfy

$$
\text{horizontal_resolution} < \text{desired_mm_per_pixel},\\
\text{vertical_resolution} < \text{desired_mm_per_pixel}.
$$

##  4.4. Evaluation

Different CARI schemes can use different strategies to acheive the highest inspection score within a finite amount of time.
The mission time starts from the moment any UAV takes off (when it's velocity exceeds 0.1m/s and it's altitude exceeds 0.1m). When the time elapses, all drones will shut down and no communication is possible.

During the mission, the GCS will receive the information regarding the captured interest points from the UAVs when there is LOS. The captures are compared and the score will be tallied and published in real time under the `/gcs` namespaces. After each mission, a log file will be generated in the folder specified under the param `log_dir` in the launch file of `caric_mission` package.

# 5. Developing your CARI scheme

##  5.1. Ground rules

The following rules should be adhered to in developing a meaningful CARI scheme:

* <u>Isolated namespaces</u>: The software processes should be isolated by the appropriate namespaces. Consider each namespace the local computer running on a unit. In the real world the processes on one computer should not be able to freely subscribe to a topic in another computer. Information exchange between the namespace is possible but should be subjected to the communication network's characteristics (see the next rule). Currently there are six namespaces used in CARIC: `/gcs`, `/jurong`, `/rafffles`, `/changi`, `/sentosa`, and `/nanyang`. Note that some topics may be published outside of these namespaces for monitoring and evaluating purposes, and should not be subscribed to by any user-defined node.

* <u>Networked communication</u>: The communication between the namespaces should be regulated by the `ppcom_router` node, which simulates a peer-to-peer broadcast network, where messages can be sent directly from a node in one namespace to another node in another namespace **when there is LOS**. Please refer to Section [5.4. Simulated networked communication](#54-simulated-networked-communication) for the instructions on how to apply the `ppcom_router` node.

* <u>No prior map</u>: Though the prior map of the structures and the locations of the interestpoints are available, they are only used for simulation. Users should develop CARI schemes that only rely on the onboard perception, and other information that are exchanged between the units via the `ppcom_router` network.

##  5.2. Estimation data

CARIC is intended for investigating cooperative control schemes, hence perception proccesses such as sensor fusion, SLAM, map merging, etc... are assumed perfect (for now). Users can obtain the accurate odometry from `/<unit_id>/ground_truth/odometry` (inclusing pose, velocity and acceleration), and pointcloud from `/<unit_id>/cloud_inW` for control, mapping, and obstacle avoidance tasks (`<unit_id>` is a namespace in the UAV fleet, for e.g. `gcs` or `raffles`). Moreover, a unit can also receive the neigbours' odometry in `/<unit_id>/nbr_odom_cloud`, as well as the neigbours' latest lidar key frame `/<unit_id>/nbr_kf_cloud`. Note that these topics only have information when there is LOS to the neighours.

##  5.3. UAV control interface

Whatever control strategy is developed, the control signal should be eventually converted to standard multi-rotor command. Specifically the UAVs are controlled using the standard ROS message `trajectory_msgs/MultiDOFJointTrajectory`. The controller subscribes to the command trajectory topic `/<unit_id>/command/trajectory`. Below are sample codes used to publish a trajectory command in `traj_gennav_node.cpp`, given 3d target states in the global(world) frame `target_pos`, `target_vel`, `target_acc` and a target yaw `target_yaw`:

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

###  5.3.1. Camera gimbal control
The camera is assummed to be installed on a camera stabilizer (gimbal) located at [`CamPosX`, `CamPosY`, `CamPosZ`] in the body frame of the drone. To be realistic, we allow the users to control the gimbal pitch and yaw angle while keeping the gimbal roll at zero. The gimbal control interface is the topic `/<unit_id>/command/gimbal` of type `geometry_msgs/Twist`. An example is shown below:
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
As explained in the comments in the sample code, the interface allows angle-based or rate-based control. When `gimbal_msg.linear.x` is set to 1.0, the fields `gimbal_msg.linear.y` and `gimbal_msg.linear.z` indicates the command pitch and yaw angle, respectively. The pitch and yaw angles are controlled independently: given a target pitch/yaw angle, the gimbal will move with the maximum pitch/yaw rate defined by the parameter `GimbalRateMax` (in degree/s) until reaching the target. In velocity control mode, the gimbal pitch/yaw rates can be set to any value in the range [`-GimbalRateMax`,`+GimbalRateMax`]. The gimbal pitch and yaw only operate in the ranges [`-GimbalPitchMax`,`+GimbalPitchMax`] and [`-GimbalYawMax`,`+GimbalYawMax`], respectively.

Here, the gimbal roll, pitch and yaw angles are defined as the euler angles ([in Z-Y-X intrinsic rotation sequence](https://en.wikipedia.org/wiki/Euler_angles#Conventions_by_intrinsic_rotations)) describing the gimbal orientation with respect to a virtual frame, whose X-axis is always parallel to the X-axis of the drone body frame, and X-Y plane is always parallel to the X-Y plane in the world frame (due to roll being stabilized). If we define the camera frame with its X-axis perpendicular to the image plane, and Z-axis pointing upward in the image plane, then, the euler angle of the camera with respect to the world frame can be obtained as
```cpp
  camera_Yaw_in_world_frame = drone_yaw_in_world_frame + gimbal_yaw;
  camera_Pitch_in_world_frame = gimbal_pitch;
  camera_roll_in_world_frame = 0.0;
```

The gimbal euler angle and angular rates are published through the topic `/<unit_id>/gimbal` of type `geometry_msgs/TwistStamped`. The fields `twist.linear` indicates the euler angle while the fields `twist.angular` indicates the angular rates (a bit of deviation from the original meaning of the message type).

###  5.3.2. Camera trigger
Two camera trigger modes are allowed. If the parameter `manual_trigger` is set to false, the robot will automatically trigger camera capture at a fixed time interval defined by the parameter `TriggerInterval`. If the parameter `manual_trigger` is set to true, the user may send camera trigger command by publishing to a topic `/<unit_id>/command/camera_trigger` of type `rotors_comm/BoolStamped`:

```cpp
rotors_comm::BoolStamped msg;
msg.header.stamp = ros::Time::now();
msg.data = true;
trigger_pub.publish(msg);  //trigger_pub has to be defined as a ros::Publisher
```
Note that in manual trigger mode, the time stamps of two consecutive trigger commands should still be separated by an interval larger than the parameter `TriggerInterval`, otherwise, the second trigger command will be ignored. The benefit of using manual trigger is that the users may send the trigger command at the exact time that results in the best capture quality.

##  5.4. Simulated networked communication

Each robot is given a unique ID in a so-called ppcom network, for e.g. gcs, firefly1, firely2. These IDs can be specified in the [description file](https://github.com/ntu-aris/rotors_simulator/blob/master/rotors_description/ppcom_network/caric_ppcom_network.txt).

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

* Below is an example to demonstrate the LOS-only communication feature:

<div align="center">
  <iframe width="800" height="450" src="https://www.youtube.com/embed/_DtOr0rsJMQ"
          title="YouTube video player" frameborder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
          allowfullscreen></iframe>
  <figcaption>Illustration of communication among ros nodes under different namespaces via the ppcom_router: In the first terminal we start the the simulation. Net the <b>ppcom_router</b> node is launch. Then <b>ppcom_firefly1_talker</b>, <b>ppcom_firefly2_talker</b>, <b>ppcom_firefly3_talker</b> are launched in different terminals. You can observe the messages sent and received by each node in the corresponding terminal. Notice how the messages are dropped when the corresponding entry in the LOS matrix turns to 0 (the <b>firefly1</b> --> <b>firefly3</b> LOS status is indicated by the entry at 2nd row, 4th column).</figcaption>
</div>

