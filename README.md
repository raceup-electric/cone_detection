# cone_detection
Perception module


IMPORTANT: when launching ouster_ros node in sensor mode (live), don't forget to specify  **use_system_default_qos:=true**

Complete command: ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os-122410001022.local use_system_default_qos:=true

##

This is the perception module of the Autonomous System stack. The node gets a continuous flow of data from the Ouster OS1 LiDAR, processing each frame to detect and classify cones present in the scene.

## Installation

This repository contains a ROS2 package, not a whole workspace, so in order to execute the code you will need to create one:

1. Create a workspace folder:

    ```bash
    mkdir -p workspace/src
    ```

2. Navigate inside the src directory:Eigen3

    ```bash
    cd workspace/src
    ```

3. Clone this repository:

    ```bash
    git clone https://github.com/raceup-electric/cone_detection
    ```

4. Clone the eufs_msgs repository:

    ```bash
    git clone https://gitlab.com/eufs/eufs_msgs
    ```

## Usage

After the installation steps:

1. Navigate to the workspace directory:

    ```bash
    cd <workspace_path>
    ```

2. Build the ROS workspace:

    ```bash
    colcon build
    ```

3. Source the ROS setup file:

    ```bash
    source install/setup.bash
    ```

4. Launch the node and RViz:

    ```bash
    ros2 launch cone_detection cone_detection_launch.py
    ```

**Note:** in order to make the node actually do anything you need to physically connect the LiDAR with the software provided at https://github.com/ouster-lidar/ouster-ros/tree/ros2-foxy or play a recorded bag.


## ROS Topics

List of ROS topics published and subscribed to by this node:

### Publishers:
        - /cone_pose
(plus other debug publishers)
### Subscribed to:
		- /ouster/points


### Launch LiDAR in sensor mode
```ros2 launch ouster_ros sensor.launch.xml        sensor_hostname:=os-122410001022.local ```
