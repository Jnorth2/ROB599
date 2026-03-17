# HW5_JN
`hw5_jn` is a ros2 package for the ROB599 HW5. It provides a custom launch file for a turtle bot nav2 stack and a ROS2 node to send waypoints to the nav2 stack. 

## Installation
- Clone the git repository to the desired directory.
- Then open the directory and build in the root directory.

```bash
cd "Path/to/Directory"  
colcon build  
source install/setup.bash  
```  

- Source ROS2 and setup turtlebot parameters (See [Getting Started](https://docs.nav2.org/getting_started/index.html))
```bash
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic
```

## Dependencies
- ROS2 Humble +
- Nav2
- Gazebo Classic 

## Usage
This package contains a launch file that launches a turtlebot nav2 stack with a gazebo house world. An additional ROS2 node `send_waypoints` is used to send poses to the nav2 `navigate_to_pose` action server and spins the robot in a 360 upon reaching the pose with the `spin` action server. Waypoints are provided by a yaml file found in the `config` directory and can be passed in through a parameter. 
- Launching the main launch
    ```bash
    ros2 launch hw5_jn hw5_launch.py
    ```
- Running the Waypoints Node (for demonstrating mapping)
    ```bash
    ros2 run hw5_jn send_waypoints
    ```
    - Parameters
        - waypoints : string
            - File path to a waypoints yaml file. 

## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Author
Jared Northrop  
jar3dnorth51@gmail.com


