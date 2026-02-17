# HW3
`HW3` is a ros2 package for the ROB599 HW3. This contains a mapping and path planning algorithm.

## Installation
- Clone the git repository to the desired directory.
- Then open the directory and build in the root directory.

```bash
cd "Path/to/Directory"  
colcon build  
source install/setup.bash  
```  

## Dependencies
- ROS2 Humble + 
- Stage
- stage_ros2
- hw2

## Usage
This package contains a launch file that launches a stage world, rviz, and the driver. The driver contains an action server to recieve points. A seperate `send_waypoint` node is also provided to send a set of test points. 
- Launching the world and driver nodes
    ```bash
    ros2 launch hw2 world.launch.py
    ```
    - Parameters
        - world : {open, hexagon} 
        - is_dwa : {True, False}
        - use_twist_stamped : {True, False}
            - This should be false for sim and True for the real robot.
- Running the Waypoints Node
    ```bash
    ros2 run hw2 send_waypoint
    ```
## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Author
Jared Northrop 
jar3dnorth51@gmail.com


