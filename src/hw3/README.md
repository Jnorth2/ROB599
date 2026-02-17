# HW3
`hw3` is a ros2 package for the ROB599 HW3. This contains a mapping and path planning algorithm.

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

- Note: `hw3` requires `hw2` to be built and uses the driver node. 

## Usage
This package contains a launch file that launches a stage world, rviz, the driver, mapper, and path planner. The driver contains an action server to recieve points. The mapper keeps a map and publishes to a `/map` topic. The path planner waits for a service call before making a path to a hard coded global point. 
- Launching the world and driver nodes
    ```bash
    ros2 launch hw3 map.launch.py
    ```
    - Parameters
        - world : {open, hexagon} 
        - is_dwa : {True, False}
        - use_twist_stamped : {True, False}
            - This should be false for sim and True for the real robot.
        - save_map : {True, False}
            - True to save the map every 5 sec and save the planned path.
- Running the Waypoints Node (for demonstrating mapping)
    ```bash
    ros2 run hw2 send_waypoint
    ```
- Trigger the Planning
    ```bash
    ros2 service call /plan_path std_srvs/srv/Trigger "{}"
    ```
## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Author
Jared Northrop 
jar3dnorth51@gmail.com


