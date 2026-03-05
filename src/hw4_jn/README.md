# HW4_JN
`hw4_jn` is a ros2 package for the ROB599 HW4. This contains a Monte Carlo Localization (MCL) algorithm.

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

- Note: `hw4_jn` requires `hw3`, `hw2`, `nav_interface` to be built and uses the driver and mapping nodes. 

## Usage
This package contains a launch file that launches a stage world, rviz, the driver, mapper and localizer. The driver contains an action server to recieve points. The mapper keeps a map and publishes to a `/map` topic. The localizer keeps track of particles in the map and publishes them to `\particles` and `\best_particle` topics. To move the robot, use the send waypoints node from `hw2`. 
- Launching the main launch
    ```bash
    ros2 launch hw4_jn hw4.launch.py
    ```
    - Parameters
        - world : {open, hexagon} 
        - is_dwa : {True, False}
        - use_twist_stamped : {True, False}
            - This should be false for sim and True for the real robot.
        - save_map : {True, False}
            - True to save the map every 5 sec.
- Running the Waypoints Node (for demonstrating mapping)
    ```bash
    ros2 run hw2 send_waypoint
    ```
- Trigger Reset Particles
    ```bash
    ros2 service call /reset_particles nav_interface/srv/ResetParticles "{
        use_pose: true,
        pose: {
            position: {x: 0.0, y: 0.0, z: 0.0},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
    }"
    ```
    - Parameters
        - use_pose : Bool
            - True to reset with the pose, false to reset uniformly
        - pose : Pose
            - Pose to initialize particles
- Trigger Save Error Plot
    ```bash
    ros2 service call /plot_error std_srvs/srv/Trigger "{}"
    ```
    - Saves a plot of the localization error to the data folder in the `hw4_jn` directory

## Liscense
This package is liscensed under the Apache 2.0 liscense.

## Author
Jared Northrop 
jar3dnorth51@gmail.com


