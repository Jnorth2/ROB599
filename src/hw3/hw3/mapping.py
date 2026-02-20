import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap, GetPlan
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, Pose, Transform, TransformStamped
from sensor_msgs.msg import LaserScan

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

import numpy as np
import scipy
from skimage.draw import line
from math import atan2, sqrt
import cv2 as cv
import os
import time

from pathlib import Path

class Mapping(Node):

    def __init__(self):
        super().__init__('mapping')

        #Launch Parameters
        self.declare_parameter('use_twist_stamped', False)
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value

        self.declare_parameter('save_map', False)
        self.save_map = self.get_parameter('save_map').value

        self.declare_parameter('use_odom', False)
        self.use_odom = self.get_parameter('use_odom').value

        self.get_logger().info(f"use odom: {self.use_odom}")
        #setup pub, sub
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        if self.use_twist_stamped:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
            self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_cb, qos)
            # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)


        else:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)

        if self.use_odom:
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, "map", 10)

        self.make_image = self.create_service(Trigger, "occ_to_img", self.occ_to_img_cb)

        #Timer to save map images for debugging
        if self.save_map:
            timer_period = 5.0 # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)

        #map Parameters
        self.cell_size = 0.05 #[m]
        self.width = 32
        self.height = 32
        self.h_cell = int(self.height/self.cell_size)
        self.w_cell = int(self.width/self.cell_size)
        self.detail_map = -np.ones((self.h_cell, self.w_cell))

        #occupancy map
        self.occupancy_threshold = 30
        self.map = OccupancyGrid()
        self.map.header.frame_id = "odom"
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.info.resolution = self.cell_size
        self.map.info.width = self.w_cell
        self.map.info.height = self.h_cell
        self.map.info.origin = Pose()
        self.map.info.origin.position.x = -16.0
        self.map.info.origin.position.y = -16.0

        #robot Params
        self.location = TransformStamped()
        self.laser_loc = None
        self.max_obj_dist = 5.0
        self.min_obj_dist = 0.0

        self.laser_sigma = 0.1
        # self.location = self.tf_buffer.lookup_transform('odom', 'laser', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        self.get_logger().info(f"Initial location: {self.location}")

        #Image saving
        self.pkg_src_path = Path(__file__).resolve().parents[1]

        self.save_dir = os.path.join(self.pkg_src_path, "data")
        os.makedirs(self.save_dir, exist_ok=True)

        if self.save_map:
            self.map_to_occmap()
            self.occmap_to_image()

    def timer_callback(self):
        self.map_to_occmap()
        self.occmap_to_image()
        return
    
    def laser_cb(self, scan):
        #get local Laser coords
        x_local, y_local, distance = self.get_laser_coords(scan)
        # self.get_logger().info(f"Current time: {self.get_clock().now().to_msg().sec}, Scan time {scan.header.stamp}")
        # print(x_local, y_local)
        x_global, y_global, ran = self.laser_local_to_map(x_local, y_local, scan.header.stamp) 
        if not ran:
            return
        x_map, y_map, robot_x, robot_y = self.transform_to_grid(x_global, y_global)
        self.update_map(x_map, y_map, robot_x, robot_y, distance)
        self.map_to_occmap()
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        # self.get_logger().info(f"Map is: {self.map.data}")
        return
    
    def transform_to_grid(self, x_global, y_global):
        """Determine the cell indexes for a set of global x, y coordinates and the robot. 

        Parameters
        ----------
        x_global : np.array(floats)
            An array of x position values in the global frame. 
        y_global : np.array(floats)
            An array of y position values in the global frame. 

        Returns
        -------
        x_map : np.array(int)
            The x indexes in the map frame.
        y_map : np.array(int)
            The y indexes in the map frame.
        robot_x : int
            The robots x index in the map frame.
        robot_y : int
            The robots y index in the map frame.  
        """
        x_map = ((x_global - self.map.info.origin.position.x) / self.map.info.resolution).astype(int)
        y_map = ((y_global - self.map.info.origin.position.y) / self.map.info.resolution).astype(int)

        # self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        if not self.use_odom:
            robot_x = int((self.laser_loc.transform.translation.x - self.map.info.origin.position.x) / self.map.info.resolution)
            robot_y = int((self.laser_loc.transform.translation.y - self.map.info.origin.position.y) /self.map.info.resolution)
        else:
            robot_x = int((self.location.transform.translation.x - self.map.info.origin.position.x) / self.map.info.resolution)
            robot_y = int((self.location.transform.translation.y - self.map.info.origin.position.y) /self.map.info.resolution)
        return x_map, y_map, robot_x, robot_y

    def update_map(self, x_map, y_map, robot_x, robot_y, distance):
        #Ray trace each lidar beam
        for i, x in enumerate(x_map):
            rr, cc = line(robot_y, robot_x, y_map[i], x_map[i])
            mu = distance[i]
            #calc distance to each cell
            line_distance = np.sqrt((rr - robot_y) ** 2 + (cc-robot_x) ** 2) * self.map.info.resolution
            #Calc probablity occupied vector form

            temp_map = np.where(self.detail_map[rr, cc] < 0.0, 0.5, self.detail_map[rr, cc])

            p_ray = np.exp(-0.5 * ((line_distance-mu) / self.laser_sigma) ** 2)
            #bayes update
            p_ray = p_ray * temp_map / (p_ray * temp_map + (1-p_ray) * (1-temp_map))
            p_ray = np.clip(p_ray, 0.00001, 0.99999)
            self.detail_map[rr, cc] = p_ray

            #Calc the probability occupied slow
            # for j in range(len(line_distance)):
            #     p_z_d = scipy.stats.norm.pdf(line_distance[j], loc=mu, scale = self.laser_sigma)
            #     p_occ = self.detail_map[rr[j], cc[j]]
            #     if p_occ == -1:
            #         p_occ = 0.5
            #     new_p_occ = p_z_d * p_occ / (p_z_d * p_occ + (1-p_z_d) * (1-p_occ))
            #     new_p_occ = max( 0.00001, min(0.99999, new_p_occ))
            #     self.detail_map[rr[j], cc[j]] = new_p_occ     
    
    def laser_local_to_map(self, x_local, y_local, stamp):
        #Get transform in 2D Parts
        # if not self.tf_buffer.can_transform('odom', 'laser', stamp, timeout=rclpy.duration.Duration(seconds=0.5)):
        #     self.get_logger().warn(f"Failed to get transform")
        #     return [], []
        if not self.use_odom:
            try: 
                self.laser_loc = self.tf_buffer.lookup_transform('odom', 'laser', stamp, timeout=rclpy.duration.Duration(seconds = 1.0))
            except Exception:
                self.get_logger().warn(f"Failed to get transform")
                return -1, -1, 0
            x_trans = self.laser_loc.transform.translation.x 
            y_trans = self.laser_loc.transform.translation.y 
            yaw = atan2(
                2.0 * (self.laser_loc.transform.rotation.w * self.laser_loc.transform.rotation.z + self.laser_loc.transform.rotation.x * self.laser_loc.transform.rotation.y),
                1.0 - 2.0 * (self.laser_loc.transform.rotation.y * self.laser_loc.transform.rotation.y + self.laser_loc.transform.rotation.z * self.laser_loc.transform.rotation.z)
            )
        else:
            x_trans = self.location.transform.translation.x 
            y_trans = self.location.transform.translation.y 
            yaw = atan2(
                2.0 * (self.location.transform.rotation.w * self.location.transform.rotation.z + self.location.transform.rotation.x * self.location.transform.rotation.y),
                1.0 - 2.0 * (self.location.transform.rotation.y * self.location.transform.rotation.y + self.location.transform.rotation.z * self.location.transform.rotation.z)
            )
        # print(f"Yaw from x axis: {yaw}")
        #Calculate Global Coordinates
        x_global = x_trans + x_local * np.cos(yaw) - y_local * np.sin(yaw)
        y_global = y_trans + y_local * np.cos(yaw) + x_local * np.sin(yaw)

        return x_global, y_global, 1

    def get_laser_coords(self, scan):
        #Get the angle range			
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        max_range = scan.range_max
        num_readings = len(scan.ranges)
        thetas = np.linspace(angle_min, angle_max, num_readings)
        ranges = np.array(scan.ranges)

        #Decompose scan into x and y coords using sin, cos
        y_readings = scan.ranges * np.sin(thetas)
        x_readings = scan.ranges * np.cos(thetas)

        #only keep points close enough (and eliminate points at the max range of the laser)
        max_range = min(max_range, self.max_obj_dist)
        indicies = np.where((ranges < max_range - 0.0000001) & (ranges > self.min_obj_dist))
        distance = ranges[indicies]

        return np.array(x_readings[indicies]), np.array(y_readings[indicies]), distance

    def transform_to_pose(self, transform : Transform):
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        pose.orientation.w = transform.rotation.w
        return pose
    
    def odom_cb(self, msg):
        """Not Used
        """
        # self.get_logger().info(f"Odom message {msg}")
        # self.get_logger().info(f"Location message {self.location}")
        self.location.header.stamp = msg.header.stamp
        self.location.header.frame_id = msg.header.frame_id
        self.location.child_frame_id = msg.child_frame_id
        self.location.transform.translation.x = msg.pose.pose.position.x
        self.location.transform.translation.y = msg.pose.pose.position.y
        self.location.transform.translation.z = msg.pose.pose.position.z
        self.location.transform.rotation = msg.pose.pose.orientation

    
    def map_to_occmap(self):
        occmap = -np.ones((self.h_cell, self.w_cell), dtype=np.int8)
        known_mask = self.detail_map >= 0
        #Scale probablity to [0,100]
        scale_range = np.clip(self.detail_map[known_mask], 0, 1) * 100
        occmap[known_mask] = scale_range.astype(np.int8)
        #Convert to occmap data list
        self.map.data = occmap.flatten().tolist()

    def occ_to_img_cb(self, request, response):
        self.map_to_occmap()
        self.occmap_to_image()
        response.success = True
        response.message = "image generated"
        return response

    def occmap_to_image(self):
        #Get map in 2d Array
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        temp_map = np.flipud(temp_map)
        #determine Seen vs unseen
        unknown_mask = np.array(temp_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - temp_map[known_mask]/100.0) * 255.0
        grey_val = grey_val.astype(np.uint8)
        #Create Image
        img = np.zeros((self.h_cell, self.w_cell, 3), dtype=np.uint8)
        img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
        #Make unseen cyan
        img[unknown_mask] = np.array([255, 255, 0], dtype=np.uint8)
        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'map_out_{time.sec}.png'), img)

def main(args=None):
    rclpy.init(args=args)

    mapping = Mapping()

    # rclpy.spin(mapping)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # mapping.destroy_node()
    # rclpy.shutdown()

    executor = MultiThreadedExecutor()
    executor.add_node(mapping)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()