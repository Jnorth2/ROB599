import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, Pose, Transform, TransformStamped
from sensor_msgs.msg import LaserScan

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point
import tf_transformations as tft

import numpy as np
from skimage.draw import line
from math import atan2, sqrt
import cv2 as cv
import os
import time
import copy

from pathlib import Path

class PrintImage(Node):

    def __init__(self):
        super().__init__('print_image')

        #Launch Parameters
        self.declare_parameter('use_twist_stamped', False)
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value

        self.declare_parameter('use_odom', True)
        self.use_odom = self.get_parameter('use_odom').value

        self.declare_parameter('num_particles', 1)
        self.num_particles = self.get_parameter('num_particles').value

        self.get_logger().info(f"use odom: {self.use_odom}")
        #setup pub, sub
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # if self.use_twist_stamped:
        #     self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_cb, qos)
        #     # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        # else:
        #     self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)

        if self.use_odom:
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.create_subscription(OccupancyGrid, "map", self.map_cb, 10) 

        self.timer_ = self.create_timer(0.5, self.timer_callback)

        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)

        #Monte carlo print_image Params
        self.particles = None
        self.got_map = False

        self.actual_loc = TransformStamped()
        self.move_thresh = 0.003
        self.rot_thresh = 0.01

        #Map Params
        self.map = None
        self.np_map = None

        #robot Params
        self.location = TransformStamped()
        self.move_location = TransformStamped()
        self.move_u = 0.00
        self.move_sigma = 0.05
        self.rot_u = 0.0
        self.rot_sigma = 0.1
        self.max_obj_dist = 5.0
        self.min_obj_dist = 0.0
        self.laser_sigma = 0.1


        #Data and Debug Path
        self.pkg_src_path = Path(__file__).resolve().parents[1]
        self.save_dir = os.path.join(self.pkg_src_path, "data")
        os.makedirs(self.save_dir, exist_ok=True)

    def map_cb(self, msg):
        self.map = msg
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        temp_map = np.flipud(temp_map)
        self.np_map = temp_map

        if not self.got_map:
            self.particles = []
            particle = {}
            particle["pose"] = Pose()
            particle['pose'].position.x = self.location.transform.translation.x
            particle['pose'].position.y = self.location.transform.translation.y
            particle["weight"] = 1.0
            self.particles.append(particle)
            self.got_map = True

    def timer_callback(self):
        self.particles_to_image(self.particles)
        # self.raw_image()
        x_map, y_map = self.transform_to_grid(self.particles[0]["pose"].position.x, self.particles[0]["pose"].position.y)
        x_global, y_global = self.grid_to_transform(x_map, y_map)
        self.get_logger().info(f"Global Position: [{x_global}, {y_global}]")
        x = self.particles[0]["pose"].position.x
        y = self.particles[0]["pose"].position.y
        self.get_logger().info(f"Global Position from particle: [{x}, {y}]")

    def odom_cb(self, msg):
        """
        """
        # self.get_logger().info(f"Odom message {msg}")
        # self.get_logger().info(f"Location message {self.location}")
        self.move_location.header.stamp = msg.header.stamp
        self.move_location.header.frame_id = msg.header.frame_id
        self.move_location.child_frame_id = msg.child_frame_id
        self.move_location.transform.translation.x = msg.pose.pose.position.x
        self.move_location.transform.translation.y = msg.pose.pose.position.y
        self.move_location.transform.translation.z = msg.pose.pose.position.z
        self.move_location.transform.rotation = msg.pose.pose.orientation
        if self.got_map:
            self.particles[0]['pose'].position.x = self.move_location.transform.translation.x
            self.particles[0]['pose'].position.y = self.move_location.transform.translation.y
    
    def grid_to_transform(self, x_map, y_map):
        x_global = x_map * self.map.info.resolution + self.map.info.origin.position.x
        y_global = y_map * self.map.info.resolution + self.map.info.origin.position.y
        return x_global, y_global 
    
    def transform_to_grid(self, x_global, y_global):
        """Determine the cell indexes for a set of global x, y coordinates. 

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
        """
        x_map = int((x_global - self.map.info.origin.position.x) / self.map.info.resolution)
        y_map = int((y_global - self.map.info.origin.position.y) / self.map.info.resolution)

        # if not self.use_odom:
        #     self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        # robot_x = int((self.location.transform.translation.x - self.map.info.origin.position.x) / self.map.info.resolution)
        # robot_y = int((self.location.transform.translation.y - self.map.info.origin.position.y) /self.map.info.resolution)

        return x_map, y_map#, robot_x, robot_y

    def raw_image(self):
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map2 = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height, self.map.info.width)
        temp_map2 = temp_map2.reshape(self.map.info.width, self.map.info.height)
        temp_map3 = copy.deepcopy(np.flipud(temp_map))
        temp_map4 = copy.deepcopy(np.flipud(temp_map2))
        maps1 = [temp_map, temp_map3]
        imgs = []
        for m in maps1:
            unknown_mask = np.array(m < 0)
            known_mask = ~unknown_mask
            #Calc Grey Values
            grey_val = (1.0 - m[known_mask]/100.0) * 255.0
            grey_val = grey_val.astype(np.uint8)
            #Create Image
            img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
            img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
            #Make unseen cyan
            img[unknown_mask] = np.array([200, 200, 0], dtype=np.uint8)
            imgs.append(img)
        maps2 = [temp_map2, temp_map4]
        for m in maps2:
            unknown_mask = np.array(m < 0)
            known_mask = ~unknown_mask
            #Calc Grey Values
            grey_val = (1.0 - m[known_mask]/100.0) * 255.0
            grey_val = grey_val.astype(np.uint8)
            #Create Image
            img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
            img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
            #Make unseen cyan
            img[unknown_mask] = np.array([200, 200, 0], dtype=np.uint8)
            imgs.append(img)

        imgs[1][10, :] = np.array([255, 0, 255], dtype=np.uint8)
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'no_flip_normal_{time.sec}.png'), imgs[0])
        cv.imwrite(os.path.join(self.save_dir, f'flip_normal_{time.sec}.png'), imgs[1])
        cv.imwrite(os.path.join(self.save_dir, f'no_flip_reshape_{time.sec}.png'), imgs[2])
        cv.imwrite(os.path.join(self.save_dir, f'flip_reshape_{time.sec}.png'), imgs[3])

    def particles_to_image(self, particles, name = ""):
        #Get map in 2d Array
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        # temp_map = np.flipud(temp_map)

        #determine Seen vs unseen
        unknown_mask = np.array(temp_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - temp_map[known_mask]/100.0) * 255.0
        grey_val = grey_val.astype(np.uint8)
        #Create Image
        img = np.zeros((self.map.info.height, self.map.info.width, 3), dtype=np.uint8)
        img[known_mask] = np.stack([grey_val, grey_val, grey_val], axis=1)
        #Make unseen cyan
        img[unknown_mask] = np.array([200, 200, 0], dtype=np.uint8)

        #Make Path cells yellow
        x = []
        y = []
        for i in range(len(particles)):
            x_map, y_map = self.transform_to_grid(particles[i]["pose"].position.x, particles[i]["pose"].position.y)
            x.append(x_map)
            y.append(y_map)
        y = np.array(y)
        x = np.array(x)
        img[y, x] = np.array([0, 0, 255], dtype=np.uint8)
        best_weight = 0
        best_x = 0
        best_y = 0
        for p in particles:
            if p["weight"] > best_weight:
                x_map, y_map = self.transform_to_grid(particles[i]["pose"].position.x, particles[i]["pose"].position.y)
                best_x = x_map
                best_y = y_map
        img[best_y, best_x] = np.array([0, 255, 0], dtype=np.uint8)

        img = np.flipud(img)

        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'Particles_{name}_{time.sec}.png'), img)
    

def main(args=None):
    rclpy.init(args=args)

    print_image = PrintImage()

    rclpy.spin(print_image)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()