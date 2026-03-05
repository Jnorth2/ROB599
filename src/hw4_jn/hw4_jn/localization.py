import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, Pose, Transform, TransformStamped, PoseArray
from sensor_msgs.msg import LaserScan
from nav_interface.srv import ResetParticles

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
import matplotlib.pyplot as plt

from pathlib import Path

class Localization(Node):

    def __init__(self):
        super().__init__('localization')

        #Launch Parameters
        self.declare_parameter('use_twist_stamped', False)
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value

        self.declare_parameter('use_odom', True)
        self.use_odom = self.get_parameter('use_odom').value

        self.declare_parameter('save_img', False)
        self.save_img = self.get_parameter('save_img').value

        self.declare_parameter('num_particles', 200)
        self.num_particles = self.get_parameter('num_particles').value

        self.get_logger().info(f"use odom: {self.use_odom}")
        #setup pub, sub
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        if self.use_twist_stamped:
            self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_cb, qos)
            # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        else:
            self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)

        if self.use_odom:
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.create_subscription(OccupancyGrid, "map", self.map_cb, 10) 

        self.pub_part = self.create_publisher(PoseArray, "particles", 10)

        self.pub_best_part = self.create_publisher(PoseArray, "best_particle", 10)

        if self.save_img:
            self.timer_ = self.create_timer(5.0, self.timer_cb)

        self.reset_service = self.create_service(ResetParticles, "reset_particles", self.reset_particles_cb)
        self.plot_error_srv = self.create_service(Trigger, "plot_error", self.plot_error_cb)

        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)

        #Monte carlo localization Params
        self.particles = None
        self.got_map = False

        self.actual_loc = TransformStamped()
        self.move_thresh = 0.003
        self.rot_thresh = 0.01

        self.error = []
        self.time_steps = []
        self.start_time = self.get_clock().now()

        #Map Params
        self.map = None
        self.np_map = None

        self.init_pose = Pose()
        self.init_pose.position.x = 0.0
        self.init_pose.position.y = 0.0
        self.init_pose.orientation.w = 1.0
        self.init_rot_sigma = 1.5708
        self.init_sigma = 0.5

        #robot Params
        self.location = TransformStamped()
        self.move_location = TransformStamped()
        self.move_u = 0.00
        self.move_sigma = 0.05
        self.rot_u = 0.0
        self.rot_sigma = 0.1
        self.max_obj_dist = 5.0
        self.min_obj_dist = 0.0
        self.laser_sigma = 0.5


        #Data and Debug Path
        self.pkg_src_path = Path(__file__).resolve().parents[1]
        self.save_dir = os.path.join(self.pkg_src_path, "data")
        os.makedirs(self.save_dir, exist_ok=True)

    def laser_cb(self, scan):
        # return
        if self.got_map:
            updated_w_move = False
            #Move first:
            move_diff, dist, yaw = self.move_diff()
            # self.get_logger().info(f"Movement change: {dist}, {abs(yaw)}")
            if abs(dist) > self.move_thresh or abs(yaw) > self.rot_thresh:
                self.importance_sample()
                self.move_update(dist, yaw)
                self.norm_w()
                updated_w_move = True
                # self.particles_to_image(self.particles)
                self.location = copy.deepcopy(self.move_location)
            #Sensor:
            if updated_w_move:
                self.importance_sample()
                x_coords, y_coords, dist, thetas = self.get_laser_coords(scan)
                self.sensor_update(x_coords, y_coords, dist, thetas)
                # self.particles_to_image(self.particles)

                particles = PoseArray()
                particles.header.frame_id = "odom"
                particles.header.stamp = self.get_clock().now().to_msg()

                for p in self.particles:
                    particles.poses.append(p["pose"])
                self.pub_part.publish(particles)

                best_particle = PoseArray()
                best_particle.header.frame_id = "odom"
                best_particle.header.stamp = self.get_clock().now().to_msg()
                best_weight = 0
                best_pose = None
                for p in self.particles:
                    # self.get_logger().info(f"particle image {p}")
                    if p["weight"] > best_weight:
                        best_pose = p["pose"]
                best_particle.poses.append(best_pose)
                self.pub_best_part.publish(best_particle)
                self.get_logger().info(f"Best Pose: x={best_pose.position.x} y={best_pose.position.y}, Actual Pose: x={self.location.transform.translation.x} y={self.location.transform.translation.y}")
                self.error.append(sqrt((best_pose.position.x-self.location.transform.translation.x) ** 2)+(best_pose.position.y-self.location.transform.translation.y) ** 2)
                time = self.get_clock().now() - self.start_time
                t = time.nanoseconds * 1e-9
                self.time_steps.append(t)

    def map_cb(self, msg):
        self.map = msg
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height, self.map.info.width)
        # temp_map = np.flipud(temp_map)
        thresh_map = (temp_map > 30).astype(np.uint8)

        self.np_map = thresh_map

        if not self.got_map:
            self.reset_particles_pose()
            # self.reset_one()
            if self.save_img:
                self.particles_to_image(self.particles)
            particles = PoseArray()
            particles.header.frame_id = "odom"
            particles.header.stamp = self.get_clock().now().to_msg()

            for p in self.particles:
                particles.poses.append(p["pose"])
            self.pub_part.publish(particles)
            self.got_map = True

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
    
    def timer_cb(self):
        self.particles_to_image(self.particles)

    def reset_particles_cb(self, request, response):
        if request.use_pose:
            self.init_pose = request.pose
            self.reset_particles_pose()
            self.get_logger().info("Reset particles with pose")
        else:
            self.reset_particles()
            self.get_logger().info("Reset particles")
        response.success = True
        response.msg = "Reset Particles"
        particles = PoseArray()
        particles.header.frame_id = "odom"
        particles.header.stamp = self.get_clock().now().to_msg()
        for p in self.particles:
            particles.poses.append(p["pose"])
        self.pub_part.publish(particles)
        return response


    def reset_one(self):
        self.particles = []
        particle = {}
        particle["pose"] = Pose()
        particle['pose'].position.x = self.location.transform.translation.x
        particle['pose'].position.y = self.location.transform.translation.y
        particle["pose"].orientation = self.location.transform.rotation

        particle["weight"] = 1.0
        # self.get_logger().info(f"particle: {particle}")
        self.particles.append(particle)

    def sensor_update(self, x_coords, y_coords, dist, thetas):
        p_sum = 0
        #choose a subset of laser ranges
        if len(dist) > 27:
            step = len(dist) // 27
            dist = dist[::step]
            thetas = thetas[::step]
        # self.get_logger().info(f"Thetas: {thetas}")
        log_weights = []
        for i, p in enumerate(self.particles):
            new_P = np.log(p["weight"])
            #This may need to be subset of laser rays or log prob since it will approach 0
            
            for j, ray in enumerate(dist):
                p_x_m, p_y_m = self.bresenham(p, thetas[j])
                # if -self.map.info.width+1 >= p_x_m or p_x_m> self.map.info.width-1 or -self.map.info.height+1 >= p_y_m or p_y_m > self.map.info.height-1:
                #     self.get_logger().info(f"Outside bound return [{p_x_m}, {p_y_m}]")
                #     x_g, y_g = self.grid_to_transform(p_x_m, p_y_m)
                #     self.get_logger().info(f"Global coords {x_g}, {y_g}")
                # if self.np_map[p_y_m, p_x_m] < 0:
                #     #Don't know if there is a wall, keep previous probablility Fails due to relative weight to particles that see stuff
                #     # p_ray = 1.0
                #     p_ray = 0.0000000001
                # else:
                p_x_g, p_y_g = self.grid_to_transform(p_x_m, p_y_m)
                p_x = p["pose"].position.x
                p_y = p["pose"].position.y
                q_current = [
                    self.particles[i]["pose"].orientation.x,
                    self.particles[i]["pose"].orientation.y,
                    self.particles[i]["pose"].orientation.z,
                    self.particles[i]["pose"].orientation.w,
                ]
                _, _, old_yaw = tft.euler_from_quaternion(q_current)
                # self.get_logger().info(f"Particle pose[{p_x}, {p_y}, {old_yaw * 180 / np.pi}] | Ray trace [{p_x_g}, {p_y_g}, {thetas[j]* 180 / np.pi}]")
                ray_dist = sqrt((p_x_g - p["pose"].position.x) ** 2 + (p_y_g - p["pose"].position.y)**2)
                p_ray = -0.5 * ((ray_dist-ray) / self.laser_sigma) ** 2
                # if self.np_map[p_x_m, p_y_m]:
                #     self.get_logger().info(f"ray_dist p{i}, r{j}: {ray_dist} | {ray} | {p_ray}")
                # if p_ray <= 0.0000000001:
                #     p_ray = 0.0000000001
                    # self.get_logger().info(f"P ray is zero. ray_dist: {ray_dist}, measured: {ray}")
                new_P = new_P + p_ray
            # non_log_p = np.exp(new_P)
            # if non_log_p < 0.0000000001:
            #     # self.get_logger().info(f"Really Small {i}, {non_log_p}")
            #     non_log_p = 0.0000000001
            # if non_log_p > 1.0:
            #     self.get_logger().info(f'Really large, {non_log_p} | log prob {new_P}')

            # p_sum = p_sum + non_log_p
            # p["weight"] =  non_log_p
            log_weights.append(new_P)
            # weight = p["weight"]
            # self.get_logger().info(f"Current normalizer {p_sum} | prob {non_log_p} | weight {weight}")
        #normalize
        # self.get_logger().info(f"Normalizer: {p_sum}")
        max_log_weight = max(log_weights)
        weights = []
        for w in log_weights:
            weights.append(np.exp(w-max_log_weight))
        p_sum = sum(weights)
        w_max = 0
        p_sum_2 = 0
        for i, p in enumerate(self.particles):
            weight = p["weight"]
            # self.get_logger().info(f"non_norm weight: {weight}")
            p["weight"] = weights[i]/ p_sum
            norm_weight = p["weight"]
            # self.get_logger().info(f"p non norm weight {weight} | norm weight {norm_weight}")
            # if p["weight"] < 0 or p["weight"] > 1:
            #     self.get_logger().info(f"p non norm weight {weight} | norm weight {norm_weight}")
            # if p["weight"] < 0.0000000001:
            #     p["weight"] = 0.0000000001
            if p["weight"] > w_max:
                w_max = p["weight"]
        # self.get_logger().info(f"max_weight {w_max} | norm sum calc v2 = {p_sum_2}")
        
    def not_free(self, map, x, y):
        return map[y,x] > 0

    def bresenham(self, particle, theta):
        """Compute a ray trace.
        This function uses a modified bresenham algorithm created by GeeksforGeeks to ray trace to the first
        non-free space in the map. GeeksforGeeks version is single octant where  this has been generalized. 
        It also only traces until a wall is seen while stopping at a maximum distance of 5 (This should get
        changed to max laser distance). 

        Parameters
        ----------
        particle : dict
            A particle which consists of a pose and weight
        theta : float
            The lasers angle relative to the robots local x-axis. 

        References
        ----------
        [1]GeeksforGeeks, “Bresenham's Line Generation Algorithm,” GeeksforGeeks, Feb. 16, 2017. https://www.geeksforgeeks.org/dsa/bresenhams-line-generation-algorithm/
        """
        #get robot position in map idx
        x_map , y_map = self.transform_to_grid(particle["pose"].position.x, particle["pose"].position.y)
        x_t = particle["pose"].position.x
        y_t = particle["pose"].position.y
        # self.get_logger().info(f"Particle Pose: [{x_map}, {y_map}] | [{x_t}, {y_t}]")
        #Calculate the global angle to ray trace
        p_quat = [
            particle["pose"].orientation.x,
            particle["pose"].orientation.y,
            particle["pose"].orientation.z,
            particle["pose"].orientation.w,
        ]
        _, _, p_euler = tft.euler_from_quaternion(p_quat)

        theta_g = (p_euler + theta)
        # self.get_logger().info(f"theta bresenham: {theta_g *180 / np.pi}, from {p_euler * 180 /np.pi}, {theta * 180 /np.pi}")

        #Create dx and dy, Since this is real slope choose sufficiently far away x2, y2

        
        max_coord_dist = int(5 / self.map.info.resolution)
        # self.get_logger().info(f"indx_distance: {max_coord_dist}")

        x2 = int(round(x_map + max_coord_dist * np.cos(theta_g)))
        y2 = int(round(y_map + max_coord_dist * np.sin(theta_g)))

        x = x_map
        y = y_map

        dir_y = -1 if y2 - y_map < 0 else 1
        dir_x = -1 if x2 - x_map < 0 else 1
        dy = abs(y2-y_map)
        dx = abs(x2 - x_map)
        line_dir = dx > dy
        if not line_dir:
            temp_dx = dx
            dx = dy
            dy = temp_dx
        m_new = 2 * dy
        slope_error_new = m_new - dx
        #Change looping conditions to either map boundry or first non-free cell
        while x > 0.0 and y > 0.0 and x < self.map.info.width-2 and y < self.map.info.height-2:
            if x == x2 and y == y2:
                return x, y
            if line_dir:
                x = x + dir_x
            else:
                y = y + dir_y
            # Slope error reached limit, time to
            # increment y and update slope error.
            if (slope_error_new >= 0):
                if line_dir:
                    y = y + dir_y
                else: 
                    x = x + dir_x
                slope_error_new = slope_error_new - 2 * dx
            if self.not_free(self.np_map, x, y):
                return x, y
            # Add slope to increment angle formed
            slope_error_new = slope_error_new + m_new
        # self.get_logger().info(f"Bound return x: {x} | y: {y}")
        return x, y
    
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
        theta = thetas[indicies]

        return np.array(x_readings[indicies]), np.array(y_readings[indicies]), distance, theta
    
    def norm_w(self):
        sum_weight = 0
        for particle in self.particles:
            sum_weight += particle["weight"]
        for particle in self.particles:
            particle["weight"] /= sum_weight

    def importance_sample(self):
        new_particles = []
        weight_sum = np.zeros(len(self.particles))
        weight_sum[0] = self.particles[0]["weight"]
        for i in range(len(self.particles)-1):
            weight_sum[i+1] = weight_sum[i] + self.particles[i+1]["weight"]

        for i in range(len(self.particles)):
            # self.get_logger().info(f"Weight sum = {weight_sum[-1]}")
            rand = np.random.uniform(0, weight_sum[-1])
            new_particle_idx = np.where(rand <= weight_sum)[0][0]
            new_particles.append(copy.deepcopy(self.particles[new_particle_idx]))
        self.particles = new_particles
    
    def move_update(self, dist, yaw):
        max_x = ((self.map.info.width-1) * self.map.info.resolution) + self.map.info.origin.position.x
        min_x = self.map.info.origin.position.x
        max_y = ((self.map.info.height-1) * self.map.info.resolution) + self.map.info.origin.position.y
        min_y = self.map.info.origin.position.y
        # self.get_logger().info(f"min and max {max_x}, {min_x}, {max_y}, {min_y}")
        for i in range(len(self.particles)):
            #Update rotation
            theta = yaw + np.random.normal(self.rot_u, self.rot_sigma)
            q_add= tft.quaternion_from_euler(0, 0, theta)
            q_current = [
                self.particles[i]["pose"].orientation.x,
                self.particles[i]["pose"].orientation.y,
                self.particles[i]["pose"].orientation.z,
                self.particles[i]["pose"].orientation.w,
            ]
            
            q_new = tft.quaternion_multiply(q_current, q_add)
            _, _, old_yaw = tft.euler_from_quaternion(q_current)
            _, _, new_yaw = tft.euler_from_quaternion(q_new)
            self.particles[i]["pose"].orientation.x = q_new[0]
            self.particles[i]["pose"].orientation.y = q_new[1]
            self.particles[i]["pose"].orientation.z = q_new[2]
            self.particles[i]["pose"].orientation.w = q_new[3]
            # self.get_logger().info(f"old yaw: {old_yaw}, New yaw = {new_yaw}")
            #update translation
            #only need x, y translation
            #probably more realistic to use distance and rotation instead of x, y, theta
            dist_moved = dist + np.random.normal(self.move_u, self.move_sigma)
            dx = dist_moved * np.cos(new_yaw)
            # if i == 0:
            #     x = self.particles[i]["pose"].position.x
            #     y = self.particles[i]["pose"].position.y
            #     self.get_logger().info(f"current pose: [{x}, {y}]")
            x_temp = self.particles[i]["pose"].position.x + dist_moved * np.cos(new_yaw)
            dy = dist_moved * np.sin(new_yaw)
            y_temp = self.particles[i]["pose"].position.y + dist_moved * np.sin(new_yaw)
            # if i == 0:
                # self.get_logger().info(f"yaw: {new_yaw} dist moved: {dist_moved} dx: {dx} dy: {dy} new_x: {x_temp} new_y: {y_temp}")
            # x_map, y_map = self.transform_to_grid(x_temp, y_temp)
            # self.get_logger().info(f"new map coord: [{x_map}, {y_map}]")

            if x_temp > max_x:
                x_temp = max_x
            elif x_temp < min_x:
                x_temp = min_x
            if y_temp > max_y:
                y_temp = max_y
            elif y_temp < min_y:
                y_temp = min_y
            # x_global, y_global = self.grid_to_transform(x_map, y_map)
            # self.get_logger().info(f"new global coord: [{x_global}, {y_global}]")
            # if i == 0:
            #     self.get_logger().info(f"Updated new_x: {x_temp} new_y: {y_temp}")
            self.particles[i]["pose"].position.x = x_temp
            self.particles[i]["pose"].position.y = y_temp

    def move_diff(self):
        move_diff = TransformStamped()
        move_diff.header = self.location.header
        #translation diff
        move_diff.transform.translation.x = self.move_location.transform.translation.x - self.location.transform.translation.x
        move_diff.transform.translation.y = self.move_location.transform.translation.y - self.location.transform.translation.y
        move_diff.transform.translation.z = 0.0 #Should be 0
        dist = sqrt(move_diff.transform.translation.x ** 2 + move_diff.transform.translation.y ** 2)
        #rotation diff
        q_loc = [
            self.location.transform.rotation.x,
            self.location.transform.rotation.y,
            self.location.transform.rotation.z,
            self.location.transform.rotation.w,
        ]
        q_move = [
            self.move_location.transform.rotation.x,
            self.move_location.transform.rotation.y,
            self.move_location.transform.rotation.z,
            self.move_location.transform.rotation.w,
        ]
        loc_inv = tft.quaternion_inverse(q_loc)
        q_diff = tft.quaternion_multiply(loc_inv, q_move)
        q_diff = q_diff / np.linalg.norm(q_diff)
        move_diff.transform.rotation.x = q_diff[0]
        move_diff.transform.rotation.y = q_diff[1]
        move_diff.transform.rotation.z = q_diff[2]
        move_diff.transform.rotation.w = q_diff[3]
        #in 2d movements axis is z
        yaw = atan2(
                2.0 * (move_diff.transform.rotation.w * move_diff.transform.rotation.z + move_diff.transform.rotation.x * move_diff.transform.rotation.y),
                1.0 - 2.0 * (move_diff.transform.rotation.y * move_diff.transform.rotation.y + move_diff.transform.rotation.z * move_diff.transform.rotation.z)
        )
        return move_diff, dist, yaw


    def is_free(self, map, x, y):
        # self.get_logger().info(f"x: {x}, y: {y}")
        return map[y, x] == 0
    
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
        # self.get_logger().info(f"global in transform: [{x_global}, {y_global}]")
        x_map = int((x_global - self.map.info.origin.position.x) / self.map.info.resolution)
        y_map = int((y_global - self.map.info.origin.position.y) / self.map.info.resolution)

        # if not self.use_odom:
        #     self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        # robot_x = int((self.location.transform.translation.x - self.map.info.origin.position.x) / self.map.info.resolution)
        # robot_y = int((self.location.transform.translation.y - self.map.info.origin.position.y) /self.map.info.resolution)

        return x_map, y_map#, robot_x, robot_y
    
    def reset_particles_pose(self):
        """

        Notes
        -----
        Particle Structure = 
            - "Pose" : Pose()
            - "weight" : float
        """
        self.particles = []
        weight = 1/self.num_particles
        max_x = ((self.map.info.width-1) * self.map.info.resolution) + self.map.info.origin.position.x
        min_x = self.map.info.origin.position.x
        max_y = ((self.map.info.height-1) * self.map.info.resolution) + self.map.info.origin.position.y
        min_y = self.map.info.origin.position.y
        for i in range(self.num_particles):
            made_particle = False
            attemps = 0
            particle = {}
            particle["pose"] = Pose()
            while not made_particle:
                #Note: in Map Coords
                # x = np.random.randint(0, self.map.info.width)
                # y = np.random.randint(0, self.map.info.height)
                x_global = np.random.normal(self.init_pose.position.x, self.init_sigma)
                y_global = np.random.normal(self.init_pose.position.y, self.init_sigma)
                if x_global > max_x:
                    x_global = max_x
                elif x_global < min_x:
                    x_global = min_x
                if y_global > max_y:
                    y_global = max_y
                elif y_global < min_y:
                    y_global = min_y
                x, y = self.transform_to_grid(x_global, y_global)

                # x_global, y_global = self.grid_to_transform(x, y)
                particle['pose'].position.x = x_global
                particle['pose'].position.y = y_global
                theta = np.random.uniform(-np.pi, np.pi)
                qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, theta)
                particle["pose"].orientation.x = qx
                particle["pose"].orientation.y = qy
                particle["pose"].orientation.z = qz
                particle["pose"].orientation.w = qw
                particle["weight"] = weight

                if self.is_free(self.np_map, x, y):
                    self.particles.append(particle)
                    made_particle = True
                elif attemps > 10:
                    self.particles.append(particle)
                    made_particle = True
                attemps += 1
                # x1 = self.particles[i]["pose"].position.x
                # y1 = self.particles[i]["pose"].position.y
            # x_map, y_map = self.transform_to_grid(x1, y1)
            # self.get_logger().info(f"Initial map coords {i} | {x} , {y}")
            # self.get_logger().info(f"particle map {i} | [{x_map}, {y_map}]")
            # self.get_logger().info(f"particle {i} | [{x1}, {y1}]")
 
    def reset_particles(self):
        """

        Notes
        -----
        Particle Structure = 
            - "Ppse" : Pose()
            - "weight" : float
        """
        self.particles = []
        weight = 1/self.num_particles
        max_x = ((self.map.info.width-1) * self.map.info.resolution) + self.map.info.origin.position.x
        min_x = self.map.info.origin.position.x
        max_y = ((self.map.info.height-1) * self.map.info.resolution) + self.map.info.origin.position.y
        min_y = self.map.info.origin.position.y
        for i in range(self.num_particles):
            made_particle = False
            attemps = 0
            particle = {}
            particle["pose"] = Pose()
            while not made_particle:
                #Note: in Map Coords
                x = np.random.randint(0, self.map.info.width)
                y = np.random.randint(0, self.map.info.height)
            
                x_global, y_global = self.grid_to_transform(x, y)
                particle['pose'].position.x = x_global
                particle['pose'].position.y = y_global
                theta = np.random.uniform(-np.pi, np.pi)
                qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, theta)
                particle["pose"].orientation.x = qx
                particle["pose"].orientation.y = qy
                particle["pose"].orientation.z = qz
                particle["pose"].orientation.w = qw
                particle["weight"] = weight
                if self.is_free(self.np_map, x, y):
                    self.particles.append(particle)
                    made_particle = True
                elif attemps > 10:
                    self.particles.append(particle)
                    made_particle = True
                attemps += 1
                # x1 = self.particles[i]["pose"].position.x
                # y1 = self.particles[i]["pose"].position.y
            # x_map, y_map = self.transform_to_grid(x1, y1)
            # self.get_logger().info(f"Initial map coords {i} | {x} , {y}")
            # self.get_logger().info(f"particle map {i} | [{x_map}, {y_map}]")
            # self.get_logger().info(f"particle {i} | [{x1}, {y1}]")

    def particles_to_image(self, particles, name = ""):
        #Get map in 2d Array
        temp_map = np.array(self.map.data, dtype=np.int8)
        temp_map = temp_map.reshape(self.map.info.height,
                             self.map.info.width)
        # temp_map = np.flipud(temp_map)
        thresh_map = (temp_map > 30).astype(np.uint8)

        #determine Seen vs unseen
        unknown_mask = np.array(thresh_map < 0)
        known_mask = ~unknown_mask
        #Calc Grey Values
        grey_val = (1.0 - thresh_map[known_mask]) * 255.0
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
            # self.get_logger().info(f"particle image {p}")
            if p["weight"] > best_weight:
                x_map, y_map = self.transform_to_grid(particles[i]["pose"].position.x, particles[i]["pose"].position.y)
                best_x = x_map
                best_y = y_map
        img[best_y, best_x] = np.array([0, 255, 0], dtype=np.uint8)

        img = np.flipud(img)
        #Save
        time = self.get_clock().now().to_msg()
        cv.imwrite(os.path.join(self.save_dir, f'Particles_{name}_{time.sec}.png'), img)

    def plot_error_cb(self, request, response):
        plt.figure()
        plt.plot(self.time_steps, self.error)
        plt.xlabel("Time (s)")
        plt.ylabel("Error")
        plt.grid(True)
        # plt.show()
        time = self.get_clock().now().to_msg()
        plt.savefig(os.path.join(self.save_dir, f'Error_Plot_{time.sec}.png'), dpi=300, bbox_inches="tight")
        plt.close()
        
        response.success = True
        response.message = "Made plot"
        return response

    

def main(args=None):
    rclpy.init(args=args)

    localization = Localization()

    rclpy.spin(localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()