"""
Driver Node
ROB599 Mobile Robotics
Author: Jared Northrop
Year: 2526

This node implements a simple proportional controller and an implemention of the dynamic window approach 
(DWA). 
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle

from geometry_msgs.msg import Twist, PointStamped, TwistStamped, TransformStamped

from nav_interface.action import NavGoal

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import time
import numpy as np
from math import atan2, tanh, sqrt, pi, fabs, cos, sin, asin

from rclpy.executors import MultiThreadedExecutor

from visualization_msgs.msg import Marker

from rclpy.qos import QoSProfile, ReliabilityPolicy

class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        #set qos for turtle bot
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.declare_parameter('is_dwa', True)
        self.is_dwa = self.get_parameter('is_dwa').value
        if self.is_dwa:
            self.get_logger().info("Running with DWA Obstacle Avoidance")
        else:
            self.get_logger().info("Running with Simple Controller")
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)


        self.declare_parameter('use_twist_stamped', False)
        self.use_twist_stamped = self.get_parameter('use_twist_stamped').value

        if self.use_twist_stamped:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.marker_pub = self.create_publisher(Marker, "goal_marker", 10)

        timer_period = 0.033  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.marker_timer = self.create_timer(timer_period, self.marker_cb)
        self.get_goal = ActionServer(
            node=self, 
            action_type=NavGoal, 
            action_name='nav_goal', 
            goal_callback=self.accept_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            execute_callback=self.waypoint_callback
        )

        self.get_dwa_goal = ActionServer(
            node = self,
            action_type=NavGoal, 
            action_name='nav_dwa_goal', 
            goal_callback=self.accept_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            execute_callback=self.waypoint_dwa_callback
        )
        if self.use_twist_stamped:
            self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_cb, qos)
            # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        else:
            self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)
            # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)


        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)

        #Goal Params
        self.goal = None
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.distance_threshold = 0.2
        self.min_distance = np.inf
        self.num_iter = 0

        self.target = PointStamped()
        self.target.point.x = 0.0
        self.target.point.y = 0.0
        self.target_marker = None

        #DWA Params
        #for Simulation Best results were [0.04, 0.04, 0.1, 0.1, 0.16, 0.2 0.12, 0.1, 5.0, 40, 50]
        #For Real Robot Best Restuls [0.02, 0.02, 0.1, 0.1, 0.16, 0.2, 0.12, 0.1, 5.0, 40, 50]
        self.delta_v = 0.04 #0.04
        self.delta_w = 0.04 #0.04
        self.dt = 0.1 #0.1
        self.sampling_res = 0.1 #0.1
        self.heading_w = 0.16 #0.16
        self.velocity_w = 0.2 #0.2
        self.clearance_w = 0.12 #0.12
        self.dist_to_obj_margin = 0.1 #0.1
        self.max_obj_dist = 5.0 # 5.0
        self.steps = 40 #40
        self.max_dist_const = 50 #50
        self.min_obj_dist = 0.00


        #robot params
        #sim [0.5, 1.0, 0.0, 2.0, 0.75, 0.0, 0.0, 0.3]
        #sim [0.15, 0.5, 0.0, 0.7, 0.2, 0.0, 0.0, 0.2]

        self.max_v = 0.5
        self.max_v_dot = 1.0
        self.min_v = 0.0
        self.max_w_dot = 02.0
        self.max_w = 0.75
        self.v = 0.0
        self.w = 0.0
        self.robot_r = 0.2
        self.location = TransformStamped()

        #stupid params
        self.last_v = 0.0
        self.last_w = 0.0
        # self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))

    def timer_callback(self):
        """Not Used
        """

        if self.goal and not self.is_dwa:
            # if not self.use_twist_stamped:
            #     self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            msg = self.get_twist()
            if self.use_twist_stamped:
                msg2 = TwistStamped()
                msg2.twist = msg
                msg2.header.frame_id = 'base_link'
                msg2.header.stamp = self.get_clock().now().to_msg()
            else:
                msg2 = msg
            self.cmd_vel_pub.publish(msg2)

    def marker_cb(self):
        #remove old Markers
        if not self.goal:
            if self.target_marker:
                self.target_marker.action = Marker.DELETE
                self.marker_pub.publish(self.target_marker)
                self.target_marker = None
            return
        #create marker
        if not self.target_marker:
            self.target_marker = Marker()
            self.target_marker.header.frame_id = self.goal.header.frame_id
            self.target_marker.id = 0
        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.position = self.goal.point
        self.target_marker.scale.x = 0.2
        self.target_marker.scale.y = 0.2
        self.target_marker.scale.z = 0.2
        self.target_marker.color.r = 0.0
        self.target_marker.color.g = 1.0
        self.target_marker.color.b = 0.0
        self.target_marker.color.a = 1.0

        self.marker_pub.publish(self.target_marker)

        self.marker_timer.cancel()
    
    def get_twist(self):
        """Find the a twist to move towards the goal with proportional speed. 

        This function uses a tanh function to proporationally control the speed as the robot approaches the 
        goal. 
        """
        t = self.zero_twist()

        t.linear.x = self.max_v * tanh(1 * self.distance_to_goal)
        t.angular.z = self.max_v * tanh(10 * self.angle_to_goal)
        return t
    
    def laser_cb(self, scan):
        if self.goal and self.is_dwa:
            if self.close_enough():
                if self.use_twist_stamped:
                    self.cmd_vel_pub.publish(self.zero_twist_stamped())
                else:
                    self.cmd_vel_pub.publish(self.zero_twist())
                return
            obstacles, min_dist = self.get_obstacles(scan)
            # self.get_logger().info(f"obstacles: {obstacles}")
            if not self.use_twist_stamped:
                try:
                    self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
                except Exception as e:
                    self.get_logger().error(f"Transform failed to process: {e}")

            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            best_pair = self.dwa2(obstacles)
            # self.get_logger().info(f"Angle to Goal: {self.angle_to_goal}")
            self.get_logger().info(f"Cmd Twist: {best_pair}")
            msg = self.zero_twist()
            msg.linear.x = best_pair[0]
            msg.angular.z = best_pair[1]
            if self.use_twist_stamped:
                msg2 = TwistStamped()
                msg2.twist = msg
                msg2.header.frame_id = 'base_link'
                msg2.header.stamp = self.get_clock().now().to_msg()
            else:
                msg2 = msg
            self.cmd_vel_pub.publish(msg2)

            self.last_v = best_pair[0]
            self.last_w = best_pair[1]
    
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

    def get_goal_in_base_link(self):
        self.target = do_transform_point(self.goal, self.location)
        #Calculate the rotation
        euler_ang = -atan2(2 * self.location.transform.rotation.z * self.location.transform.rotation.w,
                            1.0 - 2 * self.location.transform.rotation.z * self.location.transform.rotation.z)
        
        # Translate to the base link's origin
        x = self.goal.point.x - self.location.transform.translation.x
        y = self.goal.point.y - self.location.transform.translation.y

        # Do the rotation
        rot_x = x * cos(euler_ang) - y * sin(euler_ang)
        rot_y = x * sin(euler_ang) + y * cos(euler_ang)

        self.target.point.x = rot_x
        self.target.point.y = rot_y
    
    def waypoint_callback(self, goal_handle):
        self.get_logger().info(f"Recieved goal: {goal_handle.request.goal.point}")
        # Store the goal
        self.goal = PointStamped()
        self.goal.header = goal_handle.request.goal.header
        self.goal.point = goal_handle.request.goal.point
        #Create response
        # if not self.use_twist_stamped:
        #     self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        self.get_goal_in_base_link()
        self.get_distance_to_goal()
        result = NavGoal.Result()
        result.success = False

        self.get_logger().info(f"Distance to goal: {self.distance_to_goal}, target: {self.target.point}")

        self.get_logger().info(f"Current Position: {self.location.transform.translation}")

        msg = Twist()
        while not self.close_enough():
            self.get_logger().info("In While")
            # try:
            #     self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
            # except Exception:
            #     self.get_logger().error("Failed to transform")
            # self.get_goal_in_base_link()
            # self.get_distance_to_goal()
            # msg = self.get_twist()
            # if self.use_twist_stamped:
            #     msg2 = TwistStamped()
            #     msg2.twist = msg
            #     msg2.header.frame_id = 'base_link'
            #     msg2.header.stamp = self.get_clock().now().to_msg()
            # else:
            #     msg2 = msg
            # self.cmd_vel_pub.publish(msg)
            feedback = NavGoal.Feedback()
            self.get_logger().info(f"distance to goal : {self.distance_to_goal}")
            feedback.distance.data = self.distance_to_goal

            goal_handle.publish_feedback(feedback)
            time.sleep(0.03)

        self.goal = None
        if self.use_twist_stamped:
            t = self.zero_twist_stamped()
        else:
            t = self.zero_twist()
        self.cmd_vel_pub.publish(t)

        self.get_logger().info(f"Completed goal")

        # Set the succeed value on the handle
        goal_handle.succeed()

        # Set the result to True and return
        result.success = True
        return result
    
    def waypoint_dwa_callback(self, goal_handle):
        self.get_logger().info(f"Recieved goal: {goal_handle.request.goal.point}")
        # Store the goal
        self.goal = PointStamped()
        self.goal.header = goal_handle.request.goal.header
        self.goal.point = goal_handle.request.goal.point
        #Create response
        if not self.use_twist_stamped:
            self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        self.get_goal_in_base_link()
        self.get_distance_to_goal()
        result = NavGoal.Result()
        result.success = False

        self.get_logger().info(f"Distance to goal: {self.distance_to_goal}, target: {self.target.point}")
        self.get_logger().info(f"Angle to Goal: {self.angle_to_goal}")

        self.get_logger().info(f"Current Position: {self.location.transform.translation}")

        msg = Twist()
        msg2 = TwistStamped()
        while not self.close_enough():
            feedback = NavGoal.Feedback()
            # self.get_logger().info(f"distance to goal : {self.distance_to_goal}")
            feedback.distance.data = self.distance_to_goal

            goal_handle.publish_feedback(feedback)
            time.sleep(0.03)

        self.goal = None
        self.marker_timer.reset()
        if self.use_twist_stamped:
            t = self.zero_twist_stamped()
        else:
            t = self.zero_twist()
        self.cmd_vel_pub.publish(t)
        self.last_v = 0.0
        self.last_w = 0.0

        self.get_logger().info(f"Completed goal")

        # Set the succeed value on the handle
        goal_handle.succeed()

        # Set the result to True and return
        result.success = True
        return result
    
    def accept_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        self.marker_timer.reset()
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.marker_timer.reset()
        self.goal = None
        return CancelResponse.ACCEPT
    
    def zero_twist(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg
    def zero_twist_stamped(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        return msg

    def close_enough(self):
        self.get_distance_to_goal()
        if self.distance_to_goal < self.distance_threshold:
            self.min_distance = np.inf
            return True
        elif self.num_iter > 200:
            self.num_iter = 0
            self.min_distance = np.inf
            self.get_logger().info("Reached goal by not getting closer")
            return True
        return False
    
    def get_distance_to_goal(self):
        self.angle_to_goal = atan2(self.target.point.y, self.target.point.x)
        self.distance_to_goal = sqrt(self.target.point.x ** 2 + self.target.point.y ** 2)
    
    def get_obstacles(self, scan):
        #Get the angle range			
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        max_range = scan.range_max
        num_readings = len(scan.ranges)
        min_distance = min(scan.ranges)
        thetas = np.linspace(angle_min, angle_max, num_readings)
        ranges = np.array(scan.ranges)

        #Decompose scan into x and y coords using sin, cos
        y_readings = scan.ranges * np.sin(thetas)
        x_readings = scan.ranges * np.cos(thetas)

        #only keep points close enough (and eliminate points at the max range of the laser)
        max_range = min(max_range, self.max_obj_dist)
        indicies = np.where((ranges < max_range - 0.0000001) & (ranges > self.min_obj_dist))

        return list(zip(x_readings[indicies], y_readings[indicies])), min_distance

    def dwa(self, obstacles):
        """Dynamic Window Approach for obstacle avoidance. 
        This function assumes goals and obstacles are provided in the robots frame. 

        Notes
        -----
        This function was the first attempt at DWA and does not function correctly.
        """
        #Possibly cap max speed with distance to target
        t = self.get_twist()
        #only consider a v and w pair (circular trajectory pruning)
        #dynamic Window Pruning
        max_v = min(t.linear.x, self.max_v)
        # max_v = self.max_v
        self.v = self.last_v
        self.w = self.last_w
        # self.get_logger().info(f"current velocity: {self.v}, capped max:  {max_v}")
        possible_v = [max(self.v - self.max_v_dot * self.dt, self.min_v), min(self.v + self.max_v_dot * self.dt, max_v)]
        possible_w = [max(self.w - self.max_w_dot * self.dt, -1 * self.max_w), min(self.w + self.max_w_dot * self.dt, self.max_w)]
        # self.get_logger().info(f"Intervals: {possible_v}, {possible_w}")
        #Admissible Velocity Pruning and obstacle score
        possible_pairs = self.admissable_velocities(possible_v, possible_w, obstacles)
        # self.get_logger().info(f"Number of Possible Pairs: {len(possible_pairs)}")

        #evaluate pairs
        best_pair = self.eval_pairs(possible_pairs)
        return best_pair

    def eval_pairs(self, possible_pairs):
        """Evaluate the possible u, w pairs.
        Notes
        -----
        This function was used in the first attempt at DWA and does not function correctly.
        """
        best_score = 0
        best_pair = []
        v_score = []
        h_score = []
        o_score = []
        # self.get_logger().info(f"Possible pairs: {possible_pairs}")
        for pair in possible_pairs:
            #evaluate heading
            # theta = pair["w"] * self.dt 
            # h_score.append(pi-abs(self.angle_to_goal - theta))
            h_score.append(pair["head"])
            v_score.append(pair["v"]+ 0.1 * abs(pair["w"])) #
            o_score.append(pair["dist"])
        # self.get_logger().info(f"Heading: {h_score}")
        # self.get_logger().info(f"Velocity Score: {v_score}")
        # self.get_logger().info(f"Clearance Score: {o_score}")
        if v_score == []:
            return [0.0, 0.0]
        v_score = self.normalize_score(v_score)
        h_score = self.normalize_score(h_score)
        o_score = self.normalize_score(o_score)
        # self.get_logger().info(f"Heading: {h_score}")
        # self.get_logger().info(f"Velocity Score: {v_score}")
        # self.get_logger().info(f"Clearance Score: {o_score}")
        checker = []
        for i in range(len(v_score)):
            score = self.heading_w * h_score[i] + self.velocity_w * v_score[i] + self.clearance_w * o_score[i]
            checker.append(score)
            if score > best_score:
                best_score = score
                best_pair = [possible_pairs[i]["v"], possible_pairs[i]["w"]] 
        print(f"scores {checker}")
        # self.get_logger().info(f"Best Pair: {best_pair}")
        return best_pair
    
    def normalize_score(self, values):
        """Min-max normalization

        Parameters
        ----------
        values : list
            A list of value to normalize

        Returns 
        -------
        values : list
            A list of normalized values on the interval [0, 1]
        """
        values = np.array(values)
        if values.max() - values.min() == 0:
            values = [0.0 for i in range(len(values))]
        else:
            values = (values - values.min())/(values.max() - values.min())
        return values

    def admissable_velocities(self, possible_v, possible_w, obstacles):
        """Determine the Admissable Pairs
        Notes
        -----
        This function was used in the first attempt at DWA and does not function correctly.
        """
        possible_pairs = []
        number_checked = 0
        for w in np.arange(possible_w[0], possible_w[1], self.delta_w):
            for v in np.arange(possible_v[0], possible_v[1], self.delta_v):
                #Evaluate collisions based on stopping distance (Doesn't seem to be useful gets stuck easily)
                # min_stopping_dist = max(v ** 2 / 2 / self.max_v_dot, w ** 2 / 2 / self.max_w_dot)
                # #Determine points along the path to evaluate
                # if w != 0 and v !=0: #Normal Case
                #     radius = v/w
                #     theta_max = min_stopping_dist/radius
                #     num_samples = int(min_stopping_dist/self.sampling_res)
                # elif w != 0 and v == 0: #Only w case
                #     radius = 0
                #     theta_max = 0
                #     num_samples = 1
                # else: #only v case
                #     radius = 0
                #     theta_max = min_stopping_dist
                #     num_samples = int(min_stopping_dist/self.sampling_res)
                # if num_samples == 0:
                #     num_samples = 1
                # d_theta = theta_max/num_samples
                # min_dist = self.max_obj_dist
                # is_admissable = True
                # for i in range(num_samples):
                #     #Calculate the x y of the point
                #     if w != 0 and v != 0:
                #         point = [radius * ((i + 1) * d_theta), radius * (i + 1) * d_theta]
                #     elif w!=0 and v==0:
                #         point = [0, 0]
                #     else:
                #         point = [(i+1) * d_theta, 0]
                #     #Check collision With obstacles
                #     if obstacles == []:
                #         min_dist = self.max_obj_dist
                #     else:
                #         for obj in obstacles:
                #             dist = sqrt((point[0] - obj[0]) ** 2 + (point[1] - obj[1]) ** 2)
                #             if dist < self.robot_r + self.dist_to_obj_margin:
                #                 is_admissable = False
                #                 # self.get_logger().info(f"Point not Admissable")
                #                 break
                #             elif dist < min_dist:
                #                 min_dist = dist
                
                #calculate the next n steps
                min_dist = self.max_obj_dist
                is_admissable = True
                x = 0
                y = 0
                theta = 0
                for i in range(self.pre_step):
                    x = v * cos(theta) * self.dt + x
                    y = v * sin(theta) * self.dt + y
                    theta = w * self.dt + theta
                    #check for collisions
                    for obj in obstacles:
                        dist = sqrt((x - obj[0]) ** 2 + (y - obj[1]) ** 2)
                        if dist < self.robot_r + self.dist_to_obj_margin:
                            is_admissable = False
                            break
                        elif dist < min_dist:
                            min_dist = dist
                    if not is_admissable:
                        break
                if is_admissable:
                    h_score = pi-abs(self.angle_to_goal - theta)
                    possible_pairs.append({"v": v, "w": w, "dist" : min_dist, "head": h_score})
                number_checked += 1
        # self.get_logger().info(f"Number of pairs Checked: {number_checked}")
        return possible_pairs
    
    def dwa2(self, obstacles):
        """Dynamic window approach algorithm
        
        Parameters
        ----------
        obstacles : list
            A list of tuples containing x, y points for obstacles.
        
        Returns
        -------
        best_pair : list
            A list containing the best u and w pair.

        References
        ----------
        [1] robot mania, “Dynamic Window Approach Tutorial,” YouTube, Oct. 15, 2020. 
        https://www.youtube.com/watch?v=tNtUgMBCh2g (accessed Jan. 25, 2026).

        Notes
        -----
        Reference [1] was used as a guidline which mainly includes checking paths by determining the next n 
        steps.
        """
        #Create Window
        #Possibly cap max speed with distance to target
        t = self.get_twist()
        #only consider a v and w pair (circular trajectory pruning)
        #dynamic Window Pruning
        max_v = min(t.linear.x, self.max_v)
        self.v = self.last_v
        self.w = self.last_w
        range_w = self.dt * self.max_w_dot
        w_interval = [max(self.w - range_w, -1 * self.max_w), min(self.w + range_w, self.max_w)]
        range_v = self.dt * self.max_v_dot
        v_interval = [max(self.v - range_v, self.min_v), min(self.v + range_v, max_v)]
        #Create Paths to Check Admissability
        pairs = []
        for w in np.arange(w_interval[0], w_interval[1], self.delta_w):
            for v in np.arange(v_interval[0], v_interval[1], self.delta_v):
                x_path = []
                y_path = []
                theta_path = []
                x = 0
                y = 0
                theta = 0
                for i in range(self.steps):
                    x += v * cos(theta) * self.dt
                    y += v * sin(theta) * self.dt
                    theta += w * self.dt
                    x_path.append(x)
                    y_path.append(y)
                    theta_path.append(theta)
                pairs.append({"v": v, "w": w, "x": x_path, "y": y_path, "theta": theta_path})

        #determine Admissable Paths and score
        h_score = []
        v_score = []
        o_score = []
        index = []
        for i, pair in enumerate(pairs):
            temp_o = self.get_obstacle_score2(pair, obstacles)
            if temp_o >= 0:
                o_score.append(temp_o ** 2)
                index.append(i)
                temp_h = self.get_heading_score(pair)
                #incentivise turning if not moving 
                if self.v == 0:
                    temp_v = pair["v"] + 0.1 * abs(pair["w"])
                    self.get_logger().info("No translation")
                else:
                    temp_v = pair["v"]
                # temp_v = pair["v"]
                h_score.append(temp_h)
                v_score.append(temp_v)
        if index == []:
            self.get_logger().info("No Admissable Paths")
            return [0.0, 0.0]
        h_score = self.normalize_score(h_score)
        v_score = self.normalize_score(v_score)
        o_score = self.normalize_score(o_score)
        score = []
        best_score = -1
        best_pair = []

        for i in range(len(v_score)):
            score.append( self.velocity_w * v_score[i] + self.heading_w * h_score[i] + self.clearance_w * o_score[i])
            if score[-1] > best_score:
                best_score = score[-1]
                best_pair = [pairs[index[i]]["v"], pairs[index[i]]["w"]]
        return best_pair
        
    def get_heading_score(self, pair):
        """Determine the heading 
        
        Parameters
        ----------
        pair : dictionary
            A dictionary containing 'x', 'y', and 'theta' paths for the next n steps
        
        Returns
        -------
        heading_score : float
            The heading score calculated with the difference between the predicted heading and goal heading.
        """
        angle_to_goal = atan2(self.target.point.y - pair["y"][-1], self.target.point.x - pair["x"][-1])
        relative_heading = abs(self.bound_angle(angle_to_goal - pair["theta"][-1]))
        return cos(relative_heading)

            
    def bound_angle(self, angle):
        """Returns an angle between [-pi, pi]

        Parameters
        ----------
        angle : float
            An angle in radians. 
        """
        return (angle + pi) % (2 * pi) - pi
    
    def get_obstacle_score(self, pair, obstacles):
        """This function evaluates the pairs clearance/obstacle score

        Parameters
        ----------
        pair : dictionary
            A dictionary containing the 'x', 'y', 'theta' paths for the next n steps.
        obstacles : list
            A list of tuples containing x, y locations of obstacles. 
        
        Returns
        -------
        max_score : float
            The clearance/obstacle score. '-1' means not admissable.

        """
        max_score = self.max_obj_dist #max score set to  max object detection distance
        dist_to_obj = 0.0
        not_admissable = False
        for i in range(len(pair["x"])):
            for obj in obstacles:
                dist_to_obj = sqrt(((pair["x"][i] - obj[0]) ** 2) + ((pair["y"][i] - obj[1]) ** 2))
                if dist_to_obj < self.robot_r + self.dist_to_obj_margin:
                    max_score = -1 #This technically keeps the path which should be discarded
                    not_admissable = True
                    break
                elif dist_to_obj < max_score:
                    max_score = dist_to_obj
            if not_admissable:
                break
        return max_score
    
    def get_obstacle_score2(self, pair, obstacles):
        """Determine the clearance/obstacle score.
        This function first determines whether or not the given pair is admissable then assigns a score based
        on the path length to collision. Admissability is determined based on the minimum stopping distance 
        given a constant velocity. 

        Parameters
        ----------
        pair : dictionary
            A dictionary containing 'x', 'y', and 'theta' paths for the next n steps and the 'u' and 'w' pair
        obstacles : list
            A list of tuples containing the x, y locations of obstacles. 
        
        Returns
        -------
        max_score : float
            The obstacle or clearance score. '-1' means not admissable.
        """
        max_score = self.max_dist_const
        dist_to_obj = 0.0
        #find the minimum stopping distance at the current speed
        min_stoping_dist = pair["v"] ** 2 / 2 / self.max_v_dot
        #if only turning then return a high score
        if pair["v"] == 0:
            return max_score
        for i in range(len(pair["x"])):
            for obj in obstacles:
                dist_to_obj = sqrt(((pair["x"][i] - obj[0]) ** 2) + ((pair["y"][i] - obj[1]) ** 2))
                #check if we collide
                if dist_to_obj < self.robot_r + self.dist_to_obj_margin:
                    #if no turning then driving in a straight line
                    if pair["w"] == 0:
                        path_distance = pair["x"][i]
                        #not admissable if you can't stop
                        if path_distance < min_stoping_dist + self.dist_to_obj_margin:
                            return -1
                        else:
                            max_score = path_distance
                            return max_score
                    #Determine path length and check if stopping distance
                    else:
                        r = pair["v"]/pair["w"]
                        chord = sqrt((pair["x"][i] ** 2) + (pair["y"][i] ** 2))
                        ratio = chord / (2 * r)
                        ratio = min(max(ratio, -1.0), 1.0) 
                        theta = 2 * asin(ratio)
                        path_distance = r * theta
                        #not admissable if you can't stop
                        if path_distance < min_stoping_dist + self.dist_to_obj_margin:
                            return -1
                        else:
                            max_score = path_distance
                            return max_score
        return max_score

def main(args=None):
    rclpy.init(args=args)
    driver_node = Driver()
    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()