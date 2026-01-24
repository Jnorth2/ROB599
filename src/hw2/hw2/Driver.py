import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle



from geometry_msgs.msg import Twist, PointStamped

from nav_interface.action import NavGoal

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import time
import numpy as np
from math import atan2, tanh, sqrt, pi, fabs, cos, sin

from rclpy.executors import MultiThreadedExecutor



class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        self.declare_parameter('is_dwa', False)
        self.is_dwa = self.get_parameter('is_dwa').value
        if self.is_dwa:
            self.get_logger().info("Running with DWA Obstacle Avoidance")
        else:
            self.get_logger().info("Running with Simple Controller")

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.033  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
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

        self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # Create a buffer to put the transform data in
        self.tf_buffer = Buffer()
        
		# This sets up a listener for all of the transform types created
        self.transform_listener = TransformListener(self.tf_buffer, self)


        #Goal Params
        self.goal = None
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.distance_threshold = 0.1
        self.min_distance = np.inf
        self.num_iter = 0

        self.target = PointStamped()
        self.target.point.x = 0.0
        self.target.point.y = 0.0

        #DWA Params
        self.delta_v = 0.02
        self.delta_w = 0.02
        self.dt = 0.1
        self.sampling_res = 0.1
        self.heading_w = 2.0
        self.velocity_w = 0.2
        self.clearance_w = 0.2
        self.dist_to_obj_margin = 0.05
        self.min_obj_dist = 5.0
        self.pre_step = 20

        #robot params
        self.max_v = 0.5
        self.max_v_dot = 1.0
        self.min_v = 0.0
        self.max_w_dot = 1.0
        self.max_w = 0.5
        self.v = 0.0
        self.w = 0.0
        self.robot_r = 0.20
        self.location = None

        #stupid params
        self.last_v = 0.0
        self.last_w = 0.0
        # self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))



    def timer_callback(self):
        msg = Twist()

        self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
        if self.goal:
            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            msg = self.get_twist()
            self.cmd_vel_pub.publish(msg)
    
    def get_twist(self):
        t = self.zero_twist()

        t.linear.x = self.max_v * tanh(self.distance_to_goal)
        t.angular.z = self.max_v * tanh(5 * self.angle_to_goal)
        return t
    
    def laser_cb(self, scan):
        if self.goal and self.is_dwa:
            if self.close_enough():
                self.cmd_vel_pub.publish(self.zero_twist())
                return
            obstacles = self.get_obstacles(scan)
            # self.get_logger().info(f"obstacles: {obstacles}")
            self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            best_pair = self.dwa(obstacles)
            self.get_logger().info(f"Angle to Goal: {self.angle_to_goal}")
            self.get_logger().info(f"Cmd Twist: {best_pair}")
            msg = self.zero_twist()
            msg.linear.x = best_pair[0]
            msg.angular.z = best_pair[1]
            self.cmd_vel_pub.publish(msg)
            self.last_v = best_pair[0]
            self.last_w = best_pair[1]
    
    def odom_cb(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z


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
        self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        self.get_goal_in_base_link()
        self.get_distance_to_goal()
        result = NavGoal.Result()
        result.success = False

        self.get_logger().info(f"Distance to goal: {self.distance_to_goal}, target: {self.target.point}")

        self.get_logger().info(f"Current Position: {self.location.transform.translation}")

        msg = Twist()
        while not self.close_enough():
            self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            msg = self.get_twist()
            self.cmd_vel_pub.publish(msg)
            feedback = NavGoal.Feedback()
            self.get_logger().info(f"distance to goal : {self.distance_to_goal}")
            feedback.distance.data = self.distance_to_goal

            goal_handle.publish_feedback(feedback)
            time.sleep(0.03)

        self.goal = None

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
        self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        self.get_goal_in_base_link()
        self.get_distance_to_goal()
        result = NavGoal.Result()
        result.success = False

        self.get_logger().info(f"Distance to goal: {self.distance_to_goal}, target: {self.target.point}")
        self.get_logger().info(f"Angle to Goal: {self.angle_to_goal}")

        self.get_logger().info(f"Current Position: {self.location.transform.translation}")

        msg = Twist()
        while not self.close_enough():
            feedback = NavGoal.Feedback()
            # self.get_logger().info(f"distance to goal : {self.distance_to_goal}")
            feedback.distance.data = self.distance_to_goal

            goal_handle.publish_feedback(feedback)
            time.sleep(0.03)

        self.goal = None

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
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
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
        thetas = np.linspace(angle_min, angle_max, num_readings)
        ranges = np.array(scan.ranges)

        #Decompose scan into x and y coords using sin, cos
        y_readings = scan.ranges * np.sin(thetas)
        x_readings = scan.ranges * np.cos(thetas)

        #only keep points close enough
        max_range = min(max_range, self.min_obj_dist)
        indicies = np.where(ranges < max_range - 0.0000001)

        return list(zip(x_readings[indicies], y_readings[indicies]))


    def dwa(self, obstacles):
        """Dynamic Window Approach for obstacle avoidance. 
        This function assumes goals and obstacles are provided in the robots frame. 
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
        best_score = 0
        best_pair = []
        v_score = []
        h_score = []
        o_score = []
        for pair in possible_pairs:
            #evaluate heading
            theta = pair["w"] * self.dt 
            h_score.append(pi-abs(self.angle_to_goal - theta))
            v_score.append(pair["v"])
            o_score.append(pair["dist"])
        # self.get_logger().info(f"Heading: {h_score}")
        if v_score == []:
            return [0.0, 0.0]
        v_score = self.normalize_score(v_score)
        h_score = self.normalize_score(h_score)
        o_score = self.normalize_score(o_score)
        # self.get_logger().info(f"Normed: {o_score}")
        checker = []
        for i in range(len(v_score)):
            score = self.heading_w * h_score[i] + self.velocity_w * v_score[i] + self.clearance_w * o_score[i]
            checker.append(score)
            if score > best_score:
                best_score = score
                best_pair = [possible_pairs[i]["v"], possible_pairs[i]["w"]] 
        # print(f"scores {checker}")
        # self.get_logger().info(f"Best Pair: {best_pair}")
        return best_pair
    
    def normalize_score(self, values):
        values = np.array(values)
        if values.max() - values.min() == 0:
            values = [0.0 for i in range(len(values))]
        else:
            values = (values - values.min())/(values.max() - values.min())
        return values


    def admissable_velocities(self, possible_v, possible_w, obstacles):
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
                # min_dist = self.min_obj_dist
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
                #         min_dist = self.min_obj_dist
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
                min_dist = self.min_obj_dist
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
                    possible_pairs.append({"v": v, "w": w, "dist" : min_dist})
                number_checked += 1
        # self.get_logger().info(f"Number of pairs Checked: {number_checked}")
        return possible_pairs




def main(args=None):
    rclpy.init(args=args)

    driver_node = Driver()

    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.spin()

    # rclpy.spin(driver_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()