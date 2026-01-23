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

import time
import numpy as np
from math import atan2, tanh, sqrt, pi, fabs, cos, sin

from rclpy.executors import MultiThreadedExecutor



class Driver(Node):

    def __init__(self):
        super().__init__('driver')
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

        self.laser_sub = self.create_subscription(LaserScan, 'base_scan', self.laser_cb, 10)

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

        #robot params
        self.max_speed = 0.5
        self.location = None
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

        t.linear.x = self.max_speed * tanh(self.distance_to_goal)
        t.angular.z = self.max_speed * tanh(5 * self.angle_to_goal)
        return t
    
    def laser_cb(self, scan):
        return


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
            rclpy.spin_once(self)
            self.location = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds = 1.0))
            self.get_goal_in_base_link()
            self.get_distance_to_goal()
            msg = self.get_twist()
            self.cmd_vel_pub.publish(msg)
            feedback = NavGoal.Feedback()
            self.get_distance_to_goal()
            # self.get_logger().info(f"distance to goal : {self.distance_to_goal}")
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

    def dwa(self):
        return




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