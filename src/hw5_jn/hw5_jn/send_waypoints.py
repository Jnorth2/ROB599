import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from nav2_msgs.action import NavigateToPose, Spin

import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SendWaypoints(Node):

    def __init__(self):
        super().__init__('send_waypoint_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Wait for the action server to be available
        self.get_logger().info('Waiting for NavGoal action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('NavGoal action server connected!')

        self.spin_action = ActionClient(self, Spin, 'spin')
        # Wait for the action server to be available
        self.get_logger().info('Waiting for NavGoal action server...')
        self.spin_action.wait_for_server()
        self.get_logger().info('NavGoal action server connected!')

        pkg_share = get_package_share_directory("hw5_jn")
        default_path = os.path.join(pkg_share, "config", "waypoints.yaml")
        self.declare_parameter("waypoints", default_path)
        self.waypoints_path = self.get_parameter("waypoints").value
        self.waypoints = []
        self.load_waypoints_from_yaml()

        self.point_iter = 0
        self.is_done = True
        self.succeed = True

    def send_waypoint(self):
        while self.point_iter < len(self.waypoints):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.waypoints[self.point_iter]
            # goal_msg.goal.header.frame_id = "odom"
            # goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
            # goal_msg.goal.point = self.waypoints[self.point_iter]
            send_waypoint_future = self.action_client.send_goal_async(goal_msg)
            self.get_logger().info("sent goal")
            rclpy.spin_until_future_complete(self, send_waypoint_future)
            sent_goal_handle = send_waypoint_future.result()
            # Wait for result
            result_future = sent_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result_handle = result_future.result()

            spin_msg = Spin.Goal()
            spin_msg.target_yaw = 6.28
            send_spin_future = self.spin_action.send_goal_async(spin_msg)

            self.get_logger().info("sent spin")
            rclpy.spin_until_future_complete(self, send_spin_future)
            sent_goal_handle = send_spin_future.result()
            # Wait for result
            result_future = sent_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result_handle = result_future.result()

            self.point_iter += 1

    def load_waypoints_from_yaml(self):
            with open(self.waypoints_path, 'r') as f:
                data = yaml.safe_load(f)
            for wp in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = wp['frame_id']
                #print(pose.header.frame_id)
                pose.pose.position.x = wp['pose']['position']['x']
                pose.pose.position.y = wp['pose']['position']['y']
                self.waypoints.append(pose)
                #print((pose.point.x, pose.point.y))

def main(args=None):
    rclpy.init(args=args)

    send_waypoint_node = SendWaypoints()
    send_waypoint_node.send_waypoint()

    rclpy.spin(send_waypoint_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    send_waypoint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()