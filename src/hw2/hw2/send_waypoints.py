import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from nav_interface.action import NavGoal

class SendWaypoints(Node):

    def __init__(self):
        super().__init__('send_waypoint_node')
        self.action_client = ActionClient(self, NavGoal, 'nav_dwa_goal')
        # Wait for the action server to be available
        self.get_logger().info('Waiting for NavGoal action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('NavGoal action server connected!')

        

        self.waypoints = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=4.0, y=0.0, z=0.0),
            Point(x=2.0, y=-2.0, z=0.0),
            Point(x=0.0, y=-3.0, z=0.0),
            Point(x=-4.0, y=0.0, z=0.0),
            Point(x=0.0, y=3.0, z=0.0),
            Point(x=0.0, y=0.0, z=0.0)
        ]

        # self.waypoints = [
        #     Point(x = 0.0, y = 2.0, z = 0.0)
        # ]

        # self.waypoints = [
        #     Point(x = 5.0, y=0.0, z= 0.0),
        #     Point(x = 5.0, y=-4.0, z= 0.0),
        #     Point(x = 5.0, y=4.0, z= 0.0),
        #     Point(x = 0.0, y=0.0, z= 0.0),
        # ]

        #For Real Robot
        # self.waypoints = [
        #     Point(x = 3.0, y = 0.0, z = 0.0),
        #     Point(x = 3.0, y = -0.75, z = 0.0),
        #     Point(x = 0.0, y = 0.0, z = 0.0)
        # ]


        self.point_iter = 0
        self.is_done = True
        self.succeed = True

    def send_waypoint(self):
        while self.point_iter < len(self.waypoints):
            goal_msg = NavGoal.Goal()
            goal_msg.goal.header.frame_id = "odom"
            goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
            goal_msg.goal.point = self.waypoints[self.point_iter]
            send_waypoint_future = self.action_client.send_goal_async(goal_msg)
            self.get_logger().info("sent goal")
            rclpy.spin_until_future_complete(self, send_waypoint_future)
            sent_goal_handle = send_waypoint_future.result()
            # Wait for result
            result_future = sent_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result_handle = result_future.result()
            self.point_iter += 1




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