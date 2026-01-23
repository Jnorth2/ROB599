import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist

class TestTurtle(Node):

    def __init__(self):
        super().__init__('test_turtle')
        #Init Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        #Init timers
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Robot state variables
        self.max_iter = 20
        self.max_speed = 0.1
        self.max_angular_speed = self.max_speed * 5

        self.state = ["for", "back", "left", "right"]
        self.current_state = 0
        self.iter = 0
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        self.iter += 1
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.get_clock().now() - self.start_time > rclpy.duration.Duration(seconds=8):
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            return
        if self.iter > self.max_iter:
            self.iter = 0
            self.current_state = (self.current_state + 1) % len(self.state)
        
        self.get_logger().info("Current state: " + self.state[self.current_state])
        if self.state[self.current_state] == "for":
            msg.twist.linear.x = self.max_speed
        elif self.state[self.current_state] == "back":
            msg.twist.linear.x = -self.max_speed
        elif self.state[self.current_state] == "left":
            msg.twist.angular.z = self.max_angular_speed
        elif self.state[self.current_state] == "right":
            msg.twist.angular.z = -self.max_angular_speed
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    test_turtle = TestTurtle()

    rclpy.spin(test_turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()