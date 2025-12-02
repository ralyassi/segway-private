import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Throttle(Node):
    def __init__(self):
        super().__init__('throttle_cmd_vel')
        # Declare parameters
        self.declare_parameter('in_topic', 'cmd_vel')
        self.declare_parameter('out_topic', 'cmd_vel_segway')
        self.declare_parameter('rate', 20.0)      # Hz
        self.declare_parameter('timeout', 0.2)    # seconds

        # Read parameters
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        rate = self.get_parameter('rate').value
        self.timeout = self.get_parameter('timeout').value

        # State
        self.last_msg = Twist()
        self.last_time = 0.0   # no msg received yet

        # ROS interfaces
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_cb)

        self.get_logger().info(
            f"Throttling {in_topic} -> {out_topic} at {rate:.1f} Hz, timeout={self.timeout}s"
        )

    def cb(self, msg):
        self.last_msg = msg
        self.last_time = time.time()

    def timer_cb(self):
        now = time.time()
        if now - self.last_time > self.timeout:
            # Timeout reached -> publish zero Twist
            self.pub.publish(Twist())
        else:
            # Still fresh -> publish last command
            msg = Twist()
            msg.linear.x = self.last_msg.linear.x
            msg.linear.y = self.last_msg.linear.y
            msg.linear.z = self.last_msg.linear.z
            msg.angular.x = self.last_msg.angular.x
            msg.angular.y = self.last_msg.angular.y
            msg.angular.z = self.last_msg.angular.z

            # Fix: invert steering when moving backwards
            if msg.linear.x < 0.0:
                msg.angular.z *= -1.0

            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Throttle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

