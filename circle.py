import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CircleShape(Node):
    def __init__(self):
        super().__init__('circle_shape')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1
        self.timer_ = self.create_timer(self.timer_period, self.publish_twist)
        self.start_time = time.time()
        self.duration = (2 * math.pi) / 1.0  # 2π / angular_z

    def publish_twist(self):
        msg = Twist()
        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            msg.linear.x = 2.0   # forward
            msg.angular.z = 1.0  # turn
            self.publisher_.publish(msg)
        else:
            # stop
            self.publisher_.publish(Twist())
            self.get_logger().info("Circle completed, turtle stopped.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CircleShape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
