import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RectangleShape(Node):
    def __init__(self):
        super().__init__('rectangle_shape')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1
        self.timer_ = self.create_timer(self.timer_period, self.publish_twist)
        self.state = "forward"
        self.start_time = time.time()
        self.side_count = 0

    def publish_twist(self):
        msg = Twist()
        current_time = time.time()

        if self.side_count >= 4:
            # Dừng hẳn
            self.publisher_.publish(Twist())
            self.get_logger().info("Rectangle completed, turtle stopped.")
            rclpy.shutdown()
            return

        if self.state == "forward":
            msg.linear.x = 2.0
            if current_time - self.start_time > 2.0:   # go straight 2s
                self.state = "turn"
                self.start_time = current_time
        elif self.state == "turn":
            msg.angular.z = 1.57   # turning 90 degress (≈ π/2 rad/s) 
            if current_time - self.start_time > 1.0:  # turning in 1s
                self.state = "forward"
                self.start_time = current_time
                self.side_count += 1

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleShape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
