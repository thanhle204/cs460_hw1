import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random, time

class RandomShape(Node):
    def __init__(self):
        super().__init__('random_shape')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.start = time.time()
        self.timer = self.create_timer(0.5, self.move)

    def move(self):
        if time.time() - self.start > 10.0:  # run 10s
            self.pub.publish(Twist())        # stop
            self.get_logger().info("Random done.")
            rclpy.shutdown()
            return

        msg = Twist()
        msg.linear.x = random.uniform(0.5, 2.0)
        msg.angular.z = random.uniform(-2.0, 2.0)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomShape()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
