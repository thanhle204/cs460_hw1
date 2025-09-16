import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class DiamondShape(Node):
    def __init__(self):
        super().__init__('diamond_shape')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # parematers
        self.dt = 0.05            # 20 Hz
        self.v  = 2.0             # forward speed (m/s turtlesim units)
        self.w  = 1.2             # turning speed (rad/s)
        self.side_len = 2.6       # length of side (tuỳ chỉnh)

        # time
        self.t_forward = self.side_len / self.v
        self.t_turn90  = (math.pi/2) / self.w
        self.t_turn45  = (math.pi/4) / self.w

        self.state = 'turn45'     # turning 45° before draw
        self.t0 = time.time()
        self.sides_done = 0

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info('DiamondShape node started.')

    def tick(self):
        now = time.time()
        elapsed = now - self.t0
        msg = Twist()

        if self.state == 'turn45':
            # turn 45° to make square become diamond
            msg.angular.z = self.w
            if elapsed >= self.t_turn45:
                self.state = 'forward'
                self.t0 = now

        elif self.state == 'forward':
            msg.linear.x = self.v
            if elapsed >= self.t_forward:
                self.state = 'turn90'
                self.t0 = now

        elif self.state == 'turn90':
            msg.angular.z = self.w
            if elapsed >= self.t_turn90:
                self.sides_done += 1
                if self.sides_done >= 4:
                    # finshed for sides, stop
                    self.pub.publish(Twist())
                    self.get_logger().info('Diamond completed. Turtle stopped.')
                    rclpy.shutdown()
                    return
                self.state = 'forward'
                self.t0 = now

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiamondShape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
