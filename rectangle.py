import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class RectangleShape(Node):
    def __init__(self):
        super().__init__('rectangle_shape')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer_period = 0.05           # run every 0.05s (20 Hz)
        self.timer = self.create_timer(self.timer_period, self.loop)

        # simple state machine
        self.state = 'forward'             # 'forward' -> 'turn' -> 'forward' ...
        self.start_time = time.time()
        self.segment = 0                   # how many straight segments finished (0..4)

        # tweakable constants
        self.LIN_SPD = 2.0                 # forward speed
        self.FWD_TIME = 2.0                # seconds for each side length
        self.TURN_SPD = math.pi / 2        # 90 deg/s
        self.TURN_TIME = 1.06              # a bit > 1.0s to compensate drift

    def loop(self):
        msg = Twist()
        now = time.time()

        # done: stop and shutdown
        if self.segment >= 4:
            self.pub.publish(Twist())      # hard stop
            self.get_logger().info('Rectangle completed. Stopping near start point.')
            rclpy.shutdown()
            return

        if self.state == 'forward':
            # go straight
            msg.linear.x = self.LIN_SPD
            msg.angular.z = 0.0

            if now - self.start_time > self.FWD_TIME:
                # one side done
                self.segment += 1
                self.start_time = now

                # brake one tick to avoid drifting
                self.pub.publish(Twist())

                # only turn after the first 3 sides; on the 4th, just stop
                if self.segment < 4:
                    self.state = 'turn'
                else:
                    self.state = 'stop'

        elif self.state == 'turn':
            # turn left ~90 degrees
            msg.linear.x = 0.0
            msg.angular.z = self.TURN_SPD

            if now - self.start_time > self.TURN_TIME:
                # finish turning, brake, then go forward again
                self.pub.publish(Twist())
                self.start_time = now
                self.state = 'forward'

        elif self.state == 'stop':
            # final hard stop at the end of the 4th side
            self.pub.publish(Twist())
            self.get_logger().info('Rectangle completed. Turtle stopped.')
            rclpy.shutdown()
            return

        # publish command
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleShape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
