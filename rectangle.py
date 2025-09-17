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
         