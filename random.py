import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import time, random, math

class RandomShape(Node):
    def __init__(self):
        super().__init__('random_shape')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.tp_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # --- super simple random polygon near center ---
        self.n_sides = random.randint(3, 6)      # triangle..hexagon
        self.lin_speed = 1.4                     # small forward speed
        self.turn_speed = math.pi / 2            # 90 deg/s (used to time turns)
        self.side_time = 0.8                     # short side -> shape stays small
        self.turn_time = (2 * math.pi / self.n_sides) / self.turn_speed

        # add a random initial heading so the polygon looks different each run
        self.random_spin_time = random.uniform(0.0, 2 * math.pi) / self.turn_speed

        # tiny state machine
        self.state = 'spin'                      # spin -> forward -> turn (repeat) -> finish
        self.start_time = time.time()
        self.sides_done = 0

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    def loop(self):
        msg = Twist()
        now = time.time()

        # finish: hard stop + teleport back to default spawn = exact start
        if self.sides_done >= self.n_sides and self.state == 'finish':
            self.pub.publish(Twist())
            # try to snap to default spawn (5.544445, 5.544445, 0)
            if self.tp_client.wait_for_service(timeout_sec=0.5):
                req = TeleportAbsolute.Request()
                req.x, req.y, req.theta = 5.544445, 5.544445, 0.0
                self.tp_client.call_async(req)  # fire-and-forget is fine here
            self.get_logger().info('Random tiny shape done. Teleported to start and stopped.')
            rclpy.shutdown()
            return

        if self.state == 'spin':
            # random initial heading so it feels random, still small & safe
            msg.angular.z = self.turn_speed
            if now - self.start_time > self.random_spin_time:
                self.pub.publish(Twist())       # brake 1 tick
                self.start_time = now
                self.state = 'forward'

        elif self.state == 'forward':
            # go straight a short time (keeps shape small, avoids walls)
            msg.linear.x = self.lin_speed
            if now - self.start_time > self.side_time:
                self.pub.publish(Twist())
                self.start_time = now
                self.state = 'turn'

        elif self.state == 'turn':
            # turn by external angle = 360/n, time-based (simple)
            msg.angular.z = self.turn_speed
            if now - self.start_time > self.turn_time:
                self.pub.publish(Twist())
                self.start_time = now
                self.sides_done += 1
                if self.sides_done < self.n_sides:
                    self.state = 'forward'
                else:
                    self.state = 'finish'       # next tick: stop + teleport

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomShape()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
