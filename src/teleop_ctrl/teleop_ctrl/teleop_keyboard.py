#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
import sys
import tty
import termios
import threading


# WAM-V Teleoperation Node
# This node allows teleoperation of the WAM-V using keyboard inputs.
class WamvTeleopNode(Node):
    def __init__(self):
        super().__init__('wamv_teleop_node')

        # Publishers
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        # Subscribers
        self.create_subscription(Float64, '/wamv/thrusters/left/thrust', self.left_thrust_callback, 10)
        self.create_subscription(Float64, '/wamv/thrusters/left/pos', self.left_pos_callback, 10)
        self.create_subscription(Float64, '/wamv/thrusters/right/thrust', self.left_pos_callback, 10)
        self.create_subscription(Float64, '/wamv/thrusters/right/pos', self.left_pos_callback, 10)

        # Internal state updated from feedback
        self.current_thrust = 0.0
        self.current_pos = 0.0
        self.max_pos = 1.5
        self.min_pos = -1.5

        # Launch input thread
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info("WAM-V Teleop Node with Feedback Started")

        # log keyboard instructions
        msg = " Keyboard Controls:\n" \
                "  - 'w': Increase thrust\n" \
                "  - 's': Decrease thrust\n" \
                "  - 'a': Turn left\n" \
                "  - 'd': Turn right\n" \
                "  - 'q': Quit\n"
        print(msg)

    def publish_thruster_command(self):
        # Clamp position
        if self.current_pos >= self.max_pos:
            self.current_pos = self.max_pos
        elif self.current_pos <= self.min_pos:
            self.current_pos = self.min_pos

        self.left_pos_pub.publish(Float64(data=self.current_pos))
        self.right_pos_pub.publish(Float64(data=self.current_pos))
        self.left_thrust_pub.publish(Float64(data=self.current_thrust))
        self.right_thrust_pub.publish(Float64(data=self.current_thrust))

    def keyboard_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                key = sys.stdin.read(1).lower()
                if key == 'w':
                    self.current_thrust += 100.0
                    self.current_pos = 0.0
                    self.get_logger().info(f"[Input] Forward → Thrust: {self.current_thrust}")
                elif key == 's':
                    self.current_thrust -= 100.0
                    self.current_pos = 0.0
                    self.get_logger().info(f"[Input] Reverse → Thrust: {self.current_thrust}")
                elif key == 'a':
                    self.current_pos -= 0.1
                    self.get_logger().info(f"[Input] Turn Left → Pos: {self.current_pos}")
                elif key == 'd':
                    self.current_pos += 0.1
                    self.get_logger().info(f"[Input] Turn Right → Pos: {self.current_pos}")
                elif key == 'q':
                    self.current_thrust = 0.0
                    self.current_pos = 0.0
                    self.get_logger().info("[Input] Quit")
                    self.publish_thruster_command()
                    self.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0)
                else:
                    self.get_logger().info(f"[Input] Unknown key: {key}")
                    break

                self.publish_thruster_command()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # Subscription callbacks
    def left_thrust_callback(self, msg):
        self.current_thrust = msg.data

    def left_pos_callback(self, msg):
        self.current_pos = msg.data

    def right_thrust_callback(self, msg):
        self.current_thrust = msg.data

    def right_pos_callback(self, msg):
        self.current_pos = msg.data

def main(args=None):
    rclpy.init(args=args)
    # Create the WAM-V teleoperation node
    try:
        node = WamvTeleopNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt: Shutting down WAM-V Teleop Node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
