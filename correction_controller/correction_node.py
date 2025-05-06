import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import math
import time


class OpenLoopCorrectionNode(Node):
    def __init__(self):
        super().__init__('open_loop_correction_node')

        # Subscribes to Pose corrections in FLU frame
        self.subscription = self.create_subscription(
            Pose,
            '/correction_pose',
            self.pose_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',  # or remap via launch file
            self.laser_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


        # Timer for the control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Execution state
        self.state = 'idle'
        self.target_twist = None
        self.duration = 0.0
        self.elapsed_time = 0.0  # seconds of actual motion
        self.start_time = None
        self.obstacle_too_close = False
        self.was_paused = False
        
        self.declare_parameter('velocity', 0.1)
        self.vel_mag = self.get_parameter('velocity').get_parameter_value().double_value
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value


    def pose_callback(self, msg):
        if self.state != 'idle':
            self.get_logger().warn("Still executing a correction. Ignoring new command.")
            return

        dx = msg.position.x
        dy = msg.position.y
        dz = msg.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance < 0.01:
            self.get_logger().info("Correction too small. Ignoring.")
            return

        # Normalize direction
        norm_dx = dx / distance
        norm_dy = dy / distance
        norm_dz = dz / distance

        # Set target twist
        twist = Twist()
        twist.linear.x = norm_dx * self.vel_mag
        twist.linear.y = norm_dy * self.vel_mag
        twist.linear.z = norm_dz * self.vel_mag

        # Setup execution state
        self.target_twist = twist
        self.duration = distance / self.vel_mag
        self.start_time = time.time()
        self.elapsed_time = 0.0
        self.state = 'executing'

        self.get_logger().info(
            f"Starting correction: move {distance:.2f} m over {self.duration:.2f} s")

    def laser_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.obstacle_too_close = False
            return

        # Check for any obstacle closer than min threshold
        self.obstacle_too_close = any(
            r < self.min_obstacle_distance for r in msg.ranges if r > 0.0)

    def control_loop(self):
        if self.state == 'executing':
            if self.obstacle_too_close:
                if not self.was_paused:
                    self.get_logger().warn("Obstacle too close! Pausing motion.")
                    self.was_paused = True
                self.cmd_vel_pub.publish(Twist())
                return
            elif self.was_paused:
                self.get_logger().info("Obstacle cleared. Resuming motion.")
                self.was_paused = False


            # Only accumulate time when motion is allowed
            self.elapsed_time += 0.1  # Control loop runs at 10 Hz

            if self.elapsed_time < self.duration:
                self.cmd_vel_pub.publish(self.target_twist)
            else:
                self.cmd_vel_pub.publish(Twist())  # Stop
                self.get_logger().info("Correction complete.")
                self.state = 'idle'
                self.target_twist = None
                self.duration = 0.0
                self.start_time = None
                self.elapsed_time = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
