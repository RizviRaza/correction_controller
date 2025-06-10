import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
import math
import time


class OpenLoopCorrectionNode(Node):
    def __init__(self):
        super().__init__('open_loop_correction_node')

        # Subscriptions
        self.subscription = self.create_subscription(
            Pose, '/correction_pose', self.pose_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.estop_sub = self.create_subscription(
            Bool, '/estop', self.estop_callback, 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Execution state
        self.state = 'idle'
        self.target_twist = None
        self.duration = 0.0
        self.elapsed_time = 0.0

        # Obstacle detection
        self.obstacle_too_close = False
        self.was_paused = False

        self.estop_active = False

        # Parameters
        self.declare_parameter('velocity', 0.1)
        self.vel_mag = self.get_parameter('velocity').get_parameter_value().double_value

        self.declare_parameter('min_obstacle_distance', 0.5)
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value

        self.declare_parameter('angular_velocity', 0.5)
        self.angular_vel_mag = self.get_parameter('angular_velocity').get_parameter_value().double_value

        # Rotation handling
        self.target_yaw = 0.0
        self.rotation_twist = None
        self.rotation_duration = 0.0
        self.rotation_elapsed = 0.0

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn("Estop engaged! Cancelling motion and halting the drone.")
            self.state = 'idle'
            self.target_twist = None
            self.rotation_twist = None
            self.duration = 0.0
            self.elapsed_time = 0.0
            self.rotation_duration = 0.0
            self.rotation_elapsed = 0.0
            self.cmd_vel_pub.publish(Twist())  # Stop immediately
        else:
            self.get_logger().info("Estop released. Ready to accept new commands.")


    def pose_callback(self, msg):
        if self.estop_active:
            self.get_logger().warn("Estop active. Ignoring correction command.")
            return
        
        if self.state != 'idle':
            self.get_logger().warn("Still executing a correction. Ignoring new command.")
            return

        # --- Translation setup ---
        dx = msg.position.x
        dy = msg.position.y
        dz = msg.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < 0.01:
            self.get_logger().info("Correction too small. Ignoring.")
            return

        norm_dx = dx / distance
        norm_dy = dy / distance
        norm_dz = dz / distance

        twist = Twist()
        twist.linear.x = norm_dx * self.vel_mag
        twist.linear.y = norm_dy * self.vel_mag
        twist.linear.z = norm_dz * self.vel_mag

        self.target_twist = twist
        self.duration = distance / self.vel_mag
        self.elapsed_time = 0.0

        # --- Yaw setup (stored for later) ---
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.target_yaw = yaw

        self.get_logger().info(
            f"Starting translation: {distance:.2f} m over {self.duration:.2f} s, yaw target = {math.degrees(yaw):.1f}°")

        self.state = 'executing'

    def laser_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.obstacle_too_close = False
            return

        self.obstacle_too_close = any(
            r < self.min_obstacle_distance for r in msg.ranges if r > 0.0 and math.isfinite(r))

    def control_loop(self):
        if self.estop_active:
            self.cmd_vel_pub.publish(Twist())  # Force stop
            return
        
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

            self.elapsed_time += 0.1
            if self.elapsed_time < self.duration:
                self.cmd_vel_pub.publish(self.target_twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Translation complete.")

                # Start rotation if needed
                if abs(self.target_yaw) > 0.01:
                    self.rotation_duration = abs(self.target_yaw) / self.angular_vel_mag
                    self.rotation_elapsed = 0.0

                    twist = Twist()
                    twist.angular.z = self.angular_vel_mag if self.target_yaw > 0 else -self.angular_vel_mag
                    self.rotation_twist = twist

                    self.state = 'rotating'
                    self.get_logger().info(
                        f"Starting rotation: {math.degrees(self.target_yaw):.1f}° over {self.rotation_duration:.2f} s")
                else:
                    self.state = 'idle'

        elif self.state == 'rotating':
            if self.obstacle_too_close:
                if not self.was_paused:
                    self.get_logger().warn("Obstacle too close! Pausing rotation.")
                    self.was_paused = True
                self.cmd_vel_pub.publish(Twist())
                return
            elif self.was_paused:
                self.get_logger().info("Obstacle cleared. Resuming rotation.")
                self.was_paused = False

            self.rotation_elapsed += 0.1
            if self.rotation_elapsed < self.rotation_duration:
                self.cmd_vel_pub.publish(self.rotation_twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Rotation complete.")
                self.state = 'idle'
                self.rotation_twist = None
                self.rotation_duration = 0.0
                self.rotation_elapsed = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
