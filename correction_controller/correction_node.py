import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


class OpenLoopCorrectionNode(Node):
    def __init__(self):
        super().__init__('open_loop_correction_node')

        # Subscriptions
        self.subscription = self.create_subscription(
            Twist, '/visual_servo/pose_correction', self.twist_callback, 10)

        self.estop_sub = self.create_subscription(
            Bool, '/estop', self.estop_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Execution state
        self.state = 'idle'
        self.target_twist = None
        self.duration = 0.0
        self.elapsed_time = 0.0

        self.estop_active = False

        # Parameters
        self.declare_parameter('velocity', 0.1)
        self.vel_mag = self.get_parameter('velocity').get_parameter_value().double_value

        self.declare_parameter('angular_velocity', 0.1)
        self.angular_vel_mag = self.get_parameter('angular_velocity').get_parameter_value().double_value

        # Rotation handling
        self.target_angular_y = 0.0
        self.rotation_twist = None
        self.rotation_duration = 0.0
        self.rotation_elapsed = 0.0

        # Threshold: 2 degrees in radians
        self.angular_threshold_rad = math.radians(2.0)

        # Sequential translation queue
        self.translation_queue = []

    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data
        if self.estop_active:
            self.get_logger().warn("Estop engaged! Cancelling motion and halting the drone.")
            self.state = 'idle'
            self.target_twist = None
            self.rotation_twist = None
            self.translation_queue = []
            self.duration = 0.0
            self.elapsed_time = 0.0
            self.rotation_duration = 0.0
            self.rotation_elapsed = 0.0
            self.target_angular_y = 0.0
            self.cmd_vel_pub.publish(Twist())
        else:
            self.get_logger().info("Estop released. Ready to accept new commands.")

    def twist_callback(self, twist_msg: Twist):
        if self.estop_active:
            self.get_logger().warn("Estop active. Ignoring correction command.")
            return

        if self.state != 'idle':
            self.get_logger().warn("Still executing a correction. Ignoring new command.")
            return

        # Only use angular.y
        self.target_angular_y = twist_msg.angular.y

        # --------------------------------------------------
        # If angular correction is more than 2 deg:
        # execute rotation ONLY and break
        # --------------------------------------------------
        if abs(self.target_angular_y) > self.angular_threshold_rad:
            self.rotation_duration = abs(self.target_angular_y) / self.angular_vel_mag
            self.rotation_elapsed = 0.0

            twist = Twist()
            twist.angular.z = -self.angular_vel_mag if self.target_angular_y > 0 else self.angular_vel_mag
            self.rotation_twist = twist

            self.state = 'rotating'
            self.get_logger().info(
                f"Angular correction {math.degrees(self.target_angular_y):.2f} deg "
                f"> 2 deg. Executing rotation only for {self.rotation_duration:.2f} s."
            )
            return

        # --------------------------------------------------
        # Translation: convert from camera frame to drone FLU
        # Camera: x right, y down, z forward
        # Body FLU: x forward, y left, z up
        # --------------------------------------------------
        cam_x = twist_msg.linear.x
        cam_y = twist_msg.linear.y
        cam_z = twist_msg.linear.z

        dx = cam_z     # forward
        dy = -cam_x    # left
        dz = -cam_y    # up

        candidate_axes = [
            ('x', dx),
            ('y', dy),
            ('z', dz),
        ]

        queued_moves = []

        min_thresh = 0.08
        max_thresh = 0.60

        for axis_name, axis_value in candidate_axes:
            abs_val = abs(axis_value)

            # Skip tiny motions
            if abs_val < min_thresh:
                self.get_logger().info(
                    f"Skipping {axis_name}-axis: {axis_value:.3f} m "
                    f"(below minimum threshold of {min_thresh:.2f} m)."
                )
                continue

            # Clamp oversized motions to maximum threshold
            exec_val = axis_value
            if abs_val > max_thresh:
                exec_val = math.copysign(max_thresh, axis_value)
                self.get_logger().info(
                    f"Clamping {axis_name}-axis from {axis_value:.3f} m "
                    f"to {exec_val:.3f} m (maximum threshold {max_thresh:.2f} m)."
                )

            axis_twist = Twist()
            if axis_name == 'x':
                axis_twist.linear.x = self.vel_mag if exec_val > 0 else -self.vel_mag
            elif axis_name == 'y':
                axis_twist.linear.y = self.vel_mag if exec_val > 0 else -self.vel_mag
            elif axis_name == 'z':
                axis_twist.linear.z = self.vel_mag if exec_val > 0 else -self.vel_mag

            axis_duration = abs(exec_val) / self.vel_mag
            queued_moves.append((axis_name, exec_val, axis_twist, axis_duration))

        if not queued_moves:
            self.get_logger().info("No valid per-axis translation to execute.")
            return

        self.translation_queue = queued_moves
        self.start_next_translation()

    def start_next_translation(self):
        if not self.translation_queue:
            self.get_logger().info("All sequential translations complete.")
            self.state = 'idle'
            self.target_twist = None
            self.duration = 0.0
            self.elapsed_time = 0.0
            return

        axis_name, axis_value, axis_twist, axis_duration = self.translation_queue.pop(0)

        self.target_twist = axis_twist
        self.duration = axis_duration
        self.elapsed_time = 0.0
        self.state = 'executing'

        self.get_logger().info(
            f"Executing {axis_name}-axis translation only: "
            f"{axis_value:.3f} m over {axis_duration:.2f} s."
        )

    def control_loop(self):
        if self.estop_active:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.state == 'executing':
            self.elapsed_time += 0.1
            if self.elapsed_time < self.duration and self.target_twist is not None:
                self.cmd_vel_pub.publish(self.target_twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Axis translation complete.")
                self.target_twist = None
                self.duration = 0.0
                self.elapsed_time = 0.0
                self.start_next_translation()

        elif self.state == 'rotating':
            self.rotation_elapsed += 0.1
            if self.rotation_elapsed < self.rotation_duration and self.rotation_twist is not None:
                self.cmd_vel_pub.publish(self.rotation_twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Rotation complete.")
                self.state = 'idle'
                self.rotation_twist = None
                self.rotation_duration = 0.0
                self.rotation_elapsed = 0.0
                self.target_angular_y = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()