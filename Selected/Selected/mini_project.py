#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import sys

class CircleNavigator(Node):
    def __init__(self):
        super().__init__('circle_navigator')
        # Publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for the robot's trajectory
        self.trajectory_pub = self.create_publisher(Path, '/path', 10)
        self.trajectory_msg = Path()
        self.trajectory_msg.header.frame_id = 'odom'

        # Subscribe to robot's odometry data
        self.create_subscription(Odometry, '/odom', self.odom_update, 10)

        # Timer to periodically send velocity commands
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Parse speed and circle radius from command-line arguments or use defaults
        if len(sys.argv) < 3:
            self.get_logger().warn(
                "Usage: ros2 run <pkg> circle_navigator <speed> <circle_radius>; using defaults"
            )
            self.speed = 0.1  # meters per second
            self.circle_radius = 2.0  # meters
        else:
            try:
                self.speed = float(sys.argv[1])
                self.circle_radius = float(sys.argv[2])
            except ValueError:
                self.get_logger().error("Arguments must be numeric; reverting to defaults")
                self.speed = 0.1
                self.circle_radius = 2.0

        # Calculate angular velocity for circular motion (ω = v / r)
        self.rotation_rate = self.speed / self.circle_radius
        self.get_logger().info(
            f"Circular motion: speed = {self.speed} m/s, radius = {self.circle_radius} m → angular rate = {self.rotation_rate} rad/s"
        )

    def control_loop(self):
        # Publish velocity commands for circular motion
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = self.rotation_rate
        self.cmd_publisher.publish(cmd)

    def odom_update(self, odom: Odometry):
        # Update and publish the robot's trajectory
        self.trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = self.trajectory_msg.header
        pose.pose = odom.pose.pose
        self.trajectory_msg.poses.append(pose)
        self.trajectory_pub.publish(self.trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = CircleNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()