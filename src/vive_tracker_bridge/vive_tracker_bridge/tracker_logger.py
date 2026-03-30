#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
import time
import numpy as np

class TrackerCSVLogger(Node):
    def __init__(self):
        super().__init__('tracker_csv_logger')

        # Parameters (you can override with launch file or command line)
        self.declare_parameter('topic_name', '/vive_tracker/pose')
        self.declare_parameter('logging_rate_hz', 200.0)
        self.declare_parameter('output_file', 'tracker_log.csv')

        topic_name = self.get_parameter('topic_name').value
        self.logging_rate = float(self.get_parameter('logging_rate_hz').value)
        self.output_file = self.get_parameter('output_file').value

        # Make sure output is in the same folder as this script (or absolute path)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_file = os.path.join(script_dir, self.output_file)

        self.get_logger().info(f"Logging topic: {topic_name}")
        self.get_logger().info(f"Saving CSV to: {os.path.abspath(self.output_file)}")
        self.get_logger().info(f"Logging rate: ~{self.logging_rate} Hz")

        # CSV setup - headers with speed and quaternion
        self.csv_file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp_sec',    # absolute time
            'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw',
            'speed_m_s'         # computed linear speed
        ])

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            10
        )

        # Rate limiting & previous pose for speed calculation
        self.last_log_time = 0.0
        self.log_interval = 1.0 / self.logging_rate if self.logging_rate > 0 else 0.0
        self.prev_pose = None
        self.prev_time = None

        # Console print limiter (1 Hz)
        self.last_print_second = -1

    def pose_callback(self, msg: PoseStamped):
        now = time.time()

        # Rate limit logging
        if self.log_interval > 0 and (now - self.last_log_time) < self.log_interval:
            return

        # ROS timestamp (more accurate than time.time())
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Compute linear speed (m/s) from previous pose
        speed = 0.0
        if self.prev_pose is not None:
            dt = t - self.prev_time
            if dt > 0:
                dx = x - self.prev_pose[0]
                dy = y - self.prev_pose[1]
                dz = z - self.prev_pose[2]
                distance = np.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance > 0.002: 
                    speed = distance / dt
                else:
                    speed = 0.0

        # Update previous pose
        self.prev_pose = (x, y, z)
        self.prev_time = t

        # Write to CSV
        self.writer.writerow([
            f"{t:.6f}",
            f"{x:.6f}",
            f"{y:.6f}",
            f"{z:.6f}",
            f"{qx:.6f}",
            f"{qy:.6f}",
            f"{qz:.6f}",
            f"{qw:.6f}",
            f"{speed:.6f}"
        ])
        self.csv_file.flush()

        self.last_log_time = now

        # Print once per second
        current_second = int(now)
        if current_second != self.last_print_second:
            self.get_logger().info(
                f"{t:.2f}s → x={x:.3f} y={y:.3f} z={z:.3f} | speed={speed:.3f} m/s"
            )
            self.last_print_second = current_second

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed: {self.output_file}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerCSVLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user (Ctrl+C)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()