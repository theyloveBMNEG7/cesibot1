#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json

class ViveTrackerNode(Node):
    def __init__(self):
        super().__init__('vive_tracker_node')

        # Publisher
        self.publisher = self.create_publisher(PoseStamped, '/vive_tracker/pose', 10)

        # Timer: call callback every 50 ms → 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Server config
        self.host = "10.191.69.102"
        self.port = 8000
        self.tracker_name = b"tracker_1"

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

        self.get_logger().info(f"Vive tracker client started - server {self.host}:{self.port}, requesting '{self.tracker_name.decode()}'")

    def timer_callback(self):
        try:
            # Send request to server
            self.sock.sendto(self.tracker_name, (self.host, self.port))

            # Receive response (max 2048 bytes)
            data, _ = self.sock.recvfrom(2048)

            # Clean the raw bytes → string
            raw = data.decode('utf-8', errors='ignore').strip('& \r\n\t')

            # Log raw for debugging (only every 5 seconds to avoid spam)
            current_time = self.get_clock().now().nanoseconds
            if current_time % 5000000000 < 100000000:  # ~every 5s
                self.get_logger().info(f"Raw received (after strip): '{raw[:200]}...'")

            # Remove outer quotes if the server wraps the JSON in quotes
            if raw.startswith('"') and raw.endswith('"'):
                raw = raw[1:-1]

            # Unescape any escaped quotes (server sometimes escapes them)
            raw = raw.replace('\\"', '"')

            # Parse JSON
            try:
                d = json.loads(raw)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode failed: {e}")
                self.get_logger().error(f"Raw data that failed: '{raw}'")
                return

            # Check validity
            if not d.get('valid', False):
                self.get_logger().warn("Received pose marked as invalid")
                return

            # Create PoseStamped message
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'vive_world'

            # Position
            msg.pose.position.x = float(d.get('x', 0.0))
            msg.pose.position.y = float(d.get('y', 0.0))
            msg.pose.position.z = float(d.get('z', 0.0))

            # Orientation
            msg.pose.orientation.x = float(d.get('qx', 0.0))
            msg.pose.orientation.y = float(d.get('qy', 0.0))
            msg.pose.orientation.z = float(d.get('qz', 0.0))
            msg.pose.orientation.w = float(d.get('qw', 1.0))

            # Publish
            self.publisher.publish(msg)

            # Log published pose every ~2 seconds
            if current_time % 2000000000 < 100000000:
                self.get_logger().info(
                    f"Published pose: x={msg.pose.position.x:.3f}, "
                    f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
                )

        except socket.timeout:
            # Silent retry - normal if server is slow
            pass
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e} - raw: {raw}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ViveTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Vive tracker node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()