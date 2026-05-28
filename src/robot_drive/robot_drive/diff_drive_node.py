#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros
import math
import sys
import os
import time

sys.path.append(os.path.join(os.path.expanduser('~'), 'ros2_ws/src/DynamixelSDK/python/src'))
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Control Table Addresses
ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_GOAL_VELOCITY        = 104
ADDR_PRESENT_POSITION     = 132
ADDR_PRESENT_CURRENT      = 126
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_VELOCITY_LIMIT       = 44

TORQUE_ON     = 1
TORQUE_OFF    = 0
VELOCITY_MODE = 1

LEFT_ID  = 1
RIGHT_ID = 2
PORT     = '/dev/u2d2'
BAUDRATE = 57600
PROTOCOL = 2.0

VELOCITY_UNIT    = 0.229      # 1 unit = 0.229 RPM (XL430 datasheet)
TICKS_PER_REV    = 4096.0
CURRENT_UNIT     = 2.69       # mA per unit
STALL_CURRENT_MA = 900.0      # stop if current exceeds this

# Robot geometry
WHEEL_RADIUS     = 0.034      
WHEEL_SEPARATION = 0.184      

# Safety
MAX_LINEAR_MPS  = 0.25
MAX_ANGULAR_RPS = 1.5
CMD_TIMEOUT_SEC = 0.3

# Motor hardware profile
PROFILE_ACCEL  = 20
PROFILE_VEL    = 150
VELOCITY_LIMIT = 200


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_left_pos  = None
        self.last_right_pos = None

        self.last_cmd_time  = self.get_clock().now()
        self.target_linear  = 0.0
        self.target_angular = 0.0

        self.read_left_turn = True


        self.stall_counter = 0

        # Connect to motors
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL)

        if not self._connect_and_configure():
            raise RuntimeError('Motor connection failed')

        # Publishers
        self.odom_pub  = self.create_publisher(Odometry,   '/odom',         10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states',  10)
       # self.tf_br     = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Control loop at 50 Hz
        self.create_timer(0.033, self.update)

        self.get_logger().info(
            f'DiffDrive node ready | '
            f'r={WHEEL_RADIUS*1000:.1f}mm '
            f'sep={WHEEL_SEPARATION*1000:.1f}mm')

    def _connect_and_configure(self):
        """Open port and configure both motors. Returns True on success."""
        if not self.port.openPort() or not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error('Port open/baud failed')
            return False

        for mid in [LEFT_ID, RIGHT_ID]:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE,       TORQUE_OFF)
            self.packet.write1ByteTxRx(self.port, mid, ADDR_OPERATING_MODE,      VELOCITY_MODE)
            # Smooth motion profile
            self.packet.write4ByteTxRx(self.port, mid, ADDR_PROFILE_ACCELERATION, PROFILE_ACCEL)
            self.packet.write4ByteTxRx(self.port, mid, ADDR_PROFILE_VELOCITY,    PROFILE_VEL)
            # VELOCITY_LIMIT 
            self.packet.write4ByteTxRx(self.port, mid, ADDR_VELOCITY_LIMIT,      VELOCITY_LIMIT)
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE,       TORQUE_ON)

        self.get_logger().info('Motors in velocity mode with profile smoothing enabled')
        return True

    def _reconnect(self):
        """Reconnect after USB disconnection — retries every 2 seconds."""
        self.get_logger().warn('Connection lost — trying to reconnect...')
        self.port.closePort()
        while rclpy.ok():
            time.sleep(2.0)
            if self._connect_and_configure():
                # Reset state so odometry restarts cleanly
                self.last_left_pos  = None
                self.last_right_pos = None
                self.stall_counter  = 0
                self.read_left_turn = True
                self.get_logger().info('Reconnected successfully')
                return
            self.get_logger().warn('Reconnect failed, retrying...')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time  = self.get_clock().now()
        self.target_linear  = max(-MAX_LINEAR_MPS,  min(MAX_LINEAR_MPS,  msg.linear.x))
        self.target_angular = max(-MAX_ANGULAR_RPS, min(MAX_ANGULAR_RPS, msg.angular.z))

    def set_velocity(self, motor_id, rpm):
        units = int(rpm / VELOCITY_UNIT)
        if units < 0:
            units &= 0xFFFFFFFF
        self.packet.write4ByteTxRx(self.port, motor_id, ADDR_GOAL_VELOCITY, units)

    def get_position(self, motor_id):
        pos, result, _ = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_POSITION)
        if result != COMM_SUCCESS:
            return None
        if pos > 2147483647:
            pos -= 4294967296
        return pos

    def get_current(self, motor_id):
        cur, result, _ = self.packet.read2ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_CURRENT)
        if result != COMM_SUCCESS:
            return 0.0
        if cur > 32767:
            cur -= 65536
        return abs(cur) * CURRENT_UNIT

    def update(self):

        # Timeout safety
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > CMD_TIMEOUT_SEC:
            self.target_linear  = 0.0
            self.target_angular = 0.0
            self.stall_counter  = 0   # reset when robot is stopped

        # Kinematics: robot velocity 
        v = self.target_linear
        w = self.target_angular
        left_mps  = v - w * WHEEL_SEPARATION / 2.0
        right_mps = v + w * WHEEL_SEPARATION / 2.0

        left_rpm  = -(left_mps  / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0
        right_rpm =  (right_mps / (2.0 * math.pi * WHEEL_RADIUS)) * 60.0

        # Stall detection
        self.stall_counter += 1
        if self.stall_counter >= 5:
            self.stall_counter = 0
            if elapsed < CMD_TIMEOUT_SEC:
                left_cur  = self.get_current(LEFT_ID)
                right_cur = self.get_current(RIGHT_ID)
                if max(left_cur, right_cur) > STALL_CURRENT_MA:
                    self.get_logger().warn(
                        f'STALL detected! '
                        f'L:{left_cur:.0f} R:{right_cur:.0f} mA → stopping')
                    self.set_velocity(LEFT_ID,  0.0)
                    self.set_velocity(RIGHT_ID, 0.0)
                    return

        # Send velocity commands 
        self.set_velocity(LEFT_ID,  left_rpm)
        self.set_velocity(RIGHT_ID, right_rpm)

        if self.read_left_turn:
            new_left = self.get_position(LEFT_ID)
            if new_left is None:
                self._reconnect()
                return
            left_pos  = new_left
            right_pos = self.last_right_pos if self.last_right_pos is not None else new_left
        else:
            new_right = self.get_position(RIGHT_ID)
            if new_right is None:
                self._reconnect()
                return
            right_pos = new_right
            left_pos  = self.last_left_pos if self.last_left_pos is not None else new_right

        self.read_left_turn = not self.read_left_turn

        # Odometry calculation
        if self.last_left_pos is not None and self.last_right_pos is not None:

            d_left  = -(left_pos  - self.last_left_pos)  / TICKS_PER_REV * 2 * math.pi * WHEEL_RADIUS
            d_right =  (right_pos - self.last_right_pos) / TICKS_PER_REV * 2 * math.pi * WHEEL_RADIUS

            distance = (d_left + d_right) / 2.0
            d_theta  = (d_right - d_left) / WHEEL_SEPARATION

            # Midpoint integration 
            self.x     += distance * math.cos(self.theta + d_theta / 2)
            self.y     += distance * math.sin(self.theta + d_theta / 2)
            self.theta += d_theta
            self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.last_left_pos  = left_pos
        self.last_right_pos = right_pos

        # Publish /odom 
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Pose covariance (how much to trust position from encoders)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14]  = 1e6   # z
        odom.pose.covariance[21]  = 1e6   # roll
        odom.pose.covariance[28]  = 1e6   # pitch
        odom.pose.covariance[35] = 0.01   # yaw

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        # Twist covariance (how much to trust velocity estimate)
        odom.twist.covariance[0]  = 0.01   # vx
        odom.twist.covariance[7]  = 1e6   # vy
        odom.twist.covariance[14]  = 1e6   # vz 
        odom.twist.covariance[21]  = 1e6   # vroll
        odom.twist.covariance[28]  = 1e6   # vpitch
        odom.twist.covariance[35] = 0.01   # vyaw

        self.odom_pub.publish(odom)
        
        # Publish /tf 
        #t = TransformStamped()
        #t.header         = odom.header
        #t.child_frame_id = 'base_footprint'
        #t.transform.translation.x = self.x
        #t.transform.translation.y = self.y
        #t.transform.rotation      = odom.pose.pose.orientation
        #self.tf_br.sendTransform(t)

        # Publish /joint_states
        js = JointState()
        js.header.stamp = now
        js.name         = ['left_wheel_joint', 'right_wheel_joint']
        js.position     = [
             (left_pos  / TICKS_PER_REV) * 2 * math.pi,
             (right_pos / TICKS_PER_REV) * 2 * math.pi
        ]
        self.joint_pub.publish(js)

    def shutdown(self):
        self.get_logger().info('Shutting down...')
        self.set_velocity(LEFT_ID,  0.0)
        self.set_velocity(RIGHT_ID, 0.0)
        time.sleep(0.1)
        for mid in [LEFT_ID, RIGHT_ID]:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, TORQUE_OFF)
        self.port.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()