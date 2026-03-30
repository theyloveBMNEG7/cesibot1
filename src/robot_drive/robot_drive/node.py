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

# ══════════════════════════════════════════════════════════════════════════════
# MOTOR REGISTER ADDRESSES  (XL430-W250 datasheet)
# ══════════════════════════════════════════════════════════════════════════════
ADDR_OPERATING_MODE   = 11   # sets velocity / position / PWM mode
ADDR_TORQUE_ENABLE    = 64   # 1 = on, 0 = off
ADDR_PROFILE_ACCEL    = 108  # hardware acceleration ramp (units: 214.577 RPM/s²)
ADDR_PROFILE_VELOCITY = 112  # hardware max velocity cap
ADDR_GOAL_VELOCITY    = 104  # write target speed here
ADDR_PRESENT_CURRENT  = 126  # actual current draw  (2 bytes, signed)
ADDR_PRESENT_VELOCITY = 128  # actual wheel speed   (4 bytes, signed)
ADDR_PRESENT_POSITION = 132  # encoder tick count   (4 bytes, signed)

# ── Motor IDs and communication ────────────────────────────────────────────────
LEFT_ID   = 1
RIGHT_ID  = 2
PORT      = '/dev/ttyUSB0'
BAUDRATE  = 57600
PROTOCOL  = 2.0

# ── Unit conversions (XL430 datasheet) ────────────────────────────────────────
VELOCITY_UNIT  = 0.229    # 1 unit = 0.229 RPM
CURRENT_UNIT   = 2.69     # 1 unit = 2.69 mA
ACCEL_UNIT     = 214.577  # 1 unit = 214.577 RPM/s²
TICKS_PER_REV  = 4096.0

# ══════════════════════════════════════════════════════════════════════════════
# ROBOT DIMENSIONS  — your physical measurements
# ══════════════════════════════════════════════════════════════════════════════
WHEEL_RADIUS     = 0.034   # metres  (68 mm / 2)
WHEEL_SEPARATION = 0.184   # metres  (measured on floor)

# ══════════════════════════════════════════════════════════════════════════════
# TUNING PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

# ── Speed limits ───────────────────────────────────────────────────────────────
MAX_LINEAR_MPS  = 0.20   # m/s   — hard cap regardless of what cmd_vel sends
MAX_ANGULAR_RPS = 1.0    # rad/s

# ── Acceleration ramp ──────────────────────────────────────────────────────────
# How fast speed increases when a command arrives.
# 0.5 m/s² → reaches 0.20 m/s in 0.4 seconds. Smooth but responsive.
# NO deceleration ramp — robot stops instantly when cmd_vel stops.
ACCEL_LIMIT = 0.5   # m/s²

# ── Hardware profile acceleration ─────────────────────────────────────────────
# Written to the motor at startup (register 108).
# This tells the motor's own firmware to ramp internally.
# Eliminates low-speed jitter because the motor never gets abrupt jumps.
# Value 5 → 5 × 214.577 = ~1073 RPM/s² (fast enough to feel responsive)
# Set to 0 to disable hardware ramp (not recommended).
HW_PROFILE_ACCEL = 5   # Dynamixel units

# ── Hardware velocity cap ──────────────────────────────────────────────────────
# Written to the motor at startup (register 112).
# Even if we send a large RPM, the motor will not exceed this.
# XL430 max is 265 units (≈ 60 RPM). We set a safe limit.
# At 0.20 m/s → wheel RPM = (0.20 / (2π × 0.034)) × 60 ≈ 56 RPM ≈ 244 units
HW_PROFILE_VEL = 250   # Dynamixel units

# ── Dead band ─────────────────────────────────────────────────────────────────
# Below this RPM the motor jitters instead of spinning smoothly.
# If commanded RPM is less than this, we send 0 instead.
# XL430 minimum controllable speed ≈ 3-4 RPM.
DEAD_BAND_RPM = 3.0   # RPM

# ── Stall detection ────────────────────────────────────────────────────────────
# If current exceeds this while the robot is commanded to move → stall.
# XL430 rated stall current ≈ 3200 mA. We stop well before that.
STALL_CURRENT_MA  = 900.0   # milliamps
STALL_VELOCITY_RPM = 2.0    # if actual speed is below this AND current is high → stall

# ── cmd_vel timeout ────────────────────────────────────────────────────────────
CMD_TIMEOUT = 0.2   # seconds — stop immediately after this


class DiffDriveNode(Node):

    def __init__(self):
        super().__init__('diff_drive_node')

        # ── Velocity state ─────────────────────────────────────────────────────
        # target_*   = what the latest /cmd_vel asked for
        # current_*  = what we are actually sending (after ramp UP only)
        self.target_linear   = 0.0
        self.target_angular  = 0.0
        self.current_linear  = 0.0
        self.current_angular = 0.0

        # ── Odometry state ─────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_left_pos  = None
        self.last_right_pos = None
        self.last_cmd_time  = self.get_clock().now()

        # ── Connect to motors ──────────────────────────────────────────────────
        self.port   = PortHandler(PORT)
        self.packet = PacketHandler(PROTOCOL)

        if not self.port.openPort():
            self.get_logger().error('Failed to open port')
            raise RuntimeError('Cannot open port')

        if not self.port.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set baudrate')
            raise RuntimeError('Cannot set baudrate')

        self.get_logger().info('Connected to motors')

        # Configure motors (velocity mode + hardware profile)
        self._configure()

        # ── ROS 2 setup ────────────────────────────────────────────────────────
        self.odom_pub  = self.create_publisher(Odometry,   '/odom',         10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states',  10)
        self.tf_br     = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.dt = 0.02   # 50 Hz
        self.create_timer(self.dt, self.update)

        self.get_logger().info('diff_drive_node started')

    # ══════════════════════════════════════════════════════════════════════════
    # CONFIGURE MOTORS AT STARTUP
    #
    # This runs once when the node starts.
    # We write three things to each motor:
    #   1. Operating mode = velocity mode
    #   2. Profile acceleration = hardware ramp (eliminates jitter)
    #   3. Profile velocity = hardware speed cap (safety ceiling)
    #
    # Why set profile acceleration in hardware?
    #   The motor's own firmware handles the ramp at a much higher
    #   frequency than our Python loop (the motor runs at ~1 kHz internally).
    #   This gives smoother low-speed behaviour than any software ramp can.
    #   It also means even if our loop is slightly late one cycle,
    #   the motor does not jerk.
    # ══════════════════════════════════════════════════════════════════════════

    def _configure(self):
        for mid in [LEFT_ID, RIGHT_ID]:
            # Must disable torque before changing operating mode
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE,   0)
            self.packet.write1ByteTxRx(self.port, mid, ADDR_OPERATING_MODE,  1)

            # Set hardware profile acceleration
            # This makes the motor ramp up internally — eliminates jitter
            self.packet.write4ByteTxRx(self.port, mid, ADDR_PROFILE_ACCEL,    HW_PROFILE_ACCEL)

            # Set hardware velocity ceiling
            # Motor will never exceed this RPM even if we send a larger value
            self.packet.write4ByteTxRx(self.port, mid, ADDR_PROFILE_VELOCITY, HW_PROFILE_VEL)

            # Enable torque — motor is now active
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE,   1)

        self.get_logger().info(
            f'Motors configured | '
            f'HW accel: {HW_PROFILE_ACCEL} units | '
            f'HW vel cap: {HW_PROFILE_VEL} units')

    # ══════════════════════════════════════════════════════════════════════════
    # RECONNECT
    # ══════════════════════════════════════════════════════════════════════════

    def reconnect(self):
        self.get_logger().warn('Connection lost — attempting reconnect...')
        self.port.closePort()
        while rclpy.ok():
            time.sleep(2.0)
            if self.port.openPort() and self.port.setBaudRate(BAUDRATE):
                self._configure()
                self.last_left_pos  = None
                self.last_right_pos = None
                self.current_linear  = 0.0
                self.current_angular = 0.0
                self.get_logger().info('Reconnected successfully')
                return
            self.get_logger().warn('Reconnect failed, retrying...')

    # ══════════════════════════════════════════════════════════════════════════
    # MOTOR WRITE
    # ══════════════════════════════════════════════════════════════════════════

    def set_velocity(self, motor_id, rpm):
        units = int(rpm / VELOCITY_UNIT)
        if units < 0:
            units = units & 0xFFFFFFFF
        self.packet.write4ByteTxRx(self.port, motor_id, ADDR_GOAL_VELOCITY, units)

    # ══════════════════════════════════════════════════════════════════════════
    # MOTOR READ — all three values in one function
    #
    # Returns (position, velocity_rpm, current_ma) or None on failure.
    #
    # We read position, velocity, and current every cycle.
    # position  → used for odometry
    # velocity  → used for stall detection (is the wheel actually moving?)
    # current   → used for stall detection (is the motor working too hard?)
    # ══════════════════════════════════════════════════════════════════════════

    def read_motor(self, motor_id):
        # Read current (2 bytes, signed)
        cur, r1, _ = self.packet.read2ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_CURRENT)
        if r1 != COMM_SUCCESS:
            return None
        if cur > 32767:
            cur -= 65536
        current_ma = abs(cur) * CURRENT_UNIT

        # Read velocity (4 bytes, signed)
        vel, r2, _ = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_VELOCITY)
        if r2 != COMM_SUCCESS:
            return None
        if vel > 2147483647:
            vel -= 4294967296
        velocity_rpm = vel * VELOCITY_UNIT

        # Read position (4 bytes, signed)
        pos, r3, _ = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_POSITION)
        if r3 != COMM_SUCCESS:
            return None
        if pos > 2147483647:
            pos -= 4294967296

        return pos, velocity_rpm, current_ma

    # ══════════════════════════════════════════════════════════════════════════
    # CMD_VEL CALLBACK
    # ══════════════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        # Clamp to safe limits regardless of what was sent
        self.target_linear = max(
            -MAX_LINEAR_MPS,  min(MAX_LINEAR_MPS,  msg.linear.x))
        self.target_angular = max(
            -MAX_ANGULAR_RPS, min(MAX_ANGULAR_RPS, msg.angular.z))

    # ══════════════════════════════════════════════════════════════════════════
    # MAIN LOOP — 50 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def update(self):

        # ── STEP 1: Timeout check ──────────────────────────────────────────────
        # If no cmd_vel arrived recently: stop immediately.
        # We do NOT ramp down — we zero current velocity and send stop now.
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > CMD_TIMEOUT:
            self.target_linear   = 0.0
            self.target_angular  = 0.0
            self.current_linear  = 0.0   # immediate zero — no ramp down
            self.current_angular = 0.0
            self.set_velocity(LEFT_ID,  0.0)
            self.set_velocity(RIGHT_ID, 0.0)
            # Still read encoders and publish odom so Nav2 stays informed
            self._read_and_publish()
            return

        # ── STEP 2: Acceleration ramp UP only ─────────────────────────────────
        # We ramp speed UP smoothly to prevent wheel slip on tile.
        # We do NOT ramp down — when target drops, current snaps to target.
        # This gives immediate stops while still having smooth starts.
        max_step = ACCEL_LIMIT * self.dt

        # Linear velocity — ramp up, snap down
        if self.target_linear > self.current_linear:
            # Speeding up → ramp
            diff = self.target_linear - self.current_linear
            self.current_linear += min(diff, max_step)
        else:
            # Slowing down or changing direction → snap immediately
            self.current_linear = self.target_linear

        # Angular velocity — same logic
        if abs(self.target_angular) > abs(self.current_angular):
            diff = self.target_angular - self.current_angular
            if abs(diff) <= max_step:
                self.current_angular = self.target_angular
            else:
                self.current_angular += math.copysign(max_step, diff)
        else:
            self.current_angular = self.target_angular

        # ── STEP 3: Kinematics ─────────────────────────────────────────────────
        # Convert robot velocity → individual wheel RPM
        v    = self.current_linear
        w    = self.current_angular
        circ = 2.0 * math.pi * WHEEL_RADIUS

        left_mps  = v - (w * WHEEL_SEPARATION / 2.0)
        right_mps = v + (w * WHEEL_SEPARATION / 2.0)

        left_rpm  = -(left_mps  / circ) * 60.0   # negative: left motor mirrored
        right_rpm =  (right_mps / circ) * 60.0

        # ── STEP 4: Dead band — eliminate low-speed jitter ─────────────────────
        # Below DEAD_BAND_RPM the motor twitches instead of spinning smoothly.
        # We treat tiny commands as zero to keep the robot still.
        if abs(left_rpm)  < DEAD_BAND_RPM:
            left_rpm  = 0.0
        if abs(right_rpm) < DEAD_BAND_RPM:
            right_rpm = 0.0

        # ── STEP 5: Send to motors ─────────────────────────────────────────────
        self.set_velocity(LEFT_ID,  left_rpm)
        self.set_velocity(RIGHT_ID, right_rpm)

        # ── STEP 6: Read feedback and publish ──────────────────────────────────
        self._read_and_publish()

    # ══════════════════════════════════════════════════════════════════════════
    # READ FEEDBACK + STALL DETECTION + ODOMETRY + PUBLISH
    # ══════════════════════════════════════════════════════════════════════════

    def _read_and_publish(self):

        # Read all three values from both motors
        left_data  = self.read_motor(LEFT_ID)
        right_data = self.read_motor(RIGHT_ID)

        if left_data is None or right_data is None:
            self.reconnect()
            return

        left_pos,  left_vel,  left_cur  = left_data
        right_pos, right_vel, right_cur = right_data

        # ── Stall detection ────────────────────────────────────────────────────
        # A stall means the motor is commanded to move but cannot.
        # Signs: high current + near-zero actual velocity.
        # Action: log warning and disable torque to protect the motor.
        robot_commanded = (
            abs(self.current_linear)  > 0.01 or
            abs(self.current_angular) > 0.01)

        left_stalled  = (left_cur  > STALL_CURRENT_MA and
                         abs(left_vel)  < STALL_VELOCITY_RPM)
        right_stalled = (right_cur > STALL_CURRENT_MA and
                         abs(right_vel) < STALL_VELOCITY_RPM)

        if robot_commanded and (left_stalled or right_stalled):
            self.get_logger().warn(
                f'STALL DETECTED — '
                f'L: {left_cur:.0f}mA  R: {right_cur:.0f}mA — '
                f'disabling torque')
            self.set_velocity(LEFT_ID,  0.0)
            self.set_velocity(RIGHT_ID, 0.0)
            # Disable torque to protect motors from overheating
            self.packet.write1ByteTxRx(self.port, LEFT_ID,  ADDR_TORQUE_ENABLE, 0)
            self.packet.write1ByteTxRx(self.port, RIGHT_ID, ADDR_TORQUE_ENABLE, 0)
            self.current_linear  = 0.0
            self.current_angular = 0.0
            self.target_linear   = 0.0
            self.target_angular  = 0.0
            # Re-enable torque after brief pause so motor can cool
            time.sleep(0.5)
            self.packet.write1ByteTxRx(self.port, LEFT_ID,  ADDR_TORQUE_ENABLE, 1)
            self.packet.write1ByteTxRx(self.port, RIGHT_ID, ADDR_TORQUE_ENABLE, 1)
            return

        # ── Odometry ───────────────────────────────────────────────────────────
        if self.last_left_pos is None:
            self.last_left_pos  = left_pos
            self.last_right_pos = right_pos
            return

        delta_left  = left_pos  - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        self.last_left_pos  = left_pos
        self.last_right_pos = right_pos

        circ = 2.0 * math.pi * WHEEL_RADIUS
        left_dist  = -(delta_left  / TICKS_PER_REV) * circ
        right_dist =  (delta_right / TICKS_PER_REV) * circ

        distance = (left_dist + right_dist) / 2.0
        rotation = (right_dist - left_dist)  / WHEEL_SEPARATION

        self.theta += rotation
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x     += distance * math.cos(self.theta)
        self.y     += distance * math.sin(self.theta)

        # ── Publish ────────────────────────────────────────────────────────────
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.twist.linear.x  = distance / self.dt
        odom.twist.twist.angular.z = rotation / self.dt
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[7]   = 0.01
        odom.twist.covariance[35]  = 0.01
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = math.sin(self.theta / 2.0)
        t.transform.rotation.w    = math.cos(self.theta / 2.0)
        self.tf_br.sendTransform(t)

        js = JointState()
        js.header.stamp = now
        js.name         = ['wheel_left_joint', 'wheel_right_joint']
        js.position     = [
            -(left_pos  / TICKS_PER_REV) * 2.0 * math.pi,
             (right_pos / TICKS_PER_REV) * 2.0 * math.pi,
        ]
        js.velocity = [left_vel * 2.0 * math.pi / 60.0,
                       right_vel * 2.0 * math.pi / 60.0]
        self.joint_pub.publish(js)

    # ══════════════════════════════════════════════════════════════════════════
    # SHUTDOWN
    # ══════════════════════════════════════════════════════════════════════════

    def shutdown(self):
        self.get_logger().info('Stopping motors...')
        self.set_velocity(LEFT_ID,  0.0)
        self.set_velocity(RIGHT_ID, 0.0)
        for mid in [LEFT_ID, RIGHT_ID]:
            self.packet.write1ByteTxRx(self.port, mid, ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        self.get_logger().info('Done')


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()