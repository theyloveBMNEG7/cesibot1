#!/usr/bin/env python3

import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading

# ── Key bindings for AZERTY layout ──
KEY_FORWARD   = 'z'
KEY_BACKWARD  = 's'
KEY_LEFT      = 'd'
KEY_RIGHT     = 'q'
KEY_STOP      = ' '

KEY_SPEED_UP  = 'a'
KEY_SPEED_DN  = 'e'
KEY_TURN_UP   = 'r'
KEY_TURN_DN   = 'f'

KEY_QUIT      = '\x03'  # Ctrl+C

HELP_MSG = """
╔══════════════════════════════════════╗
║     AZERTY Robot Keyboard Control    ║ 
╠══════════════════════════════════════╣
║  Z          →  Forward               ║
║  S          →  Backward              ║
║  D          →  Rotate Left           ║
║  Q          →  Rotate Right          ║
║  Z + D/Q    →  Curve Left/Right      ║
║  SPACE      →  EMERGENCY STOP        ║
╠══════════════════════════════════════╣
║  A / E      →  Speed Up / Down       ║
║  R / F      →  Turn Speed Up / Down  ║
╠══════════════════════════════════════╣
║  Ctrl+C     →  Quit                  ║
╚══════════════════════════════════════╝
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # ── Publisher ──
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Speed settings ──
        self.linear_speed  = 0.2   # m/s
        self.angular_speed = 1.0   # rad/s

        self.linear_step   = 0.05  # how much each press changes speed
        self.angular_step  = 0.1

        self.max_linear    = 0.5
        self.max_angular   = 2.0
        self.min_speed     = 0.05

        # ── State ──
        self.forward  = False
        self.backward = False
        self.left     = False
        self.right    = False

        # ── Timer: publishes cmd_vel at 20Hz ──
        self.timer = self.create_timer(0.05, self.publish_velocity)

        print(HELP_MSG)
        self._print_speeds()

    def _print_speeds(self):
        print(f'  Linear speed : {self.linear_speed:.2f} m/s   '
              f'Angular speed: {self.angular_speed:.2f} rad/s')

    def process_key(self, key):
        """Called every time a key is pressed."""

        # ── Movement keys — set state ──
        if key == KEY_FORWARD:
            self.forward  = True
            self.backward = False

        elif key == KEY_BACKWARD:
            self.backward = True
            self.forward  = False

        elif key == KEY_LEFT:
            self.left  = True
            self.right = False

        elif key == KEY_RIGHT:
            self.right = True
            self.left  = False

        # ── Stop ──
        elif key == KEY_STOP:
            self._stop_all()
            print('  !! EMERGENCY STOP !!')

        # ── Speed controls ──
        elif key == KEY_SPEED_UP:
            self.linear_speed = min(self.linear_speed + self.linear_step,
                                    self.max_linear)
            self._print_speeds()

        elif key == KEY_SPEED_DN:
            self.linear_speed = max(self.linear_speed - self.linear_step,
                                    self.min_speed)
            self._print_speeds()

        elif key == KEY_TURN_UP:
            self.angular_speed = min(self.angular_speed + self.angular_step,
                                     self.max_angular)
            self._print_speeds()

        elif key == KEY_TURN_DN:
            self.angular_speed = max(self.angular_speed - self.angular_step,
                                     self.min_speed)
            self._print_speeds()

        # ── Quit ──
        elif key == KEY_QUIT:
            self._stop_all()
            raise KeyboardInterrupt

    def key_released(self, key):
        """Stop movement when key is released."""
        if key == KEY_FORWARD:
            self.forward  = False
        elif key == KEY_BACKWARD:
            self.backward = False
        elif key == KEY_LEFT:
            self.left  = False
        elif key == KEY_RIGHT:
            self.right = False

    def _stop_all(self):
        self.forward  = False
        self.backward = False
        self.left     = False
        self.right    = False

    def publish_velocity(self):
        """Runs at 20Hz — builds and sends Twist message from current state."""
        msg = Twist()

        if self.forward:
            msg.linear.x = self.linear_speed
        elif self.backward:
            msg.linear.x = -self.linear_speed

        if self.left:
            msg.angular.z = self.angular_speed
        elif self.right:
            msg.angular.z = -self.angular_speed

        self.pub.publish(msg)


def get_key(settings, timeout=0.1):
    """Read a single keypress with timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _,_ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''    
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    settings = termios.tcgetattr(sys.stdin)

    # ── Spin ROS2 in background thread ──
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)
            if key == '':
                node._stop_all()
            else:
                node.process_key(key.lower())

    except KeyboardInterrupt:
        pass

    finally:
        # Send stop before quitting
        stop = Twist()
        node.pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()