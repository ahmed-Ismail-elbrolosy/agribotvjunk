#!/usr/bin/env python3
"""Keyboard teleoperation for AgriBot — WASD + speed control."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

HELP = """
AgriBot Teleop
--------------
   w
 a s d

w/s : forward/backward
a/d : rotate left/right
t/y : increase/decrease linear speed
g/h : increase/decrease angular speed
space/k : stop
CTRL-C  : quit
"""

MOVE = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
}

SPEED = {
    't': (0.5, 0.0),
    'y': (-0.5, 0.0),
    'g': (0.0, 0.5),
    'h': (0.0, -0.5),
}


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_node')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)

    speed, turn = 0.5, 1.0
    x = th = 0.0

    try:
        print(HELP)
        print(f"Speed: {speed}  Turn: {turn}")
        while True:
            key = get_key(settings)
            if key in MOVE:
                x, _, _, th = MOVE[key]
            elif key in SPEED:
                speed = max(0.1, speed + SPEED[key][0])
                turn = max(0.1, turn + SPEED[key][1])
                print(f"Speed: {speed:.1f}  Turn: {turn:.1f}")
                x = th = 0.0
            else:
                x = th = 0.0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    finally:
        pub.publish(Twist())  # stop
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
