#!/usr/bin/env python3
"""On plandaki terminalden tek tus alip ROS teleop topic'ine iletir."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


VALID_KEYS = frozenset('wadsrfcvWADSRFCV ')


def main(args=None):
    rclpy.init(args=args)
    node = Node('keyboard_input')
    key_pub = node.create_publisher(String, '/sara/teleop/key', 10)
    alive_pub = node.create_publisher(
        Bool, '/sara/teleop/keyboard_alive', 10)

    if not sys.stdin.isatty():
        node.get_logger().error(
            'Bu dugum on plandaki gercek bir terminalde calistirilmalidir.')
        node.destroy_node()
        rclpy.shutdown()
        return

    fd = sys.stdin.fileno()
    previous_settings = termios.tcgetattr(fd)
    last_alive = 0.0
    node.get_logger().warning(
        'KLAVYE GIRISI HAZIR: W/A/D hareket, S/SPACE dur, '
        'R/F derinlik, C referans, V yuzeye cikis. Ctrl+C ile kapat.')

    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            now = time.monotonic()
            if now - last_alive >= 0.2:
                alive_pub.publish(Bool(data=True))
                last_alive = now

            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key in VALID_KEYS:
                    key_pub.publish(String(data=key))
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        stop = String(data='s')
        for _ in range(3):
            key_pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.02)
        termios.tcsetattr(fd, termios.TCSADRAIN, previous_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

