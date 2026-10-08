#!/usr/bin/env python3
"""
Node teleoperasi keyboard sederhana untuk mengontrol gerakan berjalan robot (aruku).

Kontrol:
    [W] : Jalan Maju
    [S] : Jalan Mundur
    [A] : Putar Kiri
    [D] : Putar Kanan
    [Q] : Geser Kiri (Strafe)
    [E] : Geser Kanan (Strafe)
    [SPASI] / [X] : Berhenti Jalan (Tegak)
    [Ctrl+C] : Keluar
"""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from aruku_interfaces.msg import SetWalking

HELP_MSG = """
============================================================
           Kontrol Berjalan Robot Hiro (Aruku)
============================================================
   [W] : Maju                [S] : Mundur
   [A] : Putar Kiri          [D] : Putar Kanan
   [Q] : Geser Kiri          [E] : Geser Kanan
   [SPASI] / [X] : Berhenti (Pose Tegak)
   [Ctrl+C]      : Keluar
============================================================
"""


class WalkTeleopNode(Node):

    def __init__(self):
        super().__init__('walk_teleop_node')
        self.pub = self.create_publisher(SetWalking, '/walking/set_walking', 10)
        # Tentukan step size standar yang terbukti stabil saat mundur
        self.default_step = 10.0  # mm
        self.get_logger().info('Node walk_teleop siap.')

    def send_cmd(self, run: bool, x: float = 0.0, y: float = 0.0, a: float = 0.0):
        msg = SetWalking()
        msg.run = run
        msg.x_move = float(x)
        msg.y_move = float(y)
        msg.a_move = float(a)
        msg.aim_on = False
        self.pub.publish(msg)
        if run:
            print(f'\r>> Status: BERJALAN | Maju/Mundur: {x:+.1f} mm | Geser: {y:+.1f} mm | Putar: {a:+.1f} deg   ', end='', flush=True)
        else:
            print('\r>> Status: BERHENTI (Robot Tegak Diam)      ', end='', flush=True)

    # --- Method pembantu agar pemanggilan maju dan mundur konsisten ---
    def move_forward(self, step_mm: float = None):
        """Maju dengan besaran langkah yang sama seperti mundur."""
        step = step_mm if step_mm is not None else self.default_step
        self.send_cmd(run=True, x=abs(step))

    def move_backward(self, step_mm: float = None):
        """Mundur dengan membalik tanda step."""
        step = step_mm if step_mm is not None else self.default_step
        self.send_cmd(run=True, x=-abs(step))

    def stop(self):
        """Hentikan robot dan kunci posisi diam."""
        self.send_cmd(run=False, x=0.0, y=0.0, a=0.0)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = WalkTeleopNode()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP_MSG)

    x_step = node.default_step  # 10.0 mm (stabil untuk maju & mundur)
    y_step = 10.0
    a_step = 10.0

    try:
        while rclpy.ok():
            key = get_key(settings)
            if not key:
                continue

            k = key.lower()
            if k == 'w':
                node.move_forward(x_step)
            elif k == 's':
                node.move_backward(x_step)
            elif k == 'a':
                node.send_cmd(True, x=0.0, y=0.0, a=a_step)
            elif k == 'd':
                node.send_cmd(True, x=0.0, y=0.0, a=-a_step)
            elif k == 'q':
                node.send_cmd(True, x=0.0, y=y_step, a=0.0)
            elif k == 'e':
                node.send_cmd(True, x=0.0, y=-y_step, a=0.0)
            elif k in [' ', 'x']:
                node.stop()
            elif key == '\x03':  # Ctrl+C
                break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.send_cmd(False, 0.0, 0.0, 0.0)
        print('\nTeleoperasi dimatikan.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
