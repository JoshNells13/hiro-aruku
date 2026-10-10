#!/usr/bin/env python3
"""
Node teleoperasi keyboard untuk mengontrol robot berjalan (aruku)
sekaligus otomatis mem-publish pose tangan ke bawah via /joint/set_joints.

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
from aruku_interfaces.msg import SetWalking, SetJoints
try:
    from aruku_interfaces.msg import Joint
except ImportError:
    # Alternatif jika sub-message dinamai SetJoint
    from aruku_interfaces.msg import SetJoint as Joint

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
* Catatan: Pose lengan otomatis dikirim ke bawah secara periodik.
"""

# Konfigurasi ID sendi tangan agar jatuh lurus ke samping badan:
# ID 1: right_shoulder_pitch (-75.0 deg)
# ID 2: left_shoulder_pitch (75.0 deg)
# ID 3: right_shoulder_roll (-15.0 deg)
# ID 4: left_shoulder_roll (-15.0 deg)
# ID 5: right_elbow (0.0 deg)
# ID 6: left_elbow (0.0 deg)
ARM_TARGET_JOINTS = {
    1: -75.0,
    2: 75.0,
    3: -15.0,
    4: -15.0,
    5: 0.0,
    6: 0.0,
}


class WalkTeleopNode(Node):

    def __init__(self):
        super().__init__('walk_teleop_node')
        self.pub_walking = self.create_publisher(SetWalking, '/walking/set_walking', 10)
        self.pub_joints = self.create_publisher(SetJoints, '/joint/set_joints', 10)

        self.default_step = 10.0  # mm

        # Timer 10 Hz untuk terus menahan posisi tangan ke bawah
        self.arm_timer = self.create_timer(0.1, self.publish_arms_down)

        self.get_logger().info('Node walk_teleop siap (kontrol tangan otomatis aktif).')

    def publish_arms_down(self):
        """Kirim target posisi tangan ke bawah secara berkala."""
        msg = SetJoints()
        joints_list = []
        for j_id, pos in ARM_TARGET_JOINTS.items():
            j = Joint()
            j.id = int(j_id)
            j.position = float(pos)
            joints_list.append(j)

        msg.joints = joints_list
        self.pub_joints.publish(msg)

    def send_cmd(self, run: bool, x: float = 0.0, y: float = 0.0, a: float = 0.0):
        msg = SetWalking()
        msg.run = run
        msg.x_move = float(x)
        msg.y_move = float(y)
        msg.a_move = float(a)
        msg.aim_on = False
        self.pub_walking.publish(msg)
        if run:
            print(f'\r>> Status: BERJALAN | Maju/Mundur: {x:+.1f} mm | Geser: {y:+.1f} mm | Putar: {a:+.1f} deg   ', end='', flush=True)
        else:
            print('\r>> Status: BERHENTI (Robot Tegak Diam)      ', end='', flush=True)

    def move_forward(self, step_mm: float = None):
        step = step_mm if step_mm is not None else self.default_step
        self.send_cmd(run=True, x=abs(step))

    def move_backward(self, step_mm: float = None):
        step = step_mm if step_mm is not None else self.default_step
        self.send_cmd(run=True, x=-abs(step))

    def stop(self):
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

    x_step = node.default_step
    y_step = 10.0
    a_step = 10.0

    try:
        while rclpy.ok():
            # Spin sekali non-blocking agar timer ROS (publish_arms_down) tetap terpicu
            rclpy.spin_once(node, timeout_sec=0.0)

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