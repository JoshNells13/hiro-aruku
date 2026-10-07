#!/usr/bin/env python3
"""
Interactive Teleoperation Node untuk Robot Humanoid Hiro MuJoCo (ROS 2 Jazzy).

Node ini membaca input keyboard pengguna dari terminal dan:
1. Mengirimkan target sudut sendi ke topik /op3/named_joint_command.
2. Mengirimkan perintah berjalan ke topik /walking/set_walking (Aruku walking engine).
3. Mengontrol simulasi (reset, pause).
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

try:
    from aruku_interfaces.msg import SetWalking
    ARUKU_AVAILABLE = True
except ImportError:
    ARUKU_AVAILABLE = False


USAGE_BANNER = """
============================================================
       Hiro Humanoid MuJoCo & Aruku Teleoperation
============================================================
 Kontrol Kepala:
   [W] : Tilt Atas        [S] : Tilt Bawah
   [A] : Yaw Kiri         [D] : Yaw Kanan

 Kontrol Lengan:
   [U] : Angkat Bahu Kiri [J] : Turunkan Bahu Kiri
   [I] : Angkat Bahu Kanan[K] : Turunkan Bahu Kanan
   [O] : Tekuk Siku Kiri  [L] : Tekuk Siku Kanan

 Kontrol Berjalan (Aruku Walking Engine):
   [8] / [T] : Maju (Walk Forward)
   [2] / [G] : Mundur (Walk Backward)
   [4] / [F] : Putar Kiri (Turn Left)
   [6] / [H] : Putar Kanan (Turn Right)
   [C]       : Jalan di Tempat (Walk in Place)
   [5] / [X] : Berhenti Berjalan (Stop Walking)

 Preset Pose:
   [1] : Berdiri Seimbang (Stand Pose)
   [3] : Posisi Siap / Jongkok (Squat Ready)
   [0] : Reset Semua Sudut ke 0 (Zero All)

 Fitur Simulasi:
   [R] : Reset Simulasi Fisika MuJoCo
   [P] : Jeda / Lanjutkan Simulasi (Pause/Resume)
   [Q] : Keluar (Quit)
============================================================
"""


class HiroTeleopNode(Node):
    def __init__(self):
        super().__init__('hiro_teleop_controller')

        self.cmd_pub = self.create_publisher(JointState, '/op3/named_joint_command', 10)
        self.reset_cli = self.create_client(Trigger, '/reset_simulation')
        self.pause_cli = self.create_client(SetBool, '/pause_simulation')

        if ARUKU_AVAILABLE:
            self.walking_pub = self.create_publisher(SetWalking, '/walking/set_walking', 10)
            self.get_logger().info('Publisher /walking/set_walking aktif (Aruku walking engine).')

        self.step_size = 0.08  # radian per penekanan tombol
        self.paused = False

        # State internal posisi sudut sendi Hiro
        self.current_angles = {
            'neck_yaw': 0.0,
            'neck_pitch': 0.0,
            'left_shoulder_pitch': -0.1,
            'left_shoulder_roll': 0.25,
            'left_elbow': -0.5,
            'right_shoulder_pitch': 0.1,
            'right_shoulder_roll': -0.25,
            'right_elbow': 0.5,
            'left_hip_yaw': 0.0,
            'left_hip_roll': 0.0,
            'left_hip_pitch': 0.0,
            'left_knee': 0.0,
            'left_ankle_pitch': 0.0,
            'left_ankle_roll': 0.0,
            'right_hip_yaw': 0.0,
            'right_hip_roll': 0.0,
            'right_hip_pitch': 0.0,
            'right_knee': 0.0,
            'right_ankle_pitch': 0.0,
            'right_ankle_roll': 0.0,
        }

    def send_joint_command(self, joint_name: str, delta: float):
        if joint_name in self.current_angles:
            self.current_angles[joint_name] += delta
            self.current_angles[joint_name] = max(-2.5, min(2.5, self.current_angles[joint_name]))

            msg = JointState()
            msg.name = [joint_name]
            msg.position = [self.current_angles[joint_name]]
            self.cmd_pub.publish(msg)
            print(f"\r-> Sendi {joint_name}: {self.current_angles[joint_name]:.2f} rad      ", end='', flush=True)

    def send_full_pose(self, pose_dict: dict, label: str):
        msg = JointState()
        for k, v in pose_dict.items():
            if k in self.current_angles:
                self.current_angles[k] = v
                msg.name.append(k)
                msg.position.append(v)
        self.cmd_pub.publish(msg)
        print(f"\r-> Pose Diaktifkan: [{label}]                     ", end='', flush=True)

    def send_walk_cmd(self, run: bool, x: float = 0.0, y: float = 0.0, a: float = 0.0):
        if not ARUKU_AVAILABLE:
            print("\r-> aruku_interfaces tidak tersedia untuk berjalan! ", end='', flush=True)
            return

        msg = SetWalking()
        msg.run = run
        msg.x_move = float(x)
        msg.y_move = float(y)
        msg.a_move = float(a)
        msg.aim_on = False
        self.walking_pub.publish(msg)

        status_str = f"Run={run}, X={x:.1f}, Y={y:.1f}, A={a:.1f}"
        print(f"\r-> Perintah Jalan: [{status_str}]                 ", end='', flush=True)

    def apply_stand(self):
        self.send_full_pose({
            'neck_yaw': 0.0, 'neck_pitch': 0.0,
            'left_shoulder_pitch': -0.1, 'left_shoulder_roll': 0.25, 'left_elbow': -0.5,
            'right_shoulder_pitch': 0.1, 'right_shoulder_roll': -0.25, 'right_elbow': 0.5,
            'left_hip_pitch': 0.0, 'left_knee': 0.0, 'left_ankle_pitch': 0.0,
            'right_hip_pitch': 0.0, 'right_knee': 0.0, 'right_ankle_pitch': 0.0,
        }, "Stand Pose")

    def apply_squat(self):
        self.send_full_pose({
            'neck_yaw': 0.0, 'neck_pitch': 0.1,
            'left_hip_pitch': 0.3, 'left_knee': 0.6, 'left_ankle_pitch': -0.3,
            'right_hip_pitch': 0.3, 'right_knee': 0.6, 'right_ankle_pitch': -0.3,
        }, "Squat Ready Pose")

    def apply_zero(self):
        zeroes = {k: 0.0 for k in self.current_angles.keys()}
        self.send_full_pose(zeroes, "Zero All Joints")

    def call_reset(self):
        if self.reset_cli.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.reset_cli.call_async(req)
            print("\r-> Request reset simulasi terkirim.       ", end='', flush=True)
        else:
            print("\r-> Service /reset_simulation belum aktif! ", end='', flush=True)

    def toggle_pause(self):
        if self.pause_cli.wait_for_service(timeout_sec=1.0):
            self.paused = not self.paused
            req = SetBool.Request()
            req.data = self.paused
            self.pause_cli.call_async(req)
            status = "PAUSE" if self.paused else "RESUME"
            print(f"\r-> Simulasi: {status}                      ", end='', flush=True)


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
    teleop = HiroTeleopNode()

    settings = termios.tcgetattr(sys.stdin)
    print(USAGE_BANNER)

    try:
        while rclpy.ok():
            rclpy.spin_once(teleop, timeout_sec=0.01)
            key = get_key(settings)
            if not key:
                continue

            k = key.lower()
            # Kontrol Kepala
            if k == 'w':
                teleop.send_joint_command('neck_pitch', -teleop.step_size)
            elif k == 's':
                teleop.send_joint_command('neck_pitch', teleop.step_size)
            elif k == 'a':
                teleop.send_joint_command('neck_yaw', teleop.step_size)
            elif k == 'd':
                teleop.send_joint_command('neck_yaw', -teleop.step_size)

            # Kontrol Lengan
            elif k == 'u':
                teleop.send_joint_command('left_shoulder_pitch', -teleop.step_size)
            elif k == 'j':
                teleop.send_joint_command('left_shoulder_pitch', teleop.step_size)
            elif k == 'i':
                teleop.send_joint_command('right_shoulder_pitch', teleop.step_size)
            elif k == 'k':
                teleop.send_joint_command('right_shoulder_pitch', -teleop.step_size)
            elif k == 'o':
                teleop.send_joint_command('left_elbow', teleop.step_size)
            elif k == 'l':
                teleop.send_joint_command('right_elbow', teleop.step_size)

            # Kontrol Berjalan (Aruku)
            elif k == '8' or k == 't':
                teleop.send_walk_cmd(run=True, x=15.0, y=0.0, a=0.0)
            elif k == '2' or k == 'g':
                teleop.send_walk_cmd(run=True, x=-15.0, y=0.0, a=0.0)
            elif k == '4' or k == 'f':
                teleop.send_walk_cmd(run=True, x=0.0, y=0.0, a=10.0)
            elif k == '6' or k == 'h':
                teleop.send_walk_cmd(run=True, x=0.0, y=0.0, a=-10.0)
            elif k == 'c':
                teleop.send_walk_cmd(run=True, x=0.0, y=0.0, a=0.0)
            elif k == '5' or k == 'x':
                teleop.send_walk_cmd(run=False, x=0.0, y=0.0, a=0.0)

            # Preset Pose
            elif k == '1':
                teleop.apply_stand()
            elif k == '3':
                teleop.apply_squat()
            elif k == '0':
                teleop.apply_zero()

            # Kontrol Simulasi
            elif k == 'r':
                teleop.call_reset()
            elif k == 'p':
                teleop.toggle_pause()
            elif k == 'q' or key == '\x03':  # Ctrl-C
                print("\nMenutup teleop controller.")
                break

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
