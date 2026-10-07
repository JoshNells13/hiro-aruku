#!/usr/bin/env python3
"""
Motion Player Node untuk Robot Humanoid Hiro di MuJoCo (ROS 2 Jazzy).

Node ini memutar animasi gerakan humanoid terinterpolasi halus (wave, squat)
dengan mengirimkan sekuens target sendi ke topik /op3/named_joint_command.
"""

import sys
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HiroMotionPlayer(Node):
    def __init__(self):
        super().__init__('hiro_motion_player')
        self.cmd_pub = self.create_publisher(JointState, '/op3/named_joint_command', 10)
        self.get_logger().info('hiro_motion_player siap memutar animasi gerakan.')

    def send_named_joints(self, joint_dict: dict):
        msg = JointState()
        for name, pos in joint_dict.items():
            msg.name.append(name)
            msg.position.append(float(pos))
        self.cmd_pub.publish(msg)

    def interpolate_and_play(self, waypoints: list, step_delay: float = 0.02, steps_per_segment: int = 40):
        """Melakukan interpolasi linear antar waypoint gerakan agar halus."""
        for i in range(len(waypoints) - 1):
            p_start = waypoints[i]
            p_end = waypoints[i + 1]

            all_keys = set(p_start.keys()).union(set(p_end.keys()))

            for s in range(steps_per_segment + 1):
                if not rclpy.ok():
                    return
                alpha = s / float(steps_per_segment)
                # Cosine smoothing (ease-in ease-out)
                smooth_alpha = 0.5 * (1.0 - math.cos(alpha * math.pi))

                interp = {}
                for k in all_keys:
                    v0 = p_start.get(k, 0.0)
                    v1 = p_end.get(k, 0.0)
                    interp[k] = v0 + (v1 - v0) * smooth_alpha

                self.send_named_joints(interp)
                time.sleep(step_delay)

    def play_wave_demo(self, cycles: int = 3):
        """Memutar animasi lambaian tangan salam robot Hiro."""
        self.get_logger().info(f'Memulai animasi Melambaikan Tangan ({cycles} siklus)...')

        stand = {
            'neck_yaw': 0.0, 'neck_pitch': 0.0,
            'left_shoulder_pitch': -0.1, 'left_shoulder_roll': 0.25, 'left_elbow': -0.5,
            'right_shoulder_pitch': 0.1, 'right_shoulder_roll': -0.25, 'right_elbow': 0.5,
        }
        raise_hand = {
            'neck_yaw': -0.25, 'neck_pitch': 0.1,
            'right_shoulder_pitch': 1.4, 'right_shoulder_roll': -0.9, 'right_elbow': 1.2,
        }
        wave_out = {
            'right_shoulder_roll': -0.6,
            'right_elbow': 0.8,
        }
        wave_in = {
            'right_shoulder_roll': -1.2,
            'right_elbow': 1.4,
        }

        # Angkat tangan
        self.interpolate_and_play([stand, raise_hand], steps_per_segment=50)

        # Lambaikan tangan bolak-balik
        for c in range(cycles):
            self.interpolate_and_play([raise_hand, wave_out, wave_in, raise_hand], steps_per_segment=30)

        # Kembalikan ke posisi berdiri
        self.interpolate_and_play([raise_hand, stand], steps_per_segment=50)
        self.get_logger().info('Animasi melambaikan tangan selesai.')

    def play_squat_demo(self, cycles: int = 2):
        """Memutar animasi jongkok dan berdiri robot Hiro."""
        self.get_logger().info(f'Memulai animasi Squat ({cycles} siklus)...')
        stand = {
            'left_hip_pitch': 0.0, 'left_knee': 0.0, 'left_ankle_pitch': 0.0,
            'right_hip_pitch': 0.0, 'right_knee': 0.0, 'right_ankle_pitch': 0.0,
        }
        squat = {
            'left_hip_pitch': 0.35, 'left_knee': 0.7, 'left_ankle_pitch': -0.35,
            'right_hip_pitch': 0.35, 'right_knee': 0.7, 'right_ankle_pitch': -0.35,
        }
        for _ in range(cycles):
            self.interpolate_and_play([stand, squat], steps_per_segment=60)
            time.sleep(0.5)
            self.interpolate_and_play([squat, stand], steps_per_segment=60)
            time.sleep(0.5)
        self.get_logger().info('Animasi squat selesai.')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Hiro Humanoid Motion Player Demo')
    parser.add_argument('--motion', type=str, default='wave', choices=['wave', 'squat', 'all'],
                        help='Gerakan yang ingin diputar: wave, squat, atau all')
    parser.add_argument('--cycles', type=int, default=3, help='Jumlah perulangan gerakan')
    parsed_args, _ = parser.parse_known_args()

    player = HiroMotionPlayer()

    # Beri jeda 1 detik agar subscriber MuJoCo siap
    time.sleep(1.0)

    try:
        if parsed_args.motion == 'wave' or parsed_args.motion == 'all':
            player.play_wave_demo(cycles=parsed_args.cycles)
        if parsed_args.motion == 'squat' or parsed_args.motion == 'all':
            player.play_squat_demo(cycles=parsed_args.cycles)
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
