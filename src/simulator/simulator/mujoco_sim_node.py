#!/usr/bin/env python3
"""
Node simulasi MuJoCo untuk robot humanoid Hiro + walking engine aruku.

Alur data:
    aruku  --/joint/set_joints-->  sim (posisi servo, derajat)
    sim    --/walking/imu------->  aruku (roll/pitch/yaw badan, derajat)

Pose berdiri awal dibaca dinamis dari key "init_angles" pada walking.json.
"""

import json
import math
import os
import threading
import time

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from aruku_interfaces.msg import Imu, SetJoints

try:
    import mujoco.viewer
    VIEWER_AVAILABLE = True
except ImportError:
    VIEWER_AVAILABLE = False


# ID servo aruku -> (nama joint MuJoCo, arah). Sudut MuJoCo = radians(deg) * arah
SERVO_MAP = {
    1: ('right_shoulder_pitch', -1.0),
    2: ('left_shoulder_pitch', 1.0),
    3: ('right_shoulder_roll', -1.0),
    4: ('left_shoulder_roll', 1.0),
    5: ('right_elbow', -1.0),
    6: ('left_elbow', -1.0),
    7: ('right_hip_yaw', -1.0),
    8: ('left_hip_yaw', -1.0),
    9: ('right_hip_roll', -1.0),
    10: ('left_hip_roll', -1.0),
    11: ('right_hip_pitch', 1.0),
    12: ('left_hip_pitch', -1.0),
    13: ('right_knee', 1.0),
    14: ('left_knee', -1.0),
    15: ('right_ankle_pitch', -1.0),
    16: ('left_ankle_pitch', 1.0),
    17: ('right_ankle_roll', 1.0),
    18: ('left_ankle_roll', 1.0),
    19: ('neck_yaw', 1.0),
    20: ('neck_pitch', 1.0),
}

JOINT_TO_SERVO_ID = {name: sid for sid, (name, _) in SERVO_MAP.items()}

# Pose default natural: bahu turun (~45-60 deg) dan siku sedikit menekuk
DEFAULT_STAND_POSE = {
    1: -45.0,  # right_shoulder_pitch
    2: 45.0,   # left_shoulder_pitch
    3: 10.0,   # right_shoulder_roll
    4: -10.0,  # left_shoulder_roll
    5: -30.0,  # right_elbow
    6: 30.0,   # left_elbow
    7: 0.0,
    8: 0.0,
    9: 0.0,
    10: 0.0,
    11: -8.17,
    12: 8.17,
    13: 17.93,
    14: -17.93,
    15: 9.76,
    16: -9.76,
    17: 0.0,
    18: 0.0,
    19: 0.0,
    20: 0.0,
}


def load_stand_pose_from_json(json_path: str, logger=None) -> dict:
    """Membaca pose awal dari init_angles di walking.json."""
    pose = DEFAULT_STAND_POSE.copy()
    if not os.path.isfile(json_path):
        if logger:
            logger.warn(f'File walking config tidak ditemukan di: {json_path}. Menggunakan default pose.')
        return pose

    try:
        with open(json_path, 'r') as f:
            cfg = json.load(f)

        init_angles = cfg.get("init_angles", {})

        if init_angles:
            for joint_name, deg_val in init_angles.items():
                if joint_name in JOINT_TO_SERVO_ID:
                    servo_id = JOINT_TO_SERVO_ID[joint_name]
                    pose[servo_id] = float(deg_val)

            if logger:
                logger.info(f'Berhasil memuat pose awal dari key "init_angles" di: {json_path}')
        else:
            if logger:
                logger.warn(f'Key "init_angles" tidak ditemukan pada {json_path}. Memakai default.')

    except Exception as e:
        if logger:
            logger.error(f'Gagal mem-parsing walking.json ({e}). Menggunakan default pose.')

    return pose


class MujocoSimNode(Node):

    def __init__(self):
        super().__init__('mujoco_sim_node')

        try:
            default_model = os.path.join(
                get_package_share_directory('simulator'), 'models', 'scene.xml')
        except Exception:
            default_model = os.path.abspath(os.path.join(
                os.path.dirname(__file__), '..', 'models', 'scene.xml'))

        try:
            default_walking_json = os.path.join(
                get_package_share_directory('aruku'), 'config', 'walking.json')
        except Exception:
            default_walking_json = os.path.abspath(os.path.join(
                os.path.dirname(__file__), '..', 'config', 'walking.json'))

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('walking_config_path', default_walking_json)
        self.declare_parameter('use_gui', True)
        self.declare_parameter('imu_rate', 100.0)
        self.declare_parameter('ignore_arm_from_aruku', True)

        model_path = self.get_parameter('model_path').value or default_model
        walking_json_path = self.get_parameter('walking_config_path').value or default_walking_json
        self.use_gui = self.get_parameter('use_gui').value
        imu_rate = self.get_parameter('imu_rate').value
        self.ignore_arm_from_aruku = self.get_parameter('ignore_arm_from_aruku').value

        # Deteksi status aktif aruku
        self.last_aruku_msg_time = 0.0
        self.aruku_timeout_sec = 0.5  # Detik toleransi sebelum aruku dinyatakan mati

        self.get_logger().info(f'Memuat model MuJoCo: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()

        self.stand_pose = load_stand_pose_from_json(walking_json_path, self.get_logger())
        self.torso_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso_kidsize25')

        self.servos = {}
        for servo_id, (name, sign) in SERVO_MAP.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if jid < 0 or aid < 0:
                self.get_logger().warn(f'Joint/actuator "{name}" tidak ada di model')
                continue
            self.servos[servo_id] = (self.model.jnt_qposadr[jid], aid, sign)

        self._setup_standing_pose()

        self.create_subscription(SetJoints, '/joint/set_joints', self._on_set_joints, 10)
        self.imu_pub = self.create_publisher(Imu, '/walking/imu', 10)
        self.create_timer(1.0 / imu_rate, self._publish_imu)

        self.get_logger().info('Simulasi siap. Robot berdiri tegak, menunggu aruku...')

    @property
    def is_aruku_alive(self) -> bool:
        """Cek apakah aruku sedang aktif mempublikasikan joint."""
        return (time.time() - self.last_aruku_msg_time) < self.aruku_timeout_sec

    def _setup_standing_pose(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)

        d.qpos[0:3] = [0.0, 0.0, 0.5]
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        for servo_id, deg in self.stand_pose.items():
            if servo_id in self.servos:
                qadr, aid, sign = self.servos[servo_id]
                d.qpos[qadr] = math.radians(deg) * sign
                d.ctrl[aid] = d.qpos[qadr]

        mujoco.mj_forward(m, d)
        d.qpos[2] -= self._lowest_point_z() - 0.0005
        mujoco.mj_forward(m, d)

        m.qpos0[:] = d.qpos
        self.stand_ctrl = d.ctrl.copy()

    def _lowest_point_z(self):
        m, d = self.model, self.data
        lowest = np.inf
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
                continue
            mesh = m.geom_dataid[g]
            start, num = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            verts = m.mesh_vert[start:start + num]
            world_z = verts @ d.geom_xmat[g].reshape(3, 3)[2] + d.geom_xpos[g][2]
            lowest = min(lowest, world_z.min())
        return lowest

    def reset(self):
        if self.is_aruku_alive:
            self.get_logger().warn('Reset ditolak: matikan node aruku terlebih dahulu.')
            return

        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.data.ctrl[:] = self.stand_ctrl

    def manual_set_joint(self, servo_id: int, deg_val: float):
        """Method untuk mengubah posisi joint secara manual (hanya jika aruku mati)."""
        if self.is_aruku_alive:
            self.get_logger().warn_throttle(
                2.0, 'Kontrol manual ditolak: node aruku sedang aktif.'
            )
            return

        with self.lock:
            if servo_id in self.servos:
                _, aid, sign = self.servos[servo_id]
                self.data.ctrl[aid] = math.radians(deg_val) * sign

    def _on_set_joints(self, msg: SetJoints):
        # Update heartbeat aruku
        self.last_aruku_msg_time = time.time()

        with self.lock:
            for joint in msg.joints:
                if joint.id not in self.servos:
                    continue

                _, aid, sign = self.servos[joint.id]
                self.data.ctrl[aid] = math.radians(joint.position) * sign

    def _publish_imu(self):
        with self.lock:
            R = self.data.xmat[self.torso_id].reshape(3, 3)
            roll = math.atan2(-R[2, 0], R[2, 2])
            pitch = math.atan2(R[2, 1], R[2, 2])
            yaw = math.atan2(R[1, 0], R[0, 0])

        msg = Imu()
        msg.roll = math.degrees(roll)
        msg.pitch = math.degrees(pitch)
        msg.yaw = math.degrees(yaw)
        self.imu_pub.publish(msg)

    def step(self):
        with self.lock:
            # Jika aruku aktif, bersihkan gaya luar/tarikan mouse agar tidak bisa digerakkan dari GUI
            if self.is_aruku_alive:
                self.data.xfrc_applied[:] = 0.0
                self.data.qfrc_applied[:] = 0.0

            mujoco.mj_step(self.model, self.data)


def run_realtime(node, viewer=None):
    dt = node.model.opt.timestep
    sim_start = node.data.time
    wall_start = time.perf_counter()
    last_sync = 0.0

    while rclpy.ok() and (viewer is None or viewer.is_running()):
        # Jika aruku aktif, reset interaksi mouse/perturbasi viewer secara aman
        if viewer is not None and node.is_aruku_alive:
            if hasattr(viewer, '_pert'):
                viewer._pert.active = 0

        node.step()

        now = time.perf_counter()
        if viewer is not None and now - last_sync > 1.0 / 60.0:
            viewer.sync()
            last_sync = now

        ahead = (node.data.time - sim_start) - (now - wall_start)
        if ahead > dt:
            time.sleep(ahead)
        elif ahead < -0.5:
            sim_start, wall_start = node.data.time, now
    dt = node.model.opt.timestep
    sim_start = node.data.time
    wall_start = time.perf_counter()
    last_sync = 0.0

    while rclpy.ok() and (viewer is None or viewer.is_running()):
        # Matikan perturbasi klik mouse di viewer jika aruku berjalan
        if viewer is not None and node.is_aruku_alive:
            viewer.pert.active = 0

        node.step()

        now = time.perf_counter()
        if viewer is not None and now - last_sync > 1.0 / 60.0:
            viewer.sync()
            last_sync = now

        ahead = (node.data.time - sim_start) - (now - wall_start)
        if ahead > dt:
            time.sleep(ahead)
        elif ahead < -0.5:
            sim_start, wall_start = node.data.time, now


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        if node.use_gui and VIEWER_AVAILABLE:
            with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
                viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
                viewer.cam.distance = 1.2
                viewer.cam.elevation = -15.0
                viewer.cam.azimuth = 135.0
                run_realtime(node, viewer)
        else:
            node.get_logger().info('Mode headless (tanpa GUI).')
            run_realtime(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()