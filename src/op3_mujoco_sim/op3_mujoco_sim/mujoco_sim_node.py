#!/usr/bin/env python3
"""
Node simulasi MuJoCo untuk robot humanoid Hiro + walking engine aruku.

Alur data:
    aruku  --/joint/set_joints-->  sim (posisi servo, derajat)
    sim    --/walking/imu------->  aruku (roll/pitch/yaw badan, derajat)

Saat start, robot langsung diletakkan dalam pose berdiri (lutut sedikit
ditekuk, sama seperti pose diam aruku) dengan telapak kaki tepat di lantai,
sehingga robot tegak sejak awal dan tidak jatuh.
"""

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

# Pose berdiri awal (derajat, konvensi ID servo aruku). Nilai ini sama dengan
# keluaran aruku saat robot diam (config/walking.json + kinematic.json), jadi
# robot tegak sempurna sejak awal dan tidak ada hentakan saat aruku aktif.
STAND_POSE = {
    1: 0.0,    # right_shoulder_pitch (lurus vertikal)
    2: 0.0,    # left_shoulder_pitch  (lurus vertikal)
    3: -75.0,  # right_shoulder_roll  (lurus ke bawah samping badan)
    4: -75.0,  # left_shoulder_roll   (lurus ke bawah samping badan)
    5: 0.0,    # right_elbow          (siku lurus tidak menekuk)
    6: 0.0,    # left_elbow           (siku lurus tidak menekuk)
    7: 0.0, 8: 0.0, 9: 0.0, 10: 0.0,
    11: -8.17, 12: 8.17, 13: 17.93, 14: -17.93, 15: 9.76, 16: -9.76,
    17: 0.0, 18: 0.0, 19: 0.0, 20: 0.0,
}


class MujocoSimNode(Node):

    def __init__(self):
        super().__init__('mujoco_sim_node')

        try:
            default_model = os.path.join(
                get_package_share_directory('op3_mujoco_sim'), 'models', 'scene.xml')
        except Exception:
            default_model = os.path.abspath(os.path.join(
                os.path.dirname(__file__), '..', 'models', 'scene.xml'))

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('use_gui', True)
        self.declare_parameter('imu_rate', 100.0)

        model_path = self.get_parameter('model_path').value or default_model
        self.use_gui = self.get_parameter('use_gui').value
        imu_rate = self.get_parameter('imu_rate').value

        self.get_logger().info(f'Memuat model MuJoCo: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()

        self.torso_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso_kidsize25')

        # ID servo -> (index qpos, index actuator, arah)
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

    # ------------------------------------------------------------------ setup
    def _setup_standing_pose(self):
        """Pasang pose berdiri dan letakkan telapak kaki tepat di lantai."""
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)

        # Badan tegak di atas origin
        d.qpos[0:3] = [0.0, 0.0, 0.5]
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        # Sendi = pose berdiri, aktuator menahan pose yang sama
        for servo_id, deg in STAND_POSE.items():
            if servo_id in self.servos:
                qadr, aid, sign = self.servos[servo_id]
                d.qpos[qadr] = math.radians(deg) * sign
                d.ctrl[aid] = d.qpos[qadr]

        # Turunkan badan sampai titik terendah robot menyentuh lantai (z = 0)
        mujoco.mj_forward(m, d)
        d.qpos[2] -= self._lowest_point_z() - 0.0005
        mujoco.mj_forward(m, d)

        # Jadikan pose ini sebagai pose reset (tombol Backspace di viewer)
        m.qpos0[:] = d.qpos
        self.stand_ctrl = d.ctrl.copy()

    def _lowest_point_z(self):
        """Koordinat z terendah dari semua mesh collision robot."""
        m, d = self.model, self.data
        lowest = np.inf
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
                continue  # geom visual saja
            mesh = m.geom_dataid[g]
            start, num = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            verts = m.mesh_vert[start:start + num]
            world_z = verts @ d.geom_xmat[g].reshape(3, 3)[2] + d.geom_xpos[g][2]
            lowest = min(lowest, world_z.min())
        return lowest

    def reset(self):
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.data.ctrl[:] = self.stand_ctrl

    # -------------------------------------------------------------- ROS I/O
    def _on_set_joints(self, msg: SetJoints):
        with self.lock:
            for joint in msg.joints:
                if joint.id in self.servos:
                    _, aid, sign = self.servos[joint.id]
                    self.data.ctrl[aid] = math.radians(joint.position) * sign

    def _publish_imu(self):
        with self.lock:
            R = self.data.xmat[self.torso_id].reshape(3, 3)
            # Arah gravitasi dalam frame badan (depan robot = -y, kiri = +x)
            roll = math.atan2(-R[2, 0], R[2, 2])
            pitch = math.atan2(R[2, 1], R[2, 2])
            yaw = math.atan2(R[1, 0], R[0, 0])

        msg = Imu()
        msg.roll = math.degrees(roll)
        msg.pitch = math.degrees(pitch)
        msg.yaw = math.degrees(yaw)
        self.imu_pub.publish(msg)

    # -------------------------------------------------------------- physics
    def step(self):
        with self.lock:
            mujoco.mj_step(self.model, self.data)


def run_realtime(node, viewer=None):
    """Loop fisika real-time; viewer di-refresh ~60 FPS."""
    dt = node.model.opt.timestep
    sim_start = node.data.time
    wall_start = time.perf_counter()
    last_sync = 0.0

    while rclpy.ok() and (viewer is None or viewer.is_running()):
        node.step()

        now = time.perf_counter()
        if viewer is not None and now - last_sync > 1.0 / 60.0:
            viewer.sync()
            last_sync = now

        # Jika simulasi terlalu cepat dari waktu nyata, tunggu sebentar
        ahead = (node.data.time - sim_start) - (now - wall_start)
        if ahead > dt:
            time.sleep(ahead)
        elif ahead < -0.5:
            # Tertinggal jauh (misal setelah reset): sinkronkan ulang jam
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
