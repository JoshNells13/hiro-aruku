#!/usr/bin/env python3
"""
Node Simulasi MuJoCo untuk Robot ROBOTIS OP3 pada ROS 2 Jazzy.

Node ini memuat model MJCF ROBOTIS OP3, menjalankan engine fisika MuJoCo,
membuka interactive 3D viewer (opsional), serta menjembatani komunikasi data
ke ekosistem ROS 2 (JointState, IMU, TF, dan Joint Commands).
"""

import os
import sys
import time
import threading
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, SetBool
import tf2_ros

import math
import mujoco
try:
    import mujoco.viewer
    VIEWER_AVAILABLE = True
except ImportError:
    VIEWER_AVAILABLE = False

try:
    from tachimawari_interfaces.msg import SetJoints
    TACHIMAWARI_AVAILABLE = True
except ImportError:
    TACHIMAWARI_AVAILABLE = False

try:
    from kansei_interfaces.msg import Status as KanseiStatus, Unit as KanseiUnit
    KANSEI_AVAILABLE = True
except ImportError:
    KANSEI_AVAILABLE = False


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('op3_mujoco_sim_node')

        # Deklarasi Parameter ROS 2
        self.declare_parameter('model_path', '')
        self.declare_parameter('physics_rate', 500.0)  # Hz
        self.declare_parameter('publish_rate', 50.0)   # Hz
        self.declare_parameter('use_gui', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_frame_parameter = self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')

        # Dapatkan nilai parameter
        model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
        self.physics_rate = self.get_parameter('physics_rate').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_gui = self.get_parameter('use_gui').get_parameter_value().bool_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value

        # Tentukan Path Model
        if not model_path_param:
            try:
                pkg_dir = get_package_share_directory('op3_mujoco_sim')
                self.model_path = os.path.join(pkg_dir, 'models', 'scene.xml')
            except Exception:
                # Fallback ke path relatif direktori kerja
                base_dir = os.path.abspath(os.path.dirname(__file__))
                self.model_path = os.path.join(base_dir, '..', 'models', 'scene.xml')
        else:
            self.model_path = model_path_param

        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model MuJoCo tidak ditemukan di: {self.model_path}')
            raise FileNotFoundError(f'Model MuJoCo tidak ditemukan di: {self.model_path}')

        self.get_logger().info(f'Memuat model MuJoCo: {self.model_path}')

        # Muat Model dan Data MuJoCo
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        # Set timestep fisika
        self.model.opt.timestep = 1.0 / self.physics_rate

        # Threading re-entrant lock untuk sinkronisasi akses ke data MuJoCo
        self.lock = threading.RLock()
        self.is_paused = False
        self.shutdown_requested = False

        # Petakan Sendi (Joints) & Aktuator
        self._setup_joint_mappings()

        # Petakan Sensor IMU jika ada
        self._setup_sensor_mappings()

        # Inisialisasi Publisher ROS 2
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        if KANSEI_AVAILABLE:
            self.kansei_status_pub = self.create_publisher(KanseiStatus, '/measurement/status', 10)
            self.kansei_unit_pub = self.create_publisher(KanseiUnit, '/measurement/unit', 10)
            self.get_logger().info('Publisher /measurement/status & /measurement/unit aktif (Kansei/Aruku bridge).')
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Inisialisasi Subscriber ROS 2
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/op3/joint_commands',
            self._joint_commands_callback,
            10
        )
        self.named_joint_cmd_sub = self.create_subscription(
            JointState,
            '/op3/named_joint_command',
            self._named_joint_command_callback,
            10
        )
        if TACHIMAWARI_AVAILABLE:
            self.set_joints_sub = self.create_subscription(
                SetJoints,
                '/joint/set_joints',
                self._set_joints_callback,
                10
            )
            self.get_logger().info('Berlangganan ke /joint/set_joints (Tachimawari/Aruku bridge aktif).')

        # Inisialisasi Service ROS 2
        self.reset_srv = self.create_service(Trigger, '/reset_simulation', self._reset_callback)
        self.pause_srv = self.create_service(SetBool, '/pause_simulation', self._pause_callback)

        # Timer untuk publikasi state ROS 2
        pub_period = 1.0 / self.publish_rate
        self.pub_timer = self.create_timer(pub_period, self._publish_state)

        # Set posisi awal robot berdiri
        self._apply_initial_standing_pose()

        self.get_logger().info('op3_mujoco_sim_node berhasil diinisialisasi.')
        self.get_logger().info(f'Physics Rate: {self.physics_rate} Hz | Publish Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'GUI Mode: {self.use_gui} (Viewer available: {VIEWER_AVAILABLE})')

    def _setup_joint_mappings(self):
        """Memetakan nama joint dan actuator dengan indeks pada model MuJoCo."""
        self.joint_names = []
        self.joint_qpos_adr = []
        self.joint_dof_adr = []

        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            # Lewati freejoint tanpa nama (floating base)
            if name:
                self.joint_names.append(name)
                self.joint_qpos_adr.append(self.model.jnt_qposadr[i])
                self.joint_dof_adr.append(self.model.jnt_dofadr[i])

        self.actuator_names = []
        self.actuator_map = {}  # actuator_name -> index
        self.joint_to_actuator = {}  # joint_name -> actuator index

        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator_names.append(name)
            self.actuator_map[name] = i
            # Jika actuator name berakhiran _act, petakan ke joint name
            if name.endswith('_act'):
                jname = name[:-4]
                self.joint_to_actuator[jname] = i
            elif name in self.joint_names:
                self.joint_to_actuator[name] = i

        self.get_logger().info(
            f'Terdeteksi {len(self.joint_names)} joint terukur dan {len(self.actuator_names)} actuator.'
        )

    def _setup_sensor_mappings(self):
        """Memetakan sensor akselerometer dan giroskop serta body torso."""
        self.accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_accel')
        self.gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_gyro')
        self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'torso_kidsize25')
        if self.torso_body_id == -1:
            self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'body_link')

        self.has_accel = (self.accel_id != -1)
        self.has_gyro = (self.gyro_id != -1)
        if self.has_accel and self.has_gyro:
            self.get_logger().info('Sensor IMU terkonfigurasi (imu_accel & imu_gyro).')

    def _apply_initial_standing_pose(self):
        """Mengatur pose awal robot Hiro agar stabil dan berdiri seimbang."""
        with self.lock:
            # Set ketinggian root link Hiro agar sol sepatu pas menyentuh lantai (posisi z = 0.2897m)
            if self.model.nq >= 7:
                self.data.qpos[0] = 0.0
                self.data.qpos[1] = 0.0
                self.data.qpos[2] = 0.2897
                self.data.qpos[3] = 1.0  # quat w
                self.data.qpos[4] = 0.0  # quat x
                self.data.qpos[5] = 0.0  # quat y
                self.data.qpos[6] = 0.0  # quat z

            pose = {
                'right_shoulder_pitch': math.radians(5.0),
                'right_shoulder_roll': math.radians(-15.0),
                'right_elbow': math.radians(30.0),
                'left_shoulder_pitch': math.radians(-5.0),
                'left_shoulder_roll': math.radians(15.0),
                'left_elbow': math.radians(-30.0),
                'neck_yaw': 0.0,
                'neck_pitch': 0.0,
            }
            for jname, val in pose.items():
                if jname in self.joint_to_actuator:
                    act_idx = self.joint_to_actuator[jname]
                    self.data.ctrl[act_idx] = val

            mujoco.mj_forward(self.model, self.data)

    def _joint_commands_callback(self, msg: Float64MultiArray):
        """Menerima array target posisi aktuator (Float64MultiArray)."""
        with self.lock:
            count = min(len(msg.data), self.model.nu)
            for i in range(count):
                self.data.ctrl[i] = msg.data[i]

    def _named_joint_command_callback(self, msg: JointState):
        """Menerima perintah posisi sendi berdasarkan nama (JointState)."""
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_to_actuator:
                    act_idx = self.joint_to_actuator[name]
                    self.data.ctrl[act_idx] = pos
                elif name in self.actuator_map:
                    act_idx = self.actuator_map[name]
                    self.data.ctrl[act_idx] = pos

    def _set_joints_callback(self, msg):
        """Menerima perintah posisi sendi dari paket aruku / tachimawari (SetJoints)."""
        joint_id_map = {
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
            11: ('right_hip_pitch', -1.0),
            12: ('left_hip_pitch', 1.0),
            13: ('right_knee', 1.0),
            14: ('left_knee', -1.0),
            15: ('right_ankle_pitch', -1.0),
            16: ('left_ankle_pitch', 1.0),
            17: ('right_ankle_roll', 1.0),
            18: ('left_ankle_roll', 1.0),
            19: ('neck_yaw', 1.0),
            20: ('neck_pitch', 1.0),
        }
        with self.lock:
            for j in msg.joints:
                if j.id in joint_id_map:
                    name, sign = joint_id_map[j.id]
                    if name in self.joint_to_actuator:
                        act_idx = self.joint_to_actuator[name]
                        rad = math.radians(j.position) * sign
                        self.data.ctrl[act_idx] = rad

    def _reset_callback(self, request, response):
        """Service handler untuk me-reset simulasi ke kondisi awal."""
        self.get_logger().info('Reset simulasi diminta.')
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self._apply_initial_standing_pose()
        response.success = True
        response.message = 'Simulasi MuJoCo berhasil direset.'
        return response

    def _pause_callback(self, request, response):
        """Service handler untuk pause / resume simulasi fisika."""
        self.is_paused = request.data
        status = 'dijeda' if self.is_paused else 'dilanjutkan'
        self.get_logger().info(f'Simulasi {status}.')
        response.success = True
        response.message = f'Simulasi {status}.'
        return response

    def _publish_state(self):
        """Mempublikasikan /joint_states, /imu/data, dan TF transform."""
        now = self.get_clock().now().to_msg()

        with self.lock:
            # 1. Publikasikan JointState
            js_msg = JointState()
            js_msg.header.stamp = now
            js_msg.name = self.joint_names
            js_msg.position = [float(self.data.qpos[adr]) for adr in self.joint_qpos_adr]
            js_msg.velocity = [float(self.data.qvel[adr]) for adr in self.joint_dof_adr]
            js_msg.effort = [float(self.data.qfrc_actuator[adr]) for adr in self.joint_dof_adr]
            self.joint_state_pub.publish(js_msg)

            # 2. Publikasikan Imu
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.imu_frame

            # Orientation dari floating base (MuJoCo format: [w, x, y, z] -> ROS: [x, y, z, w])
            if self.model.nq >= 7:
                imu_msg.orientation.w = float(self.data.qpos[3])
                imu_msg.orientation.x = float(self.data.qpos[4])
                imu_msg.orientation.y = float(self.data.qpos[5])
                imu_msg.orientation.z = float(self.data.qpos[6])

            if self.has_accel:
                adr = self.model.sensor_adr[self.accel_id]
                imu_msg.linear_acceleration.x = float(self.data.sensordata[adr])
                imu_msg.linear_acceleration.y = float(self.data.sensordata[adr + 1])
                imu_msg.linear_acceleration.z = float(self.data.sensordata[adr + 2])

            if self.has_gyro:
                adr = self.model.sensor_adr[self.gyro_id]
                imu_msg.angular_velocity.x = float(self.data.sensordata[adr])
                imu_msg.angular_velocity.y = float(self.data.sensordata[adr + 1])
                imu_msg.angular_velocity.z = float(self.data.sensordata[adr + 2])

            self.imu_pub.publish(imu_msg)

            # 3. Publikasikan Kansei Status & Unit untuk paket Aruku
            if KANSEI_AVAILABLE and self.torso_body_id != -1:
                R = self.data.xmat[self.torso_body_id].reshape(3, 3)
                pitch_deg = math.degrees(math.atan2(-R[1, 2], R[2, 2]))
                roll_deg = math.degrees(math.atan2(R[0, 2], R[2, 2]))
                yaw_deg = math.degrees(math.atan2(-R[0, 1], R[0, 0]))

                k_status = KanseiStatus()
                k_status.is_calibrated = True
                k_status.orientation.roll = float(roll_deg)
                k_status.orientation.pitch = float(pitch_deg)
                k_status.orientation.yaw = float(yaw_deg)
                k_status.gravity.x = float(R[0, 2] * 9.80665)
                k_status.gravity.y = float(R[1, 2] * 9.80665)
                k_status.gravity.z = float(R[2, 2] * 9.80665)
                self.kansei_status_pub.publish(k_status)

                k_unit = KanseiUnit()
                if self.has_gyro:
                    adr = self.model.sensor_adr[self.gyro_id]
                    k_unit.gyro.roll = float(self.data.sensordata[adr + 1])
                    k_unit.gyro.pitch = float(self.data.sensordata[adr])
                    k_unit.gyro.yaw = float(self.data.sensordata[adr + 2])
                if self.has_accel:
                    adr = self.model.sensor_adr[self.accel_id]
                    k_unit.accelero.x = float(self.data.sensordata[adr])
                    k_unit.accelero.y = float(self.data.sensordata[adr + 1])
                    k_unit.accelero.z = float(self.data.sensordata[adr + 2])
                self.kansei_unit_pub.publish(k_unit)

            # 4. Publikasikan TF (odom -> base_link)
            if self.publish_tf and self.model.nq >= 7:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = now
                tf_msg.header.frame_id = self.odom_frame
                tf_msg.child_frame_id = self.base_frame

                tf_msg.transform.translation.x = float(self.data.qpos[0])
                tf_msg.transform.translation.y = float(self.data.qpos[1])
                tf_msg.transform.translation.z = float(self.data.qpos[2])

                tf_msg.transform.rotation.w = float(self.data.qpos[3])
                tf_msg.transform.rotation.x = float(self.data.qpos[4])
                tf_msg.transform.rotation.y = float(self.data.qpos[5])
                tf_msg.transform.rotation.z = float(self.data.qpos[6])

                self.tf_broadcaster.sendTransform(tf_msg)

    def step_physics(self):
        """Memajukan 1 langkah kalkulasi fisika MuJoCo."""
        if not self.is_paused:
            with self.lock:
                mujoco.mj_step(self.model, self.data)


def main(args=None):
    rclpy.init(args=args)
    sim_node = MujocoSimNode()

    # Jalankan ROS executor di background thread
    executor = SingleThreadedExecutor()
    executor.add_node(sim_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    timestep = sim_node.model.opt.timestep

    try:
        # Jika mode GUI aktif dan modul viewer tersedia, jalankan MuJoCo Passive Viewer
        if sim_node.use_gui and VIEWER_AVAILABLE:
            with mujoco.viewer.launch_passive(sim_node.model, sim_node.data) as viewer:
                viewer.cam.lookat = [0.0, 0.0, 0.25]
                viewer.cam.distance = 1.2
                viewer.cam.elevation = -15.0
                viewer.cam.azimuth = 135.0

                sim_node.get_logger().info('MuJoCo Passive Viewer berhasil dibuka.')

                while rclpy.ok() and viewer.is_running() and not sim_node.shutdown_requested:
                    step_start = time.time()
                    sim_node.step_physics()
                    viewer.sync()

                    # Batasi kecepatan step ke real-time
                    elapsed = time.time() - step_start
                    sleep_time = timestep - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

        else:
            # Mode Headless (tanpa GUI Viewer)
            sim_node.get_logger().info('Menjalankan simulasi dalam mode Headless.')
            while rclpy.ok() and not sim_node.shutdown_requested:
                step_start = time.time()
                sim_node.step_physics()

                elapsed = time.time() - step_start
                sleep_time = timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        sim_node.get_logger().info('KeyboardInterrupt: mematikan simulasi...')
    finally:
        sim_node.shutdown_requested = True
        executor.shutdown()
        sim_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
