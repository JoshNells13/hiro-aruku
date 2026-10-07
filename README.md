# Simulasi Berjalan Robot Humanoid Hiro - MuJoCo & ROS 2 Jazzy

Repositori ini difokuskan **khusus untuk konsep dan implementasi berjalan (bipedal walking)** robot humanoid **Hiro Robot** menggunakan physics engine **MuJoCo** dan **ROS 2 Jazzy Jalisco** (Ubuntu 24.04 LTS).

Arsitektur dibuat **minimalis, bersih, dan simpel**:
- Robot langsung **berdiri tegak (*upright*)** dan stabil saat simulasi pertama kali dibuka.
- Semua package hardware/kompetisi yang tidak berkaitan (Dynamixel SDK, CM740, motion player, sensor eksternal) telah disingkirkan.
- Hanya menyisakan walking engine (`aruku`), library matematika (`keisan`), interface pesan ROS 2 (`aruku_interfaces`), dan node simulator MuJoCo (`op3_mujoco_sim`).

---

## Daftar Isi
1. [Struktur Paket Workspace](#struktur-paket-workspace)
2. [Alur Komunikasi Sederhana](#alur-komunikasi-sederhana)
3. [Prasyarat & Dependensi](#prasyarat--dependensi)
4. [Kompilasi (Build) Workspace](#kompilasi-build-workspace)
5. [Menjalankan Simulasi & Walking Engine](#menjalankan-simulasi--walking-engine)
6. [Mengontrol Robot Berjalan](#mengontrol-robot-berjalan)
   - [Opsi A: Teleoperasi Keyboard (Direkomendasikan)](#opsi-a-teleoperasi-keyboard-direkomendasikan)
   - [Opsi B: Perintah Topik ROS 2 Langsung](#opsi-b-perintah-topik-ros-2-langsung)
7. [Memonitor Status Berjalan & Sensor](#memonitor-status-berjalan--sensor)

---

## Struktur Paket Workspace

Workspace ini hanya berisi 4 paket inti:

```text
mujoco-op3/
├── src/
│   ├── op3_mujoco_sim/       # Simulasi fisika MuJoCo 3D (Hiro Robot) + teleop berjalan
│   │   ├── config/           # Parameter simulasi
│   │   ├── launch/           # sim.launch.py (menjalankan sim + aruku sekaligus)
│   │   ├── models/           # File MJCF scene.xml & robot.xml + 3D meshes Hiro
│   │   └── op3_mujoco_sim/   # mujoco_sim_node.py & walk_teleop.py
│   │
│   ├── aruku/                # Walking engine humanoid (Inverse Kinematics & ZMP Gait)
│   │   ├── config/           # Parameter jalan: walking.json & kinematic.json
│   │   ├── include/          # Header C++ (kinematics, walking node, joint model)
│   │   └── src/              # Implementasi gait generator & ROS 2 node
│   │
│   ├── aruku_interfaces/     # Interface pesan ROS 2 khusus walking
│   │   └── msg/              # SetWalking.msg, SetJoints.msg, Imu.msg, Status.msg, dll.
│   │
│   └── keisan/               # Library utilitas matematika (vektor, matriks, sudut)
```

---

## Alur Komunikasi Sederhana

Komunikasi antara walking engine dan simulator dibuat ringkas melalui 2 topik utama:

```text
 +-------------------------------------------------------------------------+
 |                           KONTROL BERJALAN                              |
 |                                                                         |
 |    Input Pengguna (walk_teleop / script)                                |
 |      └──> [ /walking/set_walking ] (SetWalking)                         |
 |                       │                                                 |
 |                       ▼                                                 |
 |                 aruku (Walking Engine)                                  |
 |                 - Inverse Kinematics 12-DoF Kaki                        |
 |                 - Gait Pattern Generator                                |
 |                 - Closed-loop PID Balance                               |
 +-------------------------------------------------------------------------+
             │                                        ▲
Perintah Sudut Servo (Derajat)               Orientasi Torso (Derajat)
    [ /joint/set_joints ]                         [ /walking/imu ]
             │                                        │
             ▼                                        │
 +-------------------------------------------------------------------------+
 |                   op3_mujoco_sim (MuJoCo Simulator)                     |
 |                                                                         |
 |   - Robot di-spawn langsung berdiri tegak seimbang                      |
 |   - Eksekusi dinamika fisika real-time MuJoCo (500 Hz)                  |
 |   - Ekstraksi orientasi torso (Roll/Pitch/Yaw) ke /walking/imu          |
 |   - Visualisasi interaktif 3D Passive Viewer                            |
 +-------------------------------------------------------------------------+
```

---

## Prasyarat & Dependensi

1. **Sistem Operasi:** Ubuntu 24.04 LTS (Noble Numbat)
2. **ROS 2:** Jazzy Jalisco (desktop atau base)
3. **Python & MuJoCo:**
   ```bash
   pip3 install mujoco numpy
   ```

---

## Kompilasi (Build) Workspace

Buka terminal di root workspace, source ROS 2, lalu lakukan build:

```bash
cd ~/Documents/Projek\ ROS\ 2/mujoco-op3
source /opt/ros/jazzy/setup.bash

# Build semua paket
colcon build --symlink-install
```

Setelah selesai, selalu source hasil build sebelum menjalankan perintah ROS 2:
```bash
source install/setup.bash
```

---

## Menjalankan Simulasi & Walking Engine

Cukup jalankan satu file launch. Perintah ini akan otomatis membuka **window MuJoCo 3D Interactive Viewer** dan mengaktifkan **aruku walking engine**:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch op3_mujoco_sim sim.launch.py
```

> **Catatan:**
> Robot langsung berdiri **tegak seimbang (*upright*)** dengan telapak kaki tepat menapak di lantai. Anda dapat menekan `Spasi` pada window MuJoCo untuk menjeda fisika, atau tombol `Backspace` untuk me-reset posisi robot.

Jika ingin menjalankan tanpa GUI (mode *headless*):
```bash
ros2 launch op3_mujoco_sim sim.launch.py use_gui:=false
```

---

## Mengontrol Robot Berjalan

### Opsi A: Teleoperasi Keyboard (Direkomendasikan)

Buka terminal baru, source workspace, lalu jalankan node `walk_teleop`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run op3_mujoco_sim walk_teleop
```

Gunakan tombol berikut pada terminal teleop:

| Tombol | Gerakan | Keterangan |
| :--- | :--- | :--- |
| **`W`** | **Maju** | Robot melangkah maju ke depan |
| **`S`** | **Mundur** | Robot melangkah mundur |
| **`A`** | **Putar Kiri** | Robot berbelok arah ke kiri |
| **`D`** | **Putar Kanan** | Robot berbelok arah ke kanan |
| **`Q`** | **Geser Kiri** | Langkah menyamping (*strafe left*) |
| **`E`** | **Geser Kanan** | Langkah menyamping (*strafe right*) |
| **`Spasi`** / **`X`** | **Berhenti** | Robot berhenti dan kembali ke pose tegak |
| **`Ctrl + C`** | Keluar | Menutup teleoperasi |

---

### Opsi B: Perintah Topik ROS 2 Langsung

Anda juga dapat mengirimkan perintah berjalan programatik melalui topik `/walking/set_walking`:

- **Jalan Maju:**
  ```bash
  ros2 topic pub --once /walking/set_walking aruku_interfaces/msg/SetWalking "{run: true, x_move: 15.0, y_move: 0.0, a_move: 0.0}"
  ```

- **Putar di Tempat ke Kiri:**
  ```bash
  ros2 topic pub --once /walking/set_walking aruku_interfaces/msg/SetWalking "{run: true, x_move: 0.0, y_move: 0.0, a_move: 10.0}"
  ```

- **Berhenti (Pose Tegak):**
  ```bash
  ros2 topic pub --once /walking/set_walking aruku_interfaces/msg/SetWalking "{run: false, x_move: 0.0, y_move: 0.0, a_move: 0.0}"
  ```

---

## Memonitor Status Berjalan & Sensor

Buka terminal untuk memantau data berjalan secara langsung:

- **Status & Odometri Berjalan:**
  ```bash
  ros2 topic echo /walking/status
  ```
  *(Menampilkan `is_running`, amplitudo langkah, dan akumulasi posisi `odometry.x`, `odometry.y`)*

- **Orientasi IMU Badan (Roll, Pitch, Yaw):**
  ```bash
  ros2 topic echo /walking/imu
  ```

- **Sudut Servo Kaki & Lengan:**
  ```bash
  ros2 topic echo /joint/set_joints
  ```

---

## Konfigurasi Parameter Gait (`src/aruku/config/`)

Konfigurasi pola langkah dan parameter invers kinematika dapat disesuaikan pada folder `src/aruku/config/`:
- **`kinematic.json`**: Mengatur panjang segmen kaki robot (`thigh_length`, `calf_length`, `ankle_length`), periode waktu langkah (`period_time`), tinggi angkat kaki (`foot_height`), dan ayunan badan (`swing_right_left`).
- **`walking.json`**: Mengatur gain PID penyeimbang (*pitch & roll balance gain*), koefisien odometri, dan arah orientasi sendi servo.
