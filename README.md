# Hiro & Humanoid MuJoCo Simulation - ROS 2 Jazzy

Proyek simulasi robot humanoid (**Hiro Robot**) menggunakan physics engine **MuJoCo** yang terintegrasi penuh ke dalam ekosistem **ROS 2 Jazzy Jalisco** pada sistem operasi **Ubuntu 24.04 LTS (Noble Numbat)**.

Repositori ini telah dilengkapi dengan walking engine robot humanoid **`aruku`** (kinematika dan gait generator), modul manajemen sendi Dynamixel (**`tachimawari`**), modul IMU sensor (**`kansei`**), serta panduan lengkap untuk mengimpor **model robot kustom dari luar** (misal Onshape, SolidWorks, URDF, MJCF).

---

## Daftar Isi
1. [Fitur Utama](#-fitur-utama)
2. [Arsitektur Sistem & Alur Komunikasi](#-arsitektur-sistem--alur-komunikasi)
3. [Struktur Folder Workspace](#-struktur-folder-workspace)
4. [Panduan Instalasi & Prasyarat](#-panduan-instalasi--prasyarat)
   - [Instalasi ROS 2 Jazzy](#1-instalasi-ros-2-jazzy)
   - [Instalasi MuJoCo Physics Engine](#2-instalasi-mujoco-physics-engine)
   - [Dependensi Paket Pendukung](#3-dependensi-paket-pendukung)
5. [Kompilasi Workspace (Build)](#-kompilasi-workspace-build)
6. [Panduan Lengkap Paket `aruku` (Walking Engine)](#-panduan-lengkap-paket-aruku-walking-engine)
   - [Apa itu Package `aruku`?](#1-apa-itu-package-aruku)
   - [Dependensi dan Struktur Paket](#2-dependensi-dan-struktur-paket)
   - [Konfigurasi Parameter `aruku` (`config/`)](#3-konfigurasi-parameter-aruku)
   - [Menjalankan Node `aruku`](#4-menjalankan-node-aruku)
   - [Mengontrol Gerakan Berjalan (Walking Commands)](#5-mengontrol-gerakan-berjalan-walking-commands)
   - [Memonitor Status dan Odometri](#6-memonitor-status-dan-odometri)
7. [Panduan Impor & Implementasi Model dari Luar (External Model)](#-panduan-impor--implementasi-model-dari-luar-external-model)
   - [Alur dari CAD ke MuJoCo (MJCF)](#1-alur-dari-cad-ke-mujoco-mjcf)
   - [Struktur Penempatan File Model](#2-struktur-penempatan-file-model)
   - [Standar Kebutuhan Model MJCF untuk `aruku` & Simulator](#3-standar-kebutuhan-model-mjcf-untuk-aruku--simulator)
   - [Penyesuaian Dimensi Kinematika Robot Baru](#4-penyesuaian-dimensi-kinematika-robot-baru)
   - [Menerapkan Model Baru ke Simulasi](#5-menerapkan-model-baru-ke-simulasi)
8. [Menjalankan Simulasi MuJoCo](#-menjalankan-simulasi-mujoco)
   - [Mode 3D GUI Interactive Viewer](#1-mode-3d-gui-interactive-viewer)
   - [Mode Headless (Tanpa GUI)](#2-mode-headless-tanpa-gui)
9. [Kontrol Teleoperasi & Motion Player](#-kontrol-teleoperasi--motion-player)
   - [Teleoperasi Keyboard](#1-teleoperasi-keyboard)
   - [Demo Motion Player](#2-demo-motion-player)
10. [Daftar Lengkap Topik dan Service ROS 2](#-daftar-lengkap-topik-dan-service-ros-2)
11. [Troubleshooting & Solusi](#-troubleshooting--solusi)

---

## Fitur Utama

- **Walking Engine Tingkat Lanjut (`aruku`):** Mengimplementasikan gait generator bipedal berbasis kinematika invers, Zero Moment Point (ZMP), dan stabilisasi IMU real-time.
- **Model Robot Berkualitas Tinggi:** Mendukung model standar ROBOTIS OP3 dan model robot kustom hasil ekspor CAD seperti robot Hiro (`hiro-mujoco`) dengan 20 servo Dynamixel.
- **Simulasi Fisika Cepat & Akurat:** Menggunakan MuJoCo 3+ dengan penanganan kontak dinamis, gesekan telapak kaki, dan komputasi RK4/Euler berkecepatan tinggi.
- **Bridge ROS 2 Komprehensif:**
  - Jembatan data sensor: `/joint_states` (20 sendi), `/imu/data`, `/measurement/status`, `/measurement/unit`.
  - Jembatan aktuasi: `/joint/set_joints` (Tachimawari/Aruku bridge), `/op3/named_joint_command`, `/op3/joint_commands`.
  - Broadcasting TF Dinamis: `odom` -> `base_link`.
- **Dukungan Model Luar (External CAD / OnShape / URDF / MJCF):** Alur kerja siap pakai untuk mengimpor robot baru langsung ke simulasi dan walking engine.

---

## Arsitektur Sistem & Alur Komunikasi

Simulasi dan walking engine bekerja secara berkesinambungan melalui antarmuka ROS 2 berikut:

```
 +-------------------------------------------------------------------------------+
 |                            ROBOT WALKING CONTROL                              |
 |                                                                               |
 |   User / High-Level Command                   aruku (Walking Node)            |
 |  [ /walking/set_walking ] ---------------->  - Inverse Kinematics (Gait)       |
 |                                              - PID Roll/Pitch Balance         |
 |                                              - Odometry Estimation            |
 +-------------------------------------------------------------------------------+
                                 |                          ^
     Perintah Sudut Sendi        |                          |  Data Orientasi & Gyro
     [ /joint/set_joints ]       v                          |  [ /measurement/status ]
 (tachimawari_interfaces/SetJoints)                         |  [ /measurement/unit ]
 +-------------------------------------------------------------------------------+
 |                     op3_mujoco_sim (MuJoCo Simulation Node)                   |
 |                                                                               |
 |   - Konversi ID Servo (1-20) -> Radian + Arah                                 |
 |   - Eksekusi Fisika MuJoCo (500 Hz): data.ctrl -> Forward Dynamics            |
 |   - Publikasi Sensor & TF: /joint_states, /imu/data, /tf                      |
 |   - Render Visual 3D Viewer                                                   |
 +-------------------------------------------------------------------------------+
```

---

## Struktur Folder Workspace

```text
mujoco-op3/
├── README.md                      # Dokumentasi komprehensif proyek
├── config/                        # Konfigurasi parameter walking engine & kinematika
│   ├── kinematic.json             # Dimensi link kaki, rasio ayunan, tinggi langkah
│   └── walking.json               # Sudut inisialisasi, arah putar servo, gain PID
├── src/
│   ├── op3_mujoco_sim/            # Node simulasi MuJoCo & bridge ROS 2
│   │   ├── op3_mujoco_sim/        # Kode modul Python (sim_node, teleop, motion)
│   │   ├── launch/                # File launch ROS 2 (sim.launch.py, teleop.launch.py)
│   │   ├── config/                # Konfigurasi parameter node & pose default
│   │   └── models/                # Definisi model simulasi MuJoCo
│   │       ├── scene.xml          # Scene utama (lantai, pencahayaan, include robot)
│   │       ├── robot.xml          # Model robot kustom (misal Hiro / Onshape export)
│   │       ├── op3.xml            # Model resmi ROBOTIS OP3
│   │       └── assets/            # File mesh 3D (.stl) visual dan collision
│   ├── aruku/                     # Paket C++ walking gait generation & ZMP
│   ├── aruku_interfaces/          # Definisi pesan & service ROS 2 untuk aruku
│   ├── tachimawari/               # Manajemen aktuator Dynamixel C++
│   ├── tachimawari_interfaces/    # Definisi pesan SetJoints untuk kontrol sendi
│   ├── kansei/                    # Paket C++ pemrosesan dan estimasi IMU
│   ├── kansei_interfaces/         # Definisi pesan Status & Unit IMU
│   ├── keisan/                    # Pustaka matematika & geometri 3D
│   └── jitsuyo/                   # Pustaka utilitas konfigurasi & JSON I/O
```

---

## Panduan Instalasi & Prasyarat

### 1. Instalasi ROS 2 Jazzy
Pastikan sistem operasi Anda adalah **Ubuntu 24.04 LTS (Noble Numbat)** dengan ROS 2 Jazzy Jalisco.

```bash
# 1. Konfigurasi Locale UTF-8
sudo apt update && sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Tambahkan Repositori ROS 2
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'

# 3. Pasang ROS 2 Jazzy Desktop dan Build Tools
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep

# 4. Inisialisasi rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 2. Instalasi MuJoCo Physics Engine

```bash
# Dependensi sistem grafik & OpenGL
sudo apt install -y python3-pip libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Pustaka Python MuJoCo untuk user
python3 -m pip install --user --break-system-packages mujoco glfw pyopengl
```

Verifikasi instalasi MuJoCo:
```bash
python3 -c "import mujoco; print('MuJoCo Version:', mujoco.__version__)"
```

### 3. Dependensi Paket Pendukung

Pustaka C++ yang diperlukan oleh paket `keisan`, `jitsuyo`, `kansei`, dan `aruku`:
```bash
sudo apt install -y nlohmann-json3-dev libeigen3-dev
```

Pastikan environment ROS 2 Jazzy dimuat secara default di `~/.bashrc`:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Kompilasi Workspace (Build)

Sebelum menjalankan simulasi atau node kontrol, lakukan build seluruh paket di workspace:

```bash
cd ~/Documents/mujoco-op3

# Pastikan ROS 2 aktif
source /opt/ros/jazzy/setup.bash

# Kompilasi seluruh 9 paket
colcon build --symlink-install

# Muat overlay workspace
source install/setup.bash
```

> [!TIP]
> Opsi `--symlink-install` memungkinkan perubahan skrip Python (`op3_mujoco_sim`) dan file launch langsung aktif tanpa harus mengulang `colcon build`. Jika Anda mengedit file C++ pada `aruku`, jalankan `colcon build --packages-select aruku` kembali.

---

## Panduan Lengkap Paket `aruku` (Walking Engine)

### 1. Apa itu Package `aruku`?
`aruku` (dari bahasa Jepang *歩く* yang berarti berjalan) adalah walking engine bipedal robot humanoid. Paket ini menggenerasi pola langkah kaki (gait generation) dengan menghitung invers kinematika (IK) dari 12 sendi kaki (masing-masing 6 DOF per kaki) dan lengan penyeimbang, memproses stabilisasi tubuh berdasarkan data orientasi giroskop/IMU, serta mengestimasi posisi relatif pergerakan robot (odometri).

### 2. Dependensi dan Struktur Paket
Paket `aruku` bergantung pada pustaka internal berikut di dalam `src/`:
- **`aruku_interfaces`**: Berisi format pesan `SetWalking.msg`, `Status.msg`, `Point2.msg`, dan service `SetConfig.msg`.
- **`keisan`**: Operasi vektor, matriks, sudut (degree/radian), dan kalkulasi geometri 3D.
- **`jitsuyo`**: Pembacaan konfigurasi JSON.
- **`kansei` & `kansei_interfaces`**: Penerimaan data orientasi roll, pitch, yaw dari IMU untuk stabilisasi langkah.
- **`tachimawari` & `tachimawari_interfaces`**: Pengiriman instruksi target sendi (`SetJoints.msg`) ke aktuator.

### 3. Konfigurasi Parameter `aruku`
Konfigurasi tersimpan dalam folder `config/` di root workspace:

#### A. File `config/kinematic.json`
Mengatur parameter gait, dimensi fisik robot, ayunan, dan rasio kompensasi:
```json
{
  "ratio": {
    "period_time": 600.0,       // Durasi satu siklus langkah penuh (milidetik)
    "dsp_ratio": 0.2,           // Double Support Phase ratio (fase kedua kaki menyentuh tanah)
    "foot_height": 18.0,        // Tinggi angkatan kaki saat melangkah (mm)
    "swing_right_left": 12.0,   // Amplitudo ayunan samping badan untuk memindahkan beban (mm)
    "swing_up_down": 3.0,       // Amplitudo ayunan vertikal (mm)
    "move_accel_ratio": 0.3,    // Percepatan transisi langkah
    "foot_accel_ratio": 0.3
  },
  "length": {
    "thigh_length": 134.0,      // Panjang paha (hip pitch ke knee) dalam mm
    "calf_length": 111.7,       // Panjang betis (knee ke ankle pitch) dalam mm
    "ankle_length": 34.74,      // Jarak mata kaki ke sol telapak (mm)
    "leg_length": 280.44        // Total panjang kaki saat lurus (mm)
  },
  "offset": {
    "x_offset": 0.0,            // Offset posisi badan depan-belakang
    "y_offset": 0.0,            // Offset posisi badan kiri-kanan
    "z_offset": 10.0,           // Offset penurunan pusat gravitasi (posisi lutut sedikit ditekuk)
    "yaw_offset": 0.0,
    "roll_offset": 0.0,
    "pitch_offset": 0.0
  }
}
```

#### B. File `config/walking.json`
Mengatur inisialisasi sudut servo, arah rotasi, dan gain penyeimbang PID:
- **`init_angles`**: Sudut pose awal berdiri robot untuk seluruh 20 sendi (dalam derajat).
- **`angles_direction`**: Polaritas arah pergerakan sendi (`1` atau `-1`). Menentukan apakah sudut bertambah searah jarum jam atau berlawanan arah.
- **`pid`**: Pengaturan gain penyeimbang pitch dan roll berbasis sensor IMU (`p_pitch_gain`, `p_roll_gain`).

### 4. Menjalankan Node `aruku`

Paket `aruku` menyediakan 3 executable:

#### Opsi 1: Menjalankan Node Utama (`main`) - Direkomendasikan
Node ini mengaktifkan walking manager, controller loop (125 Hz / 8 ms), publisher status, dan service konfigurasi runtime.
```bash
cd ~/Documents/mujoco-op3
source install/setup.bash
ros2 run aruku main config/
```
*Argumen `config/` mengarahkan ke path folder tempat file `kinematic.json` dan `walking.json` berada.*

#### Opsi 2: Menjalankan Walking Loop Sederhana (`walking`)
Menjalankan loop kinematika murni tanpa runtime service manager:
```bash
ros2 run aruku walking config/
```

#### Opsi 3: Menjalankan via File Kontrol (`control`)
Membaca instruksi kontinu dari file `config/control.json` secara real-time:
```bash
ros2 run aruku control config/
```

### 5. Mengontrol Gerakan Berjalan (Walking Commands)

Ketika node `aruku` berjalan, Anda dapat memberikan perintah jalan melalui topik ROS 2 `/walking/set_walking`:

#### A. Mulai Berjalan Maju
```bash
ros2 topic pub /walking/set_walking aruku_interfaces/msg/SetWalking \
  "{run: true, x_move: 20.0, y_move: 0.0, a_move: 0.0, aim_on: false}" --once
```
*Keterangan parameter:*
- `run`: `true` untuk mengaktifkan jalan, `false` untuk berhenti.
- `x_move`: Amplitudo gerak maju (+) atau mundur (-) dalam mm per langkah (rekomendasi: 10.0 s/d 30.0).
- `y_move`: Amplitudo gerak geser menyamping / crab walk (kiri +, kanan -).
- `a_move`: Kecepatan putaran / rotasi belok dalam derajat per langkah (kiri +, kanan -).
- `aim_on`: Mode penyesuaian sudut otomatis jika diaktifkan.

#### B. Berjalan Mundur
```bash
ros2 topic pub /walking/set_walking aruku_interfaces/msg/SetWalking \
  "{run: true, x_move: -15.0, y_move: 0.0, a_move: 0.0, aim_on: false}" --once
```

#### C. Berbelok di Tempat (Putar Kiri / Kanan)
```bash
# Putar ke kiri:
ros2 topic pub /walking/set_walking aruku_interfaces/msg/SetWalking \
  "{run: true, x_move: 0.0, y_move: 0.0, a_move: 10.0, aim_on: false}" --once

# Putar ke kanan:
ros2 topic pub /walking/set_walking aruku_interfaces/msg/SetWalking \
  "{run: true, x_move: 0.0, y_move: 0.0, a_move: -10.0, aim_on: false}" --once
```

#### D. Berhenti Berjalan (Smooth Stop ke Posisi Berdiri)
```bash
ros2 topic pub /walking/set_walking aruku_interfaces/msg/SetWalking \
  "{run: false, x_move: 0.0, y_move: 0.0, a_move: 0.0, aim_on: false}" --once
```

### 6. Memonitor Status dan Odometri

- **Melihat Status Berjalan Real-Time:**
  ```bash
  ros2 topic echo /walking/status
  ```
  Menampilkan apakah robot sedang melangkah (`is_running`), amplitudo pergerakan aktual (`x_amplitude`, `y_amplitude`), dan posisi odometri global robot.

- **Melihat Perubahan Posisi Tiap Langkah (Delta Position):**
  ```bash
  ros2 topic echo /walking/delta_position
  ```

---

## Panduan Impor & Implementasi Model dari Luar (External Model)

Jika Anda memiliki desain CAD robot humanoid baru (dari Onshape, SolidWorks, Autodesk Inventor, atau Blender) dan ingin mensimulasikannya dengan MuJoCo serta mengontrolnya dengan `aruku`, ikuti langkah-langkah di bawah ini:

### 1. Alur dari CAD ke MuJoCo (MJCF)

```
 +------------------+       +-------------------+       +-----------------------+
 | 3D CAD Assembly  | ----> | onshape-to-robot  | ----> | File MJCF XML         |
 | (Onshape/Blender)|       | atau urdf2mjcf    |       | + Mesh STL (assets/)  |
 +------------------+       +-------------------+       +-----------------------+
```

1. **Persiapan Assembly CAD:**
   - Berikan nama sendi (joints) sesuai standar yang jelas (misal `right_knee`, `left_shoulder_pitch`).
   - Tentukan orientasi sumbu rotasi setiap sendi secara konsisten.
2. **Ekspor Menggunakan Tool Konversi:**
   - Menggunakan [`onshape-to-robot`](https://github.com/Rhoban/onshape-to-robot) untuk menghasilkan model URDF/MJCF otomatis beserta file `.stl`.
   - Atau menggunakan fitur ekspor URDF dari SolidWorks/Fusion360, lalu diubah ke format MJCF menggunakan MuJoCo utility:
     ```bash
     python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('model.urdf'); mujoco.mj_saveLastXML('robot.xml', m)"
     ```

### 2. Struktur Penempatan File Model

Letakkan file hasil ekspor ke direktori paket `op3_mujoco_sim`:

```text
src/op3_mujoco_sim/models/
├── scene.xml                  # File scene utama MuJoCo
├── robot.xml                  # File model robot kustom Anda (MJCF)
└── assets/                    # File mesh STL visual & collision
    ├── torso.stl
    ├── femur.stl
    ├── tibia.stl
    ├── foot.stl
    └── ...
```

### 3. Standar Kebutuhan Model MJCF untuk `aruku` & Simulator

Agar model eksternal dapat dikontrol oleh `aruku` dan node `op3_mujoco_sim_node`, pastikan file XML model memenuhi 4 kriteria berikut:

#### A. Pemetaan 20 Sendi dan Aktuator (Dynamixel Standard)
Model harus memiliki 20 joint dan aktuator posisi dengan penamaan standar berikut:

| ID Servo | Nama Joint / Actuator | Keterangan Bagian Robot |
| :---: | :--- | :--- |
| **1** | `right_shoulder_pitch` | Bahu Kanan (Pitch) |
| **2** | `left_shoulder_pitch` | Bahu Kiri (Pitch) |
| **3** | `right_shoulder_roll` | Bahu Kanan (Roll / Angkat samping) |
| **4** | `left_shoulder_roll` | Bahu Kiri (Roll / Angkat samping) |
| **5** | `right_elbow` | Siku Kanan |
| **6** | `left_elbow` | Siku Kiri |
| **7** | `right_hip_yaw` | Pinggul Kanan (Yaw / Putar) |
| **8** | `left_hip_yaw` | Pinggul Kiri (Yaw / Putar) |
| **9** | `right_hip_roll` | Pinggul Kanan (Roll / Miring) |
| **10** | `left_hip_roll` | Pinggul Kiri (Roll / Miring) |
| **11** | `right_hip_pitch` | Pinggul Kanan (Pitch / Maju-Mundur) |
| **12** | `left_hip_pitch` | Pinggul Kiri (Pitch / Maju-Mundur) |
| **13** | `right_knee` | Lutut Kanan |
| **14** | `left_knee` | Lutut Kiri |
| **15** | `right_ankle_pitch` | Mata Kaki Kanan (Pitch) |
| **16** | `left_ankle_pitch` | Mata Kaki Kiri (Pitch) |
| **17** | `right_ankle_roll` | Mata Kaki Kanan (Roll) |
| **18** | `left_ankle_roll` | Mata Kaki Kiri (Roll) |
| **19** | `neck_yaw` | Leher / Kepala (Yaw / Geleng) |
| **20** | `neck_pitch` | Leher / Kepala (Pitch / Angguk) |

Definisikan aktuator pada tag `<actuator>` dalam file XML robot:
```xml
<actuator>
  <position name="neck_yaw" joint="neck_yaw" forcerange="-6.0 6.0" kp="60"/>
  <position name="right_hip_pitch" joint="right_hip_pitch" forcerange="-6.0 6.0" kp="80"/>
  <position name="right_knee" joint="right_knee" forcerange="-6.0 6.0" kp="80"/>
  <!-- Ulangi untuk seluruh 20 sendi -->
</actuator>
```

#### B. Freejoint pada Torso (Floating Base)
Badan utama robot (torso) harus memiliki `<freejoint/>` agar robot dapat berjalan dan berpindah tempat di dunia simulasi:
```xml
<body name="torso_kidsize25" pos="0 0 0" quat="1 0 0 0">
  <freejoint name="torso_freejoint"/>
  <site name="imu" pos="0 0 0.1"/>
  <!-- Geometri dan link anak di sini -->
</body>
```
*(Node simulator secara otomatis mendeteksi body `torso_kidsize25` atau `body_link`)*.

#### C. Sensor IMU (Akselerometer & Gyro)
Tambahkan sensor IMU di dalam tag `<sensor>` untuk menunjang stabilisasi `kansei` dan `aruku`:
```xml
<sensor>
  <accelerometer name="imu_accel" site="imu"/>
  <gyro name="imu_gyro" site="imu"/>
</sensor>
```

#### D. Friksi Sol Telapak Kaki (Foot Friction)
Pastikan geometri telapak kaki memiliki gesekan (friction) yang cukup agar tidak licin saat berjalan:
```xml
<geom type="mesh" mesh="shoes" friction="1.2 0.005 0.0001" solref="0.004 1" solimp="0.9 0.95 0.001 0.5 2"/>
```

### 4. Penyesuaian Dimensi Kinematika Robot Baru

Ukur dimensi mekanik robot luar Anda pada software CAD (dalam satuan milimeter) dan perbarui `config/kinematic.json`:
- **`thigh_length`**: Jarak sumbu putar `hip_pitch` ke `knee`.
- **`calf_length`**: Jarak sumbu putar `knee` ke `ankle_pitch`.
- **`ankle_length`**: Jarak sumbu putar `ankle_pitch` ke bagian dasar sol kaki.
- **`leg_length`**: Total `thigh_length + calf_length + ankle_length`.

Jika sudut nol atau arah putaran sendi pada model baru berbeda dari model referensi, sesuaikan polaritas di `config/walking.json` pada bagian `angles_direction` (`1` atau `-1`).

### 5. Menerapkan Model Baru ke Simulasi

1. **Buka file `src/op3_mujoco_sim/models/scene.xml`** dan arahkan include ke file robot baru Anda:
   ```xml
   <mujoco model="scene">
       <include file="robot.xml" />
       ...
   ```
2. **Kompilasi ulang workspace:**
   ```bash
   cd ~/Documents/mujoco-op3
   colcon build --symlink-install
   source install/setup.bash
   ```
3. **Jalankan simulasi:**
   ```bash
   ros2 launch op3_mujoco_sim sim.launch.py
   ```

---

## Menjalankan Simulasi MuJoCo

### 1. Mode 3D GUI Interactive Viewer
Jalankan file launch utama:
```bash
cd ~/Documents/mujoco-op3
source install/setup.bash
ros2 launch op3_mujoco_sim sim.launch.py
```
Jendela **MuJoCo Viewer** interaktif akan terbuka. Anda dapat berinteraksi dengan:
- **Klik Kiri + Tahan Geser:** Putar kamera (Orbit).
- **Klik Kanan + Tahan Geser:** Perbesar / perkecil (Zoom).
- **Klik Tengah / Scroll:** Geser posisi kamera (Pan).
- **Ctrl + Klik Kiri pada Robot:** Berikan gaya dorong / gangguan fisik untuk menguji keseimbangan robot saat berjalan.

### 2. Mode Headless (Tanpa GUI)
Untuk eksekusi server, training AI/RL, atau tanpa monitor:
```bash
ros2 launch op3_mujoco_sim sim.launch.py use_gui:=false
```

---

## Kontrol Teleoperasi & Motion Player

Buka terminal baru untuk menjalankan utilitas pengujian:

### 1. Teleoperasi Keyboard
```bash
cd ~/Documents/mujoco-op3
source install/setup.bash
ros2 run op3_mujoco_sim teleop_controller
```

| Tombol | Fungsi |
| :---: | :--- |
| `W` / `S` | Miringkan Kepala ke Atas / Bawah (`neck_pitch`) |
| `A` / `D` | Gelengkan Kepala ke Kiri / Kanan (`neck_yaw`) |
| `U` / `J` | Angkat / Turunkan Bahu Kiri |
| `I` / `K` | Angkat / Turunkan Bahu Kanan |
| `O` / `L` | Tekuk Siku Tangan Kiri / Kanan |
| `1` | Pose Berdiri Seimbang (*Stand Pose*) |
| `2` | Pose Melambaikan Tangan (*Wave Hand*) |
| `3` | Pose Jongkok Siap (*Squat Ready*) |
| `0` | Reset Semua Sendi ke Sudut 0 |
| `R` | Reset Fisika Simulasi MuJoCo |
| `P` | Jeda / Lanjutkan Simulasi (*Pause/Resume*) |
| `Q` | Keluar dari Teleop |

### 2. Demo Motion Player
Untuk memutar sekuens animasi interpolasi halus:
- Lambaikan tangan: `ros2 run op3_mujoco_sim motion_player --motion wave --cycles 3`
- Jongkok: `ros2 run op3_mujoco_sim motion_player --motion squat --cycles 2`

---

## Daftar Lengkap Topik dan Service ROS 2

### Topik Sensor & Bridge Simulasi (`op3_mujoco_sim`)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Posisi, kecepatan, dan torsi aktual 20 sendi |
| `/imu/data` | `sensor_msgs/msg/Imu` | Akselerasi linear, laju sudut giroskop, dan kuaternion orientasi |
| `/measurement/status` | `kansei_interfaces/msg/Status` | Data roll, pitch, yaw, gravitasi untuk input `aruku` |
| `/measurement/unit` | `kansei_interfaces/msg/Unit` | Raw data giroskop & akselerometer untuk `aruku` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transformasi koordinat dari `odom` ke `base_link` |

### Topik Perintah Gerak & Walking Engine (`aruku`)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/walking/set_walking` | `aruku_interfaces/msg/SetWalking` | Perintah aktivasi jalan (`run`, `x_move`, `y_move`, `a_move`) |
| `/joint/set_joints` | `tachimawari_interfaces/msg/SetJoints` | Output sudut 20 sendi dari `aruku` ke simulator |
| `/walking/status` | `aruku_interfaces/msg/Status` | Status aktif walking engine dan posisi odometri aktual |
| `/walking/delta_position` | `aruku_interfaces/msg/Point2` | Perpindahan langkah delta x dan y |
| `/op3/named_joint_command` | `sensor_msgs/msg/JointState` | Perintah posisi sendi berdasarkan nama spesifik |
| `/op3/joint_commands` | `std_msgs/msg/Float64MultiArray` | Perintah langsung array float ke seluruh 20 aktuator |

### Service ROS 2
| Service | Tipe Service | Deskripsi |
| :--- | :--- | :--- |
| `/reset_simulation` | `std_srvs/srv/Trigger` | Mengembalikan posisi robot dan simulasi fisika ke kondisi awal |
| `/pause_simulation` | `std_srvs/srv/SetBool` | Menghentikan sementara (`data: true`) atau melanjutkan (`data: false`) fisika |
| `/aruku/config/get_config` | `aruku_interfaces/srv/GetConfig` | Mengambil data konfigurasi walking runtime |
| `/aruku/config/save_config` | `aruku_interfaces/srv/SaveConfig` | Menyimpan perubahan konfigurasi walking ke file JSON |

---

## Troubleshooting & Solusi

1. **Error `command not found` saat menjalankan perintah:**
   Pastikan Anda tidak mengetikkan perintah di luar terminal bash, dan pastikan environment ROS 2 sudah di-source:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/Documents/mujoco-op3/install/setup.bash
   ```

2. **Robot jatuh saat mulai berjalan:**
   - Periksa nilai `period_time` di `config/kinematic.json` (coba perbesar ke `700.0` atau `800.0` ms untuk langkah yang lebih lambat dan stabil).
   - Naikkan `swing_right_left` (misal dari `12.0` ke `16.0` mm) agar pergeseran titik berat badan saat mengangkat satu kaki lebih seimbang.
   - Periksa gesekan telapak kaki di file XML robot (pastikan nilai `friction` pada sol sepatu minimal `1.0` s/d `1.5`).

3. **Mesh 3D robot tidak muncul (tampak transparan atau error `mesh not found`):**
   - Pastikan tag `<compiler angle="radian" meshdir="assets" autolimits="true"/>` sudah ada di file XML robot.
   - Pastikan seluruh file `.stl` berada di `src/op3_mujoco_sim/models/assets/`.
   - Jalankan `colcon build --symlink-install` agar file mesh diperbarui di direktori `install/`.

4. **Gerakan sendi terbalik:**
   Ubah nilai polaritas sendi yang bermasalah pada `angles_direction` di `config/walking.json` dari `1` menjadi `-1` (atau sebaliknya).
