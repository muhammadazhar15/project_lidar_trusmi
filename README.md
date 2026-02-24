<div align="center">

# 📡 LiDAR IoT Volume Measurement System

### Kerja Sama dengan **Trusmi Group**

Sistem pengukur volume berbasis **LiDAR LPXT1**  
Menggunakan **ROS 2 Jazzy** dengan dukungan **Simulasi & Perangkat Nyata**

![ROS](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![LiDAR](https://img.shields.io/badge/Sensor-LiDAR%20LPXT1-green)
![Platform](https://img.shields.io/badge/Platform-Ubuntu-orange)
![Status](https://img.shields.io/badge/Status-Development-yellow)

</div>

---

## 📖 Deskripsi
**LiDAR IoT Volume Measurement System** adalah project pengembangan alat pengukur volume berbasis **sensor LiDAR LPXT1** yang dibangun menggunakan **framework ROS 2 Jazzy**.

Project ini dikembangkan bersama **Trusmi Group** dan dirancang untuk mendukung:
- 🔹 **Mode simulasi** (tanpa hardware)
- 🔹 **Mode perangkat nyata** (terhubung langsung ke LiDAR LPXT1)

Pendekatan ini memungkinkan pengembangan, pengujian, dan validasi sistem dilakukan secara fleksibel sebelum diterapkan di lingkungan produksi.

---

## 🎯 Tujuan
- Mengukur volume objek menggunakan teknologi LiDAR
- Menyediakan sistem simulasi LiDAR berbasis ROS 2
- Integrasi sensor **LiDAR LPXT1** ke dalam ROS 2
- Menyediakan output data untuk kebutuhan **IoT**
- Mendukung implementasi industri dan riset

---

## 🛠️ Teknologi
- **ROS 2 Jazzy**
- **LiDAR LPXT1**
- **Ubuntu 24.04**
- **Python & C++**
- **RViz2**
- **Gazebo Harmonic Simulator**
- **IoT Integration (MQTT / REST API – opsional)**

---

## 🧠 Arsitektur Sistem
<img width="1536" height="1024" alt="scanner_systems" src="https://github.com/user-attachments/assets/a5a05662-a76c-4f75-9698-ecceb419092f" />

---

## ✨ Fitur Utama
- ✅ Simulasi LiDAR berbasis ROS 2
- ✅ Driver komunikasi LiDAR LPXT1
- ✅ Akuisisi data point cloud
- ✅ Perhitungan volume objek
- ✅ Visualisasi di RViz2
- ✅ Mode simulasi & mode real device
- ✅ Siap integrasi dengan sistem IoT

---

## 📦 Instalasi

### 1️⃣ Prasyarat Sistem
- Ubuntu 24.04
- ROS 2 Jazzy
- Git
- Python 3.10+
- Colcon
- Gazebo Harmonic

Dokumentasi ROS 2 Jazzy:  
https://docs.ros.org/en/jazzy/Installation.html

---

### 2️⃣ Setup Workspace ROS 2

```bash
mkdir -p ~/lidar_ws/src
cd ~/lidar_ws/src
git clone https://github.com/muhammadazhar15/project_lidar_trusmi.git
cd ~/lidar_ws
colcon build
source install/local_setup.bash
```

---

## ▶️ Menjalankan Sistem

### 🔹 Mode Simulasi LiDAR
```bash
ros2 launch scanner scanner_main.launch.py
```

---

### 🔹 Action Client Scanner LiDAR
```bash
ros2 run scanner scanner_action_client
```

---

### 🔹 Mode Perangkat Nyata (LiDAR LPXT1)
```bash
TODO NEXT
```

---

## 🧩 Node ROS 2

| Node | Fungsi |
|-----|-------|
| gz_ros_control | Control LiDAR Model in Gazebo |
| ros_gz_bridge | Bridge topic between Gazebo and ROS 2 |
| controller_manager | Manage Control in Gazebo from ROS 2 |
| forward_position_controller | Control Joint position in Gazebo |
| joint_state_broadcaster | Broadcast Joint data from Gazebo |
| laserscan_to_pointcloud | Convert Laserscan into Pointcloud2 |
| assembler | Assembly LiDAR Scan data into 3D Pointcloud |
| robot_state_publisher | Handle all tf in LiDAR model |
| rviz2 | Visualize the data |

---

## 📡 Topik ROS 2

| Topic | Tipe | Deskripsi |
|------|------|----------|
| /scan | sensor_msgs/LaserScan | LiDAR raw data |
| /cloud| sensor_msgs/PointCloud2 | Point cloud conversion result from /scan|
| /assembled_cloud| sensor_msgs/PointCloud2 | Point cloud assembly result|
| /imu/data | sensor_msgs/Imu | Data orientasi joint LiDAR|
| /forward_position_controller/commands | std_msgs/Float64MultiArray | Control LiDAR Joint |

---

## 📐 Perhitungan Volume (TODO NEXT)
- Point cloud segmentation
- Ground plane removal
- Volume estimation (voxel / convex hull)
- Output dalam satuan m³

---

## 🌐 Integrasi IoT (TODO NEXT)

Mendukung:
- MQTT
- REST API
- Dashboard

Contoh payload:
```json
{
  "volume": 12.45,
  "unit": "m3",
  "timestamp": "2026-02-10T13:58:00"
}
```

---

## 🧪 Testing & Debugging (TODO NEXT)
```bash
rviz2
ros2 node list
ros2 topic list
ros2 action list
gz sim
```

---

## 🗺️ Roadmap
- ✅ Simulasi sistem LiDAR
- ✅ Laser Assembler Reborn
- ✅ Action Server LiDAR Joint
- [ ] Integrasi Hardware LiDAR LPX-T1
- [ ] Pointcloud Filtering
- [ ] Scan Matching Algorithm
- [ ] Volume Calculation
- [ ] Integrasi QT GUI
- [ ] Integrasi Sistem IoT

---

## 🤝 Kontribusi (TODO NEXT)
1. Fork repository
2. Buat branch fitur
3. Pull request dengan deskripsi jelas

---

## 📄 Lisensi
NCSC License (No Copyright Source Code)

---

## 🙏 Kredit
Dikembangkan **Telkom University** bersama **Trusmi Group**
