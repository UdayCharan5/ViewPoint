  ViewPoint 

Rebuilding the ViewPoint software of Oculus M750D SONAR using its SDK 
# Running the Sonar & Building 2D Sonar SLAM on Ubuntu

**Blueprint Subsea · Oculus M750D**

Complete step-by-step guide: SDK setup → Point cloud extraction → 2D SLAM with sonar-SLAM (ROS/GTSAM)

**Oculus M750D** → **TCP :52100** → **Python driver** → **ROS PointCloud2** → **bruce_slam (GTSAM)** → **2D Map**

---

## Contents

1. [Understanding the Data Flow](#understanding-the-data-flow)
2. [System Prerequisites](#system-prerequisites)
3. [Python Sonar Driver (raw TCP)](#python-sonar-driver-raw-tcp)
4. [ROS2 Sonar Node — Point Cloud + Intensity Publisher](#ros2-sonar-node--point-cloud--intensity-publisher)
5. [2D SLAM Setup using sonar-SLAM repo](#2d-slam-setup-using-sonar-slam-repo)
6. [Launch & Visualize Everything](#launch--visualize-everything)
7. [Troubleshooting Reference](#troubleshooting-reference)

---

## Section 01 — Understanding the Data Flow

The Oculus M750D is a dual-frequency multibeam imaging sonar. It communicates over standard TCP/IP — no special kernel drivers needed. Here's how data moves end-to-end:

```
┌─────────────────────────────────────────────────────────────────┐
│                     Oculus M750D Sonar                          │
│   Default IP: 192.168.2.4   TCP Port: 52100                     │
└──────────────────────┬──────────────────────────────────────────┘
                       │  TCP stream of binary ping packets
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│  Python Driver  (oculus_driver.py)                               │
│  ┌─────────────────┐   ┌────────────────────────────────────┐   │
│  │ Send FireMessage│   │ Receive SimplePingResult2          │   │
│  │ (mode, range,   │   │ ├─ nBeams, nRanges, rangeResolution│   │
│  │  gain, SoS)     │   │ ├─ bearings[] (0.01° steps)        │   │
│  └─────────────────┘   │ ├─ pingStartTime, frequency        │   │
│                         │ └─ imageData[nBeams × nRanges]     │   │
│                         └────────────────────────────────────┘   │
└──────────────────────┬───────────────────────────────────────────┘
                       │ polar → cartesian conversion
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│  ROS2 Node  (sonar_ros_node.py)                                  │
│  Publishes:                                                       │
│  • /sonar/pointcloud  [sensor_msgs/PointCloud2]  (x,y,intensity) │
│  • /sonar/scan        [sensor_msgs/LaserScan]    (2D projection)  │
│  • /sonar/raw         [custom bruce_msgs]         (full ping)     │
└──────────────────────┬───────────────────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────────────────┐
│  bruce_slam  (sonar-SLAM repo)                                   │
│  GTSAM iSAM2 graph SLAM                                          │
│  → /slam/map  OccupancyGrid  (2D map)                            │
│  → /slam/path PoseArray      (trajectory)                        │
└──────────────────────────────────────────────────────────────────┘
```

### Key Data Structures (from Oculus.h SDK)

Everything you extract from the sonar comes packed in these two C structs:

| Field | Type | Meaning |
|---|---|---|
| `nBeams` | uint16 | Number of angular beams per ping (up to 512 on M750d) |
| `nRanges` | uint16 | Number of range bins (samples along each beam) |
| `rangeResolution` | double (m) | Distance represented by one range bin |
| `bearings[]` | int16[] × 0.01° | Angle of each beam in hundredths of a degree |
| `pingStartTime` | double (s) | Seconds since sonar power-on, microsecond precision |
| `frequency` | double (Hz) | Acoustic frequency used (750 kHz or 1.2 MHz for M750d) |
| `imageData` | uint8[] or uint16[] | Intensity values, row = range bin, col = beam |

The image is stored in **polar coordinates**: `image[range_bin][beam_index]`. You convert to Cartesian x,y using:

```
angle_rad = bearing_deg * π / 180
range_m   = range_bin_index * rangeResolution

x = range_m * sin(angle_rad)   ← horizontal (port/starboard)
y = range_m * cos(angle_rad)   ← forward
```

---

## Section 02 — System Prerequisites

> **ℹ️ Recommended OS:** Ubuntu 22.04 LTS. The sonar-SLAM repo targets ROS2 Humble (ships with Ubuntu 22.04). All commands below assume Ubuntu 22.04 + ROS2 Humble.

### Step 1 — Network Setup

The M750D ships with a fixed IP. Connect it directly (or via a switch) to your Ubuntu machine's Ethernet port.

**Step 1.1** — Assign a static IP to your Ubuntu NIC on the same subnet as the sonar:

```bash
# Replace eth0/enp3s0 with your actual interface name (ip link show)
sudo ip addr add 192.168.2.100/24 dev enp3s0
sudo ip link set enp3s0 up

# Verify you can ping the sonar (default sonar IP: 192.168.2.4)
ping 192.168.2.4
```

**Step 1.2** — Confirm the sonar's TCP port is reachable:

```bash
nc -zv 192.168.2.4 52100
# Expected: Connection to 192.168.2.4 52100 port [tcp] succeeded!
```

> **⚠️ If the sonar has a different IP:** Use the official Oculus Viewer (Windows/Mac) to find and reconfigure it first, or use ARP scan: `sudo arp-scan --localnet`

### Step 2 — Install System Dependencies

```bash
# Python deps for the driver
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-scipy

pip3 install numpy scipy matplotlib

# ROS2 Humble (skip if already installed)
sudo apt install -y ros-humble-desktop ros-humble-sensor-msgs \
    ros-humble-geometry-msgs ros-humble-nav-msgs \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions

# GTSAM (required by bruce_slam)
pip3 install gtsam

# Additional SLAM deps
sudo apt install -y ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-rviz2
```

### Step 3 — Clone sonar-SLAM repo

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/jake3991/sonar-SLAM.git

# The repo contains 3 packages:
# bruce/         → top-level launch files
# bruce_msgs/    → custom ROS2 message types
# bruce_slam/    → core SLAM node (GTSAM iSAM2)
```

---

## Section 03 — Python Sonar Driver

This is a standalone Python module that handles the raw TCP connection to the sonar, sends fire commands, receives ping data, and parses the binary packet format defined in `Oculus.h`.

Create this file at `~/ros2_ws/src/oculus_driver/oculus_driver.py`:



#### Test the driver (no ROS needed)

```bash
python3 oculus_driver.py
# You should see:
# [OculusDriver] Connected to 192.168.2.4:52100
# Ping 1: 256 beams × 500 ranges | 3421 points | T=22.3°C
# Ping 2: ...
# After 5 pings: saves /tmp/sonar_pointcloud.png
```

---

## Section 04 — ROS2 Sonar Node

This ROS2 node wraps the driver and publishes three topics that the sonar-SLAM pipeline consumes.

### Package Structure

```
~/ros2_ws/src/oculus_driver/
├── oculus_driver/
│   ├── __init__.py
│   ├── oculus_driver.py        ← the driver from Section 03
│   └── sonar_ros_node.py       ← this section
├── package.xml
├── setup.py
└── setup.cfg
```






### Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select oculus_driver
source install/setup.bash
```

---

## Section 05 — 2D SLAM Setup — sonar-SLAM repo

> **ℹ️ About the repo:** `jake3991/sonar-SLAM` implements graph-based SLAM with GTSAM iSAM2. It was designed for a BlueROV with a DVL + IMU + imaging sonar. For 2D-only SLAM (no DVL/IMU), we configure it with a scan-matching odometry fallback.

### Step 1 — Build sonar-SLAM

```bash
cd ~/ros2_ws
# Install Python deps for bruce_slam
pip3 install gtsam numpy scipy open3d

# Build all packages
colcon build --symlink-install
source install/setup.bash
```

### Step 2 — Add a TF (transform) broadcaster

SLAM needs a coordinate frame tree. Create `~/ros2_ws/src/oculus_driver/oculus_driver/tf_broadcaster.py`:



### Step 3 — SLAM Toolbox config (2D)

Instead of bruce_slam's full 3D pipeline, we wire our `/sonar/scan` topic into **slam_toolbox** — a battle-tested 2D SLAM system in ROS2. Create `~/ros2_ws/src/oculus_driver/config/slam_toolbox_params.yaml`:


### Step 4 — Launch file

Create `~/ros2_ws/src/oculus_driver/launch/slam.launch.py`:



Register the new executables in `setup.py` entry_points:

```python
'console_scripts': [
    'sonar_node = oculus_driver.sonar_ros_node:main',
    'tf_static  = oculus_driver.tf_broadcaster:main',
],
```

---

## Section 06 — Launch & Visualize Everything

**Terminal 1** — Build everything and launch the full stack:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch oculus_driver slam.launch.py \
    sonar_ip:=192.168.2.4 \
    range_m:=10.0 \
    gain:=40.0
```

**Terminal 2** — Verify topics are publishing:

```bash
source ~/ros2_ws/install/setup.bash

# Check all sonar topics are live
ros2 topic list | grep sonar

# Check point cloud rate (~10Hz expected)
ros2 topic hz /sonar/pointcloud

# Inspect one ping's metadata
ros2 topic echo /sonar/scan --once
```

**RViz2 Setup** — In the RViz2 window, add these displays:

- **Map** → Topic: `/map` — this is your 2D SLAM map (OccupancyGrid)
- **PointCloud2** → Topic: `/sonar/pointcloud` → Color: `intensity`
- **LaserScan** → Topic: `/sonar/scan`
- **TF** → shows coordinate frames
- Fixed Frame: `map`

### Save the Map

When you're happy with the 2D map:

```bash
# Save occupancy grid as image + YAML
ros2 run nav2_map_server map_saver_cli -f ~/my_sonar_map

# Creates:
#   ~/my_sonar_map.pgm  (greyscale map image)
#   ~/my_sonar_map.yaml (resolution, origin metadata)
```

### Using bruce_slam directly (alternative)

If you want the full GTSAM graph from the sonar-SLAM repo instead of slam_toolbox:

```bash
# Bruce SLAM expects /PointCloud topic + odometry
# Remap our topic:
ros2 launch bruce_slam slam.launch.py \
    __remapping:=/PointCloud:=/sonar/pointcloud
```

---

## Section 07 — Troubleshooting Reference

| Symptom | Likely Cause | Fix |
|---|---|---|
| `nc -zv 192.168.2.4 52100` times out | Wrong subnet or sonar IP | Run `sudo arp-scan --localnet`; set your PC IP to `192.168.2.x` |
| Connected but 0 pings received | Fire message not sending or malformed | Enable debug: check `MSG_SIMPLE_FIRE = 0x15`; send a dummy message first |
| Point cloud looks scrambled | Bearings array misaligned | Check `brg_offset = offset + PING_RESULT_SIZE` matches struct layout; print `bearings_deg.min()/.max()` |
| LaserScan has zero ranges | Intensity threshold too high | Lower `intensity_threshold` to `0.02` |
| SLAM map not updating | TF tree broken | Run `ros2 run tf2_tools view_frames`; ensure `map→odom→base_link→sonar_link` chain exists |
| Map drifts badly | No odometry (robot moving) | Add wheel odometry or DVL; or keep sonar static for mapping; increase `minimum_travel_distance` |
| `gtsam` import error | Wrong Python gtsam build | `pip3 install gtsam --force-reinstall` |
| Image shape mismatch | M750d sent 16-bit data | Check `data_size` field; use `uint16` decode path |

### Useful Debug Commands

```bash
# Record all sonar data to a bag file
ros2 bag record /sonar/pointcloud /sonar/scan /sonar/image -o sonar_bag

# Replay later (without real sonar)
ros2 bag play sonar_bag

# Check sonar image in terminal
ros2 topic echo /sonar/image --no-arr | head -20

# TF tree visualization
ros2 run tf2_tools view_frames
evince frames.pdf

# Print sonar metadata every ping
ros2 topic echo /sonar/scan | grep -E "range_min|range_max|angle"
```

> **✅ Next step after 2D SLAM works:** Add real odometry from a DVL or wheel encoders to the TF tree (`odom→base_link`), then enable the full bruce_slam GTSAM graph for loop-closure and drift correction. The driver and point cloud code you built here works unchanged for 3D as well — just add a `z` component from depth sensor data.

---

*Oculus M750D · SDK v1.15.168 · ROS2 Humble · Ubuntu 22.04 · sonar-SLAM (jake3991)*
