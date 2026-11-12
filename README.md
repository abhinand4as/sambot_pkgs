# sambot_pkgs

ROS 2 Jazzy packages for **Sambot**, a two-wheeled differential drive robot with a 2D lidar, IMU and RGBD camera, built following the [Nav2 setup guides](https://docs.nav2.org/setup_guides/index.html). Sambot can be simulated in **Gazebo Harmonic** or **NVIDIA Isaac Sim**, with SLAM mapping and Nav2 navigation on top.

---

## Packages

| Package | Description |
|---|---|
| [sambot_description](sambot_description) | Robot model (URDF/SDF), Gazebo worlds and launch, EKF, SLAM Toolbox and Nav2 configs, RViz layouts, saved maps |
| [sambot_isaac](sambot_isaac) | Isaac Sim support: robot and world USD assets, simulation script and launch file publishing the same ROS 2 topics as Gazebo |

Both simulators expose the same ROS 2 interface (`/scan`, `/demo/odom`, `/demo/imu`, `/demo/cmd_vel`, `/clock`), so the EKF, SLAM and Nav2 setup in `sambot_description` works with either.

```text
Gazebo (sambot_description) ─┐   /scan, /demo/odom,
                             ├─► /demo/imu, /clock ──► EKF ──► SLAM Toolbox / AMCL ──► Nav2
Isaac Sim (sambot_isaac) ────┘                                                          │
          ▲                                                                             │
          └──────────────────────────── /demo/cmd_vel ◄─────────────────────────────────┘
```

---

## Repository Layout

```text
sambot_pkgs/
├── README.md
├── docs/
│   └── MAPPING.md            # Step-by-step SLAM mapping guide (Gazebo)
├── sambot_description/       # Robot model, Gazebo simulation, Nav2/SLAM/EKF configs
└── sambot_isaac/             # Isaac Sim simulation
```

---

## Prerequisites

| Item | Needed for |
|---|---|
| ROS 2 Jazzy | Everything |
| Gazebo Harmonic + `ros_gz` | Gazebo simulation |
| Isaac Sim 6.0.1 (at `~/apps/robotics/isaacsim` by default) | Isaac Sim simulation |
| `robot_localization`, `slam_toolbox`, `nav2_bringup`, `teleop_twist_keyboard` | Odometry fusion, mapping, navigation, driving |

```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard
```

---

## Build

Clone into the `src/` folder of a colcon workspace:

```bash
cd ~/ws/robotics/ros2/nav2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

### Gazebo

```bash
ros2 launch sambot_description gazebo_display.launch.py world:=room_with_walls/square_room.sdf
```

### Isaac Sim

```bash
ros2 launch sambot_isaac isaac_display.launch.py
```

Launch from a fresh terminal; see [sambot_isaac/README.md](sambot_isaac/README.md#prerequisites).

### Drive the robot (either simulator)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true --remap cmd_vel:=/demo/cmd_vel
```

---

## Documentation

| Topic | Where |
|---|---|
| Robot model, sensors, Gazebo launch, Nav2 workflows | [sambot_description/README.md](sambot_description/README.md) |
| Building and saving a map with SLAM Toolbox | [docs/MAPPING.md](docs/MAPPING.md) |
| Gazebo worlds | [sambot_description/world/README.md](sambot_description/world/README.md) |
| Isaac Sim setup, topics, regenerating assets, troubleshooting | [sambot_isaac/README.md](sambot_isaac/README.md) |

---

## License

Apache 2.0
