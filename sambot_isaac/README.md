# sambot_isaac

Isaac Sim support for **Sambot**. Runs the same robot and room as the Gazebo setup in [sambot_description](../sambot_description) and publishes the **same ROS 2 topics and frames**, so the existing EKF, SLAM Toolbox and Nav2 configs are reused unchanged.

---

## Status

| Feature | State |
|---|---|
| Robot import (URDF → USD) | Working |
| Square room world (SDF → USD) | Working |
| Driving via `/demo/cmd_vel` (teleop / Nav2) | Working |
| `/clock`, `/demo/odom`, `/demo/imu`, `/joint_states`, `/scan` | Working |
| TF tree + EKF (`/odometry/filtered`) | Working |
| SLAM mapping | Not yet verified in Isaac |
| Nav2 navigation | Not yet verified in Isaac |
| RGBD camera (`/depth_camera/*`) | Not implemented |

---

## Prerequisites

| Item | Notes |
|---|---|
| Isaac Sim 6.0.1 | Default install path `~/apps/robotics/isaacsim` (override with `isaac_sim_path:=`) |
| ROS 2 Jazzy | Sourced in your shell |
| `sambot_description` | Built in the same workspace (robot model, EKF, Nav2 and SLAM configs) |
| `robot_localization`, `slam_toolbox`, `nav2_bringup`, `teleop_twist_keyboard` | `sudo apt install ros-jazzy-<name>` |

Isaac Sim runs with its own bundled ROS 2 Jazzy libraries (`exts/isaacsim.ros2.core/jazzy/lib`). The launch file sets that environment for the Isaac process only. **Do not export Isaac's library path in the shell you launch from**, or the system ROS nodes will crash (see [Troubleshooting](#troubleshooting)).

---

## Package Structure

```text
sambot_isaac/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── isaac_display.launch.py   # Isaac Sim + robot_state_publisher + EKF + relay + RViz
├── scripts/
│   ├── run_sim.py                # Loads world + robot, sensors, ROS 2 OmniGraph (Isaac Python)
│   ├── build_world.py            # Gazebo SDF world → USD world (Isaac Python)
│   ├── import_robot.py           # urdf/sambot.urdf → usd/sambot (Isaac Python)
│   └── cmd_vel_relay.py          # /demo/cmd_vel (TwistStamped) → /isaac/cmd_vel (Twist)
├── urdf/
│   └── sambot.urdf               # xacro-expanded sambot_sensors.urdf, input to the importer
└── usd/
    ├── sambot/                   # Imported robot asset (sambot.usda + payloads)
    └── worlds/
        └── square_room.usda      # Room from room_with_walls/square_room.sdf
```

`run_sim.py`, `build_world.py` and `import_robot.py` run inside **Isaac Sim's Python** (`~/apps/robotics/isaacsim/python.sh`), not the system Python.

---

## Build

```bash
cd ~/ws/robotics/ros2/nav2_ws
colcon build --symlink-install --packages-select sambot_description sambot_isaac
source install/setup.bash
```

---

## Running the Simulation

Use a **fresh terminal** for each command below.

### Terminal 1 — Isaac Sim + ROS stack

```bash
source ~/ws/robotics/ros2/nav2_ws/install/setup.bash
ros2 launch sambot_isaac isaac_display.launch.py
```

This brings up:

- Isaac Sim with the square room and Sambot; the simulation starts playing automatically
- `robot_state_publisher` (same `sambot_odometry_sensors.sdf` model as Gazebo, which defines `imu_link`)
- EKF node (`sambot_description/config/ekf.yaml`) publishing `/odometry/filtered` and `odom → base_link`
- `cmd_vel_relay`
- RViz with `sambot_description/rviz/config.rviz`

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `headless` | `False` | Run Isaac Sim without a window |
| `rviz` | `True` | Start RViz |
| `rvizconfig` | `sambot_description/rviz/config.rviz` | Absolute path to the RViz config file |
| `world` | `usd/worlds/square_room.usda` | Absolute path to a world USD |
| `model` | `sambot_description/urdf/sambot_odometry_sensors.sdf` | Robot description for `robot_state_publisher` |
| `use_sim_time` | `True` | Use Isaac Sim's `/clock` |
| `isaac_sim_path` | `~/apps/robotics/isaacsim` | Isaac Sim install directory |

### Terminal 2 — Keyboard teleop

Same command as with Gazebo:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true --remap cmd_vel:=/demo/cmd_vel
```

`i` forward, `,` backward, `j`/`l` turn, `k` stop. The robot keeps its last command, so press `k` before quitting teleop.

---

## ROS 2 Interface

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Isaac → ROS | 60 Hz, one tick per physics step |
| `/demo/odom` | `nav_msgs/msg/Odometry` | Isaac → ROS | `odom` → `base_link` |
| `/demo/imu` | `sensor_msgs/msg/Imu` | Isaac → ROS | frame `imu_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | Isaac → ROS | `drivewhl_l_joint`, `drivewhl_r_joint` |
| `/scan` | `sensor_msgs/msg/LaserScan` | Isaac → ROS | frame `lidar_link`, 0.12–3.5 m, 10 Hz |
| `/demo/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS → relay | What teleop and Nav2 publish |
| `/isaac/cmd_vel` | `geometry_msgs/msg/Twist` | relay → Isaac | Isaac's `ROS2SubscribeTwist` only accepts unstamped `Twist` |

Isaac does **not** publish `/tf`. As in the Gazebo setup, `robot_state_publisher` publishes the robot's fixed frames, the EKF owns `odom → base_link`, and SLAM Toolbox or AMCL provides `map → odom`.

### Differences from Gazebo

| | Gazebo | Isaac Sim |
|---|---|---|
| `/demo/odom` | DiffDrive plugin (wheel odometry, drifts) | Ground-truth pose of `base_link` (no drift) |
| `/scan` rate | 5 Hz | 10 Hz (`Example_Rotary_2D` RTX lidar config) |
| `/scan/points`, `/depth_camera/*` | Published | Not published yet |
| Physics | Gazebo (DART) | PhysX, 60 Hz |

Mapping in Isaac will look cleaner than in Gazebo because the odometry does not drift.

---

## Mapping and Navigation

The commands match the Gazebo workflow in [sambot_description/README.md](../sambot_description/README.md) and [docs/MAPPING.md](../docs/MAPPING.md); only Terminal 1 changes. These have not yet been verified end-to-end in Isaac.

### SLAM mapping

```bash
# Terminal 1
ros2 launch sambot_isaac isaac_display.launch.py \
  rvizconfig:=$(ros2 pkg prefix --share sambot_description)/rviz/navigation.rviz

# Terminal 2
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix --share sambot_description)/config/mapper_params_sambot.yaml

# Terminal 3: drive with teleop (see above), then save the map
ros2 run nav2_map_server map_saver_cli -f ~/maps/isaac_square_room_map \
  --ros-args -p use_sim_time:=true
```

### Navigation with a saved map

```bash
# Terminal 1
ros2 launch sambot_isaac isaac_display.launch.py \
  rvizconfig:=$(ros2 pkg prefix --share sambot_description)/rviz/navigation.rviz

# Terminal 2
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$(ros2 pkg prefix --share sambot_description)/world/room_with_walls/maps/square_room_map.yaml \
  params_file:=$(ros2 pkg prefix --share sambot_description)/config/nav2_params.yaml
```

The Isaac room is generated from the same SDF as the Gazebo room, so the Gazebo-built `square_room_map.yaml` lines up with it.

---

## Regenerating Assets

Run these with Isaac's Python from the package directory (`src/sambot_pkgs/sambot_isaac`).

### Robot (after changing the URDF)

```bash
source /opt/ros/jazzy/setup.bash
xacro ../sambot_description/urdf/sambot_sensors.urdf -o urdf/sambot.urdf
~/apps/robotics/isaacsim/python.sh scripts/import_robot.py
```

Writes `usd/sambot/`. Wheel drives, friction, IMU and lidar are added at runtime by `run_sim.py`, so they survive a re-import.

### World (after changing the SDF, or to convert another world)

```bash
~/apps/robotics/isaacsim/python.sh scripts/build_world.py
~/apps/robotics/isaacsim/python.sh scripts/build_world.py \
  --sdf ../sambot_description/world/my_world.sdf
```

Writes `usd/worlds/<sdf name>.usda`. Only SDF models made of boxes are converted; meshes, cylinders and `<include>`d models are skipped. Colours come from the SDF's `Gazebo/<Colour>` material names; edit `PALETTE` in `build_world.py` to change them. The world file can also be opened on its own in Isaac Sim (File > Open).

### Running the sim without the launch file

Useful for debugging the Isaac side alone. Isaac's environment must be set in this shell only:

```bash
export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$HOME/apps/robotics/isaacsim/exts/isaacsim.ros2.core/jazzy/lib
~/apps/robotics/isaacsim/python.sh scripts/run_sim.py [--headless] [--world WORLD.usda] [--debug-poses]
```

`--debug-poses` prints the world pose of the body and wheels every 2 s. Do not launch ROS nodes from this shell afterwards.

---

## Implementation Notes

- **One physics scene.** The world's physics scene arrives through a USD reference, which Isaac's `SimulationManager` does not detect, so it would add a second `/PhysicsScene`. Isaac's simulation time then advances once per scene per step, making `/clock` and every message stamp run 2–3× faster than physics, and the EKF overshoots. `run_sim.py` registers `/World/PhysicsScene` as the default scene and prints a warning if another scene appears.
- **Ground plane.** Physics uses an infinite collision plane; a thin scaled box let the wheels' cylinder colliders sink through.
- **Caster friction.** The URDF caster is a fixed sphere. Like the Gazebo SDF (`mu = 0.001`), it gets a near-frictionless physics material; the wheels get friction 1.0.
- **Acceleration limits.** The differential controller is limited to 1 m/s², matching the Gazebo DiffDrive plugin's `max_linear_acceleration`.
- **Faceted wheels.** The wheels look polygonal in the viewport. This is only how the renderer draws USD cylinders; the colliders are exact cylinders.

---

## Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| Launch stops with `LD_LIBRARY_PATH contains Isaac Sim's bundled ROS libs` | The shell has Isaac's library path exported (e.g. from running `python.sh` by hand). Open a fresh terminal. |
| `ekf_node`, `rviz2` or `robot_state_publisher` die with `cannot open shared object file` | Same cause as above. |
| `ROS2 Bridge startup failed` in the Isaac log | Isaac's bundled ROS libs were not found. In Isaac Sim 6.x they are in `exts/isaacsim.ros2.core/jazzy/lib` (not `isaacsim.ros2.bridge`). |
| `[run_sim] WARNING: physics scenes [...]` | A second physics scene was created; `/clock` will run faster than physics. Check the world USD contains exactly one `PhysicsScene` at `/World/PhysicsScene`. |
| Robot keeps moving after teleop exits | The controller holds the last command. Publish a zero command or restart teleop and press `k`. |
| Warnings about `MotionBVH`, deprecated `targetPrim` or `ITimeline` callbacks | Harmless. |

---

## License

Apache 2.0 — see [package.xml](package.xml) for details.
