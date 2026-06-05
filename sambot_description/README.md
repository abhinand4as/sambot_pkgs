# sambot_description

ROS 2 description package for **Sambot** — a simple two-wheeled differential drive robot built following the [Nav2 URDF setup guide](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html).

---

## Robot Overview

Sambot is a ground mobile robot with a rectangular base, two rear drive wheels, and a front caster wheel.

| Parameter | Value |
|---|---|
| Drive type | Differential drive |
| Base size (L × W × H) | 0.42 m × 0.31 m × 0.18 m |
| Wheel radius | 0.10 m |
| Wheel width | 0.04 m |
| Caster wheel | Front, fixed spherical |
| Base mass | 15 kg |
| Wheel mass | 0.5 kg each |

### Links

| Link | Type | Description |
|---|---|---|
| `base_link` | Box | Main chassis (cyan) |
| `base_footprint` | Virtual | Ground projection of the base |
| `drivewhl_l_link` | Cylinder | Left drive wheel (gray) |
| `drivewhl_r_link` | Cylinder | Right drive wheel (gray) |
| `front_caster` | Sphere | Front caster wheel (cyan) |

---

## Package Structure

```
sambot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── bridge_config.yaml      # ROS ↔ Gazebo topic bridge config
│   └── ekf.yaml                # robot_localization EKF parameters
├── launch/
│   ├── display.launch.py       # Launch robot in RViz (URDF)
│   └── gazebo_display.launch.py  # Launch robot in Gazebo + RViz (SDF)
├── rviz/
│   ├── config.rviz             # Default RViz layout
│   └── sensor_config.rviz      # RViz layout with sensor displays
├── urdf/
│   ├── sambot_base.urdf        # Base robot description (RViz only)
│   ├── sambot_base.sdf         # Base robot description (Gazebo)
│   ├── sambot_odometry.sdf     # Robot with diff-drive + odometry plugin
│   ├── sambot_odometry_sensors.sdf  # Robot with odometry, IMU, LiDAR, camera
│   └── sambot_sensors.urdf     # Robot with sensors (URDF, RViz)
└── world/
    └── my_world.sdf            # Gazebo simulation world
```

---

## Dependencies

Runtime dependencies declared in [package.xml](package.xml):

- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `xacro`

---

## Build

```bash
cd ~/ws/robotics/ros2/nav2_ws
colcon build --packages-select sambot_description
source install/setup.bash
```

---

## Viewing the Robot in RViz

`display.launch.py` starts `robot_state_publisher`, `joint_state_publisher` (or its GUI variant), and RViz with the pre-configured layout.

### Default launch (with joint GUI)

```bash
ros2 launch sambot_description display.launch.py
```

### Launch without the joint state GUI

```bash
ros2 launch sambot_description display.launch.py gui:=false
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `gui` | `True` | Enable `joint_state_publisher_gui` for interactive joint control |
| `model` | `urdf/sambot_base.urdf` | Absolute path to the URDF/xacro model |
| `rvizconfig` | `rviz/config.rviz` | Absolute path to the RViz config file |

### Custom model or RViz config

```bash
ros2 launch sambot_description display.launch.py \
  model:=/path/to/your/robot.urdf \
  rvizconfig:=/path/to/your/config.rviz
```

---

## Odometry in Gazebo

`gazebo_display.launch.py` starts a full simulation stack: Gazebo server + GUI, `robot_state_publisher`, an EKF node (`robot_localization`), RViz, and a ROS–Gazebo bridge. The default model is `sambot_odometry_sensors.sdf`, which includes the differential drive + odometry plugin and all sensors.

### Launch

```bash
ros2 launch sambot_description gazebo_display.launch.py
```

This brings up:

- Gazebo (`gz sim`) with `my_world.sdf`
- Sambot spawned at z = 0.15 m
- ROS–Gazebo bridge (configured by `bridge_config.yaml`)
- EKF node fusing odometry and IMU into `/odometry/filtered`
- RViz with the default layout

### Gazebo launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `True` | Sync ROS time with Gazebo simulation clock |
| `model` | `urdf/sambot_odometry_sensors.sdf` | Absolute path to the SDF model |
| `rvizconfig` | `rviz/config.rviz` | Absolute path to the RViz config file |

### Drive the robot with keyboard teleop

In a second terminal, run `teleop_twist_keyboard`. The `stamped:=true` flag sends `TwistStamped` messages, and the topic is remapped to match the bridge:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true --remap cmd_vel:=/demo/cmd_vel
```

| Key | Action |
|---|---|
| `i` | Move forward |
| `,` | Move backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `q` / `z` | Increase / decrease max speed |

> **Note:** `teleop_twist_keyboard` must be installed separately.
>
> ```bash
> sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
> ```

---

## Sensors in Gazebo

`sambot_odometry_sensors.sdf` adds three sensors on top of the odometry model. Pass `sensor_config.rviz` to visualise all sensor streams in RViz.

### Sensors

| Sensor | Link | Topic(s) | Rate |
| --- | --- | --- | --- |
| IMU | `imu_link` | `/demo/imu` | 100 Hz |
| 2D LiDAR (360°) | `lidar_link` | `/scan`, `/scan/points` | 5 Hz |
| RGBD Camera | `camera_link` | `/depth_camera/image_raw`, `/depth_camera/points`, `/depth_camera/camera_info` | 5 Hz |

### Launch with sensor config

```bash
ros2 launch sambot_description gazebo_display.launch.py \
  rvizconfig:=/home/abhinandas/ws/robotics/ros2/nav2_ws/src/sambot_pkgs/sambot_description/rviz/sensor_config.rviz
```

This brings up the same full simulation stack as the odometry example, with RViz configured to display the LiDAR scan, point cloud, and camera feeds.

### Bridged ROS topics

All sensor data is forwarded from Gazebo to ROS via `bridge_config.yaml`:

| Topic | Type | Direction |
| --- | --- | --- |
| `/demo/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS |
| `/scan/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS |
| `/depth_camera/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS |
| `/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS |
| `/depth_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS |
| `/demo/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/demo/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS → Gazebo |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |

---

## Mapping and Localization with Nav2

This example runs SLAM-based mapping alongside the Nav2 navigation stack so you can observe the global and local costmaps being built in real time.

Each command runs in its own terminal (source `install/setup.bash` in each).

### Terminal 1 — Simulation

Launch Gazebo with the sensor-equipped robot and open RViz with the sensor layout:

```bash
ros2 launch sambot_description gazebo_display.launch.py \
  rvizconfig:=/home/abhinandas/ws/robotics/ros2/nav2_ws/src/sambot_pkgs/sambot_description/rviz/sensor_config.rviz
```

Starts Gazebo, spawns Sambot, brings up the ROS–Gazebo bridge, EKF, and RViz pre-configured to display sensor streams (LiDAR, camera, odometry).

### Terminal 2 — SLAM Toolbox (mapping)

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Runs SLAM Toolbox in asynchronous online mode. Subscribes to `/scan` and publishes an incrementally built occupancy grid on `/map`, along with the `map → odom` transform needed by Nav2.

### Terminal 3 — Nav2 Navigation Stack

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/abhinandas/ws/robotics/ros2/nav2_ws/src/sambot_pkgs/sambot_description/config/nav2_params.yaml
```

Starts the full Nav2 stack (planner, controller, costmap servers, behaviour tree) using the tuned parameters in `config/nav2_params.yaml`. The **global costmap** inflates obstacles on the SLAM map for path planning; the **local costmap** uses live sensor data for reactive obstacle avoidance.

### What to observe in RViz

Add the following displays to watch the costmaps:

| Display | Topic |
| --- | --- |
| Map | `/map` |
| Global costmap | `/global_costmap/costmap` |
| Local costmap | `/local_costmap/costmap` |

Once all three stacks are running, use the **Nav2 Goal** tool in RViz to send a navigation goal and watch the planner generate a path through the global costmap while the controller tracks it using the local costmap.

---

## License

Apache 2.0 — see [package.xml](package.xml) for details.
