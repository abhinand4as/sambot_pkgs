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
│   └── bridge_config.yaml      # ROS ↔ Gazebo topic bridge config
├── launch/
│   ├── display.launch.py       # Launch robot in RViz (URDF)
│   └── gazebo_display.launch.py  # Launch robot in Gazebo + RViz (SDF)
├── rviz/
│   └── config.rviz             # Pre-configured RViz layout
├── urdf/
│   ├── sambot_base.urdf        # Robot description (xacro, RViz)
│   └── sambot_odometry.sdf     # Robot description with odometry plugin (Gazebo)
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

`gazebo_display.launch.py` starts a full simulation stack: Gazebo server + GUI, `robot_state_publisher`, RViz, and a ROS–Gazebo bridge for odometry and command topics. The robot model used here is `sambot_odometry.sdf`, which includes the differential drive + odometry plugin.

### Launch

```bash
ros2 launch sambot_description gazebo_display.launch.py
```

This brings up:

- Gazebo (`gz sim`) with `my_world.sdf`
- Sambot spawned at z = 0.65 m
- ROS–Gazebo bridge (configured by `bridge_config.yaml`)
- RViz with the pre-configured layout

### Gazebo launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `True` | Sync ROS time with Gazebo simulation clock |
| `model` | `urdf/sambot_odometry.sdf` | Absolute path to the SDF model |
| `rvizconfig` | `rviz/config.rviz` | Absolute path to the RViz config file |

### Drive the robot with keyboard teleop

In a second terminal, install and run `teleop_twist_keyboard`. The `stamped:=true` flag sends `TwistStamped` messages, and the topic is remapped to match the bridge:

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

## License

Apache 2.0 — see [package.xml](package.xml) for details.
