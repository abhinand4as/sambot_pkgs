# Mapping with Sambot in the Square Room World

This guide walks through building a 2-D occupancy map of the
`room_with_walls/square_room.sdf` environment using **SLAM Toolbox** and
**Nav2**, then saving it for use in autonomous navigation.

---

## Prerequisites

| Item | Notes |
| --- | --- |
| ROS 2 (Jazzy / Humble) | sourced in your shell |
| Gazebo (Harmonic) | `gz` CLI available |
| `slam_toolbox` | `sudo apt install ros-$ROS_DISTRO-slam-toolbox` |
| `nav2_bringup` | `sudo apt install ros-$ROS_DISTRO-nav2-bringup` |
| `teleop_twist_keyboard` | `sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard` |

---

## System Overview

```text
Gazebo sim
  │
  ├─ /scan          (LaserScan)   ──► SLAM Toolbox ──► /map  +  map→odom TF
  ├─ /demo/odom     (Odometry)    ──► robot_localization EKF ──► odom→base_link TF
  ├─ /demo/imu      (Imu)         ──► robot_localization EKF
  └─ /demo/cmd_vel  (TwistStamped)◄── teleop / Nav2
```

**TF tree during mapping:**

```text
map ──► odom ──► base_link ──► base_footprint
                    └──► lidar_link
                    └──► imu_link
                    └──► (wheels, caster)
```

---

## Step 1 — Build and Source the Workspace

```bash
cd ~/ws/robotics/ros2/nav2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2 — Launch the Simulation

Open a dedicated terminal and launch Gazebo with the square-room world plus
the robot, RViz, ROS-Gazebo bridges, and the EKF odometry node:

```bash
source ~/ws/robotics/ros2/nav2_ws/install/setup.bash

ros2 launch sambot_description gazebo_display.launch.py \
  world:=room_with_walls/square_room.sdf
```

> The launch file starts `gz sim -g` (GUI) and `GzServer` (physics) separately,
> so both a Gazebo window and an RViz window will open.

**Verify the robot is running:**

```bash
# Should show /scan, /demo/odom, /demo/imu, /tf, /joint_states
ros2 topic list

# Confirm laser scans are arriving
ros2 topic hz /scan
```

---

## Step 3 — Launch SLAM Toolbox

Open a **second terminal**. Pass the Sambot-tuned params file (see
[Improving Mapping Quality](#improving-mapping-quality) for what it changes):

```bash
source ~/ws/robotics/ros2/nav2_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix sambot_description)/share/sambot_description/config/mapper_params_sambot.yaml
```

`online_async_launch.py` runs the mapper in the background without blocking
the robot's motion — best suited for driving and mapping at the same time.

**What this starts:**

- `slam_toolbox` node subscribing to `/scan` and `/tf`
- Publishing `/map` (OccupancyGrid) and the `map → odom` transform

**Check the map topic is active:**

```bash
ros2 topic hz /map
```

In RViz add a **Map** display (`/map`) to watch the map grow in real time.

---

## Step 4 — Teleoperate to Build the Map

Open a **third terminal**. The robot's drive plugin expects
`geometry_msgs/msg/TwistStamped` on `/demo/cmd_vel`, so pass `stamped:=true`:

```bash
source ~/ws/robotics/ros2/nav2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/demo/cmd_vel \
  -p stamped:=true \
  -p frame_id:=base_link
```

**Keyboard controls:**

| Key | Action |
| --- | --- |
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `u` / `o` | Forward-left / Forward-right |
| `k` | Stop |
| `q` / `z` | Increase / decrease speed |

**Mapping tips for the 7 × 7 m square room:**

1. Start from the centre of the room (robot spawn position).
2. Drive slowly along each wall to get clean wall scans.
3. Circle around each of the 4 orange box obstacles.
4. Return to areas that still show grey (unknown) in the RViz map display.
5. Aim for a map where all walls and obstacles are clearly outlined in black
   and the free space is white.

---

## Step 5 — Save the Map

Once the map looks complete in RViz, open a **fourth terminal** and save:

```bash
source ~/ws/robotics/ros2/nav2_ws/install/setup.bash

mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/square_room_map \
  --ros-args -p use_sim_time:=true
```

This writes two files:

- `~/maps/square_room_map.pgm` — greyscale image of the occupancy grid
- `~/maps/square_room_map.yaml` — metadata (resolution, origin, thresholds)

**Verify the files exist:**

```bash
ls -lh ~/maps/
```

The `.yaml` file will look similar to:

```yaml
image: square_room_map.pgm
resolution: 0.05          # metres per pixel
origin: [-3.6, -3.6, 0.0] # map origin in world frame
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## Step 6 — Inspect the Saved Map (Optional)

View the raw `.pgm` image:

```bash
eog ~/maps/square_room_map.pgm
# or
display ~/maps/square_room_map.pgm   # ImageMagick
```

Reload it in RViz via `map_server`:

```bash
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=$HOME/maps/square_room_map.yaml \
  -p use_sim_time:=true
```

Then lifecycle-activate it:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

## Improving Mapping Quality

This section explains the two symptoms you may encounter and the parameter
changes in `config/mapper_params_sambot.yaml` that address them.

### Root Causes

| Symptom | Default param causing it | Value changed to |
| --- | --- | --- |
| Map lag (slow visual updates) | `map_update_interval: 5.0` s | `1.0` s |
| Drift between keyframes | `minimum_travel_distance: 0.5` m | `0.3` m |
| Drift between keyframes | `minimum_travel_heading: 0.5` rad | `0.3` rad |
| Weak scan matches accepted | `link_match_minimum_response_fine: 0.1` | `0.3` |
| Loop closure misses in 7 m room | `loop_search_maximum_distance: 3.0` m | `4.0` m |
| Loop closure slow to trigger | `loop_match_minimum_chain_size: 10` | `7` |
| Odometry weighted too high | `distance_variance_penalty: 0.5` | `0.3` |
| TF lookup failures in sim | `transform_timeout: 0.2` s | `0.5` s |

### Key Parameter Explanations

**`map_update_interval`**
Controls how often slam_toolbox redraws the `/map` OccupancyGrid topic.
The default of 5 seconds is intentionally slow to save CPU — reduce to `1.0`
for responsive visual feedback during exploration.

**`minimum_travel_distance` / `minimum_travel_heading`**
A new keyframe (scan snapshot) is only created once the robot has moved this
far or rotated this much since the last one. At the default 0.5 m, the robot
can travel half a metre before any scan correction is applied — too coarse for
a 7 m room with obstacles at 1–2 m spacing. Lowering to 0.3 m creates
keyframes more frequently, giving the scan matcher more chances to correct
small accumulated errors.

**`link_match_minimum_response_fine`**
Quality score threshold below which a scan-to-scan match is rejected.
The default 0.1 is very permissive — bad matches silently introduce drift.
Raising to 0.3 means only high-confidence matches are accepted.

**`loop_search_maximum_distance`**
Maximum distance between the current robot position and a previously visited
node for a loop-closure candidate to be considered. The default 3.0 m is too
small for a 7 m room — raised to 4.0 m so the robot can close the loop even
when approaching a corner from a different direction.

**`distance_variance_penalty` / `angle_variance_penalty`**
These scale how much slam_toolbox trusts odometry versus scan matching.
Lower values trust the laser more. In simulation there is no wheel slip, but
Gazebo sim-time jitter can cause minor TF timestamp mismatches that look like
odometry errors — leaning slightly towards scan evidence improves consistency.

### Using the Tuned Params File

The file is at `sambot_description/config/mapper_params_sambot.yaml`.
Step 3 already passes it via `slam_params_file`. To rebuild after editing:

```bash
colcon build --packages-select sambot_description --symlink-install
```

With `--symlink-install` the installed file is a symlink to the source, so
edits to the YAML take effect without rebuilding.

### Best Practices for Clean Maps

#### Drive slowly and systematically

- Keep forward speed at or below 0.2 m/s while near walls (`z` key to reduce).
- Use a coverage pattern: perimeter first, then S-curves through the interior.
- Rotate slowly in corners (< 0.5 rad/s) — fast spins blur the scan.

#### Close the loop deliberately

Drift accumulates during exploration and is corrected the moment slam_toolbox
recognises a previously-visited place (loop closure). At the end of your
mapping run, return the robot to the start position and pause for a few
seconds. Watch the RViz map — if the walls snap into alignment, loop closure
fired successfully.

#### Check scan quality before mapping

```bash
ros2 topic echo /scan --once
```

Confirm `range_min`, `range_max`, and that `ranges` is populated. If ranges
are all `inf` the LiDAR is not hitting anything (robot may be outside the
room, or the sensor link TF is wrong).

#### Verify the full TF chain

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

If this times out, slam_toolbox cannot publish the `map → odom` transform —
check that it started correctly and that `/scan` is arriving.

#### Keep `use_sim_time` consistent

All three nodes (simulation, slam_toolbox, teleop) must use the same clock.
If any node runs on wall-clock time while others use sim time, TF lookups
will fail with timestamp errors. The `use_sim_time:=true` flags in Steps 2
and 3 ensure this.

---

## What's Next — Autonomous Navigation

With the saved map you can run the full Nav2 stack (AMCL localisation +
path planning + MPPI controller):

```bash
# Terminal 1 — simulation (same as mapping)
ros2 launch sambot_description gazebo_display.launch.py \
  world:=room_with_walls/square_room.sdf

# Terminal 2 — Nav2 navigation with the saved map
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=$HOME/maps/square_room_map.yaml \
  params_file:=$(ros2 pkg prefix sambot_description)/share/sambot_description/config/nav2_params.yaml
```

Then use the **Nav2 Goal** tool in RViz (or the **2D Pose Estimate** button
to initialise AMCL) to send the robot to goal poses.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| Map not updating | `/scan` not arriving | Check `ros2 topic hz /scan`; verify bridge is running |
| Robot drifts off-map | EKF / TF issue | Confirm `odom → base_link` TF with `ros2 run tf2_ros tf2_echo odom base_link` |
| Teleop has no effect | Wrong message type | Ensure `stamped:=true` and topic is `/demo/cmd_vel` |
| Walls appear blurry | Robot moving too fast | Reduce speed with `z` key; keep `vx < 0.2 m/s` while scanning walls |
| Unknown space remains | Area not covered | Drive closer to corners and around all obstacles |
| Loop closure not firing | Search radius too small | Increase `loop_search_maximum_distance` or revisit start position |
| TF lookup failures | Sim-time mismatch | Ensure `use_sim_time:=true` on all nodes |
