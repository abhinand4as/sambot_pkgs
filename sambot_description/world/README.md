# Worlds

## Quick Preview (Gazebo only)

```bash
gz sim <path_to_sdf>
```

**Example — square room:**

```bash
gz sim /home/abhinandas/ws/robotics/ros2/nav2_ws/src/sambot_pkgs/sambot_description/world/room_with_walls/square_room.sdf
```

---

## With ROS Launch Stack (robot + RViz + bridges)

Source the workspace first:

```bash
source /home/abhinandas/ws/robotics/ros2/nav2_ws/install/setup.bash
```

Launch with the default world (`my_world.sdf`):

```bash
ros2 launch sambot_description gazebo_display.launch.py
```

Launch with a custom world:

```bash
ros2 launch sambot_description gazebo_display.launch.py \
  world:=room_with_walls/square_room.sdf
```

> **Note:** Run `colcon build` from the workspace root after adding new world files so they are installed to the share directory.

---

## Available Worlds

| File                                | Description                                                                                       |
|-------------------------------------|---------------------------------------------------------------------------------------------------|
| `my_world.sdf`                      | Default open world with a box and sphere                                                          |
| `room_with_walls/square_room.sdf`   | 7 m × 7 m square room, 4 walls (1.0 m tall), no roof, 4 box obstacles — for mapping/navigation   |
