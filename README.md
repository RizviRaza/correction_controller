# correction_controller

A ROS 2 package that performs **open-loop correction** of a drone using a provided FLU-frame `Pose`. The node applies a 3D translation and yaw rotation sequentially, using only velocity commands and no feedback. It also integrates **obstacle avoidance** via LaserScan range checking.

---

## 🚀 Features

- Receives a 6DOF correction transform (`Pose`) in FLU frame.
- Executes **open-loop linear movement** toward a given offset.
- Applies **open-loop yaw rotation** based on quaternion orientation.
- Pauses both translation and rotation if **any obstacle is within a configurable distance**.
- Accepts fully configurable topics and parameters via launch file.

---

## 🧠 Behavior

1. Subscribes to `Pose` correction (translation + rotation).
2. **Translates first** in original FLU direction (to preserve frame correctness).
3. After reaching target distance, **rotates to desired yaw**.
4. Stops and awaits the next correction.
5. Pauses motion if any LaserScan reading is too close.

---

## 📄 Files

| File | Purpose |
|------|---------|
| `correction_node.py` | Main node implementation |
| `launch/correction_launch.py` | Launch file with topic remapping and parameters |

---

## 📡 Topics

### Subscribed:
- `/correction_pose` (`geometry_msgs/Pose`): Correction transform to execute.
- `/scan` (`sensor_msgs/LaserScan`): Used to detect nearby obstacles.

### Published:
- `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands sent to drone.

---

## 🛠 Parameters

All parameters can be set via the launch file or `ros2 param set`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `velocity` | double | `0.1` | Linear velocity in m/s |
| `angular_velocity` | double | `0.5` | Angular velocity in rad/s |
| `min_obstacle_distance` | double | `0.5` | Distance threshold for obstacle pausing |

---

## 🚀 Launching

### Default launch:
```bash
ros2 launch correction_controller correction_launch.py
