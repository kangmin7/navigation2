# Navigation2 for PX4 Drone

Custom [Navigation2](https://github.com/ros-planning/navigation2) for integration
with [PX4-Autopilot](https://github.com/kangmin7/PX4-Autopilot) and [MAVROS](https://github.com/kangmin7/mavros),
targeting ROS 2 Humble

---

## Changes

- Added `nav2_bringup/launch/px4_mavros_navigation_launch.py` — Nav2 launch for PX4 + MAVROS
- Added `nav2_bringup/params/nav2_px4_params.yaml` — parameters tuned for PX4 drone (MPPI Omni controller)
- Added `nav2_bringup/scripts/px4_mavros_bridge.py` — converts Nav2 `cmd_vel` to MAVROS velocity setpoint

---

## Running

```bash
ros2 launch nav2_bringup px4_mavros_navigation_launch.py use_sim_time:=true
```
