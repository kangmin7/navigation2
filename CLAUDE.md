# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Source ROS 2 first
source /opt/ros/humble/setup.bash

# Build all packages from workspace root
cd ~/ros2_navigation
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select nav2_controller

# Build with debug info
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the workspace after building
source install/setup.bash
```

## Testing

```bash
# Run all tests for a package
colcon test --packages-select nav2_controller
colcon test-result --verbose

# Run a single GTest directly (after building)
./build/nav2_controller/test_controller_server

# Run a single pytest
pytest src/navigation2/nav2_costmap_2d/test/test_costmap.py -v

# Run linting checks
ament_cpplint src/navigation2/nav2_controller/src/
ament_flake8 src/navigation2/nav2_simple_commander/
ament_pep257 src/navigation2/nav2_simple_commander/
```

## Architecture Overview

Navigation2 is a **plugin-based, lifecycle-managed** navigation framework. All major nodes implement the ROS 2 lifecycle (configure → activate → deactivate → cleanup), managed by `nav2_lifecycle_manager`.

### Request Flow

1. **User/Application** sends a `NavigateToPose` action goal to `bt_navigator`
2. **`nav2_bt_navigator`** executes a Behavior Tree (XML file) that orchestrates the pipeline
3. **`nav2_planner`** computes a global path (`ComputePathToPose` action) using a planner plugin
4. **`nav2_controller`** executes the path (`FollowPath` action) using a controller plugin
5. **`nav2_velocity_smoother`** smooths `cmd_vel_nav` → publishes final `cmd_vel`
6. **`nav2_smoother`** can optionally smooth the global path before execution
7. **`nav2_behaviors`** handles recovery actions (Spin, BackUp, Wait) when the BT triggers them

### Plugin Interfaces (`nav2_core/`)

All algorithms are plugins loaded at runtime via `pluginlib`:
- `GlobalPlanner` — implemented by: NavFn, Smac (2D/Hybrid-A*/State Lattice), Theta*
- `Controller` — implemented by: DWB, RPP, MPPI, Graceful, RotationShim (wrapper)
- `Smoother` — implemented by: SimpleSmoother, ConstrainedSmoother
- `CostmapLayer` — static, inflation, obstacle, voxel, range, etc.
- `Behavior` — Spin, BackUp, DriveOnHeading, Wait, AssistedTeleop
- `GoalChecker` / `ProgressChecker` — configurable goal tolerance and stall detection

### Costmap System (`nav2_costmap_2d/`)

Two costmap instances run simultaneously:
- **Global costmap**: static map + inflation layers for planning
- **Local costmap**: rolling window with obstacle/voxel layers for control

Costmap plugins are stacked as layers; the master costmap merges them.

### Key Topics / Remappings

The standard `navigation_launch.py` applies these remappings for namespace support:
- `/tf` → `tf`, `/tf_static` → `tf_static`
- `cmd_vel` (controller output) → `cmd_vel_nav`
- `cmd_vel_smoothed` (velocity smoother output) → `cmd_vel` (robot input)

### Launch System (`nav2_bringup/launch/`)

| File | Purpose |
|------|---------|
| `bringup_launch.py` | Top-level: includes localization + navigation, supports SLAM mode |
| `navigation_launch.py` | Navigation stack only (controller, planner, BT, behaviors, etc.) |
| `localization_launch.py` | AMCL + map server |
| `slam_launch.py` | slam_toolbox integration |
| `tb3_simulation_launch.py` | TurtleBot3 Gazebo simulation |

All launch files accept: `namespace`, `use_sim_time`, `params_file`, `autostart`, `use_composition`, `use_respawn`, `log_level`.

`RewrittenYaml` (from `nav2_common.launch`) rewrites the params file at launch time to inject `use_sim_time`, namespace prefix, and other substitutions.

### Parameters (`nav2_bringup/params/nav2_params.yaml`)

Single YAML file configures all nodes. Key sections: `bt_navigator`, `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `velocity_smoother`, `costmap_common_params` (anchor), `local_costmap`, `global_costmap`, `amcl`, `map_server`.

For multi-robot deployments use `nav2_multirobot_params_*.yaml` which use `<robot_namespace>` template substitution.

### `nav2_common` Utilities

- `nav2_common/launch/`: `RewrittenYaml`, `ReplaceString` — runtime YAML/string substitution in launch files
- `nav2_common/cmake/`: `nav2_package()` macro used in every package's `CMakeLists.txt` to enforce C++17 and common compiler flags

### `nav2_simple_commander` (Python API)

High-level Python API for sending navigation goals programmatically. Used for scripting and integration testing without writing ROS 2 action clients directly.
