# wheeled-cling

ROS 2 (Jazzy) workspace for a three-wheeled magnetic climbing robot
("3D3S") running on Gazebo Sim. The robot climbs horizontal and vertical
steel pipes using an adhesion model and a unified cylinder/polygon
surface-graph planner with RViz Publish Point goal selection.

## Packages

| Package | Purpose |
|---|---|
| `robot_3d3s` | The full robot: URDF, Gazebo worlds, swerve controllers, adhesion plugin (C++), Python adhesion controller, surface goal planner, RViz dashboard and pipe-surface grids. |
| `swerve_controller` | ros2_control controller for swerve drive (kinematics, odometry, twist command interface). |
| `swerve_hardware` | ros2_control hardware interface used by Gazebo. |
| `omnidirectional_controllers` | Extra holonomic / omni-directional controllers. |
| `test_swerve_control` | Scaffolding tests for the swerve controller. |

## Build

Requires ROS 2 Jazzy with `ros-jazzy-ros-gz-bridge`,
`ros-jazzy-ros-gz-sim`, `ros-jazzy-robot-state-publisher`,
`ros-jazzy-tf2-ros`, `ros-jazzy-xacro`, `ros-jazzy-rviz2`,
`ros-jazzy-controller-manager`, plus a Python venv with `PyQt5`.

```bash
source /opt/ros/jazzy/setup.bash
cd <repo-root>
colcon build --packages-select robot_3d3s swerve_controller swerve_hardware \
                         omnidirectional_controllers
source install/setup.bash
```

## Run

The PyQt dashboard drives everything (scenario presets, RViz, teleop,
plot, per-wheel table):

```bash
ros2 run robot_3d3s adhesion_dashboard.py
```

Then in the dashboard:

1. Pick a **Surface preset** (Horizontal pipe / Vertical pipe / Flat
   wall + slide / Mockup pipe).
2. (Optional) Tune the spawn pose, pipe radius, robot mass, target
   angle.
3. Press **Start Launch**.

This spawns the corresponding `*_test.launch.py` from
`src/robot_3d3s/launch/` which:

1. Starts `gz sim` with the SDF world (`src/robot_3d3s/worlds/`).
2. Spawns the robot from the URDF.
3. Bridges Gazebo topics to ROS.
4. Starts the adhesion controller, surface attachment guard, contact
   monitor, the unified **surface goal planner**,
   `experimental_polygon_grid` (RViz pick-point shell), and finally
   `rviz2` with `src/robot_3d3s/config/display.rviz`.

### Selecting a goal on the pipe

In RViz:

1. Click **Publish Point** in the toolbar (crosshair icon).
2. Click anywhere on the pipe surface. You will see a small blue
   sphere appear at the click point, and a green poly-line route from
   the robot to that point.

CLI fallbacks (work without RViz Publish Point):

```bash
# Method 1: Trigger service that reads goal_x/goal_y/goal_z parameters
ros2 param set /surface_goal_planner goal_x 0.0
ros2 param set /surface_goal_planner goal_y 1.18
ros2 param set /surface_goal_planner goal_z 2.0
ros2 service call /surface_goal_planner/set_goal_from_params \
    std_srvs/srv/Trigger

# Method 2: publish a one-shot PointStamped on /clicked_point
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 0.0, y: 1.18, z: 2.0}}"
```

### Monitoring

```bash
ros2 topic echo /clicked_point
ros2 topic echo /surface_goal_planner/debug
ros2 topic echo /robot_3d3s/adhesion_status
ros2 service list   # many planner services available
```

## Key files

- `src/robot_3d3s/scripts/surface_goal_planner.py` — unified
  cylinder/polygon surface-graph planner; subscribes to
  `/clicked_point`, publishes route + goal markers on
  `/horizontal_pipe_markers` or `/surface_goal_markers`, exposes
  `~/reload_surface_graph`, `~/use_experimental_polygon_graph`,
  `~/set_goal_from_params` services.
- `src/robot_3d3s/scripts/experimental_polygon_grid.py` —
  wireframe + solid-cylinder hit-target + dense PointCloud2 shell
  on the pipe surface. The solid cylinder (namespace
  `pipe_hit_target`, alpha 0.06-0.08) is the geometry RViz Publish
  Point reliably hits.
- `src/robot_3d3s/scripts/adhesion_dashboard.py` — PyQt GUI.
- `src/robot_3d3s/config/display.rviz` — RViz configuration with
  Publish Point tool wired to `/clicked_point`.
- `src/robot_3d3s/src/kmw100_adhesion_system.cpp` — Gazebo plugin
  applying KMW100 magnetic holding force per wheel.
- `src/robot_3d3s/scripts/surface_geometry.py` — CylinderSurface,
  PolygonSurface, SurfaceGraph and the A* routing logic.

## Clean-up before next run

```bash
pkill -f surface_goal_planner.py
pkill -f experimental_polygon_grid.py
pkill -f magnetic_adhesion.py
pkill -f "ros2 launch robot_3d3s"
pkill -f "gz sim"
pkill -f rviz2
pkill -f parameter_bridge