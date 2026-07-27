# DAIB single-UAV integration

The runtime boundary is deliberately process-based:

```text
FAST-LIVO2YYY
  /daib_slam/odom ------------------------------+
  /daib_slam/planning_cloud --> DAIB-Explorer   |
                                  |              |
                                  +-- goal ------v
                                  +-- ready --> daib_ego_bridge
                                  +-- occupied cloud --> EGO grid map
                                                       |
                                                       +-- B-spline
                                                       +-- PositionCommand
```

FAST-LIVO2 owns localization, DAIB-Explorer owns exploration memory and target
selection, and EGO-Swarm owns collision checking and dynamically feasible local
trajectories. No repository links against another repository's headers.

## Build in one ROS1 workspace

Clone all three repositories directly below `catkin_ws/src` (or symlink their
ROS packages there), install the normal FAST-LIVO2 and EGO-Swarm dependencies,
then build with one worker first:

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j1
source devel/setup.bash
```

The first build on an ARM board should use `-j1` to avoid memory pressure.
Increasing build parallelism later does not change runtime performance.

## Start order

```bash
# Terminal 1: localization
roslaunch fast_livo mapping_avia.launch rviz:=false

# Terminal 2: exploration
roslaunch daib_explorer explorer.launch

# Terminal 3: trajectory planning (no simulator, no RViz, no PX4)
roslaunch ego_planner daib_single_uav.launch
```

For bag playback, pass `use_sim_time:=true` to the Explorer and EGO launch
files and play the bag with `--clock`.

## Contract

Inputs consumed by this repository:

| Topic | Type | Requirement |
|---|---|---|
| `/daib_slam/odom` | `nav_msgs/Odometry` | `camera_init`; pose and world-frame linear velocity |
| `/daib_explorer/goal` | `geometry_msgs/PoseStamped` | Timestamp and pose identify a goal |
| `/daib_explorer/generation` | `std_msgs/UInt64` | Application-level generation for acknowledgement and telemetry |
| `/daib_explorer/ready` | `std_msgs/Bool` | must be true before forwarding a goal |
| `/daib_explorer/planning_cloud` | `sensor_msgs/PointCloud2` | occupied voxel centers in `camera_init` |

Outputs:

| Topic | Meaning |
|---|---|
| `/daib_ego/accepted_generation` | goal generation accepted by the bridge |
| `/daib_ego/bridge_state` | watchdog/validation state |
| `/drone_0_planning/bspline` | collision-checked local B-spline |
| `/daib_ego/position_cmd` | high-rate controller-facing trajectory command |

`/daib_ego/position_cmd` must not be connected directly to PX4 without a
controller/offboard adapter, arming state machine, command timeout and emergency
stop path.

## Resource defaults

The DAIB launch uses a 0.25 m EGO grid over 80 x 80 x 8 m, a 10 m local update
range, 2 m/s velocity, 3 m/s² acceleration and one trajectory optimization
branch (`use_distinctive_trajs=false`). These are initial RK3588 values, not
airframe certification values. Adjust map bounds and vertical limits to the
mission coordinate origin before flight.

The bridge requires the Explorer's 1 Hz ready heartbeat and considers it stale
after 2.5 seconds. The independent occupied-cloud watchdog is one second. If
Explorer stops publishing while a trajectory is active, EGO enters its existing
emergency-stop state instead of continuing against a stale map.
