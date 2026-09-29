# catkin_ws

ROS1 Noetic workspace for PX4 wall/ceiling attachment control.

## Packages

| Package | Responsibility | Primary documentation |
|---|---|---|
| `px4_wall_ceiling_control` | Dual range sensors, ceiling/wall decisions, owner arbitration and MAVROS output | [`src/px4_wall_ceiling_control/README.md`](src/px4_wall_ceiling_control/README.md) |

Production logic belongs inside a ROS package. Workspace-root files are limited
to workspace metadata and this index; test procedures live beside the package
that owns them.

## Build

```bash
cd /home/cry/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

The main bench procedure is
[`src/px4_wall_ceiling_control/docs/bench_test_procedure.txt`](src/px4_wall_ceiling_control/docs/bench_test_procedure.txt).
