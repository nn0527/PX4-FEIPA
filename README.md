# catkin_ws

ROS1 Noetic workspace for PX4 wall/ceiling attachment control.

Use the unified `ceiling_wall_console` test console; see
[`ceiling_wall_console.md`](src/px4_wall_ceiling_control/docs/ceiling_wall_console.md).
The old 8787 console, Windows launcher and portable build entry have been removed.
The ROS bridge backend remains shared by the unified console.

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
[`bench_test_procedure.txt`](bench_test_procedure.txt).

## Collaboration

Repository: https://gitee.com/yls2026/wall-climbing-robot

Create a working branch from `develop` and submit a pull request back to
`develop`. The initial workspace import is on
`import/catkin-workspace-20261002`.

```bash
git clone git@gitee.com:yls2026/wall-climbing-robot.git catkin_ws
cd catkin_ws
git switch develop
git switch -c your-working-branch
```

Build outputs, Python environments, logs, ROS bags and generated telemetry are
excluded from Git. The PowerShell and `runtime/` observation tools contain
paths for the original Windows/WSL installation; adjust those paths for your
machine before using them.

## ceiling_wall_console

顶吸与侧吸测试台源码已迁入 `src/px4_wall_ceiling_control`。
本机离线预览可双击 `Start-CeilingWallConsole.command`（Mac）或 `Start-CeilingWallConsole.cmd`（Windows），
打开 `http://127.0.0.1:8791/login`，使用离线访问码 `offline`。
申请控制权后可切换阶段、定高及异常场景，无需连接树莓派。
真实树莓派测试台继续使用 8790 端口。
目录、启动、部署与验证说明见 [ceiling_wall_console](src/px4_wall_ceiling_control/docs/ceiling_wall_console.md)。
