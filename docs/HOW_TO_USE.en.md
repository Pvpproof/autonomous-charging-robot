# How to use

[中文](HOW_TO_USE.md) · [Home](../README.en.md) · [Agent entry](../AGENTS.md) · [Development](DEVELOPMENT.md)

## 1. Get the repository

Extract the repository ZIP and open the directory containing `start.sh`, `README.md` and `robot_originals/`, or clone it:

```bash
git clone https://github.com/Pvpproof/autonomous-charging-robot.git
cd autonomous-charging-robot
```

| Goal | Entry |
| --- | --- |
| Inspect the UI and edit existing maps/targets | `bash start.sh --offline` |
| Develop the panel, navigation or return | `src/charging_robot/`; see [development guide](DEVELOPMENT.md) |
| Restore original arm/vision/template/calibration files on the same robot | `robot_originals/`; see [recovery guide](本车恢复说明.md) |
| Operate the separately deployed arm tools | `charge_robot test` / `charge_robot home` |
| Check sources without hardware or inspect planning demos | Sections 6 and 7 below |

This is a complete repository source bundle. ROS, Python environments, camera/base drivers and actual navigation maps remain deployment dependencies. `arm_stack/` is a path-adapted reading view; `robot_originals/` is the source for same-robot restoration.

## 2. Preview the panel offline

Install system Python with PyQt5, NumPy, Pillow and PyYAML. From a desktop terminal at the repository root:

```bash
bash start.sh --offline
```

This does not connect to ROS or hardware. Inspect the three sections, import your map YAML and edit/save charging targets. Live localization, feedback and motion require the integrated hardware mode. Use the middle mouse button to pan, the wheel to zoom and the right button to cancel the current drawing operation.

## 3. Prepare the original robot environment

The originals preserve the `/home/nvidia` layout, ROS 2 Humble, the Nova2 workspace, RealSense SDK/Python bindings and `charging_robot_env`. See [environment requirements](../robot_originals/ENVIRONMENT.md) for observed versions and unverified installation steps. Calibration, home joints and compensation are specific to this robot.

For a requested same-robot restore, keep devices stationary and related services stopped, then preview:

```bash
/usr/bin/python3 robot_originals/deploy_originals.py
```

Only `--apply` copies files and backs up replaced files. Follow the [recovery guide](本车恢复说明.md) for restoration and `colcon` build steps. Copying sources does not install dependencies or start devices.

## 4. Use the mobile-base panel

### Configure hardware mode

Hardware use also requires base/lidar drivers, an integrated gated driver, the raw-odometry relay, valid Nav2 parameters and a map. Replace the example paths below with your actual installation:

```bash
export CHARGING_ROBOT_SETUP=/absolute/path/to/robot_ws/install/setup.bash
export CHARGING_GATED_DRIVER_PREFIX=/absolute/path/to/gated_driver/install
export CHARGING_NAV2_PARAMS=/absolute/path/to/nav2.yaml
export CHARGING_DOCKING_COMMAND=/home/nvidia/nova2_tools/charge_robot.sh
bash start.sh
```

See [raw odometry and driver integration](RAW_ODOM_RETURN.md) for relay build instructions and the base interface. Public configuration is `disabled` and `commissioned=false`; use a configured, accepted runtime installation for motion. A boolean change does not supply missing drivers or parameters.

### Map, outbound and return

1. **Maps/targets:** load or import a map and select a charging point. To create one, click “点击地图新增充电停靠点”, specify the stopping pose and heading, then “保存点位”. Remapping is optional and needs real sensor feedback.
2. **Connect/localize:** click “连接底盘与雷达” and “启动定位与导航”. Mark the actual pose with “设置初始位置”; once localization is ready, click “应用初始位置” and check lidar/map alignment.
3. **Outbound:** click “去程：充电点”, or set a map goal and click “去程：地图目标”. Recording starts automatically. Wait for arrival, standstill and a completed record. “停止去程” cancels outbound and does not create a complete return route.
4. **Return:** retain the same raw-odometry session and remain at the recorded endpoint. Click “开始返程” once and confirm; holding a button is unnecessary. “停止返程” or “停止底盘” cancels the base task. Re-record after restarting/resetting the base odometry.
5. **Optional arm tools:** arrival does not start them. Click `charge_robot test` or `charge_robot home` independently and follow the terminal. After ending the tool and verifying the arm is stowed, click “已收拢，释放底盘” before base motion. Arm success is not required to retain a completed outbound route.

Manual control uses “开启键盘控制”: W/S forward/reverse, A/D steering, release keys to stop; finish with “停车并退出键盘”. Base stop does not stop the arm. Recorded routes do not detect new obstacles; physical operation still requires observation and an independent emergency stop.

## 5. Run arm test / home independently

On the original robot with the originals deployed and its environment configured, invoke the commands separately:

```bash
charge_robot test
```

`test` sequences **ICP → camera alignment → V3 → 50 mm gun prepose → 75 mm linear insertion**. Each physical motion still requires terminal confirmation. Arm RViz displays the model, cloud and poses. Diagnose a readiness/planning failure before confirming further stages.

```bash
charge_robot home
```

`home` returns the arm to its saved joint pose, handling withdrawal first when the current cycle was inserted and unretracted. It does not return the base, and `test` does not invoke it automatically. See [arm workflows](ARM_WORKFLOWS.en.md) for state branches and architecture.

The original command link points to `/home/nvidia/nova2_tools/charge_robot.sh`. Without the link, use `bash /home/nvidia/nova2_tools/charge_robot.sh test` or `home`. These are hardware-facing operations, not installation checks to run in a source checkout.

## 6. Check sources without devices

From the repository root:

```bash
bash tests/run_arm.sh
```

This runs four existing suites covering source/original hashes, syntax, feedback readiness and synthetic V3 timing. It needs system Python and NumPy, and starts no camera or ROS control nodes. The full workstation regression entry is `bash tests/run_local.sh`, requiring ROS 2 Humble and Qt. Prefer tests relevant to your changes; see [development](DEVELOPMENT.md).

## 7. MoveIt / RViz planning demonstration

The repository includes `nova2_moveit` and project `charging_dock_sim`, not a full Gazebo physics setup. Install ROS/MoveIt and build both packages in a separate development environment with no real driver connected. Load that workspace:

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/simulation_ws/install/setup.bash
ros2 launch charging_dock_sim charging_dock_demo.launch.py
```

The launch includes the Nova2 MoveIt demo and socket scene. The included demo control configuration uses `mock_components/GenericSystem`. From another terminal in the same simulation environment, request planning only:

```bash
ros2 run charging_dock_sim plan_docking --ros-args --params-file "$(ros2 pkg prefix charging_dock_sim)/share/charging_dock_sim/config/docking_params.yaml"
```

`plan_docking` uses plan-only requests and does not submit trajectory execution. The simulation YAML uses 35 mm insertion, distinct from real `test` at 75 mm. Building a fresh environment and running the full demo were not validated during this publication. The original package's `moveit_commander` dependency declaration needs checking; see [environment notes](../robot_originals/ENVIRONMENT.md).
