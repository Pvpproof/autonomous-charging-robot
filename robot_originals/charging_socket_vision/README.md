# 国标交流七孔车辆插座视觉定位（离线阶段）

本工程只做 D435i 采集、点云处理、模型配准和位姿输出，不发送机械臂运动、IO 或充电指令。

## 坐标与单位

统一使用 `T_A_from_B`：把 B 坐标中的点转换到 A 坐标。

模型 `socket` 坐标来自 SolidWorks/Open3D 包：

- 单位：m；STL 原始单位为 mm，转换时缩放 0.001；
- 原点：PE 孔轴线与最前端口沿平面的交点；
- +X：正视插座向右；
- +Y：正视插座向上；
- +Z：朝观察者/接口外侧；
- 插入方向：socket `-Z`。

配准输出：

```text
P_camera = T_camera_from_socket @ P_socket
```

因此 `T_camera_from_socket[:3, 3]` 是模型原点在相机坐标中的位置，插入方向在相机坐标中为 `-T_camera_from_socket[:3, 2]`。

只有明确提供静态眼在手外标定 `T_base_from_camera` 后，才计算：

```text
T_base_from_socket = T_base_from_camera @ T_camera_from_socket
```

不使用旧的 `R_cam2gripper/t_cam2gripper` 文件代替 `T_base_from_camera`。

## 当前输入包

模型包目前位于开发机：

```text
/home/jetson/Downloads/SolidWorks_Open3D_AC_socket_v1.zip
```

解压后需要保留：

```text
socket_front_interface_m.ply
socket_front_visible_m.ply
view_templates/*.ply
open3d_conversion_report.json
```

## 依赖

```bash
python3 -m pip install --user open3d numpy scipy opencv-python
```

Jetson 上安装前应先检查现有 Python/系统包；不要执行系统级升级。

## 设计原则

- 原始 RGB、原始深度、内参、depth_scale 永久保留；
- 不平移点云到原点，不分别归一化模型/实测点云；
- 默认保留孔边、上平边和口沿，不删除“最大平面”；
- 自动配准失败时允许人工初始位姿；
- ICP 的低 RMSE 不等于真实定位正确；必须检查重叠、孔位、上平边、插入轴和重复性；
- 任何机器人目标位姿输出均为候选，不进入运动控制。
