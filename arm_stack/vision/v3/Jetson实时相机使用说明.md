# D435i RGB PnP V3：Jetson 相机测试版

这是能直接调用 D435i 的独立程序。相机启动方式参考现有 ICP：`rs.pipeline()`、原生640×480 BGR彩色流、从启动后的实际流读取内参。因为本算法只使用RGB，所以不启用深度，也不做深度对齐。

没有ROS、机械臂、TCP、cam-plan或gun-plan连接。不要替换原ICP，也不要把结果直接接入运动控制。本版用于现场观察、保存样本和验证视觉结果。

## 1. 下载与解压

把 `PnP_D435i_RGB_Live_V3.zip` 放到 Jetson 的 Downloads 或 download 目录。在 Jetson 本地桌面打开终端，复制：

```bash
package_path=""
for download_dir in "$HOME/Downloads" "$HOME/download" "$HOME/Download" "$HOME/downloads" "$HOME/下载"; do
    if [ -f "$download_dir/PnP_D435i_RGB_Live_V3.zip" ]; then
        package_path="$download_dir/PnP_D435i_RGB_Live_V3.zip"
        break
    fi
done
if [ -z "$package_path" ]; then
    echo "没有找到压缩包，请检查路径和文件名；不要继续下一步。"
else
    test_dir=$(mktemp -d "$HOME/pnp_live_test_XXXXXX")
    if [ -n "$test_dir" ] && python3 -m zipfile -e "$package_path" "$test_dir"; then
        cd "$test_dir/pnp_live_rgb_v3" && pwd
    else
        echo "解压失败，请保留报错；不要继续。"
    fi
fi
```

## 2. 准备相机和环境

先在原ICP窗口正常退出程序；也关闭占用相机的RealSense Viewer。不要同时运行两个采集程序。插好D435i，使用原来能运行ICP的Python环境；若原来需要激活虚拟环境，按原方式激活。

```bash
python3 check_environment.py
```

依赖：numpy、带窗口支持的OpenCV、pyrealsense2。无需Open3D或scipy。此命令只检查导入与必要接口，不启动相机。缺模块时先确认Python环境，不要直接升级系统依赖。

窗口需要桌面显示。普通无图形SSH终端不适合直接运行；优先在Jetson连接显示器后的桌面终端操作。

## 3. 直接启动相机

```bash
python3 -u run_live.py
```

默认640×480、15FPS彩色流，预热20帧。先将充电口完整放进画面，尽量保持静止，等待后台自动找七孔。首次定位可能需要数秒，但后台搜索时主窗口仍应显示画面；实际Jetson帧率未验证。

若需要保留终端日志，也可以使用：

```bash
python3 -u run_live.py 2>&1 | tee live_terminal.log
```

只有相机支持对应配置时才使用 `--fps 30`。默认先用15，不要为了启动成功任意修改内参或畸变模型。

## 4. 窗口与按键

- 黄色孔名与点：当前图像中的七孔测量。
- 红/绿/蓝轴：插座X/Y/Z；青色D轮廓是诊断投影，不是独立验收证明。
- SEARCHING：后台恢复搜索中。
- BACKGROUND CHECK：已有跟踪，后台在复核。
- VISUAL VALID / READ ONLY：当前视觉结果通过连续帧、距离、时效等程序检查，不代表实际定位精度已经验证。
- outside_130_150mm_working_range：可以显示候选，但不输出有效控制变换。
- confirming_fresh_frames：等待不同相机帧的连续确认。
- independent_verification_expired：复核超时，取消有效状态并重新搜索。

按键：

|按键|作用|
|---|---|
|S|保存当前RGB、叠加图、相机内参/畸变和位姿JSON|
|R|立即取消当前状态，重新锁定；之前未完成的后台任务不会跨重置采用|
|Q或Esc|退出程序，停止相机与后台进程，写无效输出|

## 5. 输出位置和含义

默认写入本测试目录的 `live_results`，不会写正式ICP的输出文件。

- `camera.json`：相机实际彩色流内参、畸变参数。
- `latest_pose_readonly.json`：最新视觉状态。
- 按S保存的 `*_rgb.png`、`*_overlay.png`、`*_camera.json`、`*_pose.json`：供回放分析。

`pose_valid=true`表示满足本版的视觉检查；`planning_ready`始终false。只有有效时，顶层 `T_camera_from_socket` 和 `pose6d_xyz_m_rotvec_rad` 才有值；否则为null，候选数据另存于 `candidate_...`。

坐标单位米，彩色光学相机坐标系，`P_camera = T_camera_from_socket @ P_socket`。插座原点为PE轴对应前口平面，+X右、+Y上、+Z朝外。6D后三项是旋转向量弧度，不是欧拉角。未包含手眼或工具坐标转换。

测量时效以主机收到帧的时间计算，同时记录相机时间戳及帧号；不是已经标定过的机器人同步时间。强制杀进程或断电时无法保证写出退出状态，任何未来消费者都必须独立检查测量年龄。

## 6. 当前处理逻辑

主进程取新帧、光流跟踪、PnP验收和显示；后台独立进程执行之前的圆环重选、局部搜索和暗色区域恢复。后台结果必须在当前帧重新跟踪、重新求位姿后才可能进入确认。首次需要3个不同时间戳的帧。

默认约每2秒请求一次独立复核；6秒没有成功复核则取消跟踪有效性；后台参考图超过8秒丢弃；单个后台任务超过12秒重启工作进程。若Jetson上搜索持续超过这些时间，应先保存日志，不要盲目放宽有效性限制。

周期复核只能提供额外检查，不能消除所有稳定错误解。此前模拟仍存在错误通过案例，实物精度、长期光流漂移、连续恢复成功率尚未验证。

## 7. 出现问题时

- 无法启动或设备忙：确认旧ICP与RealSense Viewer已退出，检查USB连接，保留完整报错。
- OpenCV窗口错误：确认使用Jetson桌面及原ICP的带GUI OpenCV环境。
- 畸变模型不支持或SDK一致性检查失败：发送报错和 `live_results/camera.json`，不要把畸变强行改成none。
- 长时间SEARCHING：先确认七孔完整、相机图像清楚，尽量静止，然后按S保存失败样本。
- 会锁但轴不合理：按S保存，再按R重锁；不要据此执行机械臂动作。

测试后把 `live_results` 中的保存样本和日志发回。最有用的是成功、失败、倾角变化、丢失后恢复各一组。

## JSON要不要改

**原ICP与原实时PnP V2的JSON不用改。** 本包必须同时保留：

1. `pnp_near.json`：沿用此前的几何验收参数。
2. `live_settings.json`：本版新增的帧率、后台复核和超时参数，首次使用保持默认。

不是只复制一个py文件：请完整解压本包，在独立目录启动。

## 验证范围

已完成16项无硬件测试，包含保存图像上的实际多进程搜索、当前帧确认、重复帧/过期结果拒绝、复核过期、工作进程超时重启、模拟SDK的RGB启动/保存/退出和失败清理。所有Python文件编译检查通过。

**本机未连接D435i，未在Jetson上启动相机，未人工验证真实窗口或相机帧率。** 这是可调用相机的测试版，不是已验证的机器人部署版。
