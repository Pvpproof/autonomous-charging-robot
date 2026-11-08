# DOBOT Nova2 上游核对 / Upstream audit

核对日期：2026-11-08。官方仓库 [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)；本轮读取固定提交 `7807ef8fb31e8f86a76cd474a4a9cddffef6eca9`，不以浮动 main 作为现场锁定版本。

与 2026-10-29 保存的现场 SHA-256 清单比较，以下 6 个文件完全一致：dobot_rviz/urdf/nova2_robot.urdf，以及 nova2_moveit/config 下 nova2_robot.srdf、nova2_robot.ros2_control.xacro、kinematics.yaml、joint_limits.yaml、nova2_robot.urdf.xacro。详见 [逐文件哈希](dobot-upstream-comparison.json)。这证明这些文件的内容对应，不证明整个现场仓库 commit 或所有本地改动相同。

Six upstream files match the historical field inventory byte-for-byte. This establishes file correspondence, not the field checkout commit or absence of local changes.

## 许可 / Licensing

该固定提交的 [根 LICENSE](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/blob/7807ef8fb31e8f86a76cd474a4a9cddffef6eca9/LICENSE) 是 MIT，版权归 Dobot。nova2_moveit/package.xml 和 cra_description/package.xml 另写 BSD；dobot_rviz/package.xml 仍为 TODO 占位。应保留这些声明并核对具体文件/资产，不能把厂家文件改署 Pvpproof MIT，亦不能仅凭 package.xml 占位断言整个仓库没有许可。

建议先作为明确版本的外部依赖。若需纳入 vendor 目录，应固定提交、保留 Dobot LICENSE/版权和文件级声明、单独标注修改，并核查 BSD 包及网格资产声明；本轮没有复制 SDK、模型或网格进公开仓库。

Root MIT is attributed to Dobot; some package manifests declare BSD or retain a TODO. Preserve upstream notices and inspect specific assets rather than relicensing them as Pvpproof code. External dependency integration is preferred until the complete version and local changes are established. No vendor assets were imported here.

## 实际缺口 / Remaining gaps

官方源中有 Nova2 模型、MoveIt 配置及仿真入口，相关内容并非完全无来源。现缺的是现场使用的完整固定版本、修改差异、执行适配器和模型/网格依赖闭包，以及项目自己的 start_nova2_real_model.sh 等包装。当前缓存有消息/驱动构建产物与历史清单，但未找到完整模型仓库 checkout。不能用官方最新分支直接冒充现场版本。

The upstream provides model/planning/simulation components. Still missing are the complete field revision, local modifications, execution adapter, complete mesh dependencies and project-specific wrappers. Cached build products and inventory are not a full checkout. This audit did not start simulation or hardware services.

上游 README 推荐 feature/v4-optimization 分支，不能把本轮核对的 main SHA 当作该推荐分支；文档所述平台为 Ubuntu 22.04 / ROS 2 Humble，控制器版本要求需按所选分支再次核对。项目视觉 V3 是自有 RGB PnP 版本名称，不是 DOBOT SDK V3。

The upstream README recommends feature/v4-optimization, distinct from the audited main SHA. Select and verify the intended branch/revision rather than assuming equivalence. Project vision V3 is not DOBOT SDK V3.
