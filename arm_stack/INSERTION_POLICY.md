# 75 mm 公开副本策略 / Public-copy insertion policy

用户明确选择其后期采用的 75 mm。共享 insertion_limits.py 将插入上限定义为 75.0 mm，规划目标定义为 0.075 m；笛卡尔规划输出、insert_execute 和旧 insert_force 入口均使用同一策略。枪头预备位仍为原有 50 mm，未混成插入距离。旧 80 mm 计划和超限/非有限/布尔距离拒绝。计划与周期绑定保留。

The owner selected 75 mm. The shared policy is used by Cartesian planning and both forward insertion executors. Planning uses metres; saved metadata uses millimetres. Prepose distance is unchanged. Old 80 mm plans remain invalid, with source-plan/cycle binding preserved.

回退模块原有 85 mm 是“实际当前位置是否处于可回退轴向范围”的检查，不是前进目标或允许插入长度，本次保留；没有重新标定或擅自统一它。几何辅助模块换成同一 2026-10-29 补丁中的版本，补齐规划器已引用的 seeded_cartesian_path，仍非今日现场版本确认。

The existing 85 mm withdrawal-position envelope is not a forward insertion target and is retained. The geometry helper now comes from the matching historical patch that includes seeded_cartesian_path. No current field identity is claimed.

7 项新增无硬件测试通过：单位、边界、75 个 1 mm 细化点、保存字段、旧计划拒绝、执行预检和计划/周期绑定。无实机或 MoveIt 运行；未安装到机器人。
