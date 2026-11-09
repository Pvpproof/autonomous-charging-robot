# 现场原版插入参数 / Field-original insertion parameters

2026-11-09 核对现场 plan_insertion_cartesian.py：depth=.075，保存 insertion_depth_mm=75.，cartesian_speed_mm_s=10.。本次恢复原件，没有继续采用昨天另写的共享 insertion_limits 模块，也没有修改现场参数。

恢复的仿真示例 insert_depth=.035 为独立虚拟场景参数，未强改为实机插入长度。原始配置 insertion_profile.json 仍有旧 80/85 mm 候选且 enabled=false，已如实保留；不能将其当作实际执行中的 75 mm 计划。

The restored field planner targets 75 mm at nominal 10 mm/s. The inactive historical profile and standalone simulation retain their original values; restoration is not a parameter redesign or an execution validation.
