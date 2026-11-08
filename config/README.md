# 公开配置 / Public configuration

`return_commissioning.json` 是被版本管理的禁用示例：`profile=disabled`、`commissioned=false`，现场授权均为 `false`，不包含真实设备身份或验收凭据。不要用它代替现场审查。

The tracked JSON is a disabled example, not commissioning evidence. All site authorizations are false; no real device identity or acceptance receipt is supplied.

本地配置仍放在此目录；`.gitignore` 默认排除其他配置文件，仅允许此说明与禁用示例。地图、日志、轨迹、标定、控制器地址和密钥不应进入公开提交。导航参数需通过 `CHARGING_NAV2_PARAMS` 显式指定已审核的外部文件；[占位参数](../examples/nav2.placeholder.yaml)不能启动导航。

Local configuration remains at the same path. Git ignores other files here by default. Supply reviewed navigation parameters explicitly; keep site data and credentials out of public commits. [Deployment](../docs/DEPLOYMENT.en.md) · [部署](../docs/DEPLOYMENT.md)
