# 项目备忘（remaind）

> 放在 src 下随仓库走。最后更新：2026-09-07 晚

## 当前状态

- ✅ 已从 RDK X3 迁移到本 Jetson（xumeng@192.168.31.235，Domain 1），全栈冒烟通过
- ✅ 导航已启动（soldier_with_nav2，9 关键节点全活，TF 全链通，amcl_pose 正常）
- ✅ git 已接 GitHub（分支 jetson-migration，作者 邓林）

## 待办

- [ ] RViz2 实车导航首验（Domain 1；2D Pose Estimate 先纠位姿 → 2D Goal Pose 发近距目标 0.5m 内）
- [ ] 复验/校准 linear_scale=1.09、angular_scale=0.55（launch 实际值，未实车验证）
- [ ] Nav2 参数实车调优（沿用 RDK 时代配置）
- [ ] 电池电压：暂挂（串口读数漂=底层故障；OLED 真值正常；先查 MPU6050 接线，不行就上 INA219）
- [ ] 云台暂不用（CH343 序列号 5B21242534 未接）
- [ ] 建 systemd 自启单元（稳定后）
- [ ] export_2d_map.py（狗侧）推送目标改本机

## 关键参数速记

| 项 | 值 |
|---|---|
| 底盘串口 | /dev/serial/by-id/usb-1a86_USB_Single_Serial_5897131917-if00 @115200 |
| 雷达 | /dev/ttyUSB0（CP2102）115200，~5.2Hz |
| 标定 | linear_scale=1.09，angular_scale=0.55，wheel_track=0.13，wheel_radius=0.0225 |
| 电压补偿 | voltage_scale/offset 参数 + 5 点中值滤波（voltage_min=9.0 / voltage_max=12.6 3S） |
| 看门狗 | 20Hz 速度帧，0.5s 无指令自动停 |
| 地图 | maps/lab_map.yaml（0.05m，原点 [-6.03,-14.4,0]） |

## 环境要点

- ssh 非交互 shell 不加载 .bashrc，远程起栈必须显式 source
- DDS：FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_udp.xml（UDP-only）；SHM 异常时 rm /dev/shm/fastrtps_*
- git 推送无凭据：走 Mac 链式（Mac clone 本机 → push GitHub）
