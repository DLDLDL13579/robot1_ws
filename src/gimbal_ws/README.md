# gimbal_control_cpp

本仓库包含一个用于控制 **Wheeltec F32C 二自由度无刷云台（电机 ID 1 为 X轴/Yaw，电机 ID 2 为 Y轴/Pitch）** 的 ROS 2 C++ 控制节点。

该节点通过 Linux 底层 POSIX 串口 API 与云台进行 115200 波特率的二进制流通信，实现了电机的加电锁定、位置控制速度限制配置，并提供了一个 ROS 2 话题接口用于动态接收微调角度指令。

## 1. 软件与硬件准备

### 软件环境
* **操作系统**: Ubuntu 22.04 / 24.04 (或兼容 Linux 系统)
* **ROS 2 版本**: Humble / Jazzy (基于 `rclcpp` 接口)
* **依赖库**: `rclcpp`, `geometry_msgs`

### 硬件连接
* **云台硬件**: Wheeltec F32C 二自由度无刷云台
* **通信模块**: USB 转 TTL 模块 (连接至 X轴 电机接口二)
* **默认串口设备**: `/dev/ttyACM0` (波特率：`115200`)
* **接线提示**: 
  * 云台 X 轴电机 `TX` $\rightarrow$ USB转TTL `RX`
  * 云台 X 轴电机 `RX` $\rightarrow$ USB转TTL `TX`
  * 确保 USB 转 TTL 模块的 `GND` 与云台供电电源的 `GND` **共地**。
  * 保证电机供电电压在 `8V ~ 15V` 之间 (推荐 12V 稳压电源或 3S 电池)。

---

## 2. 编译与安装

在您的 ROS 2 工作空间（如 `gimbal_ws`）下执行以下步骤进行包的编译：

```bash
# 1. 进入工作空间
cd ~/gkfd/myros2/gimbal/gimbal_ws

# 2. 使用 colcon 编译当前功能包
colcon build --packages-select gimbal_control_cpp

# 3. 刷新局部环境变量
source install/setup.bash