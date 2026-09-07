#!/bin/bash

# === ROS2 Humble 履带底盘驱动包 - RDK X3 部署脚本 ===
# 支持 namespace / wheel_track / slip_factor / port_name 参数化
# 帧协议：36字节二进制，帧头0x7B，帧尾0x7D，校验位buffer[34]

set -e

WORKSPACE="/home/sunrise/robot1_ws"
PKG_NAME="robot_driver"

echo "🔧 正在初始化工作区: $WORKSPACE"
mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE"

# 初始化 workspace（若尚无 colcon build）
if [ ! -f "src/COLCON_IGNORE" ]; then
  echo "📦 初始化 ROS2 workspace..."
  source /opt/ros/humble/setup.bash
  colcon build --packages-select $PKG_NAME 2>/dev/null || true
fi

echo "🏗️  创建功能包: $PKG_NAME"
cd src
rm -rf "$PKG_NAME"
ros2 pkg create --build-type ament_python "$PKG_NAME" --dependencies rclpy sensor_msgs nav_msgs tf2_ros

cd "$PKG_NAME"

# === 写入 package.xml ===
cat > package.xml << 'EOF2'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_driver</name>
  <version>0.0.1</version>
  <description>ROS2 Humble driver for履带式小车底盘 (Skid-Steering)</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

<depend>rclpy</depend> <depend>sensor_msgs</depend> <depend>nav_msgs</depend> <depend>tf2_ros</depend> <depend>std_msgs</depend> <depend>builtin_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

<export> <build_type>ament_python</build_type> </export> </package> EOF2

# === 写入 setup.py ===
cat > setup.py << 'EOF2'
from setuptools import setup

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/robot_driver_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS2 Humble driver for履带式小车底盘',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = robot_driver.robot_driver_node:main',
        ],
    },
)
EOF2

# === 创建节点目录 ===
mkdir -p robot_driver

# === 写入 robot_driver_node.py ===
cat > robot_driver/robot_driver_node.py << 'EOF2'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
from builtin_interfaces.msg import Time

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')

        # --- Parameters ---
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('wheel_track', 0.13)  # m
        self.declare_parameter('wheel_radius', 0.0225)  # m
        self.declare_parameter('slip_factor', 1.0)

        self.port_name = self.get_parameter('port_name').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.robot_namespace = self.get_parameter('robot_namespace').value.strip()
        self.wheel_track = self.get_parameter('wheel_track').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.slip_factor = self.get_parameter('slip_factor').value

        # --- Topic names with namespace ---
        ns = f"{self.robot_namespace}/" if self.robot_namespace else ""
        self.odom_pub = self.create_publisher(Odometry, f"{ns}odom", 10)
        self.imu_pub = self.create_publisher(Imu, f"{ns}imu/data_raw", 10)
        self.battery_pub = self.create_publisher(BatteryState, f"{ns}battery_state", 10)

        # --- TF broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Serial ---
        try:
            self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'✅ Serial opened: {self.port_name} @ {self.baud_rate}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open serial: {e}')
            raise

        # --- State machine ---
        self.buffer = bytearray()
        self.state = 'WAIT_HEADER'  # WAIT_HEADER, IN_FRAME, WAIT_TAIL

        # --- Timer ---
        self.create_timer(0.05, self.read_serial_callback)  # 20Hz

    def read_serial_callback(self):
        try:
            data = self.ser.read(100)
            if not data:
                return

            self.buffer.extend(data)

            while len(self.buffer) >= 36:
                # Look for header
                if self.buffer[0] != 0x7B:
                    self.buffer.pop(0)
                    continue

                # Check full frame length
                if len(self.buffer) < 36:
                    break

                # Check tail
                if self.buffer[35] != 0x7D:
                    self.buffer.pop(0)
                    continue

                # Validate checksum: XOR of bytes [0..33]
                checksum = 0
                for i in range(34):
                    checksum ^= self.buffer[i]
                if checksum != self.buffer[34]:
                    self.get_logger().warn('⚠️  Bad checksum, dropping frame')
                    self.buffer = self.buffer[1:]
                    continue

                # Parse frame (36 bytes)
                try:
                    # Skip header & flag
                    # X_speed (2 bytes, big-endian)
                    x_speed_raw = struct.unpack('>h', self.buffer[2:4])[0]
                    y_speed_raw = struct.unpack('>h', self.buffer[4:6])[0]
                    z_speed_raw = struct.unpack('>h', self.buffer[6:8])[0]

                    # Accel (6 bytes)
                    ax_raw = struct.unpack('>h', self.buffer[8:10])[0]
                    ay_raw = struct.unpack('>h', self.buffer[10:12])[0]
                    az_raw = struct.unpack('>h', self.buffer[12:14])[0]

                    # Gyro (6 bytes)
                    gx_raw = struct.unpack('>h', self.buffer[14:16])[0]
                    gy_raw = struct.unpack('>h', self.buffer[16:18])[0]
                    gz_raw = struct.unpack('>h', self.buffer[18:20])[0]

                    # Battery (2 bytes)
                    bat_raw = struct.unpack('>h', self.buffer[20:22])[0]

                    # Roll/Pitch/Yaw (6 bytes, deg*100)
                    roll_raw = struct.unpack('>h', self.buffer[22:24])[0]
                    pitch_raw = struct.unpack('>h', self.buffer[24:26])[0]
                    yaw_raw = struct.unpack('>h', self.buffer[26:28])[0]

                    # Odometry (6 bytes: X(mm), Y(mm), Theta(deg*100))
                    # (not used in this node, but available)

                    # --- Decode ---
                    x_speed = x_speed_raw / 1000.0  # m/s
                    y_speed = y_speed_raw / 1000.0
                    z_speed = z_speed_raw / 1000.0

                    ax = ax_raw / 1000.0  # g → m/s²
                    ay = ay_raw / 1000.0
                    az = az_raw / 1000.0

                    gx = gx_raw / 1000.0  # deg/s → rad/s
                    gy = gy_raw / 1000.0
                    gz = gz_raw / 1000.0

                    battery_v = bat_raw / 1000.0  # V

                    roll = math.radians(roll_raw / 100.0)
                    pitch = math.radians(pitch_raw / 100.0)
                    yaw = math.radians(yaw_raw / 100.0)

                    # --- Publish Odometry (Skid-Steering) ---
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = f"{self.robot_namespace}/odom" if self.robot_namespace else "odom"
                    odom_msg.child_frame_id = f"{self.robot_namespace}/base_link" if self.robot_namespace else "base_link"

                    # Linear velocity (X,Y,Z)
                    odom_msg.twist.twist.linear.x = x_speed
                    odom_msg.twist.twist.linear.y = y_speed
                    odom_msg.twist.twist.linear.z = 0.0

                    # Angular velocity (Z only, with slip factor)
                    odom_msg.twist.twist.angular.z = z_speed * self.slip_factor

                    # Position & orientation (we don't integrate here — use robot_localization or similar)
                    # So set pose to zero, or leave as identity
                    odom_msg.pose.pose.orientation = quaternion_from_euler(roll, pitch, yaw)

                    self.odom_pub.publish(odom_msg)

                    # --- Publish IMU ---
                    imu_msg = Imu()
                    imu_msg.header.stamp = odom_msg.header.stamp
                    imu_msg.header.frame_id = f"{self.robot_namespace}/base_link" if self.robot_namespace else "base_link"
                    imu_msg.linear_acceleration.x = ax * 9.81
                    imu_msg.linear_acceleration.y = ay * 9.81
                    imu_msg.linear_acceleration.z = az * 9.81
                    imu_msg.angular_velocity.x = math.radians(gx)
                    imu_msg.angular_velocity.y = math.radians(gy)
                    imu_msg.angular_velocity.z = math.radians(gz) * self.slip_factor  # apply slip to yaw rate
                    imu_msg.orientation = quaternion_from_euler(roll, pitch, yaw)
                    self.imu_pub.publish(imu_msg)

                    # --- Publish Battery ---
                    bat_msg = BatteryState()
                    bat_msg.header.stamp = odom_msg.header.stamp
                    bat_msg.header.frame_id = f"{self.robot_namespace}/base_link" if self.robot_namespace else "base_link"
                    bat_msg.voltage = float(battery_v)
                    bat_msg.percentage = min(max((battery_v - 6.0) / (8.4 - 6.0) * 100.0, 0.0), 100.0)
                    bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    self.battery_pub.publish(bat_msg)

                    # --- Broadcast TF ---
                    t = TransformStamped()
                    t.header.stamp = odom_msg.header.stamp
                    t.header.frame_id = f"{self.robot_namespace}/odom" if self.robot_namespace else "odom"
                    t.child_frame_id = f"{self.robot_namespace}/base_link" if self.robot_namespace else "base_link"
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0
                    t.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
                    self.tf_broadcaster.sendTransform(t)

                    # --- Consume frame ---
                    self.buffer = self.buffer[36:]

                except Exception as e:
                    self.get_logger().error(f'❌ Frame parse error: {e}')
                    self.buffer = self.buffer[1:]

        except Exception as e:
            self.get_logger().error(f'❌ Serial read error: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF2

# === 创建 launch 目录和文件 ===
mkdir -p launch
cat > launch/robot_driver_launch.py << 'EOF2'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('robot_namespace', default_value=''),
        DeclareLaunchArgument('wheel_track', default_value='0.13'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0225'),
        DeclareLaunchArgument('slip_factor', default_value='1.0'),

        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'wheel_track': LaunchConfiguration('wheel_track'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'slip_factor': LaunchConfiguration('slip_factor'),
            }],
            output='screen',
        ),
    ])
EOF2

# === 完成 ===
cd "$WORKSPACE"
echo "✅ 包已生成于: $WORKSPACE/src/$PKG_NAME"
echo ""
echo "🚀 下一步执行："
echo "  source /opt/ros/humble/setup.bash"
echo "  cd $WORKSPACE"
echo "  colcon build --packages-select robot_driver"
echo "  source install/setup.bash"
echo "  ros2 launch robot_driver robot_driver_launch.py robot_namespace:=robot1"
echo ""
echo "💡 提示："
echo "  - 确保 /dev/ttyUSB0 存在且权限正确（sudo usermod -a -G dialout \$USER）"
echo "  - 可用 'ros2 topic list' 和 'ros2 topic echo /robot1/odom' 验证"
