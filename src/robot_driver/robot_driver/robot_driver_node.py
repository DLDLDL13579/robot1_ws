#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
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
        self.cmd_vel_sub = self.create_subscription(Twist, f'{ns}cmd_vel', self.cmd_vel_callback, 10)

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
                    # ⚠️ 已禁用底盘自发布 TF (2026-07-28)
                    # 单一权威源原则: odom → base_link 的 TF 现在完全由 robot_localization EKF 发布,
                    # 因为 EKF 同时融合了本节点的 odom(twist) 和 imu/data_raw,
                    # 能给出比底盘自身更平滑的位姿估计。底盘若同时发 TF 会和 EKF 冲突,
                    # 触发 TF_REPEATED_DATA / 双源抖动。底盘只负责发原始 odom + imu 数据。
                    # 保留 self.tf_broadcaster 与 import 以便回滚。
                    # t = TransformStamped()
                    # t.header.stamp = odom_msg.header.stamp
                    # t.header.frame_id = f"{self.robot_namespace}/odom" if self.robot_namespace else "odom"
                    # t.child_frame_id = f"{self.robot_namespace}/base_link" if self.robot_namespace else "base_link"
                    # t.transform.translation.x = 0.0
                    # t.transform.translation.y = 0.0
                    # t.transform.translation.z = 0.0
                    # t.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
                    # self.tf_broadcaster.sendTransform(t)

                    # --- Consume frame ---
                    self.buffer = self.buffer[36:]

                except Exception as e:
                    self.get_logger().error(f'❌ Frame parse error: {e}')
                    self.buffer = self.buffer[1:]

        except Exception as e:
            self.get_logger().error(f'❌ Serial read error: {e}')

    def cmd_vel_callback(self, msg):
        try:
            x_speed = int(max(min(msg.linear.x * 1000.0, 32767), -32768))
            y_speed = int(max(min(msg.linear.y * 1000.0, 32767), -32768))
            z_speed = int(max(min(msg.angular.z * 1000.0, 32767), -32768))
            buffer = bytearray(11)
            buffer[0] = 0x7B
            buffer[1] = 0x00
            buffer[2] = 0x00
            import struct
            struct.pack_into('>h', buffer, 3, x_speed)
            struct.pack_into('>h', buffer, 5, y_speed)
            struct.pack_into('>h', buffer, 7, z_speed)
            checksum = 0
            for i in range(9):
                checksum ^= buffer[i]
            buffer[9] = checksum
            buffer[10] = 0x7D
            self.ser.write(buffer)
        except Exception as e:
            self.get_logger().error(f'指令发送失败: {e}')

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
