#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import serial
import math
import threading
import queue
import time

from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue


DEFAULT_WHEEL_ODOM_TOPIC = '/odom'
DEFAULT_IMU_TOPIC = '/imu/data'
DEFAULT_IMU_FRAME_ID = 'base_footprint'
DEFAULT_BASE_DRIVER_PUBLISH_TF = True


def quaternion_from_euler(roll, pitch, yaw):
    """手写四元数转换"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class BaseDriverNode(Node):
    def __init__(self):
        super().__init__('base_driver')
        
        # 参数声明与读取
        self.declare_parameter('serial_port', '/dev/ttyS9')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 2.2)
        self.declare_parameter('cmd_timeout', 0.5)  # 命令超时时间
        self.declare_parameter('reconnect_interval', 2.0)  # 重连间隔
        self.declare_parameter('attitude_in_degrees', False)
        self.declare_parameter('tilt_topic', '/base/tilt')
        self.declare_parameter('wheel_odom_topic', DEFAULT_WHEEL_ODOM_TOPIC)
        self.declare_parameter('imu_topic', DEFAULT_IMU_TOPIC)
        self.declare_parameter('imu_frame_id', DEFAULT_IMU_FRAME_ID)
        self.declare_parameter('publish_tf', DEFAULT_BASE_DRIVER_PUBLISH_TF)
        self.declare_parameter('wheel_odom_xy_std', 0.05)
        self.declare_parameter('wheel_odom_yaw_std', 0.20)
        self.declare_parameter('wheel_odom_vxy_std', 0.05)
        self.declare_parameter('wheel_odom_wz_std', 0.05)
        self.declare_parameter('imu_orientation_std', 0.03)
        self.declare_parameter('imu_angular_velocity_std', 0.05)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.attitude_in_degrees = self.get_parameter('attitude_in_degrees').value
        self.tilt_topic = self.get_parameter('tilt_topic').value
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.wheel_odom_xy_std = self.get_parameter('wheel_odom_xy_std').value
        self.wheel_odom_yaw_std = self.get_parameter('wheel_odom_yaw_std').value
        self.wheel_odom_vxy_std = self.get_parameter('wheel_odom_vxy_std').value
        self.wheel_odom_wz_std = self.get_parameter('wheel_odom_wz_std').value
        self.imu_orientation_std = self.get_parameter('imu_orientation_std').value
        self.imu_angular_velocity_std = self.get_parameter(
            'imu_angular_velocity_std'
        ).value
        
        self.get_logger().info(f"正在初始化底盘驱动 | 端口: {self.serial_port}, 波特率: {self.baud_rate}")
        self.get_logger().info(
            f"底盘上行话题: wheel_odom={self.wheel_odom_topic}, "
            f"imu={self.imu_topic}, publish_tf={self.publish_tf}"
        )
        
        # 状态变量
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_x, self.last_y = 0.0, 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()
        self.is_moving = False  # 运动状态锁，防止超时函数狂发刹车包
        self.roll = 0.0
        self.pitch = 0.0
        
        # 串口管理
        self.serial_connected = False
        self.running = True
        self.ser = None
        self.serial_lock = threading.Lock()
        self.rx_buffer = ""
        self.tx_queue = queue.Queue(maxsize=50)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_interval = rclpy.duration.Duration(seconds=0.05)
        
        # 统计信息
        self.rx_count = 0
        self.tx_count = 0
        self.error_count = 0
        
        self.connect_serial()
        
        # ROS 2 通信接口
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.wheel_odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.tilt_pub = self.create_publisher(Vector3Stamped, self.tilt_topic, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        
        # 定时器
        self.create_timer(0.5, self.check_heartbeat)
        self.create_timer(1.0, self.publish_diagnostics)
        self.create_timer(0.1, self.check_cmd_timeout)
        
        # 独立线程
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.write_thread = threading.Thread(target=self.write_serial_data, daemon=True)
        self.read_thread.start()
        self.write_thread.start()
        
        self.get_logger().info("✓ 底盘通信节点已成功启动！")

    def connect_serial(self):
        """连接串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.01,
                write_timeout=0.01,
                xonxoff=False, 
                rtscts=False, 
                dsrdtr=False
            )
            
            # DTR/RTS控制
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = True
            self.ser.rts = True
            
            # 清空缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.rx_buffer = ""
            
            self.serial_connected = True
            self.error_count = 0
            self.get_logger().info(f"✓ 串口已连接: {self.serial_port}")
            
        except Exception as e:
            self.serial_connected = False
            self.get_logger().error(f"✗ 串口连接失败: {e}")

    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        if not self.serial_connected:
            return
        
        self.last_cmd_vel_time = self.get_clock().now()
        
        # 限制发送频率
        now = self.get_clock().now()
        if (now - self.last_cmd_time) < self.cmd_interval:
            return
        self.last_cmd_time = now
        
        # 限制速度范围
        vx = max(min(msg.linear.x, self.max_linear), -self.max_linear)
        vy = max(min(msg.linear.y, self.max_linear), -self.max_linear)
        wz = max(min(msg.angular.z, self.max_angular), -self.max_angular)
        
        # 记录运动状态
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self.is_moving = False
        else:
            self.is_moving = True
        
        # 构造命令字符串
        cmd_str = f"${vx:.2f},{vy:.2f},{wz:.2f}\n"
        
        try:
            self.tx_queue.put_nowait(cmd_str.encode())
            self.tx_count += 1
        except queue.Full:
            self.get_logger().warn("⚠ 发送队列已满", throttle_duration_sec=1.0)

    def check_cmd_timeout(self):
        """检查命令超时，超时后仅发送一次停止命令"""
        if not self.serial_connected or not self.is_moving:
            return
        
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            stop_cmd = b"$0.00,0.00,0.00\n"
            try:
                self.tx_queue.put_nowait(stop_cmd)
                self.is_moving = False  # 复位状态，避免重复发送刹车指令
                self.get_logger().warn("⚠ 速度指令超时，已自动触发安全刹车！")
            except queue.Full:
                pass

    def write_serial_data(self):
        """写线程：从队列取数据发送"""
        while self.running and rclpy.ok():
            try:
                cmd_bytes = self.tx_queue.get(timeout=0.1)
                if self.serial_connected and self.ser and self.ser.is_open:
                    with self.serial_lock:
                        self.ser.write(cmd_bytes)
                        self.ser.flush()
            except queue.Empty:
                continue
            except serial.SerialTimeoutException:
                self.get_logger().warn("⚠ 串口写超时", throttle_duration_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"写入错误: {e}", throttle_duration_sec=2.0)
                self.serial_connected = False
                self.error_count += 1

    def read_serial_data(self):
        """读线程：持续读取串口数据"""
        reconnect_timer = 0.0
        
        while self.running and rclpy.ok():
            if not self.serial_connected:
                if reconnect_timer <= 0:
                    self.get_logger().info("尝试重新连接串口...")
                    self.connect_serial()
                    reconnect_timer = self.reconnect_interval
                else:
                    time.sleep(0.1)
                    reconnect_timer -= 0.1
                continue
            
            try:
                if self.ser and self.ser.is_open:
                    with self.serial_lock:
                        chunk = self.ser.read(128)
                    
                    if chunk:
                        self.rx_buffer += chunk.decode('utf-8', errors='ignore')
                        
                        # 处理完整的行
                        while '\n' in self.rx_buffer:
                            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.parse_odom_data(line)
                                self.rx_count += 1
                    else:
                        time.sleep(0.01)
                        
            except serial.SerialException as e:
                self.get_logger().warn(f"串口异常: {e}", throttle_duration_sec=2.0)
                self.serial_connected = False
                self.error_count += 1
            except Exception as e:
                self.get_logger().warn(f"读取错误: {e}", throttle_duration_sec=2.0)
                self.error_count += 1

    def parse_odom_data(self, line):
        """解析里程计数据"""
        try:
            if not line.startswith('$'):
                return
            
            data_str = line[1:]
            data = data_str.split(',')
            
            if len(data) not in (6, 10):
                return

            if len(data) == 6:
                x, y, th, vx, vy, vth = map(float, data)
                roll = pitch = wx = wy = None
            else:
                x, y, th, vx, vy, vth, roll, pitch, wx, wy = map(float, data)
            
            # 数据合理性检查
            if abs(x) > 1000 or abs(y) > 1000 or abs(vx) > 10:
                self.get_logger().warn(f"⚠ 数据异常: x={x}, y={y}, vx={vx}", throttle_duration_sec=2.0)
                return
            
            # 跳变检测
            if self.last_x != 0.0 or self.last_y != 0.0:
                jump = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if jump > 0.5:
                    self.get_logger().warn(f"⚠ 位置跳变: {jump:.2f}m", throttle_duration_sec=2.0)
                    return
            
            self.last_x, self.last_y = x, y
            self.x, self.y, self.th = x, y, th
            if roll is not None and pitch is not None:
                self.publish_tilt(roll, pitch, wx, wy)
                self.publish_imu(roll, pitch, th, wx, wy, vth)
            self.publish_odom(vx, vy, vth)
            self.last_odom_time = self.get_clock().now()
            
        except ValueError as e:
            self.get_logger().debug(f"数据解析错误: {e}")
        except Exception as e:
            self.get_logger().warn(f"解析异常: {e}", throttle_duration_sec=2.0)

    def publish_tilt(self, roll, pitch, wx=None, wy=None):
        """发布来自下位机的横滚/俯仰角，供上层激光门控使用。"""
        if self.attitude_in_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)

        self.roll = roll
        self.pitch = pitch

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = 0.0
        self.tilt_pub.publish(msg)

    def publish_imu(self, roll, pitch, yaw, wx, wy, wz):
        """发布标准 IMU 数据，供 EKF 融合姿态和角速度。"""
        if self.attitude_in_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        msg.orientation = quaternion_from_euler(roll, pitch, yaw)
        msg.angular_velocity.x = 0.0 if wx is None else float(wx)
        msg.angular_velocity.y = 0.0 if wy is None else float(wy)
        msg.angular_velocity.z = float(wz)

        orientation_var = self.imu_orientation_std ** 2
        angular_velocity_var = self.imu_angular_velocity_std ** 2
        msg.orientation_covariance = [
            orientation_var, 0.0, 0.0,
            0.0, orientation_var, 0.0,
            0.0, 0.0, orientation_var,
        ]
        msg.angular_velocity_covariance = [
            angular_velocity_var, 0.0, 0.0,
            0.0, angular_velocity_var, 0.0,
            0.0, 0.0, angular_velocity_var,
        ]
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def publish_odom(self, vx, vy, vth):
        """发布轮式里程计数据，可选兼容旧链路广播 TF。"""
        current_time = self.get_clock().now().to_msg()
        quat = quaternion_from_euler(0, 0, self.th)
        
        if self.publish_tf and self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)
        
        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.pose.covariance[0] = self.wheel_odom_xy_std ** 2
        odom.pose.covariance[7] = self.wheel_odom_xy_std ** 2
        odom.pose.covariance[35] = self.wheel_odom_yaw_std ** 2
        odom.twist.covariance[0] = self.wheel_odom_vxy_std ** 2
        odom.twist.covariance[7] = self.wheel_odom_vxy_std ** 2
        odom.twist.covariance[35] = self.wheel_odom_wz_std ** 2
        self.odom_pub.publish(odom)

    def check_heartbeat(self):
        """检查心跳"""
        elapsed = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if elapsed > 1.0 and self.serial_connected:
            self.get_logger().warn(
                f"⚠ 已有 {elapsed:.1f} 秒未收到下位机里程计数据！", 
                throttle_duration_sec=2.0
            )

    def publish_diagnostics(self):
        """发布诊断信息"""
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "Base Driver"
        status.hardware_id = self.serial_port
        status.level = DiagnosticStatus.OK if self.serial_connected else DiagnosticStatus.ERROR
        status.message = "Connected" if self.serial_connected else "Disconnected"
        
        # 添加详细信息
        status.values.append(KeyValue(key="RX Count", value=str(self.rx_count)))
        status.values.append(KeyValue(key="TX Count", value=str(self.tx_count)))
        status.values.append(KeyValue(key="Error Count", value=str(self.error_count)))
        status.values.append(KeyValue(key="Position", value=f"({self.x:.2f}, {self.y:.2f})"))
        
        diag.status.append(status)
        self.diag_pub.publish(diag)

    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info("正在关闭底盘驱动节点...")
        self.running = False
        
        # 发送停止命令
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"$0.00,0.00,0.00\n")
                time.sleep(0.1)
                self.ser.close()
        except Exception:
            pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
