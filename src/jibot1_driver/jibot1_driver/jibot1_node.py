"""
Jibot1 机械臂 ROS2 驱动节点

话题接口：
  订阅:
    /jibot1/target_pose  (geometry_msgs/Point)  - 目标坐标 x,y,z (mm)
    /jibot1/gripper       (std_msgs/Bool)        - 夹爪控制 True=闭合 False=张开
    /jibot1/raw_cmd       (std_msgs/String)      - 原始串口指令 (直接转发)
    /jibot1/joint_pwm     (std_msgs/String)      - 直接指定PWM "pwm0,pwm1,pwm2,pwm3,time"
  发布:
    /jibot1/status        (std_msgs/String)      - 状态反馈

服务:
    /jibot1/home          (std_srvs/Trigger)     - 回到初始位置
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import serial
import threading

from .kinematics import Kinematics


class Jibot1DriverNode(Node):

    def __init__(self):
        super().__init__('jibot1_driver')

        # ---- 声明参数 ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('move_time', 1000)
        self.declare_parameter('link_l0', 235)
        self.declare_parameter('link_l1', 85)
        self.declare_parameter('link_l2', 55)
        self.declare_parameter('link_l3', 205)
        self.declare_parameter('gripper_open_pwm', 1200)
        self.declare_parameter('gripper_close_pwm', 1700)
        self.declare_parameter('gripper_time', 800)
        self.declare_parameter('servo_pwm_min', 500)
        self.declare_parameter('servo_pwm_max', 2500)

        # ---- 读取参数 ----
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---- 初始化逆运动学 ----
        self._ik = Kinematics(
            l0=self.get_parameter('link_l0').value,
            l1=self.get_parameter('link_l1').value,
            l2=self.get_parameter('link_l2').value,
            l3=self.get_parameter('link_l3').value,
        )

        # ---- 初始化串口 ----
        self._serial_lock = threading.Lock()
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'串口已连接: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self._serial = None

        # ---- 订阅器 ----
        self._sub_target = self.create_subscription(
            Point, '/jibot1/target_pose', self._on_target, 10)
        self._sub_gripper = self.create_subscription(
            Bool, '/jibot1/gripper', self._on_gripper, 10)
        self._sub_raw = self.create_subscription(
            String, '/jibot1/raw_cmd', self._on_raw_cmd, 10)
        self._sub_joint = self.create_subscription(
            String, '/jibot1/joint_pwm', self._on_joint_pwm, 10)

        # ---- 发布器 ----
        self._pub_status = self.create_publisher(String, '/jibot1/status', 10)

        # ---- 服务 ----
        self._srv_home = self.create_service(
            Trigger, '/jibot1/home', self._on_home)

        # ---- 串口读取线程 ----
        if self._serial is not None:
            self._read_thread = threading.Thread(
                target=self._serial_read_loop, daemon=True)
            self._read_thread.start()

        self.get_logger().info('Jibot1 驱动节点已启动')

    # ================================================================
    #  串口通信
    # ================================================================

    def _send(self, cmd: str) -> bool:
        """线程安全地发送串口指令"""
        if self._serial is None or not self._serial.is_open:
            self.get_logger().error('串口未连接，无法发送')
            return False
        with self._serial_lock:
            self._serial.write(cmd.encode('utf-8'))
        self.get_logger().info(f'TX: {cmd}')
        return True

    def _serial_read_loop(self):
        """后台线程：持续读取串口反馈"""
        while rclpy.ok():
            try:
                if self._serial is None or not self._serial.is_open:
                    break
                with self._serial_lock:
                    if self._serial.in_waiting > 0:
                        data = self._serial.read(
                            self._serial.in_waiting).decode('utf-8', 'ignore')
                        if data.strip():
                            self.get_logger().debug(f'RX: {data.strip()}')
                            msg = String()
                            msg.data = f'RX:{data.strip()}'
                            self._pub_status.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'串口读取异常: {e}')

    def _publish_status(self, status: str):
        """发布状态"""
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    # ================================================================
    #  回调函数
    # ================================================================

    def _on_target(self, msg: Point):
        """
        接收目标坐标 (x, y, z) 单位 mm
        自动执行逆运动学求解并驱动舵机
        """
        x, y, z = msg.x, msg.y, msg.z
        move_time = self.get_parameter('move_time').value

        pwm = self._ik.solve(x, y, z)
        if pwm is None:
            self.get_logger().warn(
                f'逆运动学无解: ({x:.1f}, {y:.1f}, {z:.1f}) 超出工作空间')
            self._publish_status(f'IK_FAILED:{x},{y},{z}')
            return

        # 限幅
        pwm_min = self.get_parameter('servo_pwm_min').value
        pwm_max = self.get_parameter('servo_pwm_max').value
        pwm = [max(pwm_min, min(pwm_max, p)) for p in pwm]

        cmd = self._ik.pwm_to_cmd(pwm, move_time)
        self._send(cmd)
        self._publish_status(
            f'MOVING:x={x:.1f},y={y:.1f},z={z:.1f},pwm={pwm}')

    def _on_gripper(self, msg: Bool):
        """夹爪控制: True=闭合(夹取), False=张开(释放)"""
        grip_time = self.get_parameter('gripper_time').value
        if msg.data:
            pwm = self.get_parameter('gripper_close_pwm').value
            self._send(f'#005P{pwm:04d}T{grip_time:04d}!')
            self._publish_status('GRIPPER:CLOSE')
        else:
            pwm = self.get_parameter('gripper_open_pwm').value
            self._send(f'#005P{pwm:04d}T{grip_time:04d}!')
            self._publish_status('GRIPPER:OPEN')

    def _on_raw_cmd(self, msg: String):
        """直接转发原始指令到 ESP32"""
        self._send(msg.data)

    def _on_joint_pwm(self, msg: String):
        """
        直接指定 PWM 值
        格式: "pwm0,pwm1,pwm2,pwm3,time"
        示例: "1500,1500,1500,1500,1000"
        """
        try:
            parts = [int(v.strip()) for v in msg.data.split(',')]
            if len(parts) != 5:
                self.get_logger().error(
                    'joint_pwm 格式错误, 需要 "pwm0,pwm1,pwm2,pwm3,time"')
                return
            pwm_list = parts[:4]
            move_time = parts[4]
            cmd = self._ik.pwm_to_cmd(pwm_list, move_time)
            self._send(cmd)
        except ValueError as e:
            self.get_logger().error(f'joint_pwm 解析失败: {e}')

    def _on_home(self, request, response):
        """服务回调: 机械臂复位到直立位置"""
        cmd = '#255P1500T1500!'
        success = self._send(cmd)
        response.success = success
        response.message = '已发送复位指令' if success else '串口未连接'
        self._publish_status('HOME')
        return response

    # ================================================================
    #  生命周期
    # ================================================================

    def destroy_node(self):
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Jibot1DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
