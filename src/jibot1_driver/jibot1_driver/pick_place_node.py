"""
Jibot1 拾取放置节点 (支持仅拾取模式)

提供一个简单的拾取-放置动作序列节点，
通过话题 /jibot1/pick_place 接收拾取/放置坐标。

使用方法:
  # 模式 1：仅拾取并举起 (发3个数字) —— 适合抓球后小车继续导航
  ros2 topic pub --once /jibot1/pick_place std_msgs/msg/String "{data: '100,200,20'}"

  # 模式 2：完整的拾取并放置 (发6个数字)
  ros2 topic pub --once /jibot1/pick_place std_msgs/msg/String "{data: '100,200,20,-100,200,20'}"
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool

from .kinematics import Kinematics


DEFAULT_APPROACH_HEIGHT_MM = 80.0
DEFAULT_MOVE_DELAY_S = 2.0
DEFAULT_GRIPPER_DELAY_S = 1.0
DEFAULT_IK_REACH_CHECK_ENABLED = True
DEFAULT_LINK_L0_MM = 235
DEFAULT_LINK_L1_MM = 85
DEFAULT_LINK_L2_MM = 55
DEFAULT_LINK_L3_MM = 205


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('jibot1_pick_place')

        # 参数
        self.declare_parameter('approach_height', DEFAULT_APPROACH_HEIGHT_MM)   # 接近高度 (mm)
        self.declare_parameter('move_delay', DEFAULT_MOVE_DELAY_S)              # 运动等待时间 (s)
        self.declare_parameter('gripper_delay', DEFAULT_GRIPPER_DELAY_S)        # 夹爪动作等待 (s)
        self.declare_parameter('ik_reach_check_enabled', DEFAULT_IK_REACH_CHECK_ENABLED)
        self.declare_parameter('link_l0', DEFAULT_LINK_L0_MM)
        self.declare_parameter('link_l1', DEFAULT_LINK_L1_MM)
        self.declare_parameter('link_l2', DEFAULT_LINK_L2_MM)
        self.declare_parameter('link_l3', DEFAULT_LINK_L3_MM)

        # 发布器 —— 向驱动节点发指令
        self._pub_target = self.create_publisher(
            Point, '/jibot1/target_pose', 10)
        self._pub_gripper = self.create_publisher(
            Bool, '/jibot1/gripper', 10)
        self._pub_status = self.create_publisher(
            String, '/jibot1/status', 10)

        # 订阅器 —— 接收拾取放置指令
        self._sub_cmd = self.create_subscription(
            String, '/jibot1/pick_place', self._on_pick_place, 10)

        self._ik = Kinematics(
            l0=self.get_parameter('link_l0').value,
            l1=self.get_parameter('link_l1').value,
            l2=self.get_parameter('link_l2').value,
            l3=self.get_parameter('link_l3').value,
        )

        self.get_logger().info(
            '拾取放置节点已启动，支持 [仅拾取] 和 [拾取并放置] 双模式 | '
            f'IK预检查={self.get_parameter("ik_reach_check_enabled").value} | '
            f'连杆=({self.get_parameter("link_l0").value},'
            f'{self.get_parameter("link_l1").value},'
            f'{self.get_parameter("link_l2").value},'
            f'{self.get_parameter("link_l3").value})mm'
        )

    def _publish_target(self, x: float, y: float, z: float):
        msg = Point()
        msg.x, msg.y, msg.z = x, y, z
        self._pub_target.publish(msg)
        self.get_logger().info(f'目标: ({x:.1f}, {y:.1f}, {z:.1f})')

    def _publish_gripper(self, close: bool):
        msg = Bool()
        msg.data = close
        self._pub_gripper.publish(msg)
        self.get_logger().info(f'夹爪: {"闭合" if close else "张开"}')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    def _target_reachable(self, x: float, y: float, z: float) -> bool:
        if not self.get_parameter('ik_reach_check_enabled').value:
            return True
        return self._ik.solve(x, y, z) is not None

    def _validate_pick_targets(self, status_prefix: str, *targets) -> bool:
        bad = []
        for x, y, z in targets:
            if not self._target_reachable(x, y, z):
                bad.append((x, y, z))
        if not bad:
            return True
        details = '; '.join([f'({x:.1f},{y:.1f},{z:.1f})' for x, y, z in bad])
        self._publish_status(f'{status_prefix}:FAILED')
        self.get_logger().error(f'{status_prefix} 目标超出机械臂工作空间，已取消动作: {details}')
        return False

    def _on_pick_place(self, msg: String):
        """
        接收指令，向下兼容两种模式：
        模式 1 (仅拾取): "pick_x, pick_y, pick_z" (发3个数字)
        模式 2 (完整流水线): "pick_x, pick_y, pick_z, place_x, place_y, place_z" (发6个数字)
        """
        try:
            parts = [float(v.strip()) for v in msg.data.split(',')]
            
            if len(parts) == 3:
                # 触发【仅拾取】模式
                self.get_logger().info('触发模式：仅拾取 (Pick Only)')
                self._execute_pick_only(parts[0], parts[1], parts[2])
                
            elif len(parts) == 6:
                # 触发【完整拾取与放置】模式
                self.get_logger().info('触发模式：拾取并放置 (Pick & Place)')
                self._execute_pick_place(
                    parts[0], parts[1], parts[2],
                    parts[3], parts[4], parts[5])
                    
            else:
                self.get_logger().error('参数数量错误！请输入 3 个或 6 个数字，用逗号隔开。')

        except ValueError as e:
            self.get_logger().error(f'解析失败: {e}')

    def _execute_pick_only(self, px, py, pz):
        """执行半套流水线：张开 -> 靠近 -> 下降 -> 夹紧 -> 抬起"""
        approach_h = self.get_parameter('approach_height').value
        move_delay = self.get_parameter('move_delay').value
        grip_delay = self.get_parameter('gripper_delay').value

        if not self._validate_pick_targets(
                'PICK_ONLY',
                (px, py, approach_h),
                (px, py, pz)):
            return

        self._publish_status('PICK_ONLY:START')

        # 1. 张开夹爪
        self._publish_gripper(False)
        time.sleep(grip_delay)

        # 2. 移动到拾取点上方
        self._publish_status('PICK_ONLY:APPROACH_PICK')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        # 3. 下降到拾取位置
        self._publish_status('PICK_ONLY:DESCEND_PICK')
        self._publish_target(px, py, pz)
        time.sleep(move_delay)

        # 4. 闭合夹爪 (夹取)
        self._publish_status('PICK_ONLY:GRIP')
        self._publish_gripper(True)
        time.sleep(grip_delay)

        # 5. 提起 (抓稳了，悬在半空等待小车开走)
        self._publish_status('PICK_ONLY:LIFT')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        self._publish_status('PICK_ONLY:DONE')
        self.get_logger().info('✅ 仅拾取动作完成，机械臂已抬起并保持夹紧！')

    def _execute_pick_place(self, px, py, pz, tx, ty, tz):
        """执行完整的拾取-放置序列"""
        approach_h = self.get_parameter('approach_height').value
        move_delay = self.get_parameter('move_delay').value
        grip_delay = self.get_parameter('gripper_delay').value

        if not self._validate_pick_targets(
                'PICK_PLACE',
                (px, py, approach_h),
                (px, py, pz),
                (tx, ty, approach_h),
                (tx, ty, tz)):
            return

        self._publish_status('PICK_PLACE:START')

        # 1. 张开夹爪
        self._publish_gripper(False)
        time.sleep(grip_delay)

        # 2. 移动到拾取点上方
        self._publish_status('PICK_PLACE:APPROACH_PICK')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        # 3. 下降到拾取位置
        self._publish_status('PICK_PLACE:DESCEND_PICK')
        self._publish_target(px, py, pz)
        time.sleep(move_delay)

        # 4. 闭合夹爪 (夹取)
        self._publish_status('PICK_PLACE:GRIP')
        self._publish_gripper(True)
        time.sleep(grip_delay)

        # 5. 提起
        self._publish_status('PICK_PLACE:LIFT')
        self._publish_target(px, py, approach_h)
        time.sleep(move_delay)

        # 6. 移动到放置点上方
        self._publish_status('PICK_PLACE:APPROACH_PLACE')
        self._publish_target(tx, ty, approach_h)
        time.sleep(move_delay)

        # 7. 下降到放置位置
        self._publish_status('PICK_PLACE:DESCEND_PLACE')
        self._publish_target(tx, ty, tz)
        time.sleep(move_delay)

        # 8. 张开夹爪 (释放)
        self._publish_status('PICK_PLACE:RELEASE')
        self._publish_gripper(False)
        time.sleep(grip_delay)

        # 9. 抬起
        self._publish_status('PICK_PLACE:RETREAT')
        self._publish_target(tx, ty, approach_h)
        time.sleep(move_delay)

        self._publish_status('PICK_PLACE:DONE')
        self.get_logger().info('拾取放置完成')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
