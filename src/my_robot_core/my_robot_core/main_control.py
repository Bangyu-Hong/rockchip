#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                        参数配置区（修改这里即可）                          ║
# ╠══════════════════════════════════════════════════════════════════════════╣
# ║  所有参数集中在此处，方便调试。修改后重新 colcon build 生效。               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── 距离参数 ──────────────────────────────────────────────────────────────
# Nav2导航阶段：在球前方多远停下，切换为精确伺服接管
# 含义：车停在距球 NAV2_STOP_DISTANCE 米处，然后由 FINE_SERVO 完成最后靠近
NAV2_STOP_DISTANCE    = 0.6    # (m) 推荐 0.4~0.6

# 发现球时，距离超过此值走Nav2路径规划接近，否则直接精确伺服
SAFE_DISTANCE         = 0.7    # (m) 推荐 0.6~1.0

# 精确伺服阶段：距球多近时停车并触发机械臂抓取。
# 主控会先切到观察姿态再重新拍快照，最终是否可抓由观察姿态 TF 和 IK 预检查决定。
GRASP_DISTANCE        = 0.5   # (m) 触发切观察姿态的距离，需配合 CAM_OBSERVE 实测

# ── 运动参数 ──────────────────────────────────────────────────────────────
# 搜索时原地旋转速度，太快容易飞过球，太慢找球慢
SEARCH_ANGULAR_SPEED  = 0.7    # (rad/s) 目标约 1 秒转 90 度，供原地搜索自转使用

# 精确伺服阶段前进速度，室内环境建议 0.08~0.15
APPROACH_LINEAR_SPEED = 0.12   # (m/s)

# 转向比例增益：angular.z = -gain * cam_x
# cam_x 是球在相机坐标系的横向偏移(m)，正值=球在右侧
# 增益越大转向越猛，越小越平滑，抖动时调小
APPROACH_ANGULAR_GAIN = 0.7    # 推荐 1.0~2.0

# 转向死区：|cam_x| 小于此值时不转向，防止小幅抖动
APPROACH_DEADBAND     = 0.04   # (m) 推荐 0.03~0.06

# ── 避障参数 ──────────────────────────────────────────────────────────────
# 精确伺服阶段，激光雷达检测到前方此距离内有障碍物时紧急停车并后退
OBSTACLE_STOP_DISTANCE  = 0.2   # (m)

# 前方障碍物检测扇区：以车头正前方为中心，左右各扫描此角度
# 角度太小可能漏检侧面障碍，太大可能误把球架判断为障碍
OBSTACLE_SCAN_ANGLE_DEG = 30.0  # (度) 推荐 20~40

# /scan 原始角度中，车头正前方对应的角度。
# 当前雷达反装，base_link -> laser yaw=π，因此车头前方在 LaserScan 中约为 180°。
# 如果未来把雷达正装，改成 0.0。
SCAN_FRONT_ANGLE_OFFSET_DEG = 180.0  # (度)

# 避障后退速度（负值=后退）
RECOVERY_BACKUP_SPEED   = -0.10 # (m/s)

# 避障后退持续时间，退完后重新进入精确伺服
RECOVERY_BACKUP_TIME    = 1.5   # (s)

# 避障后横移速度。正值表示横移速度大小，方向由障碍物左右侧自动决定。
RECOVERY_SIDE_STEP_SPEED = 0.08  # (m/s)

# 避障后横移持续时间；设为0可恢复旧逻辑：只后退、不横移
RECOVERY_SIDE_STEP_TIME  = 1.5   # (s)

# 障碍物左右判断死区。障碍几乎在正前方时，按 RECOVERY_CENTER_ESCAPE_DIRECTION 处理。
OBSTACLE_SIDE_DEADBAND_DEG = 5.0  # (度)

# 障碍物在正前方时默认向哪边横移：left=向左，right=向右
RECOVERY_CENTER_ESCAPE_DIRECTION = 'right'

# ── 超时保护参数 ──────────────────────────────────────────────────────────
# Nav2接近阶段最长允许时间，超时则放弃导航重新搜索
NAV2_TIMEOUT   = 20.0  # (s)

# 精确伺服阶段最长允许时间，超时说明卡住了
SERVO_TIMEOUT  = 10.0   # (s)

# ── 丢球容忍参数 ──────────────────────────────────────────────────────────
# Nav2阶段可容忍的连续丢球帧数（视觉帧率约10fps，30帧≈3秒）
# Nav2绕弯时球可能暂时不可见，此值要设大一些
LOST_FRAMES_NAV2  = 70

# 精确伺服阶段可容忍的连续丢球帧数（15帧≈1.5秒）
LOST_FRAMES_SERVO = 15

# ── Nav2动态目标更新参数 ──────────────────────────────────────────────────
# 每隔此时间检查一次球是否移动
NAV2_UPDATE_INTERVAL  = 1.0   # (s)

# 球移动超过此距离才重新发送Nav2目标，避免频繁取消重发
NAV2_UPDATE_THRESHOLD = 0.1   # (m)

# ── 巡逻模式参数（已知地图 + Nav2 时使用）────────────────────────────────
# 是否启用定点巡逻（True=依次访问路点, False=原地旋转）
# 使用 full_nav_bringup.launch.py 时建议设为 True
USE_PATROL = False

# 主控精确伺服阶段的避障扫描输入。默认使用坡道近距安全激光。
SCAN_TOPIC = '/scan_safety'
CAMERA_LINK_FRAME_ID = 'robot_camera_link'
CAMERA_OPTICAL_FRAME_ID = 'robot_camera_color_optical_frame'
ARM_BASE_FRAME_ID = 'jibot1_arm_base'
MOTION_LOCK_STATES = {
    "INIT_POSTURE", "WAIT_INIT",
    "PRE_GRASP_POSTURE", "WAIT_PRE_GRASP", "TAKE_SNAPSHOT",
    "WAIT_FOR_GRASP_DONE", "WAIT_FOR_DROP_DONE",
}

# 巡逻路点列表：(x, y, yaw_rad)，地图坐标系
# 按顺序循环访问；需根据实际地图调整坐标
# 建议覆盖场地四角和中心，yaw使车头朝向场地内侧
PATROL_WAYPOINTS = [
    ( 1.5,  1.5,  3.14),   # 右前角，朝向左后
    (-1.5,  1.5, -1.57),   # 左前角，朝向右后
    (-1.5, -1.5,  0.0),    # 左后角，朝向右前
    ( 1.5, -1.5,  1.57),   # 右后角，朝向左前
    ( 0.0,  0.0,  0.0),    # 中心，朝向前方
]

# 每个巡逻路点的超时时间（超时则跳至下一路点）
PATROL_WAYPOINT_TIMEOUT = 20.0  # (s)

# ── 抓取参数 ──────────────────────────────────────────────────────────────
# TAKE_SNAPSHOT阶段TF变换失败最多重试次数，超出则放弃回搜索
SNAPSHOT_MAX_RETRIES = 5

# 机械臂抓取时末端的目标Z高度（毫米，在IK坐标系下）
# IK坐标系原点=机械臂底座在地面的投影，l0处理了底座到关节1的高度
# 此值=乒乓球圆心离地高度，需实测（球放平面上约20mm，球架上另算）
GRASP_Z_MM = 35.0  # (mm) 需实车调试

# 抓取前 IK 可达性预检查。这里必须和 jibot1_params.yaml / pick_place_node 保持一致。
IK_REACH_CHECK_ENABLED = True
IK_APPROACH_HEIGHT_MM = 80.0
IK_LINK_L0_MM = 235
IK_LINK_L1_MM = 85
IK_LINK_L2_MM = 55
IK_LINK_L3_MM = 205

# 等待预抓取姿态到位时间
# pick_place_node 的 _execute_pick_only 总耗时 =
#   gripper_delay(1.0) + move_delay(2.0) + move_delay(2.0) +
#   gripper_delay(1.0) + move_delay(2.0) = 8.0s
# 此值必须 > 8.0s，否则丢球指令会在抓取动作执行到一半时发出造成冲突
# 若修改了 pick_place_node 的 move_delay / gripper_delay，此值同步调整
GRASP_WAIT_TIME = 10.0  # (s) 等待 pick_place_node 完成抓取并举起

# 等待丢球动作组（$DGT:41-46）执行完毕的时间
# 根据实际动作组帧数和运动速度设置，动作组越长此值越大
DROP_WAIT_TIME = 10.0   # (s) 按实际动作组时长调整

# 等待预抓取姿态（$DGT:40）机械臂运动到位的时间
PRE_GRASP_WAIT_TIME = 2.5  # (s) 机械臂动作越慢此值越大

# 等待搜索姿态（$DGT:39）机械臂运动到位的时间
# 该等待发生在 INIT_POSTURE -> WAIT_INIT，机械臂没回到搜索姿态前不允许开始移动找球
INIT_POSTURE_WAIT_TIME = 3.0  # (s) 机械臂回搜索姿态越慢，此值越大

# ── 机械臂安装偏移（★必须实测后填入，否则抓取会有系统性偏差）────────────
# 机械臂底座转轴中心 相对于 base_footprint（底盘地面投影中心）的水平偏移
#
# 测量方法：
#   1. 在地面找到底盘几何中心的正下方投影点（base_footprint原点）
#   2. 用卷尺量 机械臂底座转轴中心 到该点的前后距离 → ARM_OFFSET_X
#   3. 用卷尺量 机械臂底座转轴中心 到该点的左右距离 → ARM_OFFSET_Y
#   4. 前为正x，左为正y
#   注：z方向由IK内部l0参数处理，此处不填z
ARM_OFFSET_X = 0.0   # (m) ★待实测
ARM_OFFSET_Y = 0.06   # (m) ★待实测

# ── 相机TF参数（★必须实测后填入，影响抓取坐标精度）────────────────────
# 描述相机光学中心 相对于 base_footprint（底盘地面投影中心）的位置和朝向
# 父坐标系为 base_footprint（地面投影中心），与IK坐标原点一致
#
# 测量方法（两种姿态都要量）：
#   x = 相机光学中心 到底盘中心 前后距离（前为正，单位米）
#   y = 相机光学中心 到底盘中心 左右距离（左为正，单位米）
#   z = 相机光学中心 离地高度（单位米）
#   p = 相机俯仰角（低头为负，-0.785≈-45°，-1.57≈-90°垂直向下）
#
# 搜索姿态（对应 $DGT:39，机械臂抬起向前看）
CAM_SEARCH = {
    'x': 0.07,    # (m) ★待实测
    'y': 0.0,     # (m) ★待实测
    'z': 0.33,    # (m) ★待实测
    'r': 0.0,     # roll，通常为0
    'p': -0.349,  # pitch，-45° 向下倾斜
    'y_aw': 0.0,  # yaw，通常为0
}

# 观察姿态（对应 $DGT:40，机械臂弯曲垂直看地面）
CAM_OBSERVE = {
    'x': 0.22,    # (m) ★待实测
    'y': 0.0,     # (m) ★待实测
    'z': 0.30,    # (m) ★待实测
    'r': 0.0,
    'p': -1.484,   # pitch，-90° 垂直向下
    'y_aw': 0.0,
}

# ══════════════════════════════════════════════════════════════════════════
# 以下为程序实现，一般不需要修改
# ══════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs
import time
import math

from jibot1_driver.kinematics import Kinematics


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = TransformStamped().transform.rotation
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class MainControlNode(Node):
    def __init__(self):
        super().__init__('main_control_node')

        # ── 从顶部常量加载参数（同时注册为ROS2参数，支持运行时ros2 param set修改）
        self.declare_parameter('nav2_stop_distance',      NAV2_STOP_DISTANCE)
        self.declare_parameter('safe_distance',           SAFE_DISTANCE)
        self.declare_parameter('grasp_distance',          GRASP_DISTANCE)
        self.declare_parameter('search_angular_speed',    SEARCH_ANGULAR_SPEED)
        self.declare_parameter('approach_linear_speed',   APPROACH_LINEAR_SPEED)
        self.declare_parameter('approach_angular_gain',   APPROACH_ANGULAR_GAIN)
        self.declare_parameter('approach_deadband',       APPROACH_DEADBAND)
        self.declare_parameter('obstacle_stop_distance',  OBSTACLE_STOP_DISTANCE)
        self.declare_parameter('obstacle_scan_angle_deg', OBSTACLE_SCAN_ANGLE_DEG)
        self.declare_parameter('scan_front_angle_offset_deg', SCAN_FRONT_ANGLE_OFFSET_DEG)
        self.declare_parameter('recovery_backup_speed',   RECOVERY_BACKUP_SPEED)
        self.declare_parameter('recovery_backup_time',    RECOVERY_BACKUP_TIME)
        self.declare_parameter('recovery_side_step_speed', RECOVERY_SIDE_STEP_SPEED)
        self.declare_parameter('recovery_side_step_time',  RECOVERY_SIDE_STEP_TIME)
        self.declare_parameter('obstacle_side_deadband_deg', OBSTACLE_SIDE_DEADBAND_DEG)
        self.declare_parameter(
            'recovery_center_escape_direction',
            RECOVERY_CENTER_ESCAPE_DIRECTION,
        )
        self.declare_parameter('nav2_timeout',            NAV2_TIMEOUT)
        self.declare_parameter('servo_timeout',           SERVO_TIMEOUT)
        self.declare_parameter('lost_frames_nav2',        LOST_FRAMES_NAV2)
        self.declare_parameter('lost_frames_servo',       LOST_FRAMES_SERVO)
        self.declare_parameter('nav2_update_interval',    NAV2_UPDATE_INTERVAL)
        self.declare_parameter('nav2_update_threshold',   NAV2_UPDATE_THRESHOLD)
        self.declare_parameter('use_patrol',              USE_PATROL)
        self.declare_parameter('scan_topic',              SCAN_TOPIC)
        self.declare_parameter('camera_frame_id',         CAMERA_OPTICAL_FRAME_ID)
        self.declare_parameter('patrol_waypoint_timeout', PATROL_WAYPOINT_TIMEOUT)
        self.declare_parameter('snapshot_max_retries',    SNAPSHOT_MAX_RETRIES)
        self.declare_parameter('grasp_z_mm',              GRASP_Z_MM)
        self.declare_parameter('ik_reach_check_enabled',  IK_REACH_CHECK_ENABLED)
        self.declare_parameter('ik_approach_height_mm',   IK_APPROACH_HEIGHT_MM)
        self.declare_parameter('ik_link_l0_mm',           IK_LINK_L0_MM)
        self.declare_parameter('ik_link_l1_mm',           IK_LINK_L1_MM)
        self.declare_parameter('ik_link_l2_mm',           IK_LINK_L2_MM)
        self.declare_parameter('ik_link_l3_mm',           IK_LINK_L3_MM)
        self.declare_parameter('arm_offset_x',            ARM_OFFSET_X)
        self.declare_parameter('arm_offset_y',            ARM_OFFSET_Y)
        self.declare_parameter('grasp_wait_time',         GRASP_WAIT_TIME)
        self.declare_parameter('drop_wait_time',          DROP_WAIT_TIME)
        self.declare_parameter('pre_grasp_wait_time',     PRE_GRASP_WAIT_TIME)
        self.declare_parameter('init_posture_wait_time',  INIT_POSTURE_WAIT_TIME)

        self.nav2_stop_distance    = self.get_parameter('nav2_stop_distance').value
        self.safe_distance         = self.get_parameter('safe_distance').value
        self.grasp_distance        = self.get_parameter('grasp_distance').value
        self.search_angular_speed  = self.get_parameter('search_angular_speed').value
        self.approach_linear_speed = self.get_parameter('approach_linear_speed').value
        self.approach_angular_gain = self.get_parameter('approach_angular_gain').value
        self.approach_deadband     = self.get_parameter('approach_deadband').value
        self.obstacle_stop_dist    = self.get_parameter('obstacle_stop_distance').value
        self.obstacle_scan_angle   = math.radians(
            self.get_parameter('obstacle_scan_angle_deg').value)
        self.scan_front_angle_offset = math.radians(
            self.get_parameter('scan_front_angle_offset_deg').value)
        self.recovery_backup_speed = self.get_parameter('recovery_backup_speed').value
        self.recovery_backup_time  = self.get_parameter('recovery_backup_time').value
        self.recovery_side_step_speed = self.get_parameter('recovery_side_step_speed').value
        self.recovery_side_step_time = self.get_parameter('recovery_side_step_time').value
        self.obstacle_side_deadband = math.radians(
            self.get_parameter('obstacle_side_deadband_deg').value)
        self.recovery_center_escape_direction = (
            str(self.get_parameter('recovery_center_escape_direction').value)
            .strip()
            .lower()
        )
        self.nav2_timeout          = self.get_parameter('nav2_timeout').value
        self.servo_timeout         = self.get_parameter('servo_timeout').value
        self.lost_frames_nav2      = self.get_parameter('lost_frames_nav2').value
        self.lost_frames_servo     = self.get_parameter('lost_frames_servo').value
        self.nav2_update_interval    = self.get_parameter('nav2_update_interval').value
        self.nav2_update_threshold   = self.get_parameter('nav2_update_threshold').value
        self.use_patrol              = self.get_parameter('use_patrol').value
        self.scan_topic              = self.get_parameter('scan_topic').value
        self.camera_frame_id         = self.get_parameter('camera_frame_id').value
        self.patrol_waypoint_timeout = self.get_parameter('patrol_waypoint_timeout').value
        self.snapshot_max_retries    = self.get_parameter('snapshot_max_retries').value
        self.grasp_z               = self.get_parameter('grasp_z_mm').value
        self.ik_reach_check_enabled = self.get_parameter('ik_reach_check_enabled').value
        self.ik_approach_height = self.get_parameter('ik_approach_height_mm').value
        self.arm_offset_x          = self.get_parameter('arm_offset_x').value
        self.arm_offset_y          = self.get_parameter('arm_offset_y').value
        self.grasp_wait_time       = self.get_parameter('grasp_wait_time').value
        self.drop_wait_time        = self.get_parameter('drop_wait_time').value
        self.init_posture_wait_time = self.get_parameter('init_posture_wait_time').value

        # 相机TF姿态参数（从顶部常量加载）
        self.tf_search  = CAM_SEARCH
        self.tf_observe = CAM_OBSERVE

        # ── 状态机 ──────────────────────────────────────────────────────
        self.state            = "INIT_POSTURE"
        self.current_posture  = "SEARCH"
        self.state_enter_time = time.time()

        # ── 运行时变量 ──────────────────────────────────────────────────
        self.lost_frames          = 0
        self.snapshot_retries     = 0
        self.last_ball_map_x      = None
        self.last_ball_map_y      = None
        self.last_nav2_send_time  = 0.0
        self._nav2_goal_handle    = None
        self._nav2_goal_pending   = False   # 防止goal response回调时状态已变
        self._nav2_result_future  = None
        self.patrol_idx           = 0       # 当前巡逻路点索引
        self._pick_done           = False
        self._pick_failed         = False
        self._last_arm_status     = ''
        self._pick_kinematics = Kinematics(
            l0=self.get_parameter('ik_link_l0_mm').value,
            l1=self.get_parameter('ik_link_l1_mm').value,
            l2=self.get_parameter('ik_link_l2_mm').value,
            l3=self.get_parameter('ik_link_l3_mm').value,
        )

        # 激光雷达缓存
        self.scan_ranges          = []
        self.scan_angle_min       = 0.0
        self.scan_angle_increment = 0.0
        self.obstacle_recovery_lateral_speed = 0.0

        # ── 话题 ────────────────────────────────────────────────────────
        self.cmd_pub      = self.create_publisher(Twist,  '/cmd_vel',           10)
        self.arm_raw_pub  = self.create_publisher(String, '/jibot1/raw_cmd',    10)
        self.arm_pick_pub = self.create_publisher(String, '/jibot1/pick_place', 10)

        self.create_subscription(
            PointStamped, '/target_ball_position_stamped', self.vision_cb, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(
            String, '/jibot1/status', self.arm_status_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(0.1,  self.loop)
        self.create_timer(0.05, self.broadcast_dynamic_tf)

        self.get_logger().info(
            f'主控节点启动 | Nav2停止={self.nav2_stop_distance}m | '
            f'切换={self.safe_distance}m | 抓取={self.grasp_distance}m | '
            f'前进={self.approach_linear_speed}m/s | 急停={self.obstacle_stop_dist}m | '
            f'避障激光={self.scan_topic} | 前方scan角={math.degrees(self.scan_front_angle_offset):.1f}° | '
            f'避障横移={self.recovery_side_step_speed}m/s x {self.recovery_side_step_time}s | '
            f'相机TF=base_footprint->{CAMERA_LINK_FRAME_ID}->{self.camera_frame_id} | '
            f'机械臂TF=base_footprint->{ARM_BASE_FRAME_ID} | '
            f'机械臂偏移=({self.arm_offset_x:.3f},{self.arm_offset_y:.3f})m | '
            f'IK预检查={self.ik_reach_check_enabled}'
        )

    # ══════════════════════════════════════════════════════════════════
    # 状态切换（统一入口）
    # ══════════════════════════════════════════════════════════════════

    def _set_state(self, new_state: str):
        """统一状态切换入口：记录日志、更新时间、重置相关计数器"""
        if new_state != self.state:
            self.get_logger().info(f'[状态] {self.state} ──→ {new_state}')
        self.state            = new_state
        self.state_enter_time = time.time()
        if new_state in MOTION_LOCK_STATES:
            self._stop_robot()
        # 进入搜索或恢复状态时重置丢球计数，防止误触发
        if new_state in ("EXPLORE", "PATROL_NAV2", "OBSTACLE_RECOVERY", "FINE_SERVO_MODE"):
            self.lost_frames = 0

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ══════════════════════════════════════════════════════════════════
    # TF 广播（20Hz）
    # ══════════════════════════════════════════════════════════════════

    def broadcast_dynamic_tf(self):
        """
        动态广播相机TF。

        TF 链路：
          base_footprint -> robot_camera_link -> camera_frame_id
          base_footprint -> jibot1_arm_base

        相机位置参数（CAM_SEARCH / CAM_OBSERVE）描述的是：
          相机普通机身坐标系 robot_camera_link 在 base_footprint 下的物理位置和朝向。
        camera_frame_id 是视觉节点发布点使用的光学坐标系：
          optical x=右、y=下、z=前，不能直接当普通 ROS 机身坐标系挂到 base 上。
        """
        now = self.get_clock().now().to_msg()
        p = self.tf_search if self.current_posture == "SEARCH" else self.tf_observe

        mount_tf = TransformStamped()
        mount_tf.header.stamp    = now
        mount_tf.header.frame_id = 'base_footprint'  # IK原点，必须用这个
        mount_tf.child_frame_id  = CAMERA_LINK_FRAME_ID
        mount_tf.transform.translation.x = p['x']
        mount_tf.transform.translation.y = p['y']
        mount_tf.transform.translation.z = p['z']
        mount_tf.transform.rotation = quaternion_from_euler(p['r'], p['p'], p['y_aw'])

        optical_tf = TransformStamped()
        optical_tf.header.stamp = now
        optical_tf.header.frame_id = CAMERA_LINK_FRAME_ID
        optical_tf.child_frame_id = self.camera_frame_id
        optical_tf.transform.translation.x = 0.0
        optical_tf.transform.translation.y = 0.0
        optical_tf.transform.translation.z = 0.0
        optical_tf.transform.rotation = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)

        arm_tf = TransformStamped()
        arm_tf.header.stamp = now
        arm_tf.header.frame_id = 'base_footprint'
        arm_tf.child_frame_id = ARM_BASE_FRAME_ID
        arm_tf.transform.translation.x = self.arm_offset_x
        arm_tf.transform.translation.y = self.arm_offset_y
        arm_tf.transform.translation.z = 0.0
        arm_tf.transform.rotation = quaternion_from_euler(0.0, 0.0, 0.0)

        self.tf_broadcaster.sendTransform([mount_tf, optical_tf, arm_tf])

    def arm_status_cb(self, msg: String):
        """读取 pick_place_node / jibot1_driver 状态，避免机械臂动作没结束就进入下一阶段。"""
        self._last_arm_status = msg.data
        if msg.data == 'PICK_ONLY:DONE':
            self._pick_done = True
            self._pick_failed = False
            self.get_logger().info('[机械臂] 收到 PICK_ONLY:DONE，抓取动作已完成')
        elif msg.data == 'PICK_ONLY:FAILED' or msg.data.startswith('IK_FAILED:'):
            self._pick_failed = True
            self._pick_done = False
            self.get_logger().error(f'[机械臂] 抓取失败状态: {msg.data}')

    def _is_pick_target_reachable(self, target_x: float, target_y: float, target_z: float) -> bool:
        """Use the same IK solver as the arm driver before publishing a pick command."""
        if not self.ik_reach_check_enabled:
            return True
        approach_ok = (
            self._pick_kinematics.solve(target_x, target_y, self.ik_approach_height)
            is not None
        )
        pick_ok = self._pick_kinematics.solve(target_x, target_y, target_z) is not None
        return approach_ok and pick_ok

    # ══════════════════════════════════════════════════════════════════
    # 激光雷达
    # ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize an angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_cb(self, msg: LaserScan):
        self.scan_ranges          = list(msg.ranges)
        self.scan_angle_min       = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def get_front_obstacle_angle(self):
        """
        返回前方避障扇区内最近障碍物相对车头的角度。

        返回值：
          None：无障碍
          angle < 0：障碍偏右
          angle > 0：障碍偏左

        LaserScan 的角度在 laser frame 下；雷达反装时，车头正前方不一定是 0°。
        inf/nan 值会被比较运算自动忽略（结果为False），无需特殊处理。
        """
        if not self.scan_ranges:
            return None

        nearest_range = math.inf
        nearest_angle = None
        for i, r in enumerate(self.scan_ranges):
            angle = self.scan_angle_min + i * self.scan_angle_increment
            angle_from_front = self._normalize_angle(angle - self.scan_front_angle_offset)
            if abs(angle_from_front) <= self.obstacle_scan_angle:
                if 0.05 < r < self.obstacle_stop_dist and r < nearest_range:
                    nearest_range = r
                    nearest_angle = angle_from_front
        return nearest_angle

    def is_obstacle_ahead(self) -> bool:
        """Compatibility wrapper for tests and old call sites."""
        return self.get_front_obstacle_angle() is not None

    def _select_recovery_lateral_speed(self, obstacle_angle) -> float:
        """选择避障横移方向：障碍偏左则右移，障碍偏右则左移。"""
        speed = abs(self.recovery_side_step_speed)
        if speed <= 0.0 or self.recovery_side_step_time <= 0.0:
            return 0.0

        if obstacle_angle is None or abs(obstacle_angle) < self.obstacle_side_deadband:
            return speed if self.recovery_center_escape_direction == 'left' else -speed

        # ROS base_link 语义：linear.y > 0 是向左横移。
        return -speed if obstacle_angle > 0.0 else speed

    @staticmethod
    def _recovery_direction_label(lateral_speed: float) -> str:
        if lateral_speed > 0.0:
            return '左'
        if lateral_speed < 0.0:
            return '右'
        return '不横移'

    # ══════════════════════════════════════════════════════════════════
    # 主循环（10Hz）：时间驱动的状态转换和超时保护
    # ══════════════════════════════════════════════════════════════════

    def loop(self):
        elapsed = time.time() - self.state_enter_time

        if self.state == "INIT_POSTURE":
            # 发送初始寻找姿态指令，然后等待机械臂到位
            self.current_posture = "SEARCH"
            self.arm_raw_pub.publish(String(data="$DGT:39-39,1!"))
            self.get_logger().info('已发送机械臂搜索姿态指令: $DGT:39-39,1!')
            self._set_state("WAIT_INIT")

        elif self.state == "WAIT_INIT":
            wait_time = self.get_parameter('init_posture_wait_time').value
            if elapsed > wait_time:
                self._set_state("EXPLORE")

        elif self.state == "EXPLORE":
            if self.use_patrol and len(PATROL_WAYPOINTS) > 0:
                # 巡逻模式：发送下一个路点，立即切换到 PATROL_NAV2
                wp_idx = self.patrol_idx % len(PATROL_WAYPOINTS)
                wp = PATROL_WAYPOINTS[wp_idx]
                self.get_logger().info(
                    f'[巡逻] 路点 {wp_idx}/{len(PATROL_WAYPOINTS)-1}: '
                    f'({wp[0]:.2f}, {wp[1]:.2f}) yaw={math.degrees(wp[2]):.0f}°'
                )
                if self._send_nav2_waypoint(wp):
                    self._set_state("PATROL_NAV2")
                else:
                    # Nav2 未就绪，降级为原地旋转等待
                    twist = Twist()
                    twist.angular.z = self.search_angular_speed
                    self.cmd_pub.publish(twist)
            else:
                # 标准模式：原地旋转搜索，视觉回调发现球后切换状态
                twist = Twist()
                twist.angular.z = self.search_angular_speed
                self.cmd_pub.publish(twist)

        elif self.state == "PATROL_NAV2":
            # 超时：跳过当前路点，前进到下一个
            if elapsed > self.patrol_waypoint_timeout:
                wp_idx = self.patrol_idx % len(PATROL_WAYPOINTS)
                self.get_logger().warn(
                    f'[巡逻] 路点 {wp_idx} 超时({self.patrol_waypoint_timeout}s)，跳至下一路点')
                self._cancel_nav2()
                self._stop_robot()
                self.patrol_idx += 1
                self._set_state("EXPLORE")

        elif self.state == "NAV2_APPROACH_MODE":
            # 超时保护：Nav2长时间未到达切换位置
            if elapsed > self.nav2_timeout:
                self.get_logger().warn(f'Nav2超时({self.nav2_timeout}s)，返回搜索')
                self._cancel_nav2()
                self._stop_robot()
                self._set_state("EXPLORE")

        elif self.state == "FINE_SERVO_MODE":
            # 超时保护：精确伺服长时间未到达抓取距离（可能被卡住）
            if elapsed > self.servo_timeout:
                self.get_logger().warn(f'精确伺服超时({self.servo_timeout}s)，返回搜索')
                self._stop_robot()
                self._set_state("EXPLORE")

        elif self.state == "OBSTACLE_RECOVERY":
            # 先后退，拉开与障碍物的距离
            if elapsed < self.recovery_backup_time:
                twist = Twist()
                twist.linear.x = self.recovery_backup_speed
                self.cmd_pub.publish(twist)
            else:
                self._stop_robot()
                if abs(self.obstacle_recovery_lateral_speed) > 0.0:
                    direction = self._recovery_direction_label(
                        self.obstacle_recovery_lateral_speed)
                    self.get_logger().info(f'避障后退完成，开始向{direction}横移')
                    self._set_state("OBSTACLE_SIDE_STEP")
                else:
                    self.get_logger().info('避障后退完成，重新进入精确伺服')
                    self._set_state("FINE_SERVO_MODE")

        elif self.state == "OBSTACLE_SIDE_STEP":
            # 再向障碍物相反方向短距离横移，避免固定障碍造成来回前进后退
            if elapsed < self.recovery_side_step_time:
                twist = Twist()
                twist.linear.y = self.obstacle_recovery_lateral_speed
                self.cmd_pub.publish(twist)
            else:
                self._stop_robot()
                self.get_logger().info('避障横移完成，重新进入精确伺服')
                self._set_state("FINE_SERVO_MODE")

        elif self.state == "PRE_GRASP_POSTURE":
            # 发送夹球观察姿态指令，等待机械臂运动到位
            self.current_posture = "OBSERVE"
            self.arm_raw_pub.publish(String(data="$DGT:40-40,1!"))
            self.get_logger().info('已发送机械臂观察/抓取姿态指令: $DGT:40-40,1!')
            self._set_state("WAIT_PRE_GRASP")

        elif self.state == "WAIT_PRE_GRASP":
            # 实时读参数，支持 ros2 param set 动态调整
            pre_grasp_wait = self.get_parameter('pre_grasp_wait_time').value
            if elapsed > pre_grasp_wait:
                self.snapshot_retries = 0
                self._set_state("TAKE_SNAPSHOT")

        elif self.state == "WAIT_FOR_GRASP_DONE":
            # 实时读参数，支持 ros2 param set 动态调整
            # 注意：此时间必须 > pick_place_node._execute_pick_only 总耗时(≈8s)
            if self._pick_failed:
                self.get_logger().error('抓取动作失败，不执行丢球动作，回搜索姿态重新接近')
                self._pick_failed = False
                self._pick_done = False
                self.current_posture = "SEARCH"
                self._set_state("INIT_POSTURE")
            elif self._pick_done:
                self.arm_raw_pub.publish(String(data="$DGT:41-47,1!"))
                self._set_state("WAIT_FOR_DROP_DONE")
            elif elapsed > self.get_parameter('grasp_wait_time').value:
                self.get_logger().warn(
                    '等待 PICK_ONLY:DONE 超时，按 grasp_wait_time 兜底继续执行丢球动作'
                )
                self.arm_raw_pub.publish(String(data="$DGT:41-47,1!"))
                self._set_state("WAIT_FOR_DROP_DONE")

        elif self.state == "WAIT_FOR_DROP_DONE":
            # 实时读参数，支持 ros2 param set 动态调整
            if elapsed > self.get_parameter('drop_wait_time').value:
                self._pick_done = False
                self.current_posture = "SEARCH"
                self._set_state("INIT_POSTURE")

    # ══════════════════════════════════════════════════════════════════
    # 视觉回调（事件驱动）：感知驱动的状态转换
    # ══════════════════════════════════════════════════════════════════

    def vision_cb(self, msg: PointStamped):
        # 只在以下状态响应视觉
        active = {
            "EXPLORE", "PATROL_NAV2", "NAV2_APPROACH_MODE",
            "FINE_SERVO_MODE", "OBSTACLE_RECOVERY", "OBSTACLE_SIDE_STEP",
            "TAKE_SNAPSHOT"
        }
        if self.state not in active:
            return

        cam_x, cam_z = msg.point.x, msg.point.z

        # ── 丢球处理 ────────────────────────────────────────────────────
        if cam_z <= 0.01:
            # 搜索/巡逻阶段视觉节点会持续发布 (0,0,0) 表示暂未检测到球。
            # 这不是追踪丢失，不应打断原地搜索旋转或巡逻导航。
            if self.state in ("EXPLORE", "PATROL_NAV2"):
                return

            # OBSTACLE_RECOVERY / OBSTACLE_SIDE_STEP 和 TAKE_SNAPSHOT 不受lost_frames控制：
            #   - OBSTACLE_RECOVERY：正在后退，由loop()驱动，视觉临时看不到是正常的
            #   - OBSTACLE_SIDE_STEP：正在横移绕开障碍，由loop()驱动
            #   - TAKE_SNAPSHOT：由snapshot_retries管理，不应被中断
            if self.state in ("OBSTACLE_RECOVERY", "OBSTACLE_SIDE_STEP", "TAKE_SNAPSHOT"):
                return

            self.lost_frames += 1
            limit = (self.lost_frames_nav2
                     if self.state in ("NAV2_APPROACH_MODE", "PATROL_NAV2")
                     else self.lost_frames_servo)
            if self.lost_frames > limit:
                self.get_logger().warn(
                    f'丢失目标>{limit}帧 (当前:{self.state})，返回搜索')
                if self.state in ("NAV2_APPROACH_MODE", "PATROL_NAV2"):
                    self._cancel_nav2()
                self._stop_robot()
                self._set_state("EXPLORE")
            return

        # 有效检测，重置丢球计数
        self.lost_frames = 0

        # ── EXPLORE / PATROL_NAV2：发现球 ───────────────────────────────
        if self.state in ("EXPLORE", "PATROL_NAV2"):
            # 巡逻中发现球：先取消当前巡逻目标
            if self.state == "PATROL_NAV2":
                self.get_logger().info('[巡逻] 发现目标！取消巡逻，切换追球模式')
                self._cancel_nav2()
            self._stop_robot()
            if cam_z > self.safe_distance:
                # 距离远：用Nav2路径规划接近
                if self._send_nav2_goal(msg):
                    self._set_state("NAV2_APPROACH_MODE")
                else:
                    # Nav2不可用（无地图/TF不通），降级为纯比例控制
                    self.get_logger().warn('Nav2不可用，降级为精确伺服')
                    self._set_state("FINE_SERVO_MODE")
            else:
                # 距离近：直接精确伺服
                self._set_state("FINE_SERVO_MODE")

        # ── NAV2_APPROACH_MODE：动态更新目标 + 切换检测 ─────────────────
        elif self.state == "NAV2_APPROACH_MODE":
            # 到达切换阈值，Nav2交棒给精确伺服
            if cam_z <= self.safe_distance:
                self._cancel_nav2()
                self.get_logger().info(f'Nav2完成，距离={cam_z:.3f}m，切换精确伺服')
                self._set_state("FINE_SERVO_MODE")
                return

            # 每隔nav2_update_interval检查球是否移动，移动了就重发Nav2目标
            now = time.time()
            if now - self.last_nav2_send_time >= self.nav2_update_interval:
                try:
                    tf_to_map   = self.tf_buffer.lookup_transform(
                        'map', msg.header.frame_id, rclpy.time.Time())
                    ball_in_map = tf2_geometry_msgs.do_transform_point(msg, tf_to_map)
                    new_bx, new_by = ball_in_map.point.x, ball_in_map.point.y

                    moved = (math.hypot(new_bx - self.last_ball_map_x,
                                        new_by - self.last_ball_map_y)
                             if self.last_ball_map_x is not None else 999.0)

                    if moved > self.nav2_update_threshold:
                        self.get_logger().info(
                            f'球移动{moved:.2f}m，更新Nav2目标')
                        self._cancel_nav2()
                        self._send_nav2_goal(msg)
                    else:
                        self.last_nav2_send_time = now

                    self.last_ball_map_x = new_bx
                    self.last_ball_map_y = new_by

                except Exception as e:
                    self.get_logger().warn(
                        f'Nav2动态更新TF失败: {e}', throttle_duration_sec=2.0)

        # ── FINE_SERVO_MODE：比例控制靠近 ───────────────────────────────
        elif self.state == "FINE_SERVO_MODE":
            # 优先级1：到达抓取距离，停车
            if cam_z <= self.grasp_distance:
                self._stop_robot()
                self.get_logger().info(f'到达抓取距离={cam_z:.3f}m，停车')
                self._set_state("PRE_GRASP_POSTURE")
                return

            # 优先级2：前方障碍物，进入后退恢复
            obstacle_angle = self.get_front_obstacle_angle()
            if obstacle_angle is not None:
                self._stop_robot()
                self.obstacle_recovery_lateral_speed = (
                    self._select_recovery_lateral_speed(obstacle_angle)
                )
                obstacle_side = (
                    '左侧' if obstacle_angle > self.obstacle_side_deadband
                    else '右侧' if obstacle_angle < -self.obstacle_side_deadband
                    else '正前方'
                )
                escape_direction = self._recovery_direction_label(
                    self.obstacle_recovery_lateral_speed)
                self.get_logger().warn(
                    f'前方{self.obstacle_stop_dist}m内有障碍物({obstacle_side})，'
                    f'后退后向{escape_direction}横移',
                    throttle_duration_sec=1.0)
                self._set_state("OBSTACLE_RECOVERY")
                return

            # 优先级3：正常比例控制
            # angular.z = -gain * cam_x：cam_x正值=球在右，需右转（负angular.z）
            twist = Twist()
            twist.angular.z = (
                -self.approach_angular_gain * cam_x
                if abs(cam_x) > self.approach_deadband else 0.0
            )
            twist.linear.x = self.approach_linear_speed
            self.cmd_pub.publish(twist)

        # ── OBSTACLE_RECOVERY / OBSTACLE_SIDE_STEP：避障中，忽略视觉（由loop()驱动）──
        elif self.state in ("OBSTACLE_RECOVERY", "OBSTACLE_SIDE_STEP"):
            pass

        # ── TAKE_SNAPSHOT：计算抓取坐标并发送给机械臂 ───────────────────
        elif self.state == "TAKE_SNAPSHOT":
            try:
                # 将球的位置从相机坐标系变换到 base_footprint（IK原点）
                trans      = self.tf_buffer.lookup_transform(
                    'base_footprint', msg.header.frame_id, rclpy.time.Time()
                )
                foot_point = tf2_geometry_msgs.do_transform_point(msg, trans)

                # ── 坐标系转换说明 ──────────────────────────────────────────
                # ROS base_footprint:  x=前, y=左
                # jibot1 IK 坐标系:    x=右(正右/负左), y=前(必须>=0)
                #
                # 转换关系：
                #   IK_x = -(ROS_y - arm_offset_y)   左→右取反
                #   IK_y =  (ROS_x - arm_offset_x)   前→前不变
                #   IK_z =  grasp_z                   使用实测抓取高度
                #
                # arm_offset_x: 机械臂底座比底盘中心更靠前的距离(m)
                # arm_offset_y: 机械臂底座比底盘中心更靠左的距离(m)

                ros_x = foot_point.point.x  # 前方距离 (m)
                ros_y = foot_point.point.y  # 左方距离 (m)

                # 减去偏移后得到相对于机械臂底座的坐标，再换算为mm
                target_x = -(ros_y - self.arm_offset_y) * 1000.0  # IK x: 右为正
                target_y =  (ros_x - self.arm_offset_x) * 1000.0  # IK y: 前为正
                target_z =  self.grasp_z

                if target_y < 0:
                    self.get_logger().warn(
                        f'IK y={target_y:.1f}mm<0（球在机械臂后方），无法抓取，返回搜索')
                    self.snapshot_retries = self.snapshot_max_retries
                    raise ValueError('ball behind arm base')

                self.get_logger().info(
                    f'抓取坐标(IK输入): ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) mm'
                    f' | 原始ROS: ({ros_x*1000:.1f}, {ros_y*1000:.1f}) mm'
                )
                if not self._is_pick_target_reachable(target_x, target_y, target_z):
                    self.get_logger().warn(
                        '抓取坐标超出机械臂工作空间，取消本次抓取并回搜索姿态 | '
                        f'IK=({target_x:.1f},{target_y:.1f},{target_z:.1f})mm | '
                        f'接近高度={self.ik_approach_height:.1f}mm | '
                        '请继续靠近、调小 GRASP_DISTANCE 或校准 CAM_OBSERVE/ARM_OFFSET'
                    )
                    self.snapshot_retries = 0
                    self._pick_done = False
                    self._pick_failed = False
                    self.current_posture = "SEARCH"
                    self._set_state("INIT_POSTURE")
                    return

                pick_cmd      = String()
                pick_cmd.data = f"{target_x:.1f},{target_y:.1f},{target_z:.1f}"
                self._pick_done = False
                self._pick_failed = False
                self.arm_pick_pub.publish(pick_cmd)
                self.snapshot_retries = 0
                self._set_state("WAIT_FOR_GRASP_DONE")

            except Exception as e:
                self.snapshot_retries += 1
                self.get_logger().error(
                    f'TF变换失败 [{self.snapshot_retries}/{self.snapshot_max_retries}]: {e}'
                )
                if self.snapshot_retries >= self.snapshot_max_retries:
                    self.get_logger().error('抓取坐标多次失败，放弃，先回搜索姿态')
                    self.snapshot_retries = 0
                    self.current_posture = "SEARCH"
                    self._set_state("INIT_POSTURE")

    # ══════════════════════════════════════════════════════════════════
    # Nav2 目标发送
    # ══════════════════════════════════════════════════════════════════

    def _send_nav2_goal(self, cam_msg: PointStamped) -> bool:
        """
        计算球正前方 nav2_stop_distance 米处的导航目标点。

        坐标推导：
          1. 球在map坐标系：(ball_x, ball_y)
          2. 机器人在map坐标系：(robot_x, robot_y)
          3. 机器人→球方向角：angle = atan2(ball_y-robot_y, ball_x-robot_x)
          4. 目标点 = 球坐标 沿反方向退后 nav2_stop_distance 米：
               goal_x = ball_x - nav2_stop_distance * cos(angle)
               goal_y = ball_y - nav2_stop_distance * sin(angle)
          5. 导航朝向 = angle（车头正对球）
        """
        try:
            # 球在map坐标系
            tf_to_map   = self.tf_buffer.lookup_transform(
                'map', cam_msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            ball_in_map = tf2_geometry_msgs.do_transform_point(cam_msg, tf_to_map)
            ball_x, ball_y = ball_in_map.point.x, ball_in_map.point.y

            # 机器人在map坐标系（用base_footprint）
            tf_robot = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            robot_x = tf_robot.transform.translation.x
            robot_y = tf_robot.transform.translation.y

            angle  = math.atan2(ball_y - robot_y, ball_x - robot_x)
            goal_x = ball_x - self.nav2_stop_distance * math.cos(angle)
            goal_y = ball_y - self.nav2_stop_distance * math.sin(angle)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.orientation = quaternion_from_euler(0, 0, angle)

            self._nav2_goal_pending = True
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._nav2_goal_response_cb)

            self.last_ball_map_x     = ball_x
            self.last_ball_map_y     = ball_y
            self.last_nav2_send_time = time.time()

            self.get_logger().info(
                f'Nav2目标: ({goal_x:.2f},{goal_y:.2f}) | '
                f'球: ({ball_x:.2f},{ball_y:.2f}) | '
                f'提前停止: {self.nav2_stop_distance}m | '
                f'朝向: {math.degrees(angle):.1f}°'
            )
            return True

        except Exception as e:
            self.get_logger().warn(f'Nav2目标发送失败: {e}')
            self._nav2_goal_pending = False
            return False

    def _nav2_goal_response_cb(self, future):
        """异步Goal响应回调：保存handle，如果状态已变则立即取消"""
        self._nav2_goal_pending = False
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Nav2目标响应读取失败: {e}')
            if self.state == "NAV2_APPROACH_MODE":
                self._set_state("FINE_SERVO_MODE")
            elif self.state == "PATROL_NAV2":
                self.patrol_idx += 1
                self._set_state("EXPLORE")
            return

        if not handle.accepted:
            self.get_logger().warn('Nav2拒绝了目标')
            if self.state == "NAV2_APPROACH_MODE":
                self.get_logger().warn('Nav2被拒绝，降级精确伺服')
                self._set_state("FINE_SERVO_MODE")
            elif self.state == "PATROL_NAV2":
                self.get_logger().warn('[巡逻] 目标被拒绝，跳至下一路点')
                self.patrol_idx += 1
                self._set_state("EXPLORE")
            return

        # 如果在等待期间状态已经切走，立即取消这个goal
        if self.state not in ("NAV2_APPROACH_MODE", "PATROL_NAV2"):
            self.get_logger().info(
                f'Nav2目标已接受但状态已变为{self.state}，立即取消')
            handle.cancel_goal_async()
            return

        self._nav2_goal_handle = handle
        self.get_logger().info('Nav2目标已接受，开始导航')
        self._nav2_result_future = handle.get_result_async()
        self._nav2_result_future.add_done_callback(
            lambda result_future, goal_handle=handle:
            self._nav2_result_cb(result_future, goal_handle)
        )

    def _nav2_result_cb(self, future, goal_handle):
        """异步Nav2结果回调：巡逻点完成后推进，追球导航失败则降级伺服。"""
        if self._nav2_goal_handle is not goal_handle:
            return

        self._nav2_goal_handle = None
        self._nav2_goal_pending = False
        self._nav2_result_future = None

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Nav2结果读取失败: {e}')
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == "PATROL_NAV2":
                wp_idx = self.patrol_idx % len(PATROL_WAYPOINTS)
                self.get_logger().info(f'[巡逻] 路点 {wp_idx} 已到达，切换下一路点')
                self.patrol_idx += 1
                self._stop_robot()
                self._set_state("EXPLORE")
            elif self.state == "NAV2_APPROACH_MODE":
                self.get_logger().info('Nav2接近目标完成，切换精确伺服')
                self._stop_robot()
                self._set_state("FINE_SERVO_MODE")
            return

        status_name = self._nav2_status_name(status)
        if self.state == "PATROL_NAV2":
            wp_idx = self.patrol_idx % len(PATROL_WAYPOINTS)
            self.get_logger().warn(
                f'[巡逻] 路点 {wp_idx} 导航结束状态={status_name}，跳至下一路点'
            )
            self.patrol_idx += 1
            self._stop_robot()
            self._set_state("EXPLORE")
        elif self.state == "NAV2_APPROACH_MODE":
            self.get_logger().warn(
                f'Nav2接近目标结束状态={status_name}，降级精确伺服'
            )
            self._stop_robot()
            self._set_state("FINE_SERVO_MODE")

    @staticmethod
    def _nav2_status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))

    def _send_nav2_waypoint(self, waypoint: tuple) -> bool:
        """
        发送固定巡逻路点的 Nav2 导航目标。

        Args:
            waypoint: (x, y, yaw_rad) 地图坐标系下的目标位置和朝向

        Returns:
            True 表示请求已发出；False 表示 Nav2 服务器未就绪。
        """
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('[巡逻] Nav2 Action服务器未就绪，降级原地旋转')
            return False

        try:
            x, y, yaw = waypoint
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            goal_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, float(yaw))

            self._nav2_goal_pending = True
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._nav2_goal_response_cb)
            return True

        except Exception as e:
            self.get_logger().warn(f'[巡逻] 路点发送异常: {e}')
            self._nav2_goal_pending = False
            return False

    def _cancel_nav2(self):
        """正确取消Nav2（通过goal handle）"""
        if self._nav2_goal_handle is not None:
            self._nav2_goal_handle.cancel_goal_async()
            self._nav2_goal_handle = None
            self._nav2_result_future = None
            self.get_logger().info('Nav2目标已取消')


def main(args=None):
    rclpy.init(args=args)
    node = MainControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
