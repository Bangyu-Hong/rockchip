"""
Jibot1 逆运动学求解器
从 ESP32 MicroPython 源码 (z_kinematics.py) 移植，参数统一提至顶部

坐标系定义：
  x: 左右方向 (正值=右侧, 负值=左侧), 单位 mm
  y: 前后方向 (正值=前方, 不可为负), 单位 mm
  z: 高度 (距离地面), 单位 mm

舵机编号：
  0: 底盘旋转 (云台 / 底座偏转)
  1: 大臂    (肩关节 theta5)
  2: 小臂    (肘关节 theta4)
  3: 腕关节  (theta3)

╔══════════════════════════════════════════════════════════════╗
║                    参数配置区（修改这里）                      ║
╚══════════════════════════════════════════════════════════════╝
"""

from math import sqrt, atan, acos, sin, cos, pi
from typing import List, Optional

# ── 连杆参数（mm，根据实际机械臂测量）────────────────────────────────────
L0 = 235   # 底盘地面 → 肩关节(舵机1)中心轴 的固定高度
L1 =  85   # 肩关节   → 肘关节(舵机2) 大臂长度
L2 =  55   # 肘关节   → 腕关节(舵机3) 小臂长度
L3 = 205   # 腕关节   → 末端夹爪闭合点 手部长度

# ── Alpha 自动搜索范围（末端与水平面夹角，单位：度，负值=向下）──────────
# alpha 代表末端（夹爪）相对水平面的俯仰角
# -25°: 末端微微向下（适合较远距离目标）
# -65°: 末端大幅向下（适合较近/较低目标）
ALPHA_SEARCH_MAX = -25   # 搜索起点（最平）
ALPHA_SEARCH_MIN = -90   # 搜索终点（最陡）

# y 小于此阈值（mm）时选最陡的 alpha（夹近处目标），否则选最平的 alpha
ALPHA_NEAR_THRESHOLD = 215   # (mm)

# ── 关节角度限制（单位：度，超出则视为无效解）────────────────────────────
# 大臂 (theta5, 肩关节)：0° = 水平前伸, 90° = 垂直朝上, 180° = 水平后折
SHOULDER_MIN =   -20.0   # 大臂下限 (°)
SHOULDER_MAX = 180.0   # 大臂上限 (°)

# 小臂 (theta4, 肘关节)：正值=向上折叠, 负值=向下折叠
ELBOW_MIN = -60.0     # 小臂下限 (°)
ELBOW_MAX =  60.0     # 小臂上限 (°)

# 腕关节 (theta3)：正值=上翘, 负值=下压
WRIST_MIN =  -90.0     # 腕关节下限 (°)
WRIST_MAX =   90.0     # 腕关节上限 (°)

# ── PWM 全局软件限位（us，防止超出舵机物理行程损坏机械结构）─────────────
PWM_GLOBAL_MIN = 500   # 所有舵机 PWM 下限
PWM_GLOBAL_MAX = 2500  # 所有舵机 PWM 上限

# ════════════════════════════════════════════════════════════════════════
# 以下为算法实现，与 ESP32 固件保持一致，一般不需要修改
# ════════════════════════════════════════════════════════════════════════


class Kinematics:
    """
    4自由度机械臂逆运动学求解器。

    算法来源：ESP32 MicroPython 固件 z_kinematics.py
    内部使用 0.1mm 精度（坐标值 ×10），PWM 公式与固件完全一致。

    默认调用方式（自动搜索最优 alpha）：
        kin = Kinematics()
        pwm = kin.solve(x_mm, y_mm, z_mm)

    指定末端俯仰角：
        pwm = kin.solve(x_mm, y_mm, z_mm, alpha=-45)  # -45° 向下
    """

    def __init__(self,
                 l0: int = L0, l1: int = L1,
                 l2: int = L2, l3: int = L3,
                 alpha_search_max: int = ALPHA_SEARCH_MAX,
                 alpha_search_min: int = ALPHA_SEARCH_MIN,
                 alpha_near_threshold: float = ALPHA_NEAR_THRESHOLD,
                 shoulder_min: float = SHOULDER_MIN,
                 shoulder_max: float = SHOULDER_MAX,
                 elbow_min: float    = ELBOW_MIN,
                 elbow_max: float    = ELBOW_MAX,
                 wrist_min: float    = WRIST_MIN,
                 wrist_max: float    = WRIST_MAX,
                 pwm_global_min: int = PWM_GLOBAL_MIN,
                 pwm_global_max: int = PWM_GLOBAL_MAX):

        # 内部放大 10 倍以提高精度（单位：0.1mm）
        self._l0 = l0 * 10
        self._l1 = l1 * 10
        self._l2 = l2 * 10
        self._l3 = l3 * 10

        self._alpha_max      = alpha_search_max
        self._alpha_min      = alpha_search_min
        self._alpha_near_thr = alpha_near_threshold

        self._shoulder_min = shoulder_min
        self._shoulder_max = shoulder_max
        self._elbow_min    = elbow_min
        self._elbow_max    = elbow_max
        self._wrist_min    = wrist_min
        self._wrist_max    = wrist_max

        self._pwm_min = pwm_global_min
        self._pwm_max = pwm_global_max

    # ── 主接口 ──────────────────────────────────────────────────────────

    def solve(self, x: float, y: float, z: float,
              alpha: Optional[float] = None) -> Optional[List[int]]:
        """
        逆运动学求解，返回四个舵机的 PWM 值。

        Args:
            x     : 目标 x 坐标 (mm)，正值=右侧，负值=左侧
            y     : 目标 y 坐标 (mm)，必须 >= 0（前方），不支持背后抓取
            z     : 目标 z 坐标 (mm)，距地面高度
            alpha : 末端与水平面夹角 (度)，负值=向下
                    None → 在 [ALPHA_SEARCH_MIN, ALPHA_SEARCH_MAX] 范围内
                    自动搜索最优解（推荐）

        Returns:
            [pwm0, pwm1, pwm2, pwm3] 已做 PWM 限位的列表；
            无解返回 None。
        """
        if y < 0:
            return None

        if alpha is not None:
            # 指定角度：直接求解
            return self._ik(x, y, z, alpha)

        # 自动搜索：从 alpha_max 到 alpha_min 步进 -1°
        candidates = []
        for a in range(self._alpha_max, self._alpha_min - 1, -1):
            result = self._ik(x, y, z, a)
            if result is not None:
                candidates.append((a, result))

        if not candidates:
            return None

        # 近处目标（y 较小）选最陡角度（alpha 最小），
        # 远处目标（y 较大）选最平角度（alpha 最大），与固件逻辑一致
        if y > self._alpha_near_thr:
            best = max(candidates, key=lambda c: c[0])   # 最平（alpha 最大）
        else:
            best = min(candidates, key=lambda c: c[0])   # 最陡（alpha 最小）
        return best[1]

    def pwm_to_cmd(self, pwm_list: List[int], move_time: int) -> str:
        """
        将 PWM 列表转换为 ESP32 协议指令字符串。

        协议格式（与 ESP32 固件完全一致）：
            {#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!}

        Args:
            pwm_list  : [pwm0, pwm1, pwm2, pwm3]
            move_time : 运动时间 (ms)

        Returns:
            协议指令字符串。
        """
        cmd = '{'
        for i, pwm in enumerate(pwm_list):
            pwm = max(self._pwm_min, min(self._pwm_max, pwm))
            cmd += '#%03dP%04dT%04d!' % (i, pwm, move_time)
        cmd += '}'
        return cmd

    def workspace_check(self, x: float, y: float, z: float) -> bool:
        """
        快速判断目标点是否可能在工作空间内（仅做距离粗校验，不完整 IK）。

        Returns:
            True  可能可达（还需完整 solve 验证）
            False 肯定不可达
        """
        if y < 0:
            return False
        yy = sqrt(x * x + y * y)
        zz = z - L0
        dist = sqrt(yy * yy + zz * zz)
        return abs(L1 - L2) <= dist <= (L1 + L2 + L3)

    # ── 内部单次 IK 计算 ─────────────────────────────────────────────────

    def _ik(self, x: float, y: float, z: float,
            alpha: float) -> Optional[List[int]]:
        """
        给定 alpha 角的单次逆运动学计算。
        与 ESP32 固件 z_kinematics.py 算法完全一致（内部 0.1mm 精度）。
        """
        x10 = x * 10
        y10 = y * 10
        z10 = z * 10

        # ── 底座偏转角 theta6 ───────────────────────────────────────────
        if x10 == 0 and y10 != 0:
            theta6 = 0.0            # 正前方，无偏转
        elif x10 > 0 and y10 == 0:
            theta6 = 90.0           # 纯右侧
        elif x10 < 0 and y10 == 0:
            theta6 = -90.0          # 纯左侧
        elif x10 == 0 and y10 == 0:
            return None             # 原点，无法求解
        else:
            theta6 = atan(x10 / y10) * 180.0 / pi   # 一般情况

        # ── 投影到竖直平面，去掉末端连杆分量 ──────────────────────────
        a_rad = alpha * pi / 180.0
        yy = sqrt(x10 * x10 + y10 * y10) - self._l3 * cos(a_rad)
        zz = z10 - self._l0 - self._l3 * sin(a_rad)

        # ── 可达性检查 ──────────────────────────────────────────────────
        if zz < -self._l0:
            return None
        dist = sqrt(yy * yy + zz * zz)
        if dist > (self._l1 + self._l2) or dist == 0:
            return None

        # ── 2R 平面逆解 ─────────────────────────────────────────────────
        # ccc: yy到dist的角度偏置
        ccc = acos(max(-1.0, min(1.0, yy / dist)))

        # bbb: 余弦定理求 l1 与 dist 的夹角
        bbb = ((yy * yy + zz * zz + self._l1 * self._l1 - self._l2 * self._l2)
               / (2 * self._l1 * dist))
        if bbb > 1.0 or bbb < -1.0:
            return None

        # zz<0 时（目标低于肩关节）角度符号取反
        zf_flag = -1 if zz < 0 else 1
        theta5 = (ccc * zf_flag + acos(bbb)) * 180.0 / pi   # 大臂角

        # ── 关节限位检查 ────────────────────────────────────────────────
        if not (self._shoulder_min <= theta5 <= self._shoulder_max):
            return None

        aaa = (-(yy * yy + zz * zz - self._l1 * self._l1 - self._l2 * self._l2)
               / (2 * self._l1 * self._l2))
        if aaa > 1.0 or aaa < -1.0:
            return None

        theta4 = 180.0 - acos(aaa) * 180.0 / pi              # 小臂角
        if not (self._elbow_min <= theta4 <= self._elbow_max):
            return None

        theta3 = alpha - theta5 + theta4                       # 腕关节角
        if not (self._wrist_min <= theta3 <= self._wrist_max):
            return None

        # ── 角度 → PWM（与 ESP32 固件公式完全一致）──────────────────────
        pwm = [
            int(1500 - 2000.0 * theta6 / 270.0),          # 底座偏转
            int(1500 + 2000.0 * (theta5 - 90) / 270.0),   # 大臂
            int(1500 + 2000.0 * theta4 / 270.0),           # 小臂
            int(1500 + 2000.0 * theta3 / 270.0),           # 腕关节
        ]
        return pwm
