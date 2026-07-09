"""
===============================================================================
                    YOLO + 颜色 + 形状 三合一增强版乒乓球3D追踪器
===============================================================================
版本: 5.3 (全频段制导终极版)
更新日志:
- 🆕 软件测距接管: 当进入红外物理盲区(<0.2m)时，无缝切换为 2D 像素大小反推深度。
- 🆕 远距截断: 当超出最大测距(>2.5m)时，输出最大值 2.5m 引导小车前进，而不是归零。
- 自动读取并应用真实的相机内参 (fx, fy) 进行测距和 3D 转换。
===============================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped  # 带时间戳和坐标系的点
from cv_bridge import CvBridge
import cv2
from rknnlite.api import RKNNLite
import numpy as np
import message_filters
from collections import deque
import time


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║                    ⚙️ 用户可调参数区 (所有开关和参数)                       ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# --- 功能开关区 ---
ENABLE_DEPTH_PREFILTER = True       # 深度图预处理滤波 (去除噪声)
ENABLE_DEPTH_JUMP_FILTER = True     # 深度跳变抑制 (防止突变)
ENABLE_SPHERICAL_COMPENSATION = True # 球面几何动态补偿 (科学家模式)
ENABLE_VISUALIZATION = True         # 主窗口 (显示检测结果)
ENABLE_DEBUG_WINDOWS = False         # 调试窗口 (颜色掩码+深度图)
FLIP_DEPTH_HORIZONTAL = True        # 水平翻转深度图 (左右相反时开启)
FLIP_COLOR_HORIZONTAL = False       # 水平翻转彩色图
ENABLE_DEPTH_PADDING = True         # 深度图黑边填充对齐
PUBLISH_WITH_TIMESTAMP = True       # 发布带时间戳的坐标 (用于TF转换)

# --- 物理参数区 ---
BALL_DIAMETER_M = 0.040             # 乒乓球直径 (米) - 国际标准40mm
BALL_RADIUS_M = 0.020               # 乒乓球半径 (米) - 20mm
DEPTH_MIN_M = 0.20                  # 最小有效硬件深度 (米) - 低于此值启用软件测距
DEPTH_MAX_M = 2.50                  # 最大有效深度 (米) - 高于此值将被截断输出
DEPTH_SCALE = 1000.0                # 深度单位转换 (Gemini输出毫米)

# --- 颜色检测参数区 ---
BALL_COLOR = 'orange'               # 可选: 'orange', 'white', 'yellow', 'auto'
ORANGE_H_MIN, ORANGE_H_MAX = 5, 25          # 色相 (0-180)
ORANGE_S_MIN, ORANGE_S_MAX = 100, 255       # 饱和度 (0-255)
ORANGE_V_MIN, ORANGE_V_MAX = 100, 255       # 亮度 (0-255)
WHITE_H_MIN, WHITE_H_MAX = 0, 180           # 白色不限色相
WHITE_S_MIN, WHITE_S_MAX = 0, 40            # 低饱和度
WHITE_V_MIN, WHITE_V_MAX = 180, 255         # 高亮度
YELLOW_H_MIN, YELLOW_H_MAX = 20, 35
YELLOW_S_MIN, YELLOW_S_MAX = 100, 255
YELLOW_V_MIN, YELLOW_V_MAX = 100, 255
COLOR_MIN_PIXELS = 30              # 最小颜色像素数 (过滤噪点)

# --- 检测算法参数区 ---
YOLO_MODEL_PATH = '/userdata/yolo_tracking/pingpang.rknn'  # RKNN模型路径
YOLO_INPUT_SIZE = 640               # 模型输入尺寸
YOLO_CONF_THRESHOLD = 0.4           # 置信度阈值 (0.0-1.0)
YOLO_IOU_THRESHOLD = 0.45           # NMS IoU阈值 (0.0-1.0)
CLASSES = ["table_tennis_ball"]     # 类别名称
CIRCULARITY_THRESHOLD = 0.6         # 圆度阈值 (0.0-1.0)
ROI_MARGIN_RATIO = 0.2              # ROI扩展比例

# --- 滤波器参数区 ---
DEPTH_FILTER_TYPE = 'median'        # 滤波类型: 'median' 或 'bilateral'
DEPTH_FILTER_SIZE = 3               # 滤波核大小 (奇数: 3, 5, 7...)
FILTER_WINDOW_SIZE = 5              # 滤波窗口 (帧数)
FRAMES_TO_RESET = 10                # 丢失多少帧后重置滤波器
DEPTH_JUMP_THRESHOLD = 0.3          # 允许的最大跳变 (米)

# --- 多球目标锁定参数区 ---
TARGET_LOCK_ENABLED = True          # 多个球同时出现时，优先跟踪上一帧锁定的球
TARGET_LOCK_MAX_PIXEL_JUMP = 160.0  # 锁定目标每帧允许的最大像素跳变
TARGET_LOCK_MAX_MISSES = 5          # 连续丢失多少帧后解除目标锁定
SHOW_FPS_ON_IMAGE = True            # 在RGB调试窗口显示视觉处理FPS
DETECTION_LOG_INTERVAL_SEC = 1.0    # 终端识别日志最短打印间隔，避免逐帧刷屏
COMPACT_DEBUG_PANEL = True          # 精简RGB窗口左上角信息，方便实车观察

# --- 相机参数区 (初始值，会被 camera_info 自动覆盖) ---
CAMERA_FX = 454.05139               # 焦距 X (像素)
CAMERA_FY = 454.05139               # 焦距 Y (像素)
CAMERA_CX = 327.56286               # 光心 X (像素)
CAMERA_CY = 244.89233               # 光心 Y (像素)
CAMERA_FRAME_ID = 'camera_color_optical_frame'  # 输出坐标的参考坐标系

# --- ROS 话题配置 ---
TOPIC_COLOR_IMAGE = '/camera/color/image_raw'
TOPIC_DEPTH_IMAGE = '/camera/depth/image_raw'
TOPIC_CAMERA_INFO = '/camera/color/camera_info'
TOPIC_BALL_POSITION = '/target_ball_position'           # geometry_msgs/Point
TOPIC_BALL_POSITION_STAMPED = '/target_ball_position_stamped'  # 带时间戳

# --- 可视化配置 ---
WINDOW_NAME = "Ball 3D Tracker"
DEBUG_WINDOW_COLOR = "DEBUG: Color Mask"
DEBUG_WINDOW_DEPTH = "DEBUG: Depth Map"

# 预计算颜色阈值数组
ORANGE_LOWER = np.array([ORANGE_H_MIN, ORANGE_S_MIN, ORANGE_V_MIN])
ORANGE_UPPER = np.array([ORANGE_H_MAX, ORANGE_S_MAX, ORANGE_V_MAX])
WHITE_LOWER = np.array([WHITE_H_MIN, WHITE_S_MIN, WHITE_V_MIN])
WHITE_UPPER = np.array([WHITE_H_MAX, WHITE_S_MAX, WHITE_V_MAX])
YELLOW_LOWER = np.array([YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN])
YELLOW_UPPER = np.array([YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX])


def filter_boxes_yolov8(inputs, conf_thres=YOLO_CONF_THRESHOLD, iou_thres=YOLO_IOU_THRESHOLD):
    """YOLOv8/v11 后处理函数"""
    out = inputs[0]
    if len(out.shape) == 3:
        out = out[0]
    if out.shape[0] < out.shape[1]:
        out = out.transpose(1, 0)

    boxes, scores, classes = [], [], []
    box_data = out[:, :4]
    class_scores = out[:, 4:]
    max_scores = np.max(class_scores, axis=1)
    class_ids = np.argmax(class_scores, axis=1)
    valid_idx = max_scores > conf_thres
    valid_boxes = box_data[valid_idx]
    valid_scores = max_scores[valid_idx]
    valid_ids = class_ids[valid_idx]

    for i in range(len(valid_boxes)):
        cx, cy, w, h = valid_boxes[i]
        boxes.append([cx - w/2, cy - h/2, cx + w/2, cy + h/2])
        scores.append(valid_scores[i])
        classes.append(valid_ids[i])

    if len(boxes) == 0:
        return [], [], []

    boxes, scores, classes = np.array(boxes), np.array(scores), np.array(classes)
    keep = []
    for c in np.unique(classes):
        idx = np.where(classes == c)[0]
        c_boxes, c_scores = boxes[idx], scores[idx]
        x1, y1, x2, y2 = c_boxes[:,0], c_boxes[:,1], c_boxes[:,2], c_boxes[:,3]
        areas = (x2-x1) * (y2-y1)
        order = c_scores.argsort()[::-1]
        while order.size > 0:
            i = order[0]
            keep.append(idx[i])
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2-xx1) * np.maximum(0, yy2-yy1)
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(ovr <= iou_thres)[0] + 1]

    return boxes[keep], classes[keep], scores[keep]


class CoordinateFilter:
    """多帧坐标滤波器 (带跳变抑制)"""

    def __init__(self, window_size=FILTER_WINDOW_SIZE):
        self.window_size = window_size
        self.x_buffer = deque(maxlen=window_size)
        self.y_buffer = deque(maxlen=window_size)
        self.z_buffer = deque(maxlen=window_size)

    def update(self, x, y, z):
        """添加新坐标并返回滤波后的结果"""
        # 如果传入的是完全无效的零坐标，不污染滤波器
        if x == 0 and y == 0 and z == 0:
            return 0.0, 0.0, 0.0

        if ENABLE_DEPTH_JUMP_FILTER and len(self.z_buffer) >= 3:
            median_z = np.median(list(self.z_buffer))
            if abs(z - median_z) > DEPTH_JUMP_THRESHOLD:
                z = median_z

        self.x_buffer.append(x)
        self.y_buffer.append(y)
        self.z_buffer.append(z)

        return (np.median(list(self.x_buffer)),
                np.median(list(self.y_buffer)),
                np.median(list(self.z_buffer)))

    def reset(self):
        self.x_buffer.clear()
        self.y_buffer.clear()
        self.z_buffer.clear()


class NPU3DTracker(Node):

    def __init__(self):
        super().__init__('npu_3d_tracker')
        self.bridge = CvBridge()
        self.declare_parameter('camera_frame_id', CAMERA_FRAME_ID)
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        # 初始化相机内参（会被 camera_info_callback 自动覆盖）
        self.fx, self.fy = CAMERA_FX, CAMERA_FY
        self.cx, self.cy = CAMERA_CX, CAMERA_CY
        self.camera_info_received = False

        self.target_pub = self.create_publisher(Point, TOPIC_BALL_POSITION, 10)
        if PUBLISH_WITH_TIMESTAMP:
            self.target_stamped_pub = self.create_publisher(PointStamped, TOPIC_BALL_POSITION_STAMPED, 10)

        self.rknn = RKNNLite()
        self.get_logger().info(f'🚀 加载模型: {YOLO_MODEL_PATH}')
        if self.rknn.load_rknn(YOLO_MODEL_PATH) != 0 or self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
            self.get_logger().error('❌ 模型或NPU初始化失败！')
            return

        self.coord_filter = CoordinateFilter()
        self.frames_without_detection = 0
        self.locked_target_px = None
        self.locked_target_py = None
        self.lock_miss_count = 0
        self.last_frame_time = None
        self.fps = 0.0
        self.last_detection_log_time = 0.0

        self.info_sub = self.create_subscription(CameraInfo, TOPIC_CAMERA_INFO, self.camera_info_callback, 10)
        color_sub = message_filters.Subscriber(self, Image, TOPIC_COLOR_IMAGE)
        depth_sub = message_filters.Subscriber(self, Image, TOPIC_DEPTH_IMAGE)
        self.ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('✅ 乒乓球3D追踪器启动！v5.3 全频段制导终极版 (硬件红外 + 软件像素双重测距)')

    def publish_target(self, x, y, z, stamp=None):
        """Publish the current target position in the configured camera frame."""
        msg = Point()
        msg.x, msg.y, msg.z = float(x), float(y), float(z)
        self.target_pub.publish(msg)

        if PUBLISH_WITH_TIMESTAMP:
            stamped_msg = PointStamped()
            stamped_msg.header.stamp = stamp or self.get_clock().now().to_msg()
            stamped_msg.header.frame_id = self.camera_frame_id
            stamped_msg.point.x = msg.x
            stamped_msg.point.y = msg.y
            stamped_msg.point.z = msg.z
            self.target_stamped_pub.publish(stamped_msg)

    def publish_no_target(self, stamp=None):
        """Publish an explicit invalid target so downstream lost-frame logic advances."""
        self.publish_target(0.0, 0.0, 0.0, stamp=stamp)

    def update_fps(self):
        """更新图像处理帧率；该值近似等于目标点发布频率。"""
        now = time.monotonic()
        if self.last_frame_time is None:
            self.last_frame_time = now
            return self.fps

        dt = now - self.last_frame_time
        self.last_frame_time = now
        if dt <= 0.0:
            return self.fps

        instant_fps = 1.0 / dt
        self.fps = instant_fps if self.fps <= 0.0 else 0.85 * self.fps + 0.15 * instant_fps
        return self.fps

    def select_target_ball(self, balls):
        """选择当前跟踪目标，避免多球时每帧跳到不同小球。"""
        if not balls:
            return None

        candidates = [b for b in balls if b['depth_m'] > 0] or list(balls)
        has_locked_target = TARGET_LOCK_ENABLED
        has_locked_target = has_locked_target and self.locked_target_px is not None
        has_locked_target = has_locked_target and self.locked_target_py is not None
        if has_locked_target:
            locked_candidates = [
                b for b in candidates
                if np.hypot(b['cx'] - self.locked_target_px,
                            b['cy'] - self.locked_target_py) <= TARGET_LOCK_MAX_PIXEL_JUMP
            ]
            if locked_candidates:
                return min(
                    locked_candidates,
                    key=lambda b: np.hypot(b['cx'] - self.locked_target_px,
                                           b['cy'] - self.locked_target_py),
                )

        if candidates[0]['depth_m'] > 0:
            return min(candidates, key=lambda b: b['depth_m'])
        return max(candidates, key=lambda b: b['conf'])

    def update_target_lock(self, ball):
        """记录当前锁定球的像素中心。"""
        if not TARGET_LOCK_ENABLED:
            return
        self.locked_target_px = float(ball['cx'])
        self.locked_target_py = float(ball['cy'])
        self.lock_miss_count = 0

    def note_target_missed(self):
        """目标丢失时递增计数，到阈值后解除锁定。"""
        if not TARGET_LOCK_ENABLED:
            return
        self.lock_miss_count += 1
        if self.lock_miss_count > TARGET_LOCK_MAX_MISSES:
            self.locked_target_px = None
            self.locked_target_py = None

    def should_log_detection(self):
        """限制检测日志频率，避免视觉热循环持续刷终端。"""
        now = time.monotonic()
        if now - self.last_detection_log_time < DETECTION_LOG_INTERVAL_SEC:
            return False
        self.last_detection_log_time = now
        return True

    def camera_info_callback(self, msg):
        """自动读取相机的真实焦距和光心参数"""
        if not self.camera_info_received:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(f'📷 成功接收相机内参: fx={self.fx:.2f}, fy={self.fy:.2f}')

    def preprocess_depth(self, depth_img):
        if not ENABLE_DEPTH_PREFILTER: return depth_img
        if DEPTH_FILTER_TYPE == 'median':
            return cv2.medianBlur(depth_img.astype(np.uint16), DEPTH_FILTER_SIZE)
        elif DEPTH_FILTER_TYPE == 'bilateral':
            return cv2.bilateralFilter(depth_img.astype(np.float32), DEPTH_FILTER_SIZE, 50, 50).astype(depth_img.dtype)
        return depth_img

    def pad_depth_to_color_size(self, depth_img, target_h, target_w):
        depth_h, depth_w = depth_img.shape[:2]
        if depth_h == target_h and depth_w == target_w: return depth_img
        pad_top = (target_h - depth_h) // 2
        pad_bottom = target_h - depth_h - pad_top
        pad_left = (target_w - depth_w) // 2
        pad_right = target_w - depth_w - pad_left
        return cv2.copyMakeBorder(depth_img, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=0)

    def refine_with_color(self, roi_bgr, ball_color=BALL_COLOR):
        if roi_bgr is None or roi_bgr.size == 0: return None
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        if ball_color == 'orange': mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        elif ball_color == 'white': mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
        elif ball_color == 'yellow': mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        else: mask = cv2.bitwise_or(cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER), cv2.bitwise_or(cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER), cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        M = cv2.moments(mask)
        if M["m00"] > COLOR_MIN_PIXELS:
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            conf = min(M["m00"] / 5000.0, 1.0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return cx, cy, conf, mask, max(contours, key=cv2.contourArea) if contours else None
        return None

    def refine_with_circle(self, roi_bgr, color_mask=None):
        if roi_bgr is None or roi_bgr.size == 0: return None
        if color_mask is not None: binary = color_mask
        else:
            binary = cv2.dilate(cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0), 50, 150), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=1)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        if area < COLOR_MIN_PIXELS: return None
        perimeter = cv2.arcLength(max_contour, True)
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
        if circularity < CIRCULARITY_THRESHOLD: return None
        (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
        return int(cx), int(cy), int(radius), circularity, max_contour

    def get_smart_depth(self, depth_img, cx, cy, radius=10):
        """硬件红外深度采样"""
        h, w = depth_img.shape[:2]
        compensated_depths = []
        for r_ratio, step in [(0.3, 45), (0.6, 30)]:
            r = max(3, int(radius * r_ratio))
            compensation = BALL_RADIUS_M * np.sqrt(1 - r_ratio ** 2) if ENABLE_SPHERICAL_COMPENSATION else BALL_RADIUS_M
            for angle in range(0, 360, step):
                rad = np.radians(angle)
                sx, sy = int(cx + r * np.cos(rad)), int(cy + r * np.sin(rad))
                if 0 <= sx < w and 0 <= sy < h:
                    d = depth_img[sy, sx]
                    if d > 0: compensated_depths.append(d / DEPTH_SCALE + compensation)

        if len(compensated_depths) >= 3: return np.median(compensated_depths)

        y_min, y_max, x_min, x_max = max(0, cy - 1), min(h, cy + 2), max(0, cx - 1), min(w, cx + 2)
        valid = depth_img[y_min:y_max, x_min:x_max][depth_img[y_min:y_max, x_min:x_max] > 0]
        if len(valid) > 0: return np.median(valid) / DEPTH_SCALE + BALL_RADIUS_M
        return None

    def pixel_to_3d(self, px, py, depth_m):
        """像素坐标转3D物理坐标 (核心矩阵变换)"""
        z = depth_m
        x = (px - self.cx) * z / self.fx
        y = (py - self.cy) * z / self.fy
        return x, y, z

    def letterbox_resize(self, img, size):
        h, w = img.shape[:2]
        scale = min(size / w, size / h)
        new_w, new_h = int(w * scale), int(h * scale)
        pad_w, pad_h = (size - new_w) // 2, (size - new_h) // 2
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[pad_h:pad_h+new_h, pad_w:pad_w+new_w] = cv2.resize(img, (new_w, new_h))
        return canvas, (scale, pad_w, pad_h)

    def letterbox_to_original(self, box, pad_info, orig_w, orig_h):
        scale, pad_w, pad_h = pad_info
        return max(0, min(orig_w, (box[0] - pad_w) / scale)), max(0, min(orig_h, (box[1] - pad_h) / scale)), max(0, min(orig_w, (box[2] - pad_w) / scale)), max(0, min(orig_h, (box[3] - pad_h) / scale))

    def draw_debug_panel(self, img, info):
        overlay = img.copy()
        panel_height = 150 if COMPACT_DEBUG_PANEL else 280
        cv2.rectangle(overlay, (5, 5), (340, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, img, 0.3, 0, img)
        cv2.putText(img, "BALL TRACKER", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        y = 50
        if COMPACT_DEBUG_PANEL:
            quality_parts = ['YOLO']
            if info.get('color', False):
                quality_parts.append('Color')
            if info.get('circle', False):
                quality_parts.append('Circle')
            quality_text = '+'.join(quality_parts) if info.get('yolo', False) else 'No target'
            quality_color = (0, 255, 0) if info.get('yolo', False) else (0, 0, 255)
            cv2.putText(img, f"Target: {quality_text}  N={info.get('balls_found', 0)}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, quality_color, 1)
            y += 22

            if SHOW_FPS_ON_IMAGE:
                cv2.putText(img, f"FPS: {info.get('fps', 0):.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y += 22

            cv2.putText(img, f"Z: {info.get('filtered_z', 0):.3f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y += 22
            cv2.putText(img, f"X: {info.get('x', 0):+.3f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
            y += 22

            dist_mode = info.get('dist_mode', 'Hardware')
            mode_color = (0, 255, 0) if dist_mode == 'Hardware' else (0, 165, 255)
            cv2.putText(img, f"Mode: {dist_mode}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)
            return img

        for name, key, conf_key in [("YOLO", 'yolo', 'yolo_conf'), ("Color", 'color', 'color_conf'), ("Circle", 'circle', 'circularity')]:
            active, conf = info.get(key, False), info.get(conf_key, 0)
            cv2.putText(img, f"{name}: {'ON ' if active else 'OFF'} ({conf:.2f})", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if active else (0, 0, 255), 1)
            y += 22

        cv2.putText(img, f"Balls Found: {info.get('balls_found', 0)}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
        y += 22

        if SHOW_FPS_ON_IMAGE:
            cv2.putText(img, f"FPS: {info.get('fps', 0):.1f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y += 22
        
        # 标明当前测距方式
        dist_mode = info.get('dist_mode', 'Hardware')
        mode_color = (0, 255, 0) if dist_mode == 'Hardware' else (0, 165, 255)
        cv2.putText(img, f"Mode: {dist_mode}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1)
        y += 22

        cv2.putText(img, f"Raw Z:  {info.get('raw_depth', 0):.4f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y += 22
        cv2.putText(img, f"Filtered Z: {info.get('filtered_z', 0):.4f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y += 22
        cv2.putText(img, f"Pixel: ({info.get('px', 0)}, {info.get('py', 0)})", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
        y += 22
        cv2.putText(img, f"X: {info.get('x', 0):+.4f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y += 22
        cv2.putText(img, f"Y: {info.get('y', 0):+.4f} m", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y += 22
        
        if info.get('fail_reason', ''): 
            cv2.putText(img, f"WARN: {info.get('fail_reason', '')}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        return img

    def sync_callback(self, color_msg, depth_msg):
        try:
            current_fps = self.update_fps()
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            if FLIP_COLOR_HORIZONTAL: cv_color = cv2.flip(cv_color, 1)
            if FLIP_DEPTH_HORIZONTAL: cv_depth = cv2.flip(cv_depth, 1)

            cv_depth = self.preprocess_depth(cv_depth)
            color_h, color_w = cv_color.shape[:2]
            depth_h_orig, depth_w_orig = cv_depth.shape[:2]

            if ENABLE_DEPTH_PADDING and (depth_h_orig != color_h or depth_w_orig != color_w):
                cv_depth = self.pad_depth_to_color_size(cv_depth, color_h, color_w)

            info = {'yolo': False, 'yolo_conf': 0, 'color': False, 'color_conf': 0, 'circle': False, 'circularity': 0, 'raw_depth': 0, 'filtered_z': 0, 'px': 0, 'py': 0, 'x': 0, 'y': 0, 'fail_reason': '', 'balls_found': 0, 'dist_mode': 'Hardware', 'fps': current_fps}
            debug_mask = np.zeros((color_h, color_w, 3), dtype=np.uint8)

            img_rgb = cv2.cvtColor(cv_color, cv2.COLOR_BGR2RGB)
            img_resized, pad_info = self.letterbox_resize(img_rgb, YOLO_INPUT_SIZE)
            outputs = self.rknn.inference(inputs=[np.expand_dims(img_resized, 0)])

            all_balls = []

            if outputs is not None:
                boxes, classes, scores = filter_boxes_yolov8(outputs)
                if len(boxes) == 0: info['fail_reason'] = 'YOLO: No detection'

                for box, score in zip(boxes, scores):
                    x1, y1, x2, y2 = self.letterbox_to_original(box, pad_info, color_w, color_h)
                    x1, y1, x2, y2 = int(max(0,x1)), int(max(0,y1)), int(min(color_w,x2)), int(min(color_h,y2))
                    if x2 <= x1 or y2 <= y1: continue

                    bw, bh = x2 - x1, y2 - y1
                    mx, my = int(bw * ROI_MARGIN_RATIO), int(bh * ROI_MARGIN_RATIO)
                    rx1, ry1 = max(0, x1 - mx), max(0, y1 - my)
                    rx2, ry2 = min(color_w, x2 + mx), min(color_h, y2 + my)
                    roi = cv_color[ry1:ry2, rx1:rx2]

                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    method = "YOLO"  # 默认一重状态
                    conf = float(score)
                    radius = (bw + bh) / 4
                    color_contour = None
                    ball_fail_reason = ''
                    depth_m = 0.0
                    dist_mode = 'Hardware' # 默认硬件测距

                    # 进入二重验证：颜色
                    cr = self.refine_with_color(roi, BALL_COLOR)
                    if cr:
                        lcx, lcy, cc, mask, color_contour = cr
                        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                        mask_bgr[:,:,0] = 0
                        mask_bgr[:,:,2] = 0
                        debug_mask[ry1:ry2, rx1:rx2] = mask_bgr

                        method = "YOLO+Color" 
                        cx, cy = rx1 + lcx, ry1 + lcy
                        conf = float(score) * cc

                        # 进入三重验证：圆度
                        circ = self.refine_with_circle(roi, mask)
                        if circ:
                            ccx, ccy, r, cir, _ = circ
                            method = "YOLO+Color+Circle" 
                            cx, cy = rx1 + ccx, ry1 + ccy
                            radius = r  # 像素半径
                            conf = float(score) * cc * cir
                        else:
                            ball_fail_reason = f'Circle: circularity < {CIRCULARITY_THRESHOLD}'
                    else:
                        ball_fail_reason = f'Color: pixels < {COLOR_MIN_PIXELS}'

                    # ========================================================
                    # 🚦 全频段测距核心逻辑 🚦
                    # ========================================================
                    
                    # 1. 尝试硬件红外深度采样
                    sampled_depth = self.get_smart_depth(cv_depth, int(cx), int(cy), int(radius))

                    # 2. 判断是否需要软件像素测距接管 (盲区)
                    if sampled_depth is None or sampled_depth < DEPTH_MIN_M:
                        pixel_diameter = radius * 2.0
                        if pixel_diameter > 0 and self.fx > 0:
                            # 针孔相机模型: Z = (fx * 物理大小) / 像素大小
                            depth_m = (self.fx * BALL_DIAMETER_M) / pixel_diameter
                            dist_mode = 'Software(Pixel)'
                            ball_fail_reason = 'Hardware Blind Zone. Auto-switched to Pixel Math.'
                        else:
                            # 实在没办法了，兜底 0
                            depth_m = 0.0
                            dist_mode = 'Failed'
                            ball_fail_reason = 'Depth & Pixel Math both failed!'
                            
                    # 3. 远距离截断 (防乱跳)
                    elif sampled_depth > DEPTH_MAX_M:
                        depth_m = DEPTH_MAX_M  # 强行截断，输出 2.5m
                        dist_mode = 'Hardware(Truncated)'
                        ball_fail_reason = f'Target too far. Output truncated to MAX {DEPTH_MAX_M}m.'
                        
                    # 4. 正常硬件红外工作区间
                    else:
                        depth_m = sampled_depth
                        dist_mode = 'Hardware'

                    all_balls.append({
                        'cx': cx, 'cy': cy, 'radius': radius,
                        'conf': conf, 'method': method,
                        'depth_m': depth_m, 'dist_mode': dist_mode,
                        'box': (x1, y1, x2, y2), 'roi': (rx1, ry1, rx2, ry2), 
                        'contour': color_contour, 'yolo_conf': float(score), 
                        'color_conf': cr[2] if cr else 0,
                        'circularity': circ[3] if cr and 'circ' in locals() and circ else 0,
                        'warn': ball_fail_reason 
                    })

                info['balls_found'] = len(all_balls)

            if all_balls:
                nearest = self.select_target_ball(all_balls)
                if nearest is None:
                    self.publish_no_target(stamp=color_msg.header.stamp)
                    return

                self.frames_without_detection = 0
                self.update_target_lock(nearest)
                info.update({
                    'yolo': True, 'yolo_conf': nearest['yolo_conf'], 
                    'color': nearest['color_conf'] > 0, 'color_conf': nearest['color_conf'], 
                    'circle': nearest['circularity'] > 0, 'circularity': nearest['circularity'], 
                    'raw_depth': nearest['depth_m'], 'px': nearest['cx'], 'py': nearest['cy'], 
                    'fail_reason': nearest['warn'], 'dist_mode': nearest['dist_mode']
                })

                if nearest['depth_m'] > 0:
                    # 无论是红外测出来的，还是像素算出来的，统统丢给矩阵转换成 3D X, Y
                    rx, ry, rz = self.pixel_to_3d(nearest['cx'], nearest['cy'], nearest['depth_m'])
                else:
                    rx, ry, rz = 0.0, 0.0, 0.0 

                # 送入滤波器平滑 (这里出来的就是稳定可靠的 3D 坐标了)
                fx, fy, fz = self.coord_filter.update(rx, ry, rz)
                info.update({'filtered_z': fz, 'x': fx, 'y': fy})

                self.publish_target(fx, fy, fz)

                # 打印日志标明当前使用的测距模式
                if self.should_log_detection():
                    mode_icon = "📷" if "Software" in nearest['dist_mode'] else "🔴"
                    self.get_logger().info(f"🎯 [{nearest['method']}] X={fx:+.4f}m Y={fy:+.4f}m Z={fz:.4f}m | FPS={current_fps:.1f} | {mode_icon} Mode: {nearest['dist_mode']}")

                if ENABLE_VISUALIZATION:
                    for i, ball in enumerate(all_balls):
                        x1, y1, x2, y2 = ball['box']
                        bcx, bcy = int(ball['cx']), int(ball['cy'])

                        # 🎨 颜色分级标识系统 🎨
                        if ball['method'] == "YOLO+Color+Circle": draw_color = (0, 255, 0)      
                        elif ball['method'] == "YOLO+Color": draw_color = (0, 255, 255)   
                        else: draw_color = (0, 0, 255)                            

                        if i == 0:
                            if "Circle" in ball['method']: cv2.circle(cv_color, (bcx, bcy), int(ball['radius']), draw_color, 2)
                            elif "Color" in ball['method'] and ball['contour'] is not None:
                                contour_shifted = ball['contour'] + np.array([ball['roi'][0], ball['roi'][1]])
                                cv2.drawContours(cv_color, [contour_shifted], -1, draw_color, 2)
                            else: cv2.rectangle(cv_color, (x1, y1), (x2, y2), draw_color, 2)

                            cv2.drawMarker(cv_color, (bcx, bcy), draw_color, cv2.MARKER_CROSS, 15, 2)
                            
                            # 根据测距模式改变文字颜色，方便观察什么时候触发了像素测距
                            text_color = (0, 165, 255) if "Software" in ball['dist_mode'] else draw_color
                            cv2.putText(cv_color, f"Z:{ball['depth_m']:.3f}m", (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                        else:
                            cv2.rectangle(cv_color, (x1, y1), (x2, y2), (128, 128, 128), 1)

            else:
                self.frames_without_detection += 1
                self.note_target_missed()
                if self.frames_without_detection > FRAMES_TO_RESET: self.coord_filter.reset()
                self.publish_no_target(stamp=color_msg.header.stamp)

            if ENABLE_VISUALIZATION:
                cv_color = self.draw_debug_panel(cv_color, info)
                cv2.imshow(WINDOW_NAME, cv_color)

                if ENABLE_DEBUG_WINDOWS:
                    cv2.imshow(DEBUG_WINDOW_COLOR, debug_mask)
                    depth_vis = np.clip(cv_depth.astype(np.float32) / 3000 * 255, 0, 255).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    for i, ball in enumerate(all_balls):
                        cv2.drawMarker(depth_vis, (int(ball['cx']), int(ball['cy'])), (255, 255, 255) if i == 0 else (128, 128, 128), cv2.MARKER_CROSS, 15, 2)
                    cv2.imshow(DEBUG_WINDOW_DEPTH, depth_vis)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'❌ 异常: {e}')
            import traceback; traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    node = NPU3DTracker()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node, 'rknn'): node.rknn.release()
        node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__': main()
