"""Record RGB-D camera video and extracted training frames from ROS image topics."""

import json
import os
from datetime import datetime
from typing import Optional

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_OUTPUT_DIR = '/userdata/yolo_tracking/dataset_capture'
DEFAULT_COLOR_TOPIC = '/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/depth/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/color/camera_info'

# Runtime debug switch. Keep off by default on RK3588 to avoid extra GUI load.
DEFAULT_SHOW_PREVIEW = False


class RgbdDatasetRecorder(Node):
    """Save color video plus extracted RGB/depth frames for YOLO dataset collection."""

    def __init__(self) -> None:
        super().__init__('rgbd_dataset_recorder')
        self.bridge = CvBridge()

        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('session_name', '')
        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('record_video', True)
        self.declare_parameter('extract_frames', True)
        self.declare_parameter('save_depth_frames', False)
        self.declare_parameter('frame_stride', 10)
        self.declare_parameter('video_fps', 15.0)
        self.declare_parameter('image_ext', 'jpg')
        self.declare_parameter('jpeg_quality', 95)
        self.declare_parameter('flip_color_horizontal', False)
        self.declare_parameter('flip_depth_horizontal', True)
        self.declare_parameter('max_frames', 0)
        self.declare_parameter('show_preview', DEFAULT_SHOW_PREVIEW)
        self.declare_parameter('min_reliable_depth_m', 0.2)
        self.declare_parameter('max_reliable_depth_m', 2.5)

        self.output_dir = str(self.get_parameter('output_dir').value)
        session_name = str(self.get_parameter('session_name').value)
        if not session_name:
            session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_name = session_name

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        self.sync_slop = float(self.get_parameter('sync_slop').value)
        self.record_video = bool(self.get_parameter('record_video').value)
        self.extract_frames = bool(self.get_parameter('extract_frames').value)
        self.save_depth_frames = bool(self.get_parameter('save_depth_frames').value)
        self.frame_stride = max(1, int(self.get_parameter('frame_stride').value))
        self.video_fps = float(self.get_parameter('video_fps').value)
        self.image_ext = str(self.get_parameter('image_ext').value).lstrip('.').lower()
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.flip_color_horizontal = bool(self.get_parameter('flip_color_horizontal').value)
        self.flip_depth_horizontal = bool(self.get_parameter('flip_depth_horizontal').value)
        self.max_frames = int(self.get_parameter('max_frames').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.min_reliable_depth_m = float(
            self.get_parameter('min_reliable_depth_m').value
        )
        self.max_reliable_depth_m = float(
            self.get_parameter('max_reliable_depth_m').value
        )

        self.session_dir = os.path.join(self.output_dir, self.session_name)
        self.images_dir = os.path.join(self.session_dir, 'images')
        self.depth_dir = os.path.join(self.session_dir, 'depth')
        self.video_dir = os.path.join(self.session_dir, 'video')
        os.makedirs(self.images_dir, exist_ok=True)
        if self.save_depth_frames:
            os.makedirs(self.depth_dir, exist_ok=True)
        if self.record_video:
            os.makedirs(self.video_dir, exist_ok=True)

        self.frame_count = 0
        self.saved_image_count = 0
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.camera_info_written = False

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            self.sync_queue_size,
            self.sync_slop,
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            'RGB-D dataset recorder started | session=%s color=%s depth=%s '
            'stride=%d output=%s'
            % (
                self.session_name,
                self.color_topic,
                self.depth_topic,
                self.frame_stride,
                self.session_dir,
            )
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Write camera intrinsics once for later distance review."""
        if self.camera_info_written:
            return
        payload = {
            'frame_id': msg.header.frame_id,
            'width': msg.width,
            'height': msg.height,
            'k': list(msg.k),
            'd': list(msg.d),
            'r': list(msg.r),
            'p': list(msg.p),
            'distortion_model': msg.distortion_model,
            'reliable_depth_range_m': [
                self.min_reliable_depth_m,
                self.max_reliable_depth_m,
            ],
        }
        info_path = os.path.join(self.session_dir, 'camera_info.json')
        with open(info_path, 'w', encoding='utf-8') as file_obj:
            json.dump(payload, file_obj, indent=2)
        self.camera_info_written = True
        self.get_logger().info(f'Camera info saved: {info_path}')

    def ensure_video_writer(self, color_img: np.ndarray) -> None:
        """Create the video writer after the first image reveals frame size."""
        if not self.record_video or self.video_writer is not None:
            return
        height, width = color_img.shape[:2]
        video_path = os.path.join(self.video_dir, f'{self.session_name}.avi')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(
            video_path,
            fourcc,
            self.video_fps,
            (width, height),
        )
        if not self.video_writer.isOpened():
            self.get_logger().error(f'Failed to open video writer: {video_path}')
            self.video_writer = None
            return
        self.get_logger().info(f'Color video recording to: {video_path}')

    def save_extracted_frame(
        self,
        color_img: np.ndarray,
        depth_img: np.ndarray,
        stamp_text: str,
    ) -> None:
        """Save one RGB training frame and optionally the matching depth frame."""
        if not self.extract_frames:
            return
        image_name = f'{self.session_name}_{self.saved_image_count:06d}.{self.image_ext}'
        image_path = os.path.join(self.images_dir, image_name)
        if self.image_ext in ('jpg', 'jpeg'):
            cv2.imwrite(image_path, color_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        else:
            cv2.imwrite(image_path, color_img)

        if self.save_depth_frames:
            depth_name = f'{self.session_name}_{self.saved_image_count:06d}.png'
            depth_path = os.path.join(self.depth_dir, depth_name)
            if depth_img.dtype != np.uint16:
                depth_to_save = depth_img.astype(np.uint16)
            else:
                depth_to_save = depth_img
            cv2.imwrite(depth_path, depth_to_save)

        meta_path = os.path.join(self.session_dir, 'frames.csv')
        write_header = not os.path.exists(meta_path)
        with open(meta_path, 'a', encoding='utf-8') as file_obj:
            if write_header:
                file_obj.write('index,image,depth,stamp\n')
            depth_name = f'{self.session_name}_{self.saved_image_count:06d}.png'
            file_obj.write(
                f'{self.saved_image_count},{image_name},'
                f'{depth_name if self.save_depth_frames else ""},{stamp_text}\n'
            )
        self.saved_image_count += 1

    def sync_callback(self, color_msg: Image, depth_msg: Image) -> None:
        """Record synchronized color/depth frames."""
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if self.flip_color_horizontal:
                color_img = cv2.flip(color_img, 1)
            if self.flip_depth_horizontal:
                depth_img = cv2.flip(depth_img, 1)

            self.ensure_video_writer(color_img)
            if self.video_writer is not None:
                self.video_writer.write(color_img)

            stamp = color_msg.header.stamp
            stamp_text = f'{stamp.sec}.{stamp.nanosec:09d}'
            if self.frame_count % self.frame_stride == 0:
                self.save_extracted_frame(color_img, depth_img, stamp_text)
                self.get_logger().info(
                    'Saved frame %d from raw frame %d'
                    % (self.saved_image_count, self.frame_count),
                    throttle_duration_sec=1.0,
                )

            if self.show_preview:
                preview = color_img.copy()
                cv2.putText(
                    preview,
                    f'session={self.session_name} saved={self.saved_image_count}',
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow('RGB-D Dataset Recorder', preview)
                cv2.waitKey(1)

            self.frame_count += 1
            if self.max_frames > 0 and self.frame_count >= self.max_frames:
                self.get_logger().info('Reached max_frames, shutting down recorder')
                rclpy.shutdown()

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'RGB-D recording callback failed: {exc}')

    def destroy_node(self) -> bool:
        """Close video writer and preview window."""
        if self.video_writer is not None:
            self.video_writer.release()
        if self.show_preview:
            cv2.destroyAllWindows()
        self.get_logger().info(
            'RGB-D recording stopped | frames=%d saved_images=%d output=%s'
            % (self.frame_count, self.saved_image_count, self.session_dir)
        )
        return super().destroy_node()


def main(args=None) -> None:
    """ROS2 entrypoint."""
    rclpy.init(args=args)
    node = RgbdDatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
