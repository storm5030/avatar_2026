import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import pyrealsense2 as rs


class realsense_camera:
    is_opened = False
    config = None
    intr = None

    def __init__(self, height=480, width=640, fps=30, use_color=True, use_depth=False):
        self.height = height
        self.width = width
        self.fps = fps
        self.use_depth = use_depth
        self.use_color = use_color

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)

        if self.can_connect():
            pipeline_profile = self.config.resolve(self.pipeline_wrapper)
            device = pipeline_profile.get_device()

            found_rgb = False
            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True

            if not found_rgb:
                self.use_color = False

            if self.use_color:
                self.config.enable_stream(
                    rs.stream.color,
                    self.width,
                    self.height,
                    rs.format.bgr8,
                    self.fps
                )

            if self.use_color:
                self.pipeline.start(self.config)
                self.is_opened = True
                self.intr = (
                    self.pipeline
                    .get_active_profile()
                    .get_stream(rs.stream.color)
                    .as_video_stream_profile()
                    .get_intrinsics()
                )

    def can_connect(self):
        return self.config.can_resolve(self.pipeline_wrapper)

    def isOpened(self):
        return self.is_opened

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(100)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None
            color_image = np.asanyarray(color_frame.get_data())
            return True, color_image
        except:
            return False, None

    def release(self):
        if self.is_opened:
            self.pipeline.stop()


class RealSenseRGBPublisher(Node):

    def __init__(self):
        super().__init__('realsense_rgb_publisher')

        self.publisher = self.create_publisher(
            Image,
            '/realsense/color/image_raw',
            10
        )

        self.bridge = CvBridge()

        self.cam = realsense_camera(
            width=640,
            height=480,
            fps=30,
            use_color=True,
            use_depth=False
        )

        if not self.cam.isOpened():
            self.get_logger().error('RealSense camera not opened')
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('RealSense RGB publisher started')

    def timer_callback(self):
        ret, frame = self.cam.read()
        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'realsense_color_frame'

        self.publisher.publish(msg)

        # 로컬 화면 출력
        cv2.imshow('RealSense RGB Publisher View', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBPublisher()
    rclpy.spin(node)
    node.cam.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
