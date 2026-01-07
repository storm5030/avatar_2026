import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RealSenseRGBSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_rgb_subscriber')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/realsense/color/image_raw',   # publisher와 동일
            self.callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /realsense/color/image_raw'
        )

    def callback(self, msg):
        # ROS Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 화면 출력
        cv2.imshow('RealSense RGB (subscriber)', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
