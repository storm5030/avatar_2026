import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Int32MultiArray

import mediapipe as mp


class MediaPipeHandTracker(Node):
    def __init__(self):
        super().__init__("mp_hand_tracker_node")

        # ===== 파라미터 =====
        # 0: 첫 번째 손(화면에서 더 잘 잡히는 손)만
        self.declare_parameter("publish_one_hand", True)
        self.publish_one_hand = bool(self.get_parameter("publish_one_hand").value)

        # 손 선택 기준: "center"면 화면 중앙에 가까운 손 1개 선택
        self.declare_parameter("select_mode", "center")  # "center" or "first"
        self.select_mode = str(self.get_parameter("select_mode").value)

        # ===== ROS =====
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            "/realsense/color/image_raw/compressed",
            self.image_callback,
            10
        )

        self.det_pub = self.create_publisher(
            Detection2DArray,
            "/vision/detections",
            10
        )

        self.id_pub = self.create_publisher(
            Int32MultiArray,
            "/vision/track_ids",
            10
        )

        # ===== MediaPipe Hands =====
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,          # 0 더 빠름 / 1 기본 / 2 더 무거움
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("MediaPipe Hands tracker started (sub: /realsense/color/image_raw/compressed)")

    def image_callback(self, msg):
        # 1) CompressedImage -> cv2 BGR
        try:
            frame_bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image convert failed: {e}")
            return

        H, W = frame_bgr.shape[:2]
        cx_img, cy_img = W * 0.5, H * 0.5

        # 2) MediaPipe는 RGB 입력
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # 3) 손 추론
        results = self.hands.process(frame_rgb)

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        track_ids_msg = Int32MultiArray()
        track_ids_msg.data = []

        if not results.multi_hand_landmarks:
            # 손 없음 -> 빈 publish
            self.det_pub.publish(det_arr)
            self.id_pub.publish(track_ids_msg)

            cv2.imshow("MediaPipe Hands", frame_bgr)
            cv2.waitKey(1)
            return

        # 4) 손 후보들에서 bbox 계산
        candidates = []
        for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
            xs = []
            ys = []
            for lm in hand_landmarks.landmark:
                xs.append(lm.x * W)
                ys.append(lm.y * H)

            x1 = float(max(0.0, min(xs)))
            x2 = float(min(W - 1.0, max(xs)))
            y1 = float(max(0.0, min(ys)))
            y2 = float(min(H - 1.0, max(ys)))

            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            if w <= 2.0 or h <= 2.0:
                continue

            cx = x1 + w * 0.5
            cy = y1 + h * 0.5
            dist2 = (cx - cx_img) ** 2 + (cy - cy_img) ** 2

            candidates.append((dist2, hand_idx, x1, y1, x2, y2, w, h, cx, cy, hand_landmarks))

        if not candidates:
            self.det_pub.publish(det_arr)
            self.id_pub.publish(track_ids_msg)

            cv2.imshow("MediaPipe Hands", frame_bgr)
            cv2.waitKey(1)
            return

        # 5) 손 1개 선택 로직
        if self.select_mode == "center":
            candidates.sort(key=lambda x: x[0])  # 중앙 가까운 손
            chosen = candidates[0]
        else:
            # first: 그냥 첫 번째 손
            chosen = candidates[0]

        _, hand_idx, x1, y1, x2, y2, w, h, cx, cy, hand_landmarks = chosen

        # 6) Detection2D 1개만 만들어서 publish
        det = Detection2D()
        det.header = msg.header

        bbox = BoundingBox2D()
        bbox.center.position.x = float(cx)
        bbox.center.position.y = float(cy)
        bbox.center.theta = 0.0
        bbox.size_x = float(w)
        bbox.size_y = float(h)
        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "hand"
        hyp.hypothesis.score = 1.0  # MediaPipe는 YOLO처럼 conf를 바로 안 줘서 1.0 고정
        det.results.append(hyp)

        det_arr.detections.append(det)

        # track id는 고정 1로 내보냄 (너 목 제어는 ID가 사실상 의미 없을 거라 이게 제일 안정적)
        track_ids_msg.data.append(1)

        self.det_pub.publish(det_arr)
        self.id_pub.publish(track_ids_msg)

        # 7) 시각화
        xi1, yi1, xi2, yi2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(frame_bgr, (xi1, yi1), (xi2, yi2), (0, 255, 0), 2)
        cv2.circle(frame_bgr, (int(cx_img), int(cy_img)), 5, (255, 255, 255), -1)
        cv2.circle(frame_bgr, (int(cx), int(cy)), 6, (0, 255, 0), -1)
        cv2.putText(frame_bgr, "HAND", (xi1, yi1 - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("MediaPipe Hands", frame_bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()