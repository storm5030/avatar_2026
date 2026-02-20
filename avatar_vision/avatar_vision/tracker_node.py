import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # 압축 이미지 메시지 타입
from cv_bridge import CvBridge # ROS-OpenCV 변환 도구

import cv2
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32MultiArray

class YoloDeepSortSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_deepsort_node")

        # ===== 설정 =====
        self.conf_thres = 0.2
        
        self.target_classes = [67]  # cell phone class id
        # 참고용: 주요 COCO class id 일부
        # 0 → person,39 → bottle, 41 → cup,  67 → cell phone

        self.bridge = CvBridge()

        # ===== 모델 로드 =====
        self.model = YOLO("yolov8n.pt")
        self.tracker = DeepSort(
            max_age=80,
            n_init=3,
            max_iou_distance=0.95
        )

        # ===== 구독자 설정 =====
        # RealSense의 압축 이미지 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/realsense/color/image_raw/compressed',
            self.image_callback,
            10 # 큐 사이즈
        )

        # ===== 퍼블리셔 (vision_msgs) =====
        # 트래킹/디텍션 결과를 Detection2DArray로 퍼블리시
        self.det_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            10
        )
        
        # track_id를 별도 토픽으로 같이 퍼블리시 (인덱스 대응)
        # 같은 프레임에서 Detection2DArray의 detections[i]와 track_ids.data[i]를 매칭해서 쓰는 방식
        self.id_pub = self.create_publisher(
            Int32MultiArray,
            '/vision/track_ids',
            10
        )
        
        self.get_logger().info("YOLOv8 + DeepSORT 구독 노드 시작됨 (Topic: /realsense/color/image_raw/compressed)")

    def image_callback(self, msg):
        # 1) 압축 이미지 -> cv2
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # 2) YOLO 추론
        results = self.model.predict(
            frame,
            conf=self.conf_thres,
            classes=self.target_classes,
            imgsz=416,
            verbose=False
        )[0]

        detections = []
        for box in results.boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            if self.target_classes is not None and cls_id not in self.target_classes:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w, h = x2 - x1, y2 - y1
            detections.append(([x1, y1, w, h], conf, cls_id))

        # 3) DeepSORT 업데이트
        tracks = self.tracker.update_tracks(detections, frame=frame)

        # 4) publish msg 준비
        det_arr = Detection2DArray()
        det_arr.header = msg.header

        track_ids_msg = Int32MultiArray()
        track_ids_msg.data = []

        H, W = frame.shape[:2]
        cx_img, cy_img = W * 0.5, H * 0.5

        # 5) confirmed 트랙 후보 수집 + 중앙거리 계산
        candidates = []
        for t in tracks:
            if not t.is_confirmed():
                continue

            x1, y1, x2, y2 = map(float, t.to_ltrb())

            # clamp
            x1 = max(0.0, min(x1, W - 1.0))
            x2 = max(0.0, min(x2, W - 1.0))
            y1 = max(0.0, min(y1, H - 1.0))
            y2 = max(0.0, min(y2, H - 1.0))

            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            if w <= 1.0 or h <= 1.0:
                continue

            cx = x1 + w * 0.5
            cy = y1 + h * 0.5
            dist2 = (cx - cx_img) ** 2 + (cy - cy_img) ** 2

            candidates.append((dist2, t, x1, y1, x2, y2, w, h))

        # 후보 없으면 빈 publish + 화면만
        if not candidates:
            self.det_pub.publish(det_arr)
            self.id_pub.publish(track_ids_msg)
            cv2.imshow("YOLOv8 + DeepSORT (RealSense)", frame)
            cv2.waitKey(1)
            return

        # 6) 중앙에 가장 가까운 1개 선택
        candidates.sort(key=lambda x: x[0])
        _, best_t, x1, y1, x2, y2, w, h = candidates[0]

        track_id = int(best_t.track_id)
        label = best_t.get_det_class()

        # conf 추출
        conf = 0.0
        raw_conf = getattr(best_t, "det_conf", None)
        if raw_conf is not None:
            conf = float(raw_conf)
        else:
            last_det = getattr(best_t, "last_detection", None)
            if last_det is not None:
                maybe = getattr(last_det, "confidence", None)
                if maybe is not None:
                    conf = float(maybe)

        # 7) Detection2D 1개만 채우기
        det = Detection2D()
        det.header = msg.header

        bbox = BoundingBox2D()
        bbox.center.position.x = float(x1 + w / 2.0)
        bbox.center.position.y = float(y1 + h / 2.0)
        bbox.center.theta = 0.0
        bbox.size_x = float(w)
        bbox.size_y = float(h)
        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(label)
        hyp.hypothesis.score = conf if conf > 0.0 else 0.0
        det.results.append(hyp)

        det_arr.detections.append(det)
        track_ids_msg.data.append(track_id)

        # 8) 시각화 (1개만)
        xi1, yi1, xi2, yi2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(frame, (xi1, yi1), (xi2, yi2), (0, 255, 0), 2)
        cv2.putText(frame, f"BEST ID {track_id} | {label}",
                    (xi1, yi1 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, (int(cx_img), int(cy_img)), 5, (255, 255, 255), -1)

        # 9) publish + show
        self.det_pub.publish(det_arr)
        self.id_pub.publish(track_ids_msg)

        cv2.imshow("YOLOv8 + DeepSORT (RealSense)", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloDeepSortSubscriber()
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