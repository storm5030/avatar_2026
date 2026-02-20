import rclpy
from rclpy.node import Node

import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort


class YoloDeepSortNode(Node):
    def __init__(self):
        super().__init__("yolo_deepsort_node")

        # ===== 설정 =====
        self.source = 0
        self.conf_thres = 0.35  # ✅ 누락되어 있던 confidence threshold 추가
        self.target_classes = None  # 예: [0] 이면 사람만

        # ===== 모델 로드 =====
        self.model = YOLO("yolov8n.pt")
        self.tracker = DeepSort(
            max_age=30,
            n_init=1,
            max_iou_distance=0.7
        )

        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패")
            return

        # 30 FPS 정도
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("YOLOv8 + DeepSORT 노드 시작됨")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 수신 실패")
            return

        # ✅ YOLO 추론
        results = self.model.predict(
            frame, conf=self.conf_thres, verbose=False
        )[0]

        # ✅ DeepSORT 입력 detections 만들기
        # deep_sort_realtime 형식: ( [x, y, w, h], confidence, class )
        detections = []
        for box in results.boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])  # ✅ 여기 뒤에 끼어든 파일경로 텍스트 제거됨

            # 클래스 필터링 옵션
            if self.target_classes is not None and cls_id not in self.target_classes:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
            w = x2 - x1
            h = y2 - y1

            detections.append(([x1, y1, w, h], conf, cls_id))

        # ✅ 트래커 업데이트
        tracks = self.tracker.update_tracks(detections, frame=frame)

        # ✅ 시각화
        for t in tracks:
            if not t.is_confirmed():
                continue

            track_id = t.track_id
            x1, y1, x2, y2 = map(int, t.to_ltrb())
            label = t.get_det_class()  # 우리가 cls_id 넣었으니 보통 숫자로 나옴

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"ID {track_id} | CLS {label}",
                (x1, max(0, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        cv2.imshow("YOLOv8 + DeepSORT (ROS2)", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDeepSortNode()

    try:
        rclpy.spin(node)
    finally:
        # ✅ 종료 처리 안전하게
        if hasattr(node, "cap") and node.cap is not None:
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()