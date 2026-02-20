# ros2 run avatar_vision angle_pub --ros-args -p target_id:=3
import rclpy
from rclpy.node import Node
import math
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class AnglePublisher(Node):
    def __init__(self):
        super().__init__("angle_publisher")

        # 1. 파라미터 설정
        self.declare_parameter("target_id", -1)
        self.target_id = int(self.get_parameter("target_id").value)

        # 2. Realsense Intrinsic 파라미터 (D435/D455 기본값 예시)
        # 실제 정확한 값은 'rs-enumerate-devices -c' 또는 카메라 토픽(/camera/camera_info)에서 확인 필요
        self.fx = 615.1111450195312  # Focal length x
        self.fy = 615.2798461914062  # Focal length y
        self.ppx = 318.12139892578125 # Principal point x (Center)
        self.ppy = 250.6387481689453 # Principal point y (Center)
        
        self.latest_detections = None
        self.latest_track_ids = None

        # ===== 구독 =====
        self.det_sub = self.create_subscription(
            Detection2DArray,
            "/vision/detections",
            self.det_callback,
            10
        )
        self.id_sub = self.create_subscription(
            Int32MultiArray,
            "/vision/track_ids",
            self.id_callback,
            10
        )

        # ===== 퍼블리시 =====
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            "/avatar/target_angles",
            10
        )

        self.get_logger().info(f"joint change publisher 시작됨 (target_id={self.target_id})")

    def calculate_angles(self, cx, cy):
        """픽셀 좌표를 카메라 기준 라디안 각도로 변환"""
        # 정중앙을 0,0으로 만들기 위해 주점(ppx, ppy)을 뺍니다.
        yaw = math.atan2((cx - self.ppx), self.fx)
        pitch = -math.atan2((cy - self.ppy), self.fy)
        
        return yaw, pitch

    def id_callback(self, msg: Int32MultiArray):
        self.latest_track_ids = msg
        self.try_publish()

    def det_callback(self, msg: Detection2DArray):
        self.latest_detections = msg
        self.try_publish()

    def try_publish(self):
        if self.latest_detections is None or self.latest_track_ids is None:
            return

        dets = self.latest_detections.detections
        ids = self.latest_track_ids.data

        if len(dets) != len(ids):
            self.get_logger().warn(
                f"길이 불일치: detections={len(dets)} vs track_ids={len(ids)} (이번 프레임 스킵)"
            )
            return

        out = Float32MultiArray()
        out.data = []

        for det, track_id in zip(dets, ids):
            track_id = int(track_id)

            # ===== 여기서 필터링 =====
            if self.target_id != -1 and track_id != self.target_id:
                continue

            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)

            yaw, pitch = self.calculate_angles(cx, cy)

            out.data.extend([float(track_id), float(yaw), float(pitch)])

        # target_id 지정했는데 못 찾으면 아예 publish 안 함
        if len(out.data) == 0:
            # 다음 프레임용 초기화만 하고 종료
            self.latest_detections = None
            self.latest_track_ids = None
            return

        self.angle_pub.publish(out)

def main():
    rclpy.init()
    node = AnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()