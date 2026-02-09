import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64MultiArray

class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        topic_name = '/world/default/model/follower/link/left_link_gripper_1/sensor/left_gripper_bumper/contact'

        # 1. 범퍼 센서 구독 (충돌 감지)
        self.subscription = self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link_gripper_1/sensor/left_gripper_bumper/contact',
            self.collision_callback,
            10)
            
        # 2. 모터 컨트롤러 퍼블리셔 (멈춤 명령용)
        # (사용하시는 컨트롤러 토픽 이름으로 바꾸세요!)
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands', 
            10)

        self.is_stopped = False

    def collision_callback(self, msg):
        # msg.states 리스트가 비어있지 않으면 충돌 중이라는 뜻!
        if len(msg.states) > 0:
            if not self.is_stopped:
                self.get_logger().warn('충돌 발생! 로봇 정지!')
                self.stop_robot()
                self.is_stopped = True
        else:
            self.is_stopped = False

    def stop_robot(self):
        stop_msg = Float64MultiArray()
        # 두 팔 로봇이라면 0.0을 더 많이 넣어야겠죠?
        # 예: [왼쪽1, ..., 왼쪽6, 오른쪽1, ..., 오른쪽6]
        stop_msg.data = [0.0] * 16  # 관절이 16개라면 이렇게!
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()