import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
        
        self.sub_right = self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link_gripper_1/sensor/right_gripper_bumper/contact', # 오른손 긴 주소 (확인 필요!)
            self.collision_callback,
            10)
            
        # 2. 모터 컨트롤러 퍼블리셔 (멈춤 명령용)
        # (사용하시는 컨트롤러 토픽 이름으로 바꾸세요!)
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory', 
            10)
        
        self.is_stopped = False

        self.joint_names = [
            'right_joint1',
            'right_joint2', 
            'right_joint3',
            'right_joint4',
            'right_joint5',
            'right_joint6',
            'right_joint_gripper',
            'left_joint1',
            'left_joint2',
            'left_joint3',
            'left_joint4',
            'left_joint5',
            'left_joint6',
            'left_joint_gripper',
            'neck_joint1',
            'neck_joint2'
        ]

    def collision_callback(self, msg):
        # msg.contacts 리스트가 비어있지 않으면 충돌 중이라는 뜻!
        if len(msg.contacts) > 0:
            if not self.is_stopped:
                self.get_logger().warn('충돌 발생! 로봇 정지!')
                self.stop_robot()
                self.is_stopped = True
        else:
            self.is_stopped = False

    def stop_robot(self):
        stop_msg = JointTrajectory()
        stop_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names) # 현재 위치를 유지하려면 별도 로직 필요
        point.velocities = [0.0] * len(self.joint_names) # 속도를 0으로!
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000 # 0.1초 안에 멈춰라

        stop_msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()