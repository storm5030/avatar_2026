import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        # Contact sensor subscriptions for all links
        # Base link
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/base_link/sensor/base_link_bumper/contact',
            self.collision_callback,
            10)
        
        # Neck links
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/neck_link1_1/sensor/neck_link1_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/neck_link2_1/sensor/neck_link2_1_bumper/contact',
            self.collision_callback,
            10)
        
        # Left arm links
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link1_1/sensor/left_link1_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link2_1/sensor/left_link2_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link3_1/sensor/left_link3_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link4_1/sensor/left_link4_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link5_1/sensor/left_link5_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link6_1/sensor/left_link6_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/left_link_gripper_1/sensor/left_gripper_bumper/contact',
            self.collision_callback,
            10)
        
        # Right arm links
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link1_1/sensor/right_link1_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link2_1/sensor/right_link2_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link3_1/sensor/right_link3_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link4_1/sensor/right_link4_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link5_1/sensor/right_link5_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link6_1/sensor/right_link6_1_bumper/contact',
            self.collision_callback,
            10)
        
        self.create_subscription(
            Contacts,
            '/world/default/model/follower/link/right_link_gripper_1/sensor/right_gripper_bumper/contact',
            self.collision_callback,
            10)
            
        # Motor controller publisher (stop command)
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
                self.get_logger().warn('🚨 충돌 감지! 현재 위치에서 긴급 정지합니다!')
                self.is_stopped = True
                self.stop_robot()
        else:
            # 🌟 3. 논리 수정: 닿은 것이 아무것도 없을 때 상태를 풀어줌
            if self.is_stopped:
                self.get_logger().info('충돌 해제. 다시 움직일 수 있습니다.')
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
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()