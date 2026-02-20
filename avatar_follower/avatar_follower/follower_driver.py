from cmath import pi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Int32MultiArray, Float32MultiArray


class FollowerPassthroughDriver(Node):
    def __init__(self):
        super().__init__('follower_passthrough_driver')

        # Action Client for FollowJointTrajectory
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )

        self.leader_sub = self.create_subscription(
            JointState,
            '/leader/joint_states',
            self.leader_callback,
            10
        )

        self.vision_sub = self.create_subscription(
            Float32MultiArray,
            '/avatar/target_angles',
            self.vision_callback,
            10
        )
        
        self.get_logger().info('Follower Passthrough Driver Node has been started.')

    def leader_callback(self, msg: JointState):
        # 액션 서버가 준비되었는지 확인
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('Action server not available yet. Waiting...')
            return

        # JointState -> JointTrajectory 변환
        traj = JointTrajectory()
        traj.joint_names = list(msg.name)

        point = JointTrajectoryPoint()
        for i in range(len(msg.position)):
            point.positions.append(msg.position[i] - pi)  # 180도 오프셋 적용
            if (msg.name[i]=="right_joint_gripper"):
                point.positions[i] = - 2.0 * point.positions[i]  # 그리퍼 각도 반전
            if (msg.name[i]=="left_joint_gripper"):
                point.positions[i] = - 2.0 * point.positions[i]
            
        
        point.time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s (50 Hz)

        traj.points = [point]

        # 액션 목표(Goal) 생성 및 데이터 복사
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # 비동기적으로 목표 전송
        self._action_client.send_goal_async(goal_msg)

    def vision_callback(self, msg: Float32MultiArray):
        # Vision 데이터를 처리하는 로직을 여기에 추가할 수 있습니다.
        # 예시로, Vision 데이터를 로그로 출력합니다.

        
        traj = JointTrajectory()
        traj.joint_names = list(["neck_joint_1", "neck_joint_2"])
        traj.points = [msg.data[2], msg.data[1]]
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # 비동기적으로 목표 전송
        self._action_client.send_goal_async(goal_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = FollowerPassthroughDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
