from cmath import pi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float32MultiArray


class FollowerPassthroughDriver(Node):
    def __init__(self):
        super().__init__('follower_passthrough_driver')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/leader/joint_trajectory', 
            10
        )
        
        self.trajectory_pub_sim = self.create_publisher(
                    JointTrajectory, '/arm_controller/joint_trajectory', 10) # 가제보용

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

        self.follower_joint_state_sub = self.create_subscription(
            JointState,
            '/real/joint_states',
            self.follower_joint_state_callback,
            10
        )
        
        self.get_logger().info('Follower Passthrough Driver Node has been started.')

    def leader_callback(self, msg: JointState):

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

        traj.points = [point]

        # [수정] Goal로 감싸지 않고 바로 publish 합니다.
        self.trajectory_pub.publish(traj)
        self.trajectory_pub_sim.publish(traj) # 가제보용
    

    def follower_joint_state_callback(self, msg: JointState):
        if not msg.name or not msg.position:
            return

        updated = False
        for joint_name in self.neck_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_neck_position[joint_name] = msg.position[idx]
                updated = True

        if updated:
            self.has_neck_state = True
        

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
