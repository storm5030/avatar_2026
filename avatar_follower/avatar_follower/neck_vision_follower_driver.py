from cmath import pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32MultiArray


class NeckVisionFollowerDriver(Node):
    def __init__(self):
        super().__init__('neck_vision_follower_driver')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/leader/joint_trajectory', 
            10
        )
        
        self.trajectory_pub_sim = self.create_publisher(
                    JointTrajectory, '/arm_controller/joint_trajectory', 10) # 가제보용

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


        self.neck_joint_names = ["neck_joint1", "neck_joint2"]
        self.current_neck_position = {
            "neck_joint1": 0.0,
            "neck_joint2": 0.0,
        }
        self.has_neck_state = False
        self.warned_no_neck_state = False

        # 오차가 이 값보다 작으면 목을 움직이지 않는다 (deadband).
        self.pitch_deadband = 0.1
        self.yaw_deadband = 0.1
        
        self.get_logger().info('Neck Vision Follower Driver Node has been started.')

    def vision_callback(self, msg: Float32MultiArray):   
        if len(msg.data) < 3:
            self.get_logger().warn("target_angles must contain at least 3 values.")
            return

        if not self.has_neck_state:
            if not self.warned_no_neck_state:
                self.get_logger().warn("Neck joint state not received yet. Skipping vision command.")
                self.warned_no_neck_state = True
            return

        traj = JointTrajectory()
        traj.joint_names = list(self.neck_joint_names)
        traj.points = [JointTrajectoryPoint()]

        pitch_error = -msg.data[2]
        yaw_error = -msg.data[1]

        if abs(pitch_error) < self.pitch_deadband:
            pitch_error = 0.0
        if abs(yaw_error) < self.yaw_deadband:
            yaw_error = 0.0

        # 입력 각도를 절대 목표가 아닌 delta로 적용한다.
        # 너무 큰 점프를 막기 위해 callback 당 이동량을 제한한다.
        max_pitch_delta = 0.05
        max_yaw_delta = 0.05

        pitch_delta = max(-max_pitch_delta, min(max_pitch_delta, pitch_error))
        yaw_delta = max(-max_yaw_delta, min(max_yaw_delta, yaw_error))

        current_pitch = self.current_neck_position["neck_joint1"]
        current_yaw = self.current_neck_position["neck_joint2"]

        target_pitch = current_pitch + pitch_delta
        target_yaw = current_yaw + yaw_delta

        traj.points[0].positions = [target_pitch, target_yaw]
        traj.points[0].time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s (50 Hz)

        # Goal(action) 대신 JointTrajectory를 토픽으로 직접 전송
        self.trajectory_pub.publish(traj)
        self.trajectory_pub_sim.publish(traj)
        self.get_logger().info(
            f"Vision delta cmd: raw(pitch={pitch_error:.4f}, yaw={yaw_error:.4f}) "
            f"-> target(pitch={target_pitch:.4f}, yaw={target_yaw:.4f})"
        )

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
    node = NeckVisionFollowerDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
