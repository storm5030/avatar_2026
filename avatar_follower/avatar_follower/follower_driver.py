import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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

        self.MODE_GRIPPER = 0
        self.MODE_NECK_YAW = 1
        self.MODE_NECK_PITCH = 2
        self.control_mode = self.MODE_GRIPPER

        self.gripper_names = ["right_joint_gripper", "left_joint_gripper"]
        self.last_gripper_targets = {name: None for name in self.gripper_names}
        self.gripper_pressed = {name: False for name in self.gripper_names}
        self.click_press_threshold = math.radians(10.0)
        self.click_release_threshold = math.radians(5.0)
        self.double_click_window = 1.0 # 더블클릭 인식 초
        self.last_click_time = None
        self.click_count = 0

        self.gripper_activation_threshold = math.radians(10.0)
        self.neck_yaw_speed = 0.1    # rad/s
        self.neck_pitch_speed = 0.2  # rad/s
        self.last_control_time = None
        self.last_neck_speed_log_time = None
        self.neck_speed_log_interval = 0.2
        
        self.get_logger().info('Follower Passthrough Driver Node has been started.')

    def leader_callback(self, msg: JointState):
        self._init_neck_position_from_leader(msg)

        right_abs = self._extract_gripper_abs(msg, "right_joint_gripper")
        left_abs = self._extract_gripper_abs(msg, "left_joint_gripper")
        self._update_mode_from_gripper_click(right_abs, left_abs)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_control_time is None:
            dt = 0.02
        else:
            dt = max(0.001, min(0.1, now_sec - self.last_control_time))
        self.last_control_time = now_sec

        # JointState -> JointTrajectory 변환
        traj = JointTrajectory()
        traj.joint_names = list(msg.name)

        point = JointTrajectoryPoint()
        for i in range(len(msg.position)):
            joint_name = msg.name[i]
            target = msg.position[i] - math.pi  # 180도 오프셋 적용

            if joint_name in self.gripper_names:
                transformed = -2.0 * target  # 그리퍼 각도 반전
                if self.control_mode == self.MODE_GRIPPER:
                    target = transformed
                    self.last_gripper_targets[joint_name] = transformed
                else:
                    hold = self.last_gripper_targets[joint_name]
                    if hold is None:
                        hold = transformed
                        self.last_gripper_targets[joint_name] = hold
                    target = hold

            point.positions.append(target)

        if self.control_mode != self.MODE_GRIPPER and self.has_neck_state:
            right_cmd = self._abs_to_cmd(right_abs)
            left_cmd = self._abs_to_cmd(left_abs)
            if self.control_mode == self.MODE_NECK_YAW:
                neck_joint = "neck_joint2"
                neck_speed_cmd = (right_cmd - left_cmd) * self.neck_yaw_speed
                neck_delta = neck_speed_cmd * dt
            else:
                neck_joint = "neck_joint1"
                neck_speed_cmd = (right_cmd - left_cmd) * self.neck_pitch_speed
                neck_delta = neck_speed_cmd * dt

            self._log_neck_speed(now_sec, neck_joint, neck_speed_cmd)

            if abs(neck_delta) > 0.0:
                target = self.current_neck_position[neck_joint] + neck_delta
                traj.joint_names.append(neck_joint)
                point.positions.append(target)

        point.time_from_start = Duration(sec=0, nanosec=20_000_000)  # 0.02s (50 Hz)

        traj.points = [point]

        # [수정] Goal로 감싸지 않고 바로 publish 합니다.
        self.trajectory_pub.publish(traj)
        self.trajectory_pub_sim.publish(traj) # 가제보용


    #목 제어를 위한 헬퍼 함수들
    def _abs_to_cmd(self, angle):
        if angle is None or abs(angle) < self.gripper_activation_threshold:
            return 0.0
        return 1.0

    def _extract_gripper_abs(self, msg: JointState, joint_name: str):
        if joint_name not in msg.name:
            return None
        idx = msg.name.index(joint_name)
        return msg.position[idx] - math.pi

    def _init_neck_position_from_leader(self, msg: JointState):
        if self.has_neck_state:
            return

        initialized = False
        for joint_name in self.neck_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_neck_position[joint_name] = msg.position[idx] - math.pi
                initialized = True

        if initialized:
            self.has_neck_state = True
            self.get_logger().info("Initialized neck position from /leader/joint_states")

    def _log_neck_speed(self, now_sec, neck_joint, neck_speed_cmd):
        if self.last_neck_speed_log_time is not None and (now_sec - self.last_neck_speed_log_time) < self.neck_speed_log_interval:
            return
        self.last_neck_speed_log_time = now_sec
        self.get_logger().info(
            f"Neck cmd speed | {neck_joint}: {neck_speed_cmd:+.3f} rad/s ({math.degrees(neck_speed_cmd):+.1f} deg/s)"
        )

    def _update_mode_from_gripper_click(self, right_abs, left_abs):
        completed_clicks = 0
        for joint_name, angle in (
            ("right_joint_gripper", right_abs),
            ("left_joint_gripper", left_abs),
        ):
            if angle is None:
                continue

            abs_angle = abs(angle)
            if (not self.gripper_pressed[joint_name]) and abs_angle >= self.click_press_threshold:
                self.gripper_pressed[joint_name] = True
            elif self.gripper_pressed[joint_name] and abs_angle <= self.click_release_threshold:
                self.gripper_pressed[joint_name] = False
                completed_clicks += 1

        if completed_clicks == 0:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        for _ in range(completed_clicks):
            if self.last_click_time is not None and (now - self.last_click_time) <= self.double_click_window:
                self.click_count += 1
            else:
                self.click_count = 1
            self.last_click_time = now
            self.get_logger().info(f"Click count: {self.click_count}/2")

            if self.click_count >= 2:
                self.control_mode = (self.control_mode + 1) % 3
                self.click_count = 0
                mode_text = {
                    self.MODE_GRIPPER: "GRIPPER",
                    self.MODE_NECK_YAW: "NECK_YAW",
                    self.MODE_NECK_PITCH: "NECK_PITCH",
                }[self.control_mode]
                self.get_logger().info(f"Control mode changed: {mode_text}")
    

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
