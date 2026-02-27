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
        self.commanded_neck_position = {
            "neck_joint1": None,
            "neck_joint2": None,
        }
        self.has_neck_state = False

        self.MODE_GRIPPER = 0
        self.MODE_NECK_YAW = 1
        self.MODE_NECK_PITCH = 2
        self.control_mode = self.MODE_GRIPPER

        self.gripper_names = ["right_joint_gripper", "left_joint_gripper"]
        self.initial_gripper_angles = {name: None for name in self.gripper_names}
        self.last_gripper_targets = {name: None for name in self.gripper_names}
        self.gripper_pressed = {name: False for name in self.gripper_names}
        self.click_press_threshold = math.radians(10.0)
        self.click_release_threshold = math.radians(5.0)
        self.double_click_window = 1.0 # 더블클릭 인식 초
        self.last_click_time = {name: None for name in self.gripper_names}
        self.click_count = {name: 0 for name in self.gripper_names}

        self.gripper_activation_threshold = math.radians(10.0)
        self.neck_yaw_speed = 0.2    # rad/s
        self.neck_pitch_speed = 0.2  # rad/s
        self.last_control_time = None
        self.resync_neck_from_follower = False
        
        self.get_logger().info('Follower Passthrough Driver Node has been started.')

        # Joint command limits in radians (converted from follower DXL min/max).
        # min: ceil to 2 decimals in deg, max: floor to 2 decimals in deg.
        self.joint_limits = {
            "right_joint1": (math.radians(-131.83), math.radians(131.83)),
            "right_joint2": (math.radians(-81.91), math.radians(4.57)),
            "right_joint3": (math.radians(-90.00), math.radians(90.00)),
            "right_joint4": (math.radians(0.00), math.radians(90.00)),
            "right_joint5": (math.radians(-90.00), math.radians(90.00)),
            "right_joint6": (math.radians(-69.60), math.radians(69.60)),
            "right_joint_gripper": (math.radians(0.00), math.radians(111.79)),
            "left_joint1": (math.radians(-131.83), math.radians(131.83)),
            "left_joint2": (math.radians(-4.57), math.radians(81.91)),
            "left_joint3": (math.radians(-90.00), math.radians(90.00)),
            "left_joint4": (math.radians(-90.00), math.radians(0.00)),
            "left_joint5": (math.radians(-90.00), math.radians(90.00)),
            "left_joint6": (math.radians(-69.60), math.radians(69.60)),
            "left_joint_gripper": (math.radians(-111.79), math.radians(0.00)),
            "neck_joint1": (math.radians(-29.17), math.radians(29.17)),
            "neck_joint2": (math.radians(-43.94), math.radians(43.94)),
        }

    def leader_callback(self, msg: JointState):
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
        for joint_name, joint_pos in zip(msg.name, msg.position):
            if joint_name in self.neck_joint_names and self.control_mode != self.MODE_GRIPPER:
                # In neck modes, ignore incoming neck targets to avoid overwriting
                # locally integrated neck command.
                continue

            if joint_name in self.gripper_names:
                initial = self.initial_gripper_angles[joint_name]
                if initial is None:
                    initial = joint_pos
                    self.initial_gripper_angles[joint_name] = initial
                # Gripper uses startup-relative angle, not global -pi offset.
                transformed = -2.0 * (joint_pos - initial)
                if self.control_mode == self.MODE_GRIPPER:
                    target = transformed
                    self.last_gripper_targets[joint_name] = transformed
                else:
                    hold = self.last_gripper_targets[joint_name]
                    if hold is None:
                        hold = transformed
                        self.last_gripper_targets[joint_name] = hold
                    target = hold
            else:
                target = joint_pos - math.pi  # 180도 오프셋 적용

            target = self._clamp_joint_target(joint_name, target)

            point.positions.append(target)

        if self.control_mode != self.MODE_GRIPPER and self.has_neck_state:
            right_cmd = self._abs_to_cmd(right_abs)
            left_cmd = self._abs_to_cmd(left_abs)
            cmd = left_cmd - right_cmd
            if self.control_mode == self.MODE_NECK_YAW:
                neck_joint = "neck_joint2"
                neck_speed_cmd = cmd * self.neck_yaw_speed
                neck_delta = neck_speed_cmd * dt
            else:
                neck_joint = "neck_joint1"
                neck_speed_cmd = cmd * self.neck_pitch_speed
                neck_delta = neck_speed_cmd * dt

            if abs(neck_delta) > 0.0:
                base = self.commanded_neck_position[neck_joint]
                if base is None:
                    base = self.current_neck_position[neck_joint]
                target = base + neck_delta
                target = self._clamp_joint_target(neck_joint, target)
                self.commanded_neck_position[neck_joint] = target
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

    def _clamp_joint_target(self, joint_name: str, target: float) -> float:
        limits = self.joint_limits.get(joint_name)
        if limits is None:
            return target
        lower, upper = limits
        return max(lower, min(upper, target))

    # 그리퍼 각도를 절대값으로 추출하여 초기값에서의 delta로 반환. 그리퍼 클릭 감지에도 사용.
    def _extract_gripper_abs(self, msg: JointState, joint_name: str):
        if joint_name not in msg.name:
            return None
        idx = msg.name.index(joint_name)
        current = msg.position[idx]

        if self.initial_gripper_angles[joint_name] is None:
            self.initial_gripper_angles[joint_name] = current

        # Use delta from startup baseline to reject static motor offset drift.
        return current - self.initial_gripper_angles[joint_name]

    # 그리퍼 클릭 감지 및 제어 모드 전환
    def _update_mode_from_gripper_click(self, right_abs, left_abs):
        completed_clicks = []
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
                completed_clicks.append(joint_name)

        if not completed_clicks:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        for joint_name in completed_clicks:
            last_t = self.last_click_time[joint_name]
            if last_t is not None and (now - last_t) <= self.double_click_window:
                self.click_count[joint_name] += 1
            else:
                self.click_count[joint_name] = 1
            self.last_click_time[joint_name] = now
            self.get_logger().info(f"{joint_name} click count: {self.click_count[joint_name]}/2")

            if self.click_count[joint_name] >= 2:
                self.control_mode = (self.control_mode + 1) % 3
                self.click_count[joint_name] = 0
                mode_text = {
                    self.MODE_GRIPPER: "GRIPPER",
                    self.MODE_NECK_YAW: "NECK_YAW",
                    self.MODE_NECK_PITCH: "NECK_PITCH",
                }[self.control_mode]
                self.get_logger().info(f"Control mode changed by {joint_name}: {mode_text}")
                if self.control_mode == self.MODE_GRIPPER:
                    self.resync_neck_from_follower = True
                break
    

    # 현재 로봇 목 각도 받아오기 (그리퍼 모드로 변경될 때 새로 받음)
    def follower_joint_state_callback(self, msg: JointState):
        if not msg.name or not msg.position:
            return

        updated = False
        for joint_name in self.neck_joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_neck_position[joint_name] = msg.position[idx]
                if self.commanded_neck_position[joint_name] is None:
                    self.commanded_neck_position[joint_name] = msg.position[idx]
                updated = True

        if updated:
            self.has_neck_state = True
            if self.resync_neck_from_follower:
                for joint_name in self.neck_joint_names:
                    self.commanded_neck_position[joint_name] = self.current_neck_position[joint_name]
                self.resync_neck_from_follower = False
        

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
