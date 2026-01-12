import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class FollowerDriver(Node):
    def __init__(self):
        super().__init__('follower_driver')

        # Publisher for joint trajectory commands
        self.follower_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.leader_trajectory_sub = self.create_subscription(
            JointTrajectory
            '/leader/joint_trajectory',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

    def joint_state_callback(self, msg: JointTrajectory):
        # recieve leader joint trajectory

        # republish to follower joint trajectory
        self.follower_trajectory_pub.publish(msg)

    def gripper_mapping(self, leader_position):
        # Map leader gripper position to follower gripper position
        # Example: simple linear mapping
        follower_position = leader_position  # Modify as needed
        return follower_position
    

def main(args=None):
    rclpy.init(args=args)
    follower_driver = FollowerDriver()
    rclpy.spin(follower_driver)
    follower_driver.destroy_node()
    rclpy.shutdown()
