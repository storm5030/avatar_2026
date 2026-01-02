from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baud = LaunchConfiguration('serial_baud')
    dxl_id = LaunchConfiguration('dxl_id')
    topic = LaunchConfiguration('topic')
    hz = LaunchConfiguration('hz')
    joint_name = LaunchConfiguration('joint_name')
    frame_id = LaunchConfiguration('frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='57600'),
        DeclareLaunchArgument('dxl_id', default_value='1'),
        DeclareLaunchArgument('topic', default_value='/joint_states'),
        DeclareLaunchArgument('hz', default_value='50.0'),
        DeclareLaunchArgument('joint_name', default_value='joint1'),
        DeclareLaunchArgument('frame_id', default_value=''),

        Node(
            package='avatar_control',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baud': serial_baud,
                'dxl_id': dxl_id,
                'topic': topic,
                'hz': hz
                'joint_name': joint_name,
                'frame_id': frame_id,
            }]
        ),
    ])
