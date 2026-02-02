from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_share = FindPackageShare('avatar_bringup')
    pkg_path = get_package_share_directory('avatar_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'follower', 'follower.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'follower.world')
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'follower', 'hardware_controller_manager.yaml'])


    # xacro → URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    robot_description_content = ParameterValue(
    Command([
        'xacro ',
        xacro_file,
        ' use_sim:=false',
        ' use_mock_hardware:=true',
        ' mock_sensor_commands:=false'
    ]),
    value_type=str
)


    # Gazebo 실행
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # 로봇 스폰 (딜레이: 1초)
    spawn_entity = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'follower', '-x', '0', '-y', '0', '-z', '0.3'],
                output='screen'
            )
        ]
    )

    # 컨트롤러 스폰
    control_node = Node(폰
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_file],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])