#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup_path = get_package_share_directory('avatar_bringup')
    pkg_description_path = get_package_share_directory('avatar_description')

    xacro_file = os.path.join(pkg_description_path, 'urdf', 'follower', 'follower.urdf.xacro')
    world_file = os.path.join(pkg_description_path, 'worlds', 'follower.world')

    declared_arguments = [
        DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false', description='Enable mock sensor commands.'),
        DeclareLaunchArgument('init_position', default_value='false', description='Whether to launch the init_position node'),
        DeclareLaunchArgument('ros2_control_type', default_value='follower', description='Type of ros2_control'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyACM0', description='Port name for the hardware interface'),
    ]

    prefix = LaunchConfiguration('prefix')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    port_name = LaunchConfiguration('port_name')

    # 설정 파일 경로
    controller_manager_config = os.path.join(pkg_bringup_path, 'config', 'follower', 'hardware_controller_manager.yaml')
    trajectory_params_file = os.path.join(pkg_bringup_path, 'config', 'follower', 'initial_positions.yaml')


    # 실제 하드웨어용 URDF
    real_urdf_cmd = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file, ' ',
        'prefix:=', prefix, ' ', 'use_sim:=false ', 'use_mock_hardware:=false ',
        'mock_sensor_commands:=', mock_sensor_commands, ' ',
        'ros2_control_type:=', ros2_control_type, ' ', 'port_name:=', port_name,
    ])
    real_robot_desc = ParameterValue(real_urdf_cmd, value_type=str)

    # 가제보 시뮬레이션용 URDF
    sim_urdf_cmd = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file, ' ',
        'prefix:=', prefix, ' ', 'use_sim:=true ', 'use_mock_hardware:=false ',
        'mock_sensor_commands:=', mock_sensor_commands, ' ',
        'ros2_control_type:=', ros2_control_type, ' ', 'port_name:=', port_name,
    ])
    sim_robot_desc = ParameterValue(sim_urdf_cmd, value_type=str)


    real_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='real_robot_state_publisher',
        parameters=[{'robot_description': real_robot_desc, 'use_sim_time': False}],
        remappings=[('robot_description', 'real_robot_description'), ('joint_states', 'real_joint_states')],
        output='both'
    )

    real_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='real_controller_manager', # 충돌 방지를 위해 이름 변경
        parameters=[{'robot_description': real_robot_desc}, controller_manager_config],
        remappings=[
            ('~/robot_description', 'real_robot_description'),
            ('joint_states', 'real_joint_states'),
            ('/arm_controller/joint_trajectory', '/leader/joint_trajectory') # 제어 동기화
        ],
        output='both',
    )

    real_jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='real_jsb_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/real_controller_manager'], output='both'
    )

    real_arm_spawner = Node(
        package='controller_manager', executable='spawner', name='real_arm_spawner',
        arguments=['arm_controller', '--controller-manager', '/real_controller_manager'], output='both'
    )

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_sim_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + os.path.join(pkg_description_path, '..')
    else:
        gz_sim_resource_path = os.path.join(pkg_description_path, '..')
    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_sim_resource_path)

    sim_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sim_robot_state_publisher',
        parameters=[{'robot_description': sim_robot_desc, 'use_sim_time': True}],
        remappings=[('robot_description', 'sim_robot_description'), ('joint_states', 'sim_joint_states')],
        output='both'
    )

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file, '--ros-args', '-r', '/arm_controller/joint_trajectory:=/leader/joint_trajectory'],
        output='screen'
    )

    sim_spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', sim_robot_desc, '-name', 'follower', '-z', '0.3'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/world/default/model/follower/link/base_link/sensor/base_link_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/neck_link1_1/sensor/neck_link1_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/neck_link2_1/sensor/neck_link2_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link1_1/sensor/left_link1_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link2_1/sensor/left_link2_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link3_1/sensor/left_link3_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link4_1/sensor/left_link4_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link5_1/sensor/left_link5_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link6_1/sensor/left_link6_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/left_link_gripper_1/sensor/left_gripper_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link1_1/sensor/right_link1_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link2_1/sensor/right_link2_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link3_1/sensor/right_link3_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link4_1/sensor/right_link4_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link5_1/sensor/right_link5_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link6_1/sensor/right_link6_1_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/world/default/model/follower/link/right_link_gripper_1/sensor/right_gripper_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
        ],
        remappings=[('/joint_states', 'sim_joint_states')], # 브릿지에서 나오는 joint_states 토픽 분리
        output='screen'
    )

    sim_jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='sim_jsb_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'
    )

    sim_arm_spawner = Node(
        package='controller_manager', executable='spawner', name='sim_arm_spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'], output='screen'
    )

    safety_stop = Node(
        package='avatar_follower', executable='safety_stop', name='safety_stop_node', output='screen'
    )


    joint_trajectory_executor = Node(
        package='avatar_bringup', executable='joint_trajectory_executor',
        parameters=[trajectory_params_file], output='both', condition=IfCondition(init_position),
    )

    load_sim_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=sim_spawn_entity,
            on_exit=[sim_jsb_spawner, sim_arm_spawner, safety_stop],
        )
    )

    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=real_arm_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )


    return LaunchDescription(
        declared_arguments + [
            # Real
            real_rsp, real_control_node, real_jsb_spawner, real_arm_spawner,
            # Sim
            set_gz_resource_path, gz_sim, sim_rsp, sim_spawn_entity, bridge, load_sim_controllers,
            # Common
            delay_joint_trajectory_executor_after_controllers
        ]
    )