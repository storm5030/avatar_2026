import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share_path = get_package_share_directory('avatar_bringup')
    pkg_description_path = get_package_share_directory('avatar_description')

    xacro_file = os.path.join(pkg_description_path, 'urdf', 'follower', 'follower.urdf.xacro')
    world_file = os.path.join(pkg_description_path, 'worlds', 'follower.world')

    # [중요] Gazebo가 모델/메쉬 파일을 찾을 수 있도록 환경변수 설정
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_sim_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + os.path.join(pkg_description_path, '..')
    else:
        gz_sim_resource_path = os.path.join(pkg_description_path, '..')

    set_gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_sim_resource_path)

    # 1. Robot Description 생성 (use_sim:=true로 설정해야 가제보 플러그인 사용)
    # 주의: xacro 파일 내부에서 $(arg use_sim)을 처리하도록 되어 있어야 합니다.
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_sim:=true',             # 시뮬레이션 모드 ON
        ' use_mock_hardware:=false'   # Mock 하드웨어 OFF (가제보가 하드웨어임)
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # 2. Robot State Publisher (TF 발행)
    # use_sim_time을 True로 해야 가제보 시간과 동기화됩니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Gazebo 실행
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 4. 로봇 스폰
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'follower', '-z', '0.3'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 시계 (필수)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # 2. 관절 상태 (필수)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # 3. [복구됨] 충돌 센서 브릿지
            # 주의: 이 토픽 이름이 ign topic -l 에서 나온 것과 토씨 하나 안 틀리고 똑같아야 합니다!
            '/world/default/model/follower/link/left_link_gripper_1/sensor/left_gripper_bumper/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
        ],
        output='screen'
    )

    # 6. 컨트롤러 스폰 (로봇이 생성된 후에 실행해야 안전함)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    safety_stop = Node(
        package='avatar_follower',      # 패키지 이름 확인!
        executable='safety_stop', # setup.py에 등록된 실행 파일 이름
        name='safety_stop_node',
        output='screen'
    )

    # 실행 순서 제어: 로봇 스폰 -> 컨트롤러 실행
    # spawn_entity가 끝나면(Exit) -> 컨트롤러를 실행한다
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster, arm_controller, safety_stop],
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,
        robot_state_publisher_node,
        spawn_entity,
        load_controllers,
    ])