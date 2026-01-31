import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():
    pkg_fr5 = FindPackageShare('fr5')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_moveit_config = FindPackageShare('robot_fr5_moveit_config')

    # 1. URDF & SRDF
    xacro_file = os.path.join(os.getcwd(), 'src', 'fr5', 'urdf', 'fr5.urdf.xacro')
    if not os.path.exists(xacro_file):
        xacro_file = '/home/baitiao/dev_ws_a/src/fr5/urdf/fr5.urdf.xacro'
    
    doc = os.popen(f'xacro {xacro_file} use_fake_hardware:=false').read()
    robot_description = {'robot_description': doc}

    robot_description_semantic_content = ""
    try:
        moveit_config_path = get_package_share_directory('robot_fr5_moveit_config')
        srdf_path = os.path.join(moveit_config_path, 'config', 'fr5.srdf')
        with open(srdf_path, 'r') as f:
            robot_description_semantic_content = f.read()
    except Exception as e:
        print(f"Error loading SRDF: {e}")

    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    kinematics_yaml = load_yaml('robot_fr5_moveit_config', 'config/kinematics.yaml')
    
    # -----------------------------------------------------------------
    # [核心修正] 构建 MoveIt Pipeline 参数结构
    # -----------------------------------------------------------------
    # 1. 读取基础的 OMPL 配置 (包含 minipulator 组配置)
    ompl_yaml_content = load_yaml('robot_fr5_moveit_config', 'config/ompl_planning.yaml')
    
    if ompl_yaml_content is None:
        print("【严重警告】未找到 ompl_planning.yaml！规划器将无法启动！")
        ompl_yaml_content = {}

    # 2. 将配置包装进 'ompl' 命名空间，并指定 pipeline 参数
    # 这种嵌套结构是 ROS 2 MoveIt 识别规划器的标准方式
    move_group_planning_config = {
        'planning_pipelines': ['ompl'],        # 声明可用的管道列表
        'default_planning_pipeline': 'ompl',   # 强制指定默认管道为 ompl
        'ompl': ompl_yaml_content              # 将具体的 yaml 内容放入 'ompl' 键下
    }
    # -----------------------------------------------------------------

    moveit_controllers = load_yaml('robot_fr5_moveit_config', 'config/moveit_controllers.yaml')

    # 2. Gazebo
    world_path = PathJoinSubstitution([pkg_fr5, 'worlds', 'vision_task.world'])
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world_path}.items()
    )

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # 【修改处】强制指定 -x -y -z 确保它必须在 0,0,0 生成
        arguments=['-topic', 'robot_description', '-entity', 'fr5', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )
    
    # 5. Controllers
    spawn_controllers = [
        Node(package="controller_manager", executable="spawner", arguments=["fr5_joint_trajectory_controller", "-c", "/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller", "-c", "/controller_manager"], output="screen"),
    ]

    # 6. MoveIt MoveGroup
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            move_group_planning_config,  # <--- 使用修正后的嵌套参数
            moveit_controllers,
            {'use_sim_time': True},
            {"trajectory_execution": {"execution_duration_monitoring": False}}, 
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 7. RViz
    rviz_config_file = PathJoinSubstitution([pkg_moveit_config, 'config', 'moveit.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': True}
        ]
    )

    # 8. Scene Setup
    scene_setup_script = '/home/baitiao/dev_ws_a/src/fr5/scripts/scene_setup.py'
    if not os.path.exists(scene_setup_script):
        scene_setup_script = os.path.join(os.getcwd(), 'src', 'fr5', 'scripts', 'scene_setup.py')
    
    run_scene_setup = ExecuteProcess(
        cmd=['python3', scene_setup_script],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        node_robot_state_publisher,
        spawn_entity,
        *spawn_controllers,
        move_group_node,
        rviz_node,
        TimerAction(period=5.0, actions=[run_scene_setup])
    ])