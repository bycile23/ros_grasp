import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 定义路径变量
    fr5_pkg_share = get_package_share_directory('fr5')
    moveit_config_share = get_package_share_directory('robot_fr5_moveit_config')
    
    # 获取 xacro 文件路径
    xacro_file = os.path.join(fr5_pkg_share, 'urdf', 'fr5.urdf.xacro')
    
    # 获取控制器配置文件路径
    controllers_file = os.path.join(fr5_pkg_share, 'config', 'fr5_controllers.yaml')

    # 2. 加载机器人描述 (Robot Description)
    # 【关键点】这里传入 use_fake_hardware:=true，激活我们在 URDF 里写的伪造硬件
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_fake_hardware:=true", 
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 3. 加载语义描述 (SRDF) - MoveIt 需要知道谁是手臂，谁是夹爪
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(moveit_config_share, "config", "fr5.srdf"),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # 4. 启动 ros2_control_node (控制器管理器)
    # 这就是负责加载 Mock 硬件的核心节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

    # 5. 启动 Robot State Publisher (发布 TF)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 6. 加载控制器 (Spawner)
    # 负责把 fr5_joint_trajectory_controller 生出来
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr5_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # 7. 启动 Move Group (MoveIt 的大脑)
    # 直接引用 MoveIt 生成好的启动文件
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "move_group.launch.py")
        ),
        # 传入机器人描述参数，防止 Move Group 找不到模型
        launch_arguments={
            "robot_description": robot_description_content,
            "robot_description_semantic": robot_description_semantic_content,
        }.items(),
    )

    # 8. 启动 RViz (可视化)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "moveit_rviz.launch.py")
        ),
        launch_arguments={
            "robot_description": robot_description_content,
            "robot_description_semantic": robot_description_semantic_content,
        }.items(),
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        move_group_launch,
        rviz_launch
    ])