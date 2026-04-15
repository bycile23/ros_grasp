from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration # 新增导入

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fr5", package_name="robot_fr5_moveit_config").to_moveit_configs()
    moveit_config_dict = moveit_config.to_dict()

    # 声明使用仿真时间变量，默认为 False
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            {"trajectory_execution": {"execution_duration_monitoring": False}},
            # 【修复】使用动态配置，而不是强制 True
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([run_move_group_node])