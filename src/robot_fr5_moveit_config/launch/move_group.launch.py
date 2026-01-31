from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载配置
    moveit_config = MoveItConfigsBuilder("fr5", package_name="robot_fr5_moveit_config").to_moveit_configs()

    # 2. 获取配置字典
    moveit_config_dict = moveit_config.to_dict()

    # 3. 【核心修改】手动构建 Node，强行注入参数
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            # 【强制注入】关闭执行时间监控，解决 Timeout
            {"trajectory_execution": {"execution_duration_monitoring": False}},
            # 【强制注入】使用仿真时间
            {"use_sim_time": True},
        ],
    )

    # 4. 返回 LaunchDescription
    return LaunchDescription([
        run_move_group_node
    ])