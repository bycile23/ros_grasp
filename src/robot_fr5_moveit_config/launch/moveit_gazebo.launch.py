import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter # <--- 引入 SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取各个包的路径
    fr5_pkg = get_package_share_directory('fr5')
    moveit_config_pkg = get_package_share_directory('robot_fr5_moveit_config')

    # ============================================================
    # 【关键修改】全局强制设置 use_sim_time 为 True
    # 这会让在这个 Launch 文件启动的所有节点（包括 MoveIt）都听 Gazebo 的时间
    # ============================================================
    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

    # 2. 包含 Gazebo 仿真启动文件
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fr5_pkg, 'launch', 'gazebo_sim.launch.py')
        )
    )

    # 3. 启动 MoveIt 的核心节点 (Move Group)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # 4. 启动 RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    return LaunchDescription([
        set_use_sim_time, # <--- 这一行必须加上
        gazebo_sim,
        move_group,
        rviz
    ])