import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions
import launch.launch_description_sources

def generate_launch_description():
    # 1. 获取功能包 share 路径
    fr5_package_path = get_package_share_directory('fr5')
    
    # 2. 设置 Xacro 文件路径
    default_xacro_path = os.path.join(fr5_package_path, 'urdf', 'fr5.urdf.xacro')
    
    # 3. 声明 model 参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='加载模型文件路径'
    )

    # 4. 通过 xacro 命令处理文件
    substitutions_Command_result = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_Command_result,
        value_type=str
    )

    # 5. 启动 robot_state_publisher (发布静态 TF 和模型描述)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
  
    # 6. 启动 Gazebo
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('verbose', 'true')]
    )

    # 7. 生成机器人实体 (Spawn Entity)
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fr5'],
    )

    # ========================================================================
    # 新增部分：加载控制器 (根据 fr5_controllers.yaml)
    # ========================================================================

    # 8. 加载关节状态广播器 (joint_state_broadcaster)
    # 它的作用是读取 Gazebo 的关节数据并发布到 /joint_states
    load_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 9. 加载关节轨迹控制器 (fr5_joint_trajectory_controller)
    # 它的作用是接收 MoveIt 的控制指令
    load_joint_trajectory_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr5_joint_trajectory_controller"],
        output="screen",
    )

    # 10. 设置事件处理器 (Event Handler)
    # 只有当 'spawn_entity' (生成机器人) 结束后，才去加载控制器。
    # 这样可以防止 Gazebo 还没准备好，控制器就尝试连接导致的报错。
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher, 
        action_launch_gazebo,
        action_spawn_entity,
        
        # 注册事件：当 spawn_entity 退出（完成）时，执行加载控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])