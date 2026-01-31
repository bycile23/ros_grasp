#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import math
import copy

# ROS 消息类型
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState

# TF 坐标变换库
from tf2_ros import Buffer, TransformListener

class CartesianDemoNode(Node):
    def __init__(self):
        super().__init__('moveit_cartesian_demo_node')
        
        # ==========================================
        # 1. 关键配置
        # ==========================================
        self.group_name = "minipulator"      
        self.base_frame = "world"        # 确保这里是 world
        self.end_effector_link = "link6"     
        
        # 2. 通信接口
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/fr5_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.current_joint_state = None
        self.joint_state_received = False

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True

    # ==================================================
    # 核心修复：带 Spin 的等待函数
    # ==================================================
    def spin_and_sleep(self, duration):
        """在等待的同时处理回调，保证 TF 和关节数据能更新"""
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_services_and_data(self):
        """初始化等待流程"""
        # 1. 等待 MoveIt 服务
        self.get_logger().info('正在连接 /compute_cartesian_path 服务...')
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt Cartesian 服务未上线，继续等待...')
            rclpy.spin_once(self, timeout_sec=0.1) # 即使在这里也要 spin
        self.get_logger().info('服务连接成功！')

        # 2. 等待关节数据
        self.get_logger().info('正在等待 /joint_states 数据...')
        cnt = 0
        while not self.joint_state_received:
            # 【关键】这里必须 spin，否则回调永远不会触发
            rclpy.spin_once(self, timeout_sec=1.0)
            cnt += 1
            if cnt % 5 == 0:
                self.get_logger().warn(f'还没收到关节数据 (已等待 {cnt} 秒)... 请检查 demo.launch.py')
        self.get_logger().info('已收到关节状态数据！')

    def get_current_pose(self):
        """获取当前位姿"""
        # 尝试 20 次，每次 1 秒
        for i in range(20):
            # 每次尝试前都 spin 一下，更新 TF 树
            rclpy.spin_once(self, timeout_sec=0.1)
            
            try:
                if not self.tf_buffer.can_transform(self.base_frame, self.end_effector_link, rclpy.time.Time()):
                    self.get_logger().warn(f"TF 变换不存在 ({self.base_frame} -> {self.end_effector_link})，重试中 ({i+1}/20)...")
                    self.spin_and_sleep(1.0) # 使用带 spin 的 sleep
                    continue

                t = self.tf_buffer.lookup_transform(
                    self.base_frame, 
                    self.end_effector_link, 
                    rclpy.time.Time())
                
                pose = Pose()
                pose.position.x = t.transform.translation.x
                pose.position.y = t.transform.translation.y
                pose.position.z = t.transform.translation.z
                pose.orientation = t.transform.rotation
                return pose
            except Exception as e:
                self.get_logger().warn(f"查询 TF 失败: {e}，重试中...")
                self.spin_and_sleep(1.0)
        
        self.get_logger().error("彻底无法获取 TF！")
        return None

    def call_cartesian_service(self, waypoints):
        if self.current_joint_state is None:
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = RobotState()
        request.start_state.joint_state = self.current_joint_state
        request.group_name = self.group_name
        request.link_name = self.end_effector_link
        request.waypoints = waypoints
        request.max_step = 0.01        
        request.jump_threshold = 0.0   
        request.avoid_collisions = True

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val == 1:
            return response.solution
        else:
            self.get_logger().error(f"规划失败，错误码: {response.error_code.val}")
            return None

    def execute_trajectory(self, robot_trajectory):
        if not robot_trajectory or len(robot_trajectory.joint_trajectory.points) == 0:
            return

        msg = robot_trajectory.joint_trajectory
        msg.header.frame_id = self.base_frame
        
        time_per_point = 0.1 
        for i, point in enumerate(msg.points):
            time_from_start = (i + 1) * time_per_point
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)

        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        self.publisher_.publish(msg)
        self.get_logger().info(f">>> 执行路径 (共 {len(msg.points)} 个点)...")
        
        total_time = len(msg.points) * time_per_point
        # 使用带 spin 的等待，确保机器人运动期间还能接收数据
        self.spin_and_sleep(total_time + 1.0)

    def go_home(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.spin_and_sleep(2.5)

def main(args=None):
    rclpy.init(args=args)
    node = CartesianDemoNode()
    
    # 初始化等待
    node.wait_for_services_and_data()

    try:
        print("=== 1. 回到 Home ===")
        node.go_home()

        print("正在查询当前坐标...")
        start_pose = node.get_current_pose()
        
        if start_pose is None:
            return 

        print(f"当前位置: x={start_pose.position.x:.3f}, y={start_pose.position.y:.3f}, z={start_pose.position.z:.3f}")

        # === 生成路点 (严格复刻 ROS 1 逻辑) ===
        waypoints = []
        
        # 1. 记录起点
        waypoints.append(copy.deepcopy(start_pose))

        # 2. 路点 1 (P1): 向下走 0.2米
        wpose = copy.deepcopy(start_pose)
        wpose.position.z -= 0.2
        waypoints.append(copy.deepcopy(wpose))
        print("添加路点 1 (P1): Z - 0.2 (下)")

        # 3. 路点 2 (P2): 向前走 0.15米
        wpose.position.x += 0.15
        waypoints.append(copy.deepcopy(wpose))
        print("添加路点 2 (P2): X + 0.15 (前)")

        # 4. 路点 3 (P3): 向侧走 0.1米
        wpose.position.y += 0.1
        waypoints.append(copy.deepcopy(wpose))
        print("添加路点 3 (P3): Y + 0.1 (侧)")

        # 5. 路点 4 (P4): 回到 P1
        # 在 ROS 1 中：x -= 0.15, y -= 0.1 
        # 这意味着回到 (Start_X, Start_Y, Start_Z-0.2) 即 P1 点
        wpose.position.x -= 0.15
        wpose.position.y -= 0.1
        waypoints.append(copy.deepcopy(wpose))
        print("添加路点 4 (P4): 回到 P1 (闭合三角形)")

        # === 规划与执行 ===
        print("=== 4. 开始计算路径 (ROS 1 复刻版)... ===")
        trajectory = node.call_cartesian_service(waypoints)

        if trajectory:
            node.execute_trajectory(trajectory)
            print("路径执行完毕。")
        else:
            print("规划失败！")

        print("=== 5. 返回 Home ===")
        node.go_home()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()