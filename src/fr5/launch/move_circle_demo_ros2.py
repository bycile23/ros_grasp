#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import copy

# ROS 消息类型
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

# TF 坐标变换库
from tf2_ros import Buffer, TransformListener

class MoveItCircleDemo(Node):
    def __init__(self):
        super().__init__('moveit_circle_demo_node')
        
        # 1. 基础配置
        self.group_name = "minipulator"
        self.base_frame = "world"        
        self.end_effector_link = "link6" 
        
        # 2. 服务客户端
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # 3. 轨迹发布者
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/fr5_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # 4. 关节状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.current_joint_state = None

        # 5. TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('正在连接 MoveIt 服务...')
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('规划服务未上线...')
        self.get_logger().info('MoveIt 服务连接成功！')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def get_current_pose(self):
        """获取当前末端位姿"""
        for i in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                if not self.tf_buffer.can_transform(self.base_frame, self.end_effector_link, rclpy.time.Time()):
                    time.sleep(0.5)
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
            except Exception:
                time.sleep(0.5)
        return None

    def call_cartesian_service(self, waypoints):
        """调用 MoveIt 服务计算圆弧路径"""
        #if self.current_joint_state is None:
        #    self.get_logger().error("未收到关节状态")
        #    return None, 0.0

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()

        #request.start_state = RobotState()
        #request.start_state.joint_state = self.current_joint_state
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
            return response.solution, response.fraction
        else:
            self.get_logger().error(f"规划失败，错误码: {response.error_code.val}")
            return None, 0.0

    def retime_trajectory(self, trajectory, default_time_step=0.1):
        """
        根据关节变化量动态调整时间，避免速度过快。
        """
        new_traj = JointTrajectory()
        new_traj.header = trajectory.joint_trajectory.header
        new_traj.joint_names = trajectory.joint_trajectory.joint_names
        
        # 假设所有关节的最大速度是 1.0 rad/s (保守值，可根据 FR5 规格调整)
        max_joint_vel = 0.5 
        current_time = 0.0

        # 获取初始位置 (第一个点)
        prev_positions = trajectory.joint_trajectory.points[0].positions
        
        # 处理第一个点
        p0 = copy.deepcopy(trajectory.joint_trajectory.points[0])
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        new_traj.points.append(p0)

        for i in range(1, len(trajectory.joint_trajectory.points)):
            point = trajectory.joint_trajectory.points[i]
            curr_positions = point.positions
            
            # 计算这一步最大的关节变化量
            max_delta = 0.0
            for j in range(len(curr_positions)):
                delta = abs(curr_positions[j] - prev_positions[j])
                if delta > max_delta:
                    max_delta = delta
            
            # 这一步需要的最少时间 = 最大距离 / 允许速度
            # 如果计算出的时间太小，就用默认的最小时间步长，保证平滑
            dt = max(max_delta / max_joint_vel, default_time_step)
            
            current_time += dt
            
            new_point = JointTrajectoryPoint()
            new_point.positions = curr_positions
            # 注意：GetCartesianPath 默认不给速度和加速度，设为0或不设
            new_point.velocities = [] 
            new_point.accelerations = []
            
            new_point.time_from_start.sec = int(current_time)
            new_point.time_from_start.nanosec = int((current_time - int(current_time)) * 1e9)
            
            new_traj.points.append(new_point)
            prev_positions = curr_positions
        
        return new_traj, current_time

    def execute_trajectory(self, trajectory):
        final_traj, total_time = self.retime_trajectory(trajectory, time_per_point=0.05) 
        self.get_logger().info(f"执行路径... 预计耗时 {total_time:.1f} 秒")
        self.trajectory_publisher.publish(final_traj)
        
        end_time = time.time() + total_time + 1.0
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_direct_trajectory(self, positions):
        """发送关节指令"""
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3 # 给3秒慢慢移动
        msg.points.append(point)
        self.trajectory_publisher.publish(msg)
        # 带 spin 的等待，确保数据不断
        end_time = time.time() + 3.5
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MoveItCircleDemo()
    time.sleep(1.0) 

    try:
        # === 1. 关键修改：前往“预备姿态” (Ready Pose) ===
        print("=== 1. 前往预备姿态 (避开全0奇异点) ===")
        # 这是一个舒服的姿态：大臂抬起，小臂下垂，远离奇异点
        ready_joints = [0.0, -0.174, 0.0, 0.0, 0.0, 0.0]
        node.send_direct_trajectory(ready_joints)

        # 2. 获取圆心参考点 (现在是在舒服的位置获取的)
        print("正在查询当前坐标...")
        start_pose = node.get_current_pose()
        if start_pose is None:
            print("无法获取当前位姿，退出。")
            return

        print(f"当前起点 (圆弧起点): x={start_pose.position.x:.3f}, y={start_pose.position.y:.3f}, z={start_pose.position.z:.3f}")

        # === 3. 生成圆弧路点 (终极解决方案：过冲法) ===
        waypoints = []
        center_y = start_pose.position.y
        center_z = start_pose.position.z
        radius = 0.15 

        # 【核心修改】
        # 不再追求精确停止在 2*pi。
        # 而是让角度多走一点点 (例如多走 0.2 弧度)，形成重叠。
        # 2 * math.pi 约为 6.28，我们走到 6.5 左右。
        extra_angle = 0.2
        angles = np.arange(0.0, 2 * math.pi + extra_angle, 0.05)
        
        print(f"将生成从 0 到 {angles[-1]:.2f} 弧度的路径 (包含重叠)")

        for th in angles:
            wpose = copy.deepcopy(start_pose)
            # 在 YZ 平面画圆
            real_center_z = center_z - radius
            wpose.position.y = center_y - radius * math.sin(th)
            wpose.position.z = real_center_z + radius * math.cos(th)
            waypoints.append(wpose)

        print(f"生成了 {len(waypoints)} 个过冲路点")

        # === 4. 规划与执行 ===
        print("=== 4. 开始计算圆弧路径... ===")
        trajectory, fraction = node.call_cartesian_service(waypoints)

        if trajectory:
            print(f"规划成功！覆盖率: {fraction * 100:.1f}%")
            
            start_point = trajectory.joint_trajectory.points[0].positions
            current_joints = node.current_joint_state.position # 注意顺序需要对齐

            diff = [s - c for s, c in zip(start_point, current_joints)]
            print(f"IK起步偏差: {diff}")
        
            node.execute_trajectory(trajectory)
            print("圆弧绘制完成。")
        else:
            print("规划失败！")

        # 5. 结束
        print("=== 5. 演示结束 ===")
        # 保持姿态，或者回 Home (这里注释掉回Home，方便你看圆)
        node.send_direct_trajectory([0.0]*6) 

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()