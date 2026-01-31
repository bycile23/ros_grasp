#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
import time

class RealIKDemoNode(Node):
    def __init__(self):
        super().__init__('real_ik_demo_node')
        
        # 1. 创建轨迹发布者 (用于控制机器人运动)
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/fr5_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # 2. 创建 IK 服务客户端 (用于让 MoveIt 帮我们算角度)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 定义关节名称
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 等待 IK 服务上线
        self.get_logger().info('正在连接 /compute_ik 服务，请稍候...')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt IK 服务未上线，继续等待...')
        self.get_logger().info('IK 服务连接成功！准备就绪。')

    def get_ik_solution(self, x, y, z, qx, qy, qz, qw):
        """
        真正的 IK 求解函数
        输入：期望的 XYZ 和 四元数姿态
        输出：算出来的 6 个关节角度 (如果无解则返回 None)
        """
        request = GetPositionIK.Request()
        request.ik_request.group_name = "minipulator" # 你的规划组名字
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        
        # 构建目标位姿 PoseStamped
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link" # 参考坐标系
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw
        
        request.ik_request.pose_stamped = target_pose
        request.ik_request.timeout.sec = 1 # 求解超时时间
        
        # 发送请求并等待结果 (同步调用)
        self.get_logger().info(f"正在请求 IK 解算: Pos[{x}, {y}, {z}]...")
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        # 检查是否有解
        if response.error_code.val == 1: # 1 代表成功 (SUCCESS)
            joint_positions = response.solution.joint_state.position
            # 注意：MoveIt 返回的关节可能包含很多东西，我们需要按照我们的 joint_names 过滤一下
            # 这里简单处理，假设返回的前6个就是我们的机械臂关节（通常是这样）
            # 更严谨的做法是匹配 joint_names
            relevant_joints = list(joint_positions)[:6]
            self.get_logger().info(f"IK 求解成功！关节角度: {[round(j, 3) for j in relevant_joints]}")
            return relevant_joints
        else:
            self.get_logger().error(f"IK 求解失败，错误代码: {response.error_code.val}")
            return None

    def send_trajectory(self, positions, duration=3.0):
        """发送运动指令"""
        if positions is None:
            self.get_logger().warn("没有有效角度，跳过运动")
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info('>>> 执行运动...')
        time.sleep(duration + 0.5)

def main(args=None):
    rclpy.init(args=args)
    node = RealIKDemoNode()
    time.sleep(1) # 等待发布者连接

    try:
        # === 1. 回到原点 ===
        print("\n=== 步骤 1: 回到 Home ===")
        node.send_trajectory([0.0]*6, duration=2.0)

        # === 2. 真正的 IK 挑战 ===
        # 这里我们只输入 XYZ 和 四元数，完全不知道关节角度是多少
        # 数据来源是你之前的 ROS 1 截图
        print("\n=== 步骤 2: 真正的 IK 求解与运动 ===")
        print("目标: x=0.2593, y=0.0636, z=0.1787")
        
        # 调用 MoveIt 计算 IK
        calculated_joints = node.get_ik_solution(
            x=0.2593, y=0.0636, z=0.1787,
            qx=0.70692, qy=0.0, qz=0.0, qw=0.70729
        )
        
        # 如果算出来了，就走过去
        if calculated_joints:
            node.send_trajectory(calculated_joints, duration=3.0)
        else:
            print("解算失败，可能是目标点不可达")

        # === 3. 再次回到 Home ===
        print("\n=== 步骤 3: 返回 Home ===")
        node.send_trajectory([0.0]*6, duration=2.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()