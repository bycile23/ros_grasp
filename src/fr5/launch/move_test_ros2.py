#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class FKDemoNode(Node):
    def __init__(self):
        super().__init__('fk_demo_node')
        
        # 创建发布者，发送给我们在 yaml 里配置的控制器话题
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/fr5_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # 关节名称 (必须和你的 yaml 以及 urdf 里的一致)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.get_logger().info('FK Demo Node 已启动！')

    def send_goal(self, positions, duration=2.0):
        """发送关节角度目标"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # 创建一个轨迹点
        point = JointTrajectoryPoint()
        point.positions = positions
        # 设置期望到达时间 (越小速度越快)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送目标: {positions}')
        
        # 等待动作执行完
        time.sleep(duration + 0.5)

def main(args=None):
    rclpy.init(args=args)
    node = FKDemoNode()
    
    # 等待发布者连接成功
    time.sleep(1) 

    try:
        # 1. 回到原点 (Home) -> 全 0
        print("=== 动作 1: 回到原点 ===")
        node.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=2.0)

        # 2. 移动到指定位置 (你截图里的数据)
        print("=== 动作 2: 移动到指定姿态 ===")
        target_pos = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        node.send_goal(target_pos, duration=3.0)

        # 3. 再次回到原点
        print("=== 动作 3: 返回原点 ===")
        node.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=2.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()