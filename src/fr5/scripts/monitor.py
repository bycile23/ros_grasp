#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        
        # 订阅关节状态
        self.sub_joints = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10)
            
        self.get_logger().info("🤖 监控器已启动...")
        self.get_logger().info("如果你看到 Joint 1 不在 0.0 附近，说明它还在漂移！")

    def joint_callback(self, msg):
        try:
            # 找到 joint1 和 joint6 的索引
            idx_1 = msg.name.index('joint1')
            idx_6 = msg.name.index('joint6')
            
            pos_1 = msg.position[idx_1]
            pos_6 = msg.position[idx_6]
            
            # 打印监控信息
            # 如果 pos_1 持续偏离 0.0 (比如 -0.27)，说明阻尼没加上
            status = "✅ 正常"
            if abs(pos_1) > 0.05:
                status = "❌ 正在向右漂移！"
            
            print(f"\r[监控] Joint 1 (基座): {pos_1:.4f} | Joint 6 (手腕): {pos_6:.4f} | {status}", end="")
            
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()