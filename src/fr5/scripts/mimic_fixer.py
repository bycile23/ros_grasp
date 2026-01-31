#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MimicFixer(Node):
    def __init__(self):
        super().__init__('mimic_fixer_node')
        # 订阅真实的关节状态
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        # 发布补充的关节状态
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("🔧 夹爪模仿修复补丁已启动")

    def callback(self, msg):
        # 如果消息里包含左指，但不包含右指
        if 'left_finger_joint' in msg.name and 'right_finger_joint' not in msg.name:
            try:
                idx = msg.name.index('left_finger_joint')
                pos = msg.position[idx]
                
                # 创建一个新的消息，补充右指的信息
                new_msg = JointState()
                new_msg.header = msg.header
                # 这里的逻辑是：右指位置 = 左指位置 (或者 -pos，取决于您的坐标系)
                # 既然是模仿，通常是一样的数值（Prismatic）
                new_msg.name = ['right_finger_joint']
                new_msg.position = [pos] 
                new_msg.velocity = [0.0]
                new_msg.effort = [0.0]
                
                self.pub.publish(new_msg)
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = MimicFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()