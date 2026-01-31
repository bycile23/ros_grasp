#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class SimpleVision(Node):
    def __init__(self):
        super().__init__('simple_vision_node')
        self.target_pub = self.create_publisher(PointStamped, '/vision/target_point', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("👀 视觉调试模式: 发送修正后的坐标...")

    def timer_callback(self):
        p = PointStamped()
        p.header.frame_id = "base_link"
        p.header.stamp = self.get_clock().now().to_msg()
        
        # 【核心修正】
        # World文件里绿柱子在 X=0.7
        # 我们设为 0.68，保证手指能伸过去包住它，而不是停在它前面
        p.point.x = 0.65
        p.point.y = 0.00
        # 高度微调：0.35 是物体中心，抓取点应该再高一点点防止蹭桌子
        p.point.z = 0.35  
        
        self.target_pub.publish(p)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()