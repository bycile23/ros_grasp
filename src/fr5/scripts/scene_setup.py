#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import time

class SceneSetup(Node):
    def __init__(self):
        super().__init__('scene_setup_node')
        self.pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        time.sleep(1.0)
        
        # ==========================================
        # 【修改点】：暂时注释掉生成方块和绿色罐子的调用
        # ==========================================
        # self.add_table()
        # self.add_cylinder()

    def add_cylinder(self):
        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = "target_cylinder"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.1, 0.03]

        pose = Pose()
        # 【位置修正】0.65 (桌子边缘舒适区)
        pose.position.x = 0.65
        pose.position.y = 0.00
        pose.position.z = 0.35 
        pose.orientation.w = 1.0
        
        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        self.publish_co(co, "绿色圆柱")

    def add_table(self):
        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = "table_obstacle"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        
        # 【恢复真实高度】我们不再欺骗MoveIt，安全第一
        primitive.dimensions = [0.3, 0.3, 0.3] 

        pose = Pose()
        # 【位置修正】桌子中心放 0.68
        # 这样桌子前沿 = 0.68 - 0.15 = 0.53
        # 物体刚好在桌子最前沿，机械臂不用跨过桌子
        pose.position.x = 0.8
        pose.position.y = 0.00
        pose.position.z = 0.15 
        pose.orientation.w = 1.0
        
        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        self.publish_co(co, "黑色底座(真实高度)")

    def publish_co(self, co, name):
        for i in range(5):
            self.pub.publish(co)
            time.sleep(0.1)
        self.get_logger().info(f"🟢 [场景] {name} 已发布 (X={co.primitive_poses[0].position.x})")

def main(args=None):
    rclpy.init(args=args)
    node = SceneSetup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()