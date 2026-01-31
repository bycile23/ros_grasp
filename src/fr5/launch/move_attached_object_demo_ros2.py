#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import copy
import math

# ROS 消息类型
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, MotionPlanRequest, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class AttachedObjectDemo(Node):
    def __init__(self):
        #父类初始化
        super().__init__('moveit_attached_object_demo')
        
        self.group_name = "minipulator"#机械臂名字
        self.base_frame = "world"   #基准坐标系     
        self.end_effector_link = "link6" #末端执行器
        
        # 1. 规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 2. 场景服务客户端
        self.apply_scene_service = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        
        # 3. 轨迹发布者 (执行)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/fr5_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # 4. 关节状态订阅 (获取规划起点)
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.current_joint_state = None #当前关节状态

        self.get_logger().info('正在连接 MoveIt 规划服务...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('规划服务 (/plan_kinematic_path) 未上线...')
        while not self.apply_scene_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('场景服务未上线...')
        self.get_logger().info('MoveIt 服务连接成功！')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
    #把包装的scene-msg发送
    def apply_scene(self, scene_msg):
        """同步应用场景"""
        request = ApplyPlanningScene.Request()
        request.scene = scene_msg
        future = self.apply_scene_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def add_obstacle(self):
        """添加障碍物"""
        obstacle = CollisionObject()
        obstacle.header.frame_id = self.base_frame#世界坐标系下
        obstacle.id = "table"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # 尺寸：长0.2, 宽0.8, 厚0.02
        primitive.dimensions = [0.05, 0.4, 0.02] 
        
        pose = Pose()
        pose.position.x = 0.50
        pose.position.y = 0.0
        pose.position.z = 0.55
        pose.orientation.w = 1.0
        
        obstacle.primitives.append(primitive)
        obstacle.primitive_poses.append(pose)
        obstacle.operation = CollisionObject.ADD

        scene_msg = PlanningScene()
        scene_msg.world.collision_objects.append(obstacle)# 把它加到“世界”的障碍物列表里
        scene_msg.is_diff = True # 重要！意思是“这只是一个更新，不要把原来的东西删了”
        self.apply_scene(scene_msg) # 发送
        self.get_logger().info(f"已添加桌子障碍")
        

    def attach_tool(self):
        """附着工具"""
        tool = CollisionObject()
        tool.header.frame_id = self.end_effector_link
        tool.id = "tool"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.2, 0.02, 0.02] 
        
        pose = Pose()
        pose.position.x = 0.1 
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        
        tool.primitives.append(primitive)
        tool.primitive_poses.append(pose)
        tool.operation = CollisionObject.ADD

        attached_tool = AttachedCollisionObject()
        attached_tool.link_name = self.end_effector_link
        attached_tool.object = tool
        attached_tool.touch_links = ['link6', 'link5', 'link4'] 
        # touch_links (白名单): 既然拿在手里，工具肯定会碰到手。
        # 如果不把 link6 加到这里，MoveIt 就会尖叫：“啊！有东西撞到 link6 了！”

        scene_msg = PlanningScene()
        # =================================================================
        # 【核心修正】
        # 这里绝对不能把 tool 加到 world.collision_objects 里！
        # 如果加了，MoveIt 就会认为原地有一个“静态”的工具。
        # 同时你也 Attach 了一个工具。
        # 结果就是：手上的工具撞到了原地的静态工具 -> 报错初始位置碰撞。
        # =================================================================
        # scene_msg.world.collision_objects.append(tool)  <-- 删除这一行
        scene_msg.robot_state.attached_collision_objects.append(attached_tool)
        scene_msg.is_diff = True
        self.apply_scene(scene_msg)
        self.get_logger().info("已附着工具")

    def cleanup_scene(self):
        """清理场景"""
        self.get_logger().info("清理场景...")
        # 1. Detach
        detach_tool = AttachedCollisionObject()#定义消息类型
        detach_tool.object.id = "tool"#确定名字
        detach_tool.object.operation = CollisionObject.REMOVE#去除附着
        detach_tool.link_name = self.end_effector_link#哪个link上的
        scene_msg_detach = PlanningScene()
        scene_msg_detach.robot_state.attached_collision_objects.append(detach_tool)
        scene_msg_detach.is_diff = True
        self.apply_scene(scene_msg_detach)#发送
        time.sleep(0.5)

        # 2. Remove
        remove_obs = CollisionObject()
        remove_obs.id = "table"
        remove_obs.operation = CollisionObject.REMOVE
        # 既然我们没有添加到 world，其实不需要从 world remove tool，
        # 但写着也无妨，MoveIt 找不到ID会忽略。
        remove_tool = CollisionObject()
        remove_tool.id = "tool"
        remove_tool.operation = CollisionObject.REMOVE

        scene_msg_remove = PlanningScene()
        scene_msg_remove.world.collision_objects.append(remove_obs)
        scene_msg_remove.world.collision_objects.append(remove_tool)
        scene_msg_remove.is_diff = True
        self.apply_scene(scene_msg_remove)

    def retime_trajectory(self, trajectory, avg_speed=0.5):
        new_traj = JointTrajectory()
        new_traj.header = trajectory.joint_trajectory.header
        new_traj.header.stamp = self.get_clock().now().to_msg()
        
        new_traj.joint_names = trajectory.joint_trajectory.joint_names
        dt = 0.2 
        for i, point in enumerate(trajectory.joint_trajectory.points):
            new_point = JointTrajectoryPoint()
            new_point.positions = point.positions
            new_point.velocities = point.velocities
            new_point.accelerations = point.accelerations
            time_from_start = (i + 1) * dt
            new_point.time_from_start.sec = int(time_from_start)
            new_point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            new_traj.points.append(new_point)
        return new_traj, (len(new_traj.points) * dt)

    def plan_path(self, target_joints):
        if self.current_joint_state is None:
            self.get_logger().error("未收到关节状态")
            return None

        self.get_logger().info(">>> MoveIt 规划开始 (RRTConnect)...")

        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = self.base_frame
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        
        # 保持纯净的 Diff 模式，不要手动塞 start_state
        req.start_state.is_diff = True
        
        req.group_name = self.group_name
        req.planner_id = "RRTConnectkConfigDefault" 
        req.num_planning_attempts = 50 
        req.allowed_planning_time = 10.0 
        
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        for i, angle in enumerate(target_joints):#为每个关节创建约束
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        req.goal_constraints.append(constraints)

        service_req = GetMotionPlan.Request()
        service_req.motion_plan_request = req
        
        future = self.plan_client.call_async(service_req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.motion_plan_response.error_code.val == 1: 
            self.get_logger().info("规划成功！")
            return response.motion_plan_response.trajectory
        else:
            self.get_logger().error(f"规划失败，错误码: {response.motion_plan_response.error_code.val}")
            return None

    def execute_trajectory(self, trajectory):
        final_traj, total_time = self.retime_trajectory(trajectory)
        self.get_logger().info(f"执行路径... 预计耗时 {total_time:.1f} 秒")
        self.trajectory_publisher.publish(final_traj)
        time.sleep(total_time + 1.0)

    def send_direct_trajectory(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        msg.points.append(point)
        self.trajectory_publisher.publish(msg)
        time.sleep(3.5)

def main(args=None):
    rclpy.init(args=args)
    node = AttachedObjectDemo()
    time.sleep(1.0) 

    try:
        node.cleanup_scene()
        time.sleep(1.0)

        # 1. 回到 Home
        print("=== 1. 回到 Home ===")
        node.send_direct_trajectory([0.0]*6)

        # 2. 添加障碍物
        print("=== 2. 添加桌子障碍 (x=0.5, z=0.55) ===")
        node.add_obstacle()
        time.sleep(1.0)

        # 3. 附着工具
        print("=== 3. 附着工具 ===")
        node.attach_tool()
        time.sleep(2.0) 

        # 4. 自动规划
        print("=== 4. 规划目标：Elbow Up 吊车姿态 ===")
        target_joints_visual = [0.3, -0.38, 0.57, 0.95, 0.3, 0.0]
        
        trajectory = node.plan_path(target_joints_visual)
        
        if trajectory:
            node.execute_trajectory(trajectory)
        else:
            print("规划失败！请检查机械臂初始位置是否穿模。")

        # 5. 回到 Home
        print("=== 5. 返回 Home ===")
        node.send_direct_trajectory([0.0]*6)

        # 6. 清理
        print("=== 6. 移除物体 ===")
        node.cleanup_scene()
        time.sleep(2.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()