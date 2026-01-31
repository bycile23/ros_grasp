#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, PoseStamped
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import tf2_geometry_msgs 
from tf2_ros import TransformException
import time
import math
from tf_transformations import quaternion_from_euler
import threading
import sys

class GraspingDemo(Node):
    def __init__(self):
        super().__init__('grasping_demo_node')
        
        # 使用 ReentrantCallbackGroup 允许并发回调，防止死锁
        self.cb_group = ReentrantCallbackGroup()
        
        # 标志位
        self.mission_completed = False 
        self.robot_is_ready = False 
        self.stop_flag = False  # 【关键】退出标志位

        # ==========================================
        # 🔧【参数设置】
        # ==========================================
        self.GRIPPER_LENGTH = 0.155  # 爪长 (根据实际微调)
        self.APPROACH_DISTANCE = 0.05 # 接近距离
        self.SAFETY_Z_LIFT     = 0.20 # 抬起高度
        
        # 姿态：水平向前 (Pitch = 90度)
        self.grasp_roll  = 0.0                
        self.grasp_pitch = 1.5708             
        self.grasp_yaw   = 0.0                

        # 容差设置 (适当放宽以防止 "Goal aborted")
        self.position_tolerance = 0.02    
        self.orientation_tolerance = 0.1 
        # ==========================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Action Clients
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd', callback_group=self.cb_group)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb_group)
        
        # Vision Subscriber
        self.target_sub = self.create_subscription(PointStamped, '/vision/target_point', self.target_callback, 10, callback_group=self.cb_group)
        
        self.robot_base_frame = "base_link"
        self.planning_group = "minipulator" 

        # 启动初始化定时器
        self.init_timer = self.create_timer(1.0, self.delayed_init, callback_group=self.cb_group)

    def delayed_init(self):
        self.init_timer.cancel()
        if self.stop_flag: return

        self.get_logger().info("⏳ 等待 Action Server 连接...")
        
        # 等待服务，每秒检查一次 stop_flag
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            if self.stop_flag: return
            self.get_logger().info("... 等待夹爪服务")
            
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            if self.stop_flag: return
            self.get_logger().info("... 等待 MoveIt 服务")

        self.get_logger().info("✅ 服务就绪，开始复位...")
        # 启动复位线程
        threading.Thread(target=self.reset_robot, daemon=True).start()

    def reset_robot(self):
        if self.stop_flag: return
        self.get_logger().info("🔄 [复位中] 机械臂归位...")
        
        # 先张开爪子
        self.control_gripper(0.04) 
        
        # 回到初始姿态 (全零)
        if self.move_arm_to_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]):
             self.get_logger().info("✅ [复位完成] 机械臂已就绪！")
             self.robot_is_ready = True 
        else:
             self.get_logger().error("❌ [复位失败] 请检查 MoveIt 或控制器")

    def target_callback(self, msg):
        if self.mission_completed or not self.robot_is_ready or self.stop_flag: 
            return
        
        self.mission_completed = True # 锁定任务，只执行一次
        
        try:
            # TF 变换：将相机坐标系下的点转换到基座坐标系
            if not self.tf_buffer.can_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn("⚠️ 无法获取 TF 变换")
                self.mission_completed = False # 允许重试
                return

            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time())
            target_in_base = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.get_logger().info(f"🎯 收到目标 [X={target_in_base.point.x:.3f}]，启动执行线程...")
            
            # 启动执行线程 (Daemon 模式确保主程序退出时它也会退出)
            threading.Thread(target=self._execute_mission_thread, args=(target_in_base,), daemon=True).start()
            
        except Exception as ex:
            self.get_logger().error(f"TF 异常: {ex}")
            self.mission_completed = False

    def _execute_mission_thread(self, target_in_base):
        # 再次检查停止标志
        if self.stop_flag: return
        
        # 简单的延时，给系统一点反应时间
        time.sleep(1.0) 

        # === 📐 坐标计算 ===
        obj_x = target_in_base.point.x
        obj_y = target_in_base.point.y
        obj_z = target_in_base.point.z

        # 手腕目标 X = 物体 X - 爪长 - 预留距离
        wrist_target_x = obj_x - self.GRIPPER_LENGTH - self.APPROACH_DISTANCE
        # 手腕目标 Z = 物体 Z (通常需要微调，这里假设物体中心高度合适)
        wrist_target_z = obj_z 
        
        self.get_logger().info(f"🚀 执行序列: 物体X {obj_x:.3f} -> 手腕去X {wrist_target_x:.3f}")

        # 调用核心执行函数
        self.execute_grasp_sequence(wrist_target_x, obj_y, wrist_target_z)

    def control_gripper(self, position):
        """ 控制夹爪: 0.0=闭合, 0.04=张开 """
        if self.stop_flag: return
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 200.0 # 给足力气
        
        future = self.gripper_client.send_goal_async(goal)
        
        # 同步等待结果 (带超时)
        start_time = time.time()
        while not future.done():
            if self.stop_flag: return
            if time.time() - start_time > 5.0:
                self.get_logger().warn("⚠️ 夹爪响应超时")
                return
            time.sleep(0.05)
            
        time.sleep(0.5) # 物理稳定时间

    def _create_constraints(self, x, y, z):
        """ 创建笛卡尔目标约束 """
        p = PoseStamped()
        p.header.frame_id = self.robot_base_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        q = quaternion_from_euler(self.grasp_roll, self.grasp_pitch, self.grasp_yaw) 
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        pc = PositionConstraint()
        pc.header = p.header
        pc.link_name = "tool0"
        pc.constraint_region.primitive_poses.append(p.pose)
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.position_tolerance])) 
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = p.header
        oc.link_name = "tool0"
        oc.orientation = p.pose.orientation
        oc.absolute_x_axis_tolerance = self.orientation_tolerance 
        oc.absolute_y_axis_tolerance = self.orientation_tolerance
        oc.absolute_z_axis_tolerance = self.orientation_tolerance
        oc.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        return constraints

    def move_arm_to_joints(self, joint_values):
        if self.stop_flag: return False
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = self.robot_base_frame
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints_list = Constraints()
        for name, val in zip(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints_list.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints_list)
        
        self.get_logger().info(f"💪 关节运动: {joint_values}")
        return self._send_move_goal(goal_msg)

    def move_arm_to_pose(self, x, y, z, is_home=False): 
        if self.stop_flag: return False
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = self.robot_base_frame
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.group_name = self.planning_group
        
        # 笛卡尔目标约束
        goal_msg.request.goal_constraints.append(self._create_constraints(x, y, z))

        return self._send_move_goal(goal_msg)

    def _send_move_goal(self, goal_msg):
        """ 发送 MoveIt 目标并等待结果的通用函数 """
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        # 等待服务器接受
        while not send_goal_future.done(): 
            if self.stop_flag: return False
            time.sleep(0.05)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ 规划请求被拒绝")
            return False
            
        # 等待执行结果
        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done(): 
            if self.stop_flag: 
                # 尝试取消目标 (可选)
                # goal_handle.cancel_goal_async()
                return False
            time.sleep(0.05)
            
        result = get_result_future.result()
        if result.result.error_code.val == 1: # SUCCESS
            return True
        else:
            self.get_logger().error(f"❌ 运动失败 Error Code: {result.result.error_code.val}")
            return False

    def execute_grasp_sequence(self, x, y, z):
        if self.stop_flag: return
        self.get_logger().info(f"--- 启动抓取流程 ---")
        
        # 1. 【预备姿态】 
        # 这个姿态比较关键，要保证是一个“不会导致碰撞”且“离物体较近”的姿态
        # 建议使用类似 [0, -1.57, 1.57, -1.57, -1.57, 0] 这样的“拱门形”姿态
        ready_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.get_logger().info("--- 1. 关节预备 ---")
        if not self.move_arm_to_joints(ready_joints): return
        
        time.sleep(1.0) # 冷却一下
        if self.stop_flag: return

        # 2. 【对齐物体前方】
        self.get_logger().info("--- 2. 对齐物体 ---")
        if not self.move_arm_to_pose(x, y, z): return
        
        if self.stop_flag: return

        # 3. 【直线突刺】
        self.get_logger().info(f"--- 3. 直线突刺 ({self.APPROACH_DISTANCE}m) ---")
        # 这里的 x 已经是减去预留距离后的，所以要加上 approach_distance 才是物体表面
        final_grasp_x = x + self.APPROACH_DISTANCE
        if not self.move_arm_to_pose(final_grasp_x, y, z): return
        
        if self.stop_flag: return
        
        # 4. 【闭合夹爪】
        self.get_logger().info("--- 4. 抓取 ---")
        self.control_gripper(0.0) # 0.0 = 闭合
        time.sleep(1.0) 
        
        if self.stop_flag: return

        # 5. 【抬起】
        self.get_logger().info("--- 5. 抬起 ---")
        if not self.move_arm_to_pose(final_grasp_x, y, z + self.SAFETY_Z_LIFT): return
        
        # 6. 【回家】
        self.get_logger().info("--- 6. 回家 ---")
        # 保持夹爪闭合回家
        self.move_arm_to_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        
        self.get_logger().info("🎉 演示结束")

def main(args=None):
    rclpy.init(args=args)
    node = GraspingDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # 在守护线程中运行 executor，这样主线程退出时它也会自动退出
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # 主线程只是在等待 Ctrl+C
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 收到退出信号，正在停止...")
        node.stop_flag = True # 通知所有子线程停止
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # 不需要 join spin_thread，因为它是 daemon

if __name__ == '__main__':
    main()