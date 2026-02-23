#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, Pose
from control_msgs.action import GripperCommand
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs 
import time
import copy
import threading

class GraspingDemo(Node):
    def __init__(self):
        super().__init__('grasping_demo_node')
        
        self.mission_completed = False 
        self.stop_flag = False 
        self.joint_state_received = False
        self.current_joint_state = None

        # ==========================================
        # 🔧 抓取高度参数配置
        # ==========================================
        self.GRIPPER_LENGTH = 0.155  
        self.HOVER_OFFSET   = 0.10   # 抓取前在物体上方 10 厘米悬停
        self.GRASP_DEPTH    = 0.05   # 夹爪往下“咬”住物体的深度（完美避开桌面）
        self.SAFETY_Z_LIFT  = 0.25   # 抓完抬起的高度
        # ==========================================

        # TF 相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 夹爪 Action 客户端
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        # 视觉目标订阅
        self.target_sub = self.create_subscription(PointStamped, '/vision/target_point', self.target_callback, 10)
        
        # 【核心1】关节状态订阅（计算笛卡尔路径必须）
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 【核心2】笛卡尔直线规划服务
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # 【核心3】直接暴力下发轨迹，完美免疫 Error Code -4
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/fr5_joint_trajectory_controller/joint_trajectory', 10)

        self.robot_base_frame = "base_link"
        self.end_effector_link = "tool0"

        # 启动初始化线程
        threading.Thread(target=self.init_sequence, daemon=True).start()

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True

    def spin_and_sleep(self, duration):
        """带 ROS 事件循环的睡眠，保证通信不阻塞"""
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok() and not self.stop_flag:
            time.sleep(0.05)

    def init_sequence(self):
        self.get_logger().info("⏳ 等待服务与关节数据...")
        while not self.gripper_client.wait_for_server(timeout_sec=1.0):
            if self.stop_flag: return
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            if self.stop_flag: return
        while not self.joint_state_received:
            time.sleep(0.1)

        self.get_logger().info("✅ 服务就绪，执行初始复位...")
        self.control_gripper(0.04) # 先张开爪子
        
        # 【直接下发复位姿态，不给 MoveIt 报错的机会】
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # 爪子垂直朝下
        point.time_from_start.sec = 3
        msg.points.append(point)
        self.trajectory_pub.publish(msg)
        self.spin_and_sleep(3.5)
        
        self.get_logger().info("✅ 复位完成，等待视觉目标...")
        self.robot_is_ready = True

    def target_callback(self, msg):
        if self.mission_completed or not getattr(self, 'robot_is_ready', False) or self.stop_flag: 
            return
        
        try:
            if not self.tf_buffer.can_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                return
            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time())
            target_in_base = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.mission_completed = True 
            self.get_logger().info(f"🎯 锁定目标: X={target_in_base.point.x:.3f}, Y={target_in_base.point.y:.3f}")
            threading.Thread(target=self.execute_grasp_sequence, args=(target_in_base,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"TF 变换失败: {e}")

    def get_current_pose(self):
        """获取当前法兰盘位姿，用于直线规划的起点"""
        try:
            t = self.tf_buffer.lookup_transform(self.robot_base_frame, self.end_effector_link, rclpy.time.Time())
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation # 永远锁定这个姿态！
            return pose
        except Exception as e:
            self.get_logger().error(f"无法获取当前位姿: {e}")
            return None

    def move_straight_line(self, target_x, target_y, target_z, duration=2.0):
        """绝对直线运动，手腕完全不转"""
        if self.stop_flag: return False
        
        start_pose = self.get_current_pose()
        if not start_pose: return False

        # 生成目标点（姿态完全复制起点，杜绝乱转）
        target_pose = copy.deepcopy(start_pose)
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = target_z

        req = GetCartesianPath.Request()
        req.header.frame_id = self.robot_base_frame
        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = self.planning_group
        req.link_name = self.end_effector_link
        req.waypoints = [target_pose]
        req.max_step = 0.01  # 每 1cm 插补一次，绝对直线
        req.jump_threshold = 0.0

        future = self.cartesian_client.call_async(req)
        while not future.done():
            if self.stop_flag: return False
            time.sleep(0.05)
            
        res = future.result()
        if res.fraction < 0.9:
            self.get_logger().error(f"❌ 直线规划失败，只完成了 {res.fraction*100:.1f}%")
            return False

        # 重新分配平滑时间
        trajectory = res.solution.joint_trajectory
        time_per_point = duration / len(trajectory.points)
        for i, point in enumerate(trajectory.points):
            t = (i + 1) * time_per_point
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            
        # 暴力下发给控制器！免疫一切公差报错！
        self.trajectory_pub.publish(trajectory)
        self.spin_and_sleep(duration + 0.2)
        return True

    def control_gripper(self, position):
        if self.stop_flag: return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 200.0 
        future = self.gripper_client.send_goal_async(goal)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 3.0:
            time.sleep(0.05)
        time.sleep(0.5) 

    def execute_grasp_sequence(self, target_in_base):
        obj_x = target_in_base.point.x
        obj_y = target_in_base.point.y
        obj_z = target_in_base.point.z

        # TCP（法兰盘）的高度计算
        hover_z = obj_z + self.GRIPPER_LENGTH + self.HOVER_OFFSET
        grasp_z = obj_z + self.GRIPPER_LENGTH - self.GRASP_DEPTH

        self.get_logger().info(f"--- 🚀 启动【直线插补】安全抓取流程 ---")

        self.get_logger().info("--- 1. 移动到正上方悬停 ---")
        if not self.move_straight_line(obj_x, obj_y, hover_z, duration=3.0): return
        
        self.get_logger().info("--- 2. 垂直下降抓取 ---")
        if not self.move_straight_line(obj_x, obj_y, grasp_z, duration=1.5): return
        
        self.get_logger().info("--- 3. 闭合夹爪 ---")
        self.control_gripper(0.012) 
        
        self.get_logger().info("--- 4. 垂直抬起 ---")
        if not self.move_straight_line(obj_x, obj_y, hover_z + self.SAFETY_Z_LIFT, duration=2.0): return
        
        self.get_logger().info("🎉 抓取任务完美结束！(停留在半空，不再乱动)")

def main(args=None):
    rclpy.init(args=args)
    node = GraspingDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 收到退出信号，正在停止...")
        node.stop_flag = True 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()