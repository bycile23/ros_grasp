#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Float64MultiArray  
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
        
        # --- 节点状态标志位 ---
        self.mission_completed = False 
        self.stop_flag = False 
        self.robot_is_ready = False
        self.joint_state_received = False
        self.current_joint_state = None
        
        # --- 核心参数配置 ---
        self.planning_group = "minipulator"
        self.robot_base_frame = "base_link"
        self.end_effector_link = "tool0"

        # 抓取姿态与高度配置
        self.GRIPPER_LENGTH = 0.155   
        self.HOVER_OFFSET   = 0.10    
        self.GRASP_DEPTH    = 0.14    # 抓取深度
        self.SAFETY_Z_LIFT  = 0.10    
        self.X_OFFSET       = -0.118  # 末端法兰的X轴偏置
        self.GRIPPER_CLOSE  = 0.0095  # 夹爪闭合参数
        self.GRIPPER_OPEN   = 0.0     # 夹爪张开参数
        
        # 机械臂初始/结束复位位置 (全0)
        self.INIT_JOINTS = {f'joint{i}': 0.0 for i in range(1, 7)}

        # --- ROS 2 通信接口声明 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/fr5_joint_trajectory_controller/joint_trajectory', 10)
        
        self.target_sub = self.create_subscription(PointStamped, '/vision/target_point', self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        # 启动后台初始化线程
        threading.Thread(target=self._init_sequence, daemon=True).start()

    # =====================================================================
    # ROS 2 回调函数
    # =====================================================================
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        self.joint_state_received = True

    def target_callback(self, msg: PointStamped):
        if self.mission_completed or not self.robot_is_ready or self.stop_flag: 
            return
            
        try:
            # 将视觉目标点转换到机器人基坐标系下
            if not self.tf_buffer.can_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                return
            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time())
            target_in_base = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.mission_completed = True 
            self.get_logger().info(f"📍 成功锁定视觉目标: X={target_in_base.point.x:.3f}, Y={target_in_base.point.y:.3f}")
            
            # 启动抓取流水线线程
            threading.Thread(target=self.execute_grasp_sequence, args=(target_in_base,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"❌ TF 坐标变换失败: {e}")

    # =====================================================================
    # 主流程与流水线控制
    # =====================================================================
    def _init_sequence(self):
        """后台初始化序列，确保服务和数据就绪后执行复位"""
        self.get_logger().info("⏳ 等待服务与关节数据就绪...")
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            if self.stop_flag: return
        while not self.joint_state_received:
            time.sleep(0.1)

        self.get_logger().info("✅ 服务就绪，执行初始复位...")
        self.control_gripper(self.GRIPPER_OPEN) 
        self.move_joints_dict(self.INIT_JOINTS, duration=2.5)
        
        self.robot_is_ready = True
        self.get_logger().info("🎯 复位完成，等待视觉目标输入...")

    def execute_grasp_sequence(self, target_in_base: PointStamped):
        """执行跨空间 Pick & Place 流水线"""
        obj_x = target_in_base.point.x
        obj_y = target_in_base.point.y
        obj_z = target_in_base.point.z

        # 高度关键点计算
        hover_z = obj_z + self.GRIPPER_LENGTH + self.HOVER_OFFSET
        grasp_z = obj_z + self.GRIPPER_LENGTH - self.GRASP_DEPTH
        safe_z  = hover_z + self.SAFETY_Z_LIFT

        self.get_logger().info("=== 🚀 启动【跨空间 Pick & Place】流水线 ===")
        
        # ------------------ 阶段 1: 抓取 (Pick) ------------------
        self.get_logger().info(">>> 1. 移动至目标正上方悬停点")
        if not self.move_straight_line(obj_x, obj_y, hover_z, duration=3.0): return
        
        self.get_logger().info(">>> 2. 垂直下降至计算抓取高度")
        if not self.move_vertical(grasp_z, duration=1.5): return
        
        self.get_logger().info(">>> 3. 闭合夹爪执行抓取")
        self.control_gripper(self.GRIPPER_CLOSE)  
        time.sleep(1.5) # 等待物理引擎稳定
        
        self.get_logger().info(">>> 4. 抓取成功，平稳垂直抬起")
        if not self.move_vertical(safe_z, duration=2.0): return
        
        # ------------------ 阶段 2: 转移 (Transfer) ------------------
        self.get_logger().info(">>> 5. 旋转基座(Joint1)平滑转移至目标工作台")
        current_j1 = self._get_joint_position('joint1')
        place_j1 = current_j1 - 1.5708  # 顺时针旋转90度
        
        if not self.move_joints_dict({'joint1': place_j1}, duration=3.0): return
        self._wait_for_joint_arrival('joint1', place_j1) # 物理安检门：严禁抢跑
        
        # ------------------ 阶段 3: 放置 (Place) ------------------
        self.get_logger().info(">>> 6. 纯垂直下降至新桌面高度")
        # 锁定旋转后完美的 XY 姿态，纯垂直下降，保留 1.5cm 缓冲防止物理弹跳
        if not self.move_vertical(grasp_z + 0.015, duration=2.0): return

        self.get_logger().info(">>> 7. 张开夹爪，平稳释放物体")
        self.control_gripper(self.GRIPPER_OPEN)
        time.sleep(1.0) 

        self.get_logger().info(">>> 8. 释放完成，机械臂安全抬起")
        if not self.move_vertical(safe_z, duration=2.0): return

        # ------------------ 阶段 4: 复位 (Return to Home) ------------------
        self.get_logger().info(">>> 9. 任务圆满完成，机械臂优雅退回原位")
        if not self.move_joints_dict(self.INIT_JOINTS, duration=3.0): return

        self.get_logger().info("🎉 恭喜！跨空间全自动 Pick & Place 完美通关！")

    # =====================================================================
    # 运动控制核心库
    # =====================================================================
    def move_joints_dict(self, target_dict: dict, duration: float = 3.0) -> bool:
        """多关节绝对位置同步运动"""
        if self.stop_flag: return False
        
        arm_joints = [f'joint{i}' for i in range(1, 7)]
        msg = JointTrajectory()
        msg.joint_names = arm_joints
        
        # 构建起点
        start_point = JointTrajectoryPoint()
        start_positions = [self._get_joint_position(name) for name in arm_joints]
        start_point.positions = start_positions
        start_point.time_from_start.sec = 0
        msg.points.append(start_point)

        # 构建终点
        end_point = JointTrajectoryPoint()
        end_positions = list(start_positions)
        for i, name in enumerate(arm_joints):
            if name in target_dict:
                end_positions[i] = target_dict[name]
        
        end_point.positions = end_positions
        end_point.time_from_start.sec = int(duration)
        end_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points.append(end_point)
        
        self.trajectory_pub.publish(msg)
        self._spin_and_sleep(duration + 0.2)
        return True

    def move_vertical(self, target_z: float, duration: float = 2.0) -> bool:
        """保持当前XY位置，仅在Z轴方向垂直运动"""
        target_pose = self._get_current_pose()
        if not target_pose: return False
        target_pose.position.z = target_z 
        return self._execute_cartesian_path(target_pose, duration)

    def move_straight_line(self, target_x: float, target_y: float, target_z: float, duration: float = 2.0) -> bool:
        """空间直线运动到指定XYZ绝对坐标"""
        target_pose = self._get_current_pose()
        if not target_pose: return False
        
        target_pose.position.x = target_x + self.X_OFFSET
        target_pose.position.y = target_y 
        target_pose.position.z = target_z
        return self._execute_cartesian_path(target_pose, duration)

    def control_gripper(self, position: float):
        """控制夹爪开合"""
        if self.stop_flag: return
        msg = Float64MultiArray()
        msg.data = [position, position]
        self.gripper_pub.publish(msg)

    # =====================================================================
    # 私有辅助方法 (Protected Utils)
    # =====================================================================
    def _execute_cartesian_path(self, target_pose: Pose, duration: float) -> bool:
        """底层方法：请求MoveIt进行笛卡尔直线规划并执行"""
        if self.stop_flag: return False
        
        req = GetCartesianPath.Request()
        req.header.frame_id = self.robot_base_frame
        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = self.planning_group
        req.link_name = self.end_effector_link
        req.waypoints = [target_pose]
        req.max_step = 0.01 
        req.jump_threshold = 0.0

        future = self.cartesian_client.call_async(req)
        while not future.done():
            if self.stop_flag: return False
            time.sleep(0.05)
            
        res = future.result()
        if res.fraction < 0.9:
            self.get_logger().error(f"❌ 直线规划失败，仅完成 {res.fraction*100:.1f}%")
            return False

        self._filter_and_execute_trajectory(res.solution.joint_trajectory, duration)
        return True

    def _filter_and_execute_trajectory(self, raw_trajectory: JointTrajectory, duration: float):
        """安检门：过滤轨迹中的夹爪等无关关节，提取纯机械臂关节信息进行发布"""
        if self.stop_flag: return
        
        arm_joints = [f'joint{i}' for i in range(1, 7)]
        filtered_msg = JointTrajectory()
        filtered_msg.joint_names = arm_joints
        
        indices = []
        for name in arm_joints:
            if name in raw_trajectory.joint_names:
                indices.append(raw_trajectory.joint_names.index(name))
            else:
                self.get_logger().error(f"❌ 轨迹缺失关键关节: {name}")
                return
                
        time_per_point = duration / len(raw_trajectory.points)
        
        for i, raw_point in enumerate(raw_trajectory.points):
            pt = JointTrajectoryPoint()
            pt.positions = [raw_point.positions[idx] for idx in indices]
            t = (i + 1) * time_per_point
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            filtered_msg.points.append(pt)
            
        self.trajectory_pub.publish(filtered_msg)
        self._spin_and_sleep(duration + 0.2)

    def _wait_for_joint_arrival(self, joint_name: str, target_angle: float, tolerance: float = 0.05):
        """物理安检门：阻塞线程，直到 Gazebo 中的真实关节抵达目标角度"""
        self.get_logger().info(f"⏳ 正在等待物理引擎：机械臂 {joint_name} 旋转到位中...")
        while rclpy.ok() and not self.stop_flag:
            current_angle = self._get_joint_position(joint_name)
            if abs(current_angle - target_angle) < tolerance:
                break
            time.sleep(0.1)
        self.get_logger().info("✅ 物理落点已精准对齐，准许执行后续动作！")

    def _get_joint_position(self, joint_name: str) -> float:
        """从当前关节状态中快速获取指定关节的位置"""
        if not self.current_joint_state or joint_name not in self.current_joint_state.name:
            return 0.0
        idx = self.current_joint_state.name.index(joint_name)
        return self.current_joint_state.position[idx]

    def _get_current_pose(self) -> Pose:
        """获取末端执行器的当前TF位姿"""
        try:
            t = self.tf_buffer.lookup_transform(self.robot_base_frame, self.end_effector_link, rclpy.time.Time())
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation 
            return pose
        except Exception:
            return None

    def _spin_and_sleep(self, duration: float):
        """带中断保护的睡眠函数"""
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok() and not self.stop_flag:
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = GraspingDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 收到键盘退出信号...")
        node.stop_flag = True 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()