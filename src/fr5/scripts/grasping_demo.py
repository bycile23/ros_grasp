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
        
        self.mission_completed = False 
        self.stop_flag = False 
        self.joint_state_received = False
        self.current_joint_state = None
        
        # MoveIt 规划组名称
        self.planning_group = "minipulator"

        # ==========================================
        # 🔧 抓取姿态与高度核心参数配置
        # ==========================================
        self.GRIPPER_LENGTH = 0.155   # 夹爪的物理长度
        self.HOVER_OFFSET   = 0.10    # 抓取前在目标正上方悬停的安全距离
        self.GRASP_DEPTH    = 0.14    # 抓取深度（根据目标物体高度精确测算，对准中间偏上）
        self.SAFETY_Z_LIFT  = 0.10    # 抓取完成后垂直抬起的安全高度
        self.X_OFFSET       = -0.118  # 夹爪中心点与末端法兰(tool0)的坐标系X轴偏置
        # ==========================================

        # TF2 坐标变换监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 夹爪控制器发布者 (Float64MultiArray 用于同时控制双指)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        
        # 订阅视觉识别发布的目标点与关节状态
        self.target_sub = self.create_subscription(PointStamped, '/vision/target_point', self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Cartesian 直线插补路径规划客户端
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/fr5_joint_trajectory_controller/joint_trajectory', 10)

        self.robot_base_frame = "base_link"
        self.end_effector_link = "tool0"

        # 启动初始化线程
        threading.Thread(target=self.init_sequence, daemon=True).start()

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True

    def spin_and_sleep(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok() and not self.stop_flag:
            time.sleep(0.05)

    def init_sequence(self):
        self.get_logger().info("⏳ 等待服务与关节数据就绪...")
        
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            if self.stop_flag: return
        while not self.joint_state_received:
            time.sleep(0.1)

        self.get_logger().info("✅ 服务就绪，执行初始复位...")
        self.control_gripper(0.0) # 张开爪子，准备抓取
        
        # 构造复位轨迹 (移动至观测姿态)
        msg = JointTrajectory()
        msg.joint_names = list(self.current_joint_state.name)
        
        start_point = JointTrajectoryPoint()
        start_point.positions = list(self.current_joint_state.position)
        start_point.time_from_start.sec = 0
        msg.points.append(start_point)

        end_point = JointTrajectoryPoint()
        # 默认的观测等待姿态
        target_positions = list(self.current_joint_state.position)
        target_positions[0:6] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        end_point.positions = target_positions
        end_point.time_from_start.sec = 2 
        msg.points.append(end_point)
        
        self.trajectory_pub.publish(msg)
        self.spin_and_sleep(2.5)
        
        self.get_logger().info("🎯 复位完成，等待视觉目标输入...")
        self.robot_is_ready = True

    def target_callback(self, msg):
        # 仅处理一次抓取任务
        if self.mission_completed or not getattr(self, 'robot_is_ready', False) or self.stop_flag: 
            return
        
        try:
            # 将视觉目标点转换至 base_link 坐标系
            if not self.tf_buffer.can_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                return
            transform = self.tf_buffer.lookup_transform(self.robot_base_frame, msg.header.frame_id, rclpy.time.Time())
            target_in_base = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            self.mission_completed = True 
            self.get_logger().info(f"📍 成功锁定目标: X={target_in_base.point.x:.3f}, Y={target_in_base.point.y:.3f}")
            
            # 开启异步线程执行抓取流
            threading.Thread(target=self.execute_grasp_sequence, args=(target_in_base,), daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"❌ TF 坐标变换失败: {e}")

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.robot_base_frame, self.end_effector_link, rclpy.time.Time())
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation 
            return pose
        except Exception as e:
            self.get_logger().error(f"无法获取当前末端位姿: {e}")
            return None

    def move_straight_line(self, target_x, target_y, target_z, duration=2.0):
        if self.stop_flag: return False
        
        start_pose = self.get_current_pose()
        if not start_pose: return False

        target_pose = copy.deepcopy(start_pose)
        target_pose.position.x = target_x + self.X_OFFSET
        target_pose.position.y = target_y 
        target_pose.position.z = target_z

        # 请求笛卡尔空间直线插补
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
            self.get_logger().error(f"❌ 直线规划失败，仅完成 {res.fraction*100:.1f}%，为了安全中止任务")
            return False

        trajectory = res.solution.joint_trajectory
        time_per_point = duration / len(trajectory.points)
        for i, point in enumerate(trajectory.points):
            t = (i + 1) * time_per_point
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            
        self.trajectory_pub.publish(trajectory)
        self.spin_and_sleep(duration + 0.2)
        return True

    def control_gripper(self, position):
        if self.stop_flag: return
        msg = Float64MultiArray()
        # 双指同步控制
        msg.data = [position, position]
        self.gripper_pub.publish(msg)

    def execute_grasp_sequence(self, target_in_base):
        obj_x = target_in_base.point.x
        obj_y = target_in_base.point.y
        obj_z = target_in_base.point.z

        hover_z = obj_z + self.GRIPPER_LENGTH + self.HOVER_OFFSET
        grasp_z = obj_z + self.GRIPPER_LENGTH - self.GRASP_DEPTH

        self.get_logger().info(f"=== 🚀 启动【视觉引导定位+直线插补】抓取流程 ===")
        
        self.get_logger().info(">>> 1. 移动至目标正上方悬停点")
        if not self.move_straight_line(obj_x, obj_y, hover_z, duration=3.0): return
        
        self.get_logger().info(">>> 2. 垂直下降至计算抓取高度")
        if not self.move_straight_line(obj_x, obj_y, grasp_z, duration=1.5): return
        
        self.get_logger().info(">>> 3. 闭合夹爪执行抓取")
        # 🌟 核心物理参数：夹爪全开距 7.8cm，易拉罐直径 6.0cm。
        # 理论单边刚好贴合距离为 (7.8-6.0)/2 = 0.9cm (0.009m)。
        # 设定 0.0095 产生 0.5mm 的微小挤压，防止 Gazebo 底层产生排斥力滑移。
        self.control_gripper(0.0095)  
        time.sleep(1.5) # 给予物理引擎和吸附插件充足的时间达到稳态
        
        self.get_logger().info(">>> 4. 抓取成功，平稳垂直抬起")
        if not self.move_straight_line(obj_x, obj_y, hover_z + self.SAFETY_Z_LIFT, duration=2.0): return
        
        self.get_logger().info("🎉 恭喜！流水线抓取任务完美结束！")

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