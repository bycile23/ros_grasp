#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import copy

# ROS 消息类型
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ContinuousTrajectoryDemo(Node):
    def __init__(self):
        super().__init__('move_continuous_trajectory_demo')
        
        # === 配置 ===
        self.group_name = "minipulator"
        self.base_frame = "world"
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 1. 规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 2. 轨迹发布者
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/fr5_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # 3. 关节状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.current_joint_state = None

        # 等待服务
        self.get_logger().info('正在连接 MoveIt 规划服务...')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务上线...')
        self.get_logger().info('MoveIt 服务连接成功！')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def spin_and_sleep(self, duration_sec):
        """防止 RViz 闪烁的呼吸等待"""
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_joint_state(self):
        while self.current_joint_state is None:
            self.get_logger().info("等待关节状态数据...")
            rclpy.spin_once(self, timeout_sec=0.1)

    def plan_path(self, start_joint_state, target_joints):
        """
        通用规划函数
        :param start_joint_state:起始关节状态 (JointState msg)，如果为 None 则使用当前状态
        :param target_joints: 目标关节角度列表
        """
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = self.base_frame
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        
        # 设置起点
        req.start_state = RobotState()
        if start_joint_state is None:
            # 如果没指定起点，就用当前这一刻的真实状态
            if self.current_joint_state is None: return None
            req.start_state.joint_state = self.current_joint_state
        else:
            # 使用指定的起点 (用于第二段轨迹规划)
            req.start_state.joint_state = start_joint_state

        req.group_name = self.group_name
        req.planner_id = "RRTConnectkConfigDefault"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        
        # 速度缩放 (对应 C++ 中的 velScale)
        req.max_velocity_scaling_factor = 0.8
        req.max_acceleration_scaling_factor = 0.8

        # 设置目标约束
        constraints = Constraints()
        for i, angle in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = angle
            jc.tolerance_above = 0.001 # C++ 代码中的 setGoalJointTolerance(0.001)
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        service_req = GetMotionPlan.Request()
        service_req.motion_plan_request = req
        
        future = self.plan_client.call_async(service_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.motion_plan_response.error_code.val == 1:
            return res.motion_plan_response.trajectory.joint_trajectory
        else:
            self.get_logger().error(f"规划失败: {res.motion_plan_response.error_code.val}")
            return None

    def stitch_trajectories(self, traj1, traj2):
        """
        将两条轨迹缝合在一起
        对应 C++ 中的 trajectory.joint_trajectory.points.push_back(...)
        """
        if not traj1 or not traj2:
            return None

        combined_traj = JointTrajectory()
        combined_traj.header = traj1.header
        combined_traj.joint_names = traj1.joint_names

        # 1. 加入第一条轨迹的所有点
        for p in traj1.points:
            combined_traj.points.append(p)

        # 获取第一条轨迹结束的时间 (用于偏移第二条轨迹的时间)
        last_point = traj1.points[-1]
        offset_sec = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

        # 2. 加入第二条轨迹的点
        # 注意：通常要跳过 traj2 的第一个点，因为它和 traj1 的最后一个点是重合的
        # C++代码里是: for (size_t j = 1; ...)
        for i, p in enumerate(traj2.points):
            if i == 0:
                continue # 跳过重叠点
            
            new_point = JointTrajectoryPoint()
            new_point.positions = p.positions
            new_point.velocities = p.velocities
            new_point.accelerations = p.accelerations
            
            # 关键：时间偏移计算
            # T_new = T_traj2 + T_end_traj1
            current_p_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            new_total_sec = current_p_sec + offset_sec
            
            new_point.time_from_start.sec = int(new_total_sec)
            new_point.time_from_start.nanosec = int((new_total_sec - int(new_total_sec)) * 1e9)
            
            combined_traj.points.append(new_point)
            
        return combined_traj

    def trajectory_point_to_joint_state(self, point):
        """辅助工具：将轨迹点转为 JointState，用于作为下一次规划的起点"""
        js = JointState()
        js.name = self.joint_names
        js.position = point.positions
        # 必须给 velocities 赋值，否则 MoveIt 可能会报错
        js.velocity = [0.0] * 6 
        return js

    def execute_trajectory(self, trajectory):
        if not trajectory: return
        
        # 时间戳处理
        now = self.get_clock().now()
        delay = Duration(seconds=0.2)
        trajectory.header.stamp = (now + delay).to_msg()
        
        self.trajectory_publisher.publish(trajectory)
        
        last_point = trajectory.points[-1]
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        
        self.get_logger().info(f"发送连续轨迹，总时长: {duration:.2f} 秒")
        self.spin_and_sleep(duration + 0.5)

    def go_home(self):
        self.get_logger().info("回到 Home 点...")
        home_joints = [0.0] * 6
        traj = self.plan_path(None, home_joints)
        if traj:
            self.execute_trajectory(traj)

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousTrajectoryDemo()
    node.wait_for_joint_state()
    node.spin_and_sleep(1.0)

    try:
        # 1. 先回到 Home
        node.go_home()
        node.spin_and_sleep(0.5)

        # 准备数据
        # 获取当前位置作为 P0
        if node.current_joint_state is None:
            print("无法获取当前位置")
            return
        
        start_joints = list(node.current_joint_state.position) # P0

        # === 设定目标点 P1 ===
        # C++: joint_group_positions[0] = -0.6
        target_p1 = copy.deepcopy(start_joints)
        target_p1[0] = -0.6

        # === 设定目标点 P2 ===
        # C++: joint_group_positions[0] = -1.2; joint_group_positions[1] = -0.5;
        target_p2 = copy.deepcopy(start_joints)
        target_p2[0] = -1.2
        target_p2[1] = -0.5

        print("\n=== 开始计算连续轨迹 ===")

        # 2. 计算第一条轨迹 (P0 -> P1)
        print("1. 规划路径 A (Home -> P1)...")
        traj1 = node.plan_path(None, target_p1) # None 表示从当前真实位置开始
        
        if traj1:
            # 3. 准备第二条轨迹的起点
            # 我们必须拿 traj1 的最后一个点作为 traj2 的起点
            # 这样规划器才知道机器人未来会在哪里
            traj1_end_point = traj1.points[-1]
            virtual_start_state = node.trajectory_point_to_joint_state(traj1_end_point)

            # 4. 计算第二条轨迹 (P1 -> P2)
            print("2. 规划路径 B (P1 -> P2)...")
            traj2 = node.plan_path(virtual_start_state, target_p2)

            if traj2:
                # 5. 缝合轨迹
                print("3. 缝合轨迹 (Stitching)...")
                full_trajectory = node.stitch_trajectories(traj1, traj2)
                
                # 6. 执行
                print(">>> 执行连续运动 <<<")
                node.execute_trajectory(full_trajectory)
            else:
                print("路径 B 规划失败")
        else:
            print("路径 A 规划失败")

        # 7. 结束回 Home
        print("\n=== 演示结束，返回 Home ===")
        node.go_home()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()