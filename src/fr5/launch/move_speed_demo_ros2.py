#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time

# ROS 消息
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class MoveSpeedDemo(Node):
    def __init__(self):
        super().__init__('move_speed_demo_node')
        
        self.group_name = "minipulator" 
        self.base_frame = "world"
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 1. 服务与发布
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/fr5_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # 2. 状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.current_joint_state = None

        # 3. 等待服务
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt 服务...')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def spin_and_sleep(self, duration_sec):
        """防止闪烁的呼吸等待"""
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_joint_state(self):
        while self.current_joint_state is None:
            self.get_logger().info("等待关节数据...")
            rclpy.spin_once(self, timeout_sec=0.1)

    def plan_path(self, target_joints):
        """规划路径"""
        if self.current_joint_state is None: return None

        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = self.base_frame
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        
        req.start_state.joint_state = self.current_joint_state
        req.group_name = self.group_name
        req.planner_id = "RRTConnectkConfigDefault"
        req.num_planning_attempts = 1 
        req.allowed_planning_time = 2.0
        
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        constraints = Constraints() #创建约束
        for i, angle in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = angle
            jc.tolerance_above = 0.001
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
            self.get_logger().error("规划失败")
            return None

    def scale_trajectory_speed(self, trajectory, scale):
        """变速处理"""
        if not trajectory: return None
        new_traj = JointTrajectory()
        new_traj.header = trajectory.header
        new_traj.joint_names = trajectory.joint_names

        for old_point in trajectory.points:
            new_point = JointTrajectoryPoint()
            new_point.positions = old_point.positions
            
            time_scale = 1.0 / scale
            old_sec = old_point.time_from_start.sec
            old_nanosec = old_point.time_from_start.nanosec
            total_old_sec = old_sec + (old_nanosec * 1e-9)
            total_new_sec = total_old_sec * time_scale
            
            new_point.time_from_start.sec = int(total_new_sec)
            new_point.time_from_start.nanosec = int((total_new_sec - int(total_new_sec)) * 1e9)
            
            if old_point.velocities:
                new_point.velocities = [v * scale for v in old_point.velocities]
            if old_point.accelerations:
                new_point.accelerations = [a * scale * scale for a in old_point.accelerations]
                
            new_traj.points.append(new_point)
        return new_traj

    def execute_trajectory(self, trajectory):
        """执行轨迹"""
        if not trajectory: return

        # 增加缓冲时间，防止瞬移
        now = self.get_clock().now()
        delay = Duration(seconds=0.5)
        trajectory.header.stamp = (now + delay).to_msg()
        
        self.trajectory_publisher.publish(trajectory)
        
        last_point = trajectory.points[-1]
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        
        self.get_logger().info(f"执行中... 耗时: {duration:.2f} 秒")
        self.spin_and_sleep(duration + 0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MoveSpeedDemo()
    node.wait_for_joint_state()
    node.spin_and_sleep(1.0)

    try:
        target_joints = [0.39, -0.67, -0.37, 0.0, 1.05, 0.45]
        home_joints = [0.0] * 6

        # 流程：去(快) -> 回(快) -> 去(慢) -> 回(快)

        # --- 1. 去程 (快) ---
        print("\n=== 1/4 去程: 快速 (Scale 1.0) ===")
        traj_to_target = node.plan_path(target_joints)
        if traj_to_target:
            node.execute_trajectory(traj_to_target)
        
        # --- 2. 回程 (快) ---
        print("\n=== 2/4 回程: 快速归位 ===")
        traj_to_home = node.plan_path(home_joints)
        if traj_to_home:
            node.execute_trajectory(traj_to_home)
        
        node.spin_and_sleep(0.5)

        # --- 3. 去程 (慢) ---
        print("\n=== 3/4 去程: 慢速执行 (Scale 0.4) ===")
        # 复用第一次规划的路径并减速
        # 提示：如果你想让最后一次回程也是慢速，可以在步骤4里用同样的方法
        slow_traj = node.scale_trajectory_speed(traj_to_target, 0.4)
        node.execute_trajectory(slow_traj)

        node.spin_and_sleep(0.5)

        # --- 4. 回程 (快 - 结束动作) ---
        print("\n=== 4/4 回程: 结束演示，快速归位 ===")
        # 此时机械臂在目标点，我们重新规划一条回 Home 的路径
        traj_final_home = node.plan_path(home_joints)
        if traj_final_home:
            node.execute_trajectory(traj_final_home)
        
        print("\n=== 演示圆满结束 ===")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()