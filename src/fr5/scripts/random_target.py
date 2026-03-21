#!/usr/bin/env python3
import os
import random

def main():
    print("\n" + "★"*50)
    print("🎲 [智能视觉自证] 正在为易拉罐生成随机盲盒落点...")
    
    # ---------------------------------------------------------
    # 🌟 核心修改：限定在机械臂最佳工作区，且严格保留 2 位小数 (厘米级)
    # X范围：0.66 到 0.78
    # Y范围：-0.10 到 0.10 (桌子中轴线左右 10 厘米内，防止掉下桌子)
    # ---------------------------------------------------------
    rand_x = round(random.uniform(0.66, 0.78), 2)
    rand_y = round(random.uniform(-0.10, 0.10), 2)
    
    # 组装 Gazebo 服务调用命令，强行改写 green_can 的绝对坐标
    cmd = f'ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState ' \
          f'"{{state: {{name: \'green_can\', reference_frame: \'world\', ' \
          f'pose: {{position: {{x: {rand_x}, y: {rand_y}, z: 0.35}}, ' \
          f'orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}}}" > /dev/null 2>&1'
    
    # 执行瞬间传送
    result = os.system(cmd)
    
    if result == 0:
        print("✅ 传送完成！请查看 Gazebo 画面，易拉罐已经瞬移了！")
        print("\n" + "🔥"*50)
        print("                 【 核 对 专 用 面 板 】")
        print(f"      👉 真实的物理 X 坐标 (前后):   {rand_x}")
        print(f"      👉 真实的物理 Y 坐标 (左右):   {rand_y}")
        print("🔥"*50 + "\n")
        print("👀 请立刻运行 vision_manager，核对它算出的数字！")
        print("   (重点看 X 和 Y 有没有搞反，正负号有没有对不上)")
    else:
        print("⚠️ 传送失败！请确保你的 Gazebo 仿真环境已经完全启动。")

if __name__ == '__main__':
    main()