import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class VisionManager(Node):
    def __init__(self):
        super().__init__('vision_manager')
        self.bridge = CvBridge()

        # ==========================================
        # 1. 物理环境与校准参数
        # ==========================================
        self.table_length = 0.3    # 桌子物理长度 (米)
        self.table_breadth = 0.3   # 桌子物理宽度 (米)
        
        # [核心校准参数] 摄像头的"等效"世界坐标
        # 注意：此处的值 (0.528, 0.107) 包含了"物理安装位置" + "视觉系统误差补偿"
        # 实际物理物体在 X=0.5, Y=0.0，通过调整此参数消除了 dx=0.028, dy=0.107 的系统误差
        self.camera_x = 0.528  
        self.camera_y = 0.107  

        # ==========================================
        # 2. 图像参数
        # ==========================================
        self.img_width = 640
        self.img_height = 480
        self.img_centre_x = self.img_width / 2
        self.img_centre_y = self.img_height / 2

        # 像素/米 比例尺 (在 detect_table 中动态计算)
        self.pixels_permm_x = 0.0
        self.pixels_permm_y = 0.0
        self.last_print_time = 0

        # ==========================================
        # 3. 通信配置
        # ==========================================
        self.image_sub = self.create_subscription(
            Image,
            '/narrow_stereo/image_raw', 
            self.image_callback,
            10
        )
        # 调试话题发布
        self.table_detect_pub = self.create_publisher(Image, '/vision/table_debug', 10)
        self.object_detect_pub = self.create_publisher(Image, '/vision/object_debug', 10)
        self.mask_debug_pub = self.create_publisher(Image, '/vision/mask_debug', 10)

        self.get_logger().info(f"🚀 视觉节点已启动 | 已加载校准参数: X={self.camera_x}, Y={self.camera_y}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # 1. 检测桌子 (用于确立坐标系比例尺)
        table_rect = self.detect_table(cv_image.copy())

        # 2. 检测物体 (基于颜色分割)
        if table_rect is not None:
            obj_pixel_x, obj_pixel_y = self.detect_2d_object(cv_image.copy(), table_rect)

            # 3. 坐标转换与输出
            if obj_pixel_x is not None:
                # 3.1 像素坐标 -> 相对米 (相对于图像中心)
                rel_x, rel_y = self.pixels_to_meters(obj_pixel_x, obj_pixel_y)
                
                # 3.2 相对米 -> 绝对世界坐标 (Gazebo/Base_link Frame)
                abs_x, abs_y = self.calculate_world_pose(rel_x, rel_y)

                # 3.3 打印日志 (1Hz 频率)
                current_time = time.time()
                if current_time - self.last_print_time > 1.0: 
                    self.print_debug_info(rel_x, rel_y, abs_x, abs_y)
                    self.last_print_time = current_time

    def calculate_world_pose(self, cam_rel_x, cam_rel_y):
        """
        [核心逻辑] 将相机相对坐标转换为机器人世界坐标
        输入: 
            cam_rel_x: 图像水平方向偏差 (米)
            cam_rel_y: 图像垂直方向偏差 (米)
        输出: 
            world_x, world_y
        """
        
        # --- 坐标系映射说明 (垂直向下安装) ---
        # 图像坐标系 -> 世界坐标系
        # Image Y (垂直像素) -> World X (前后距离)
        # Image X (水平像素) -> World Y (左右距离)
        
        # 计算公式：真实位置 = 相机校准位置 - 相对偏差
        # 这里使用减号是因为：图像上物体越靠下(y变大)，在世界坐标中实际上离机器人越近(X变小，如果相机在前)
        # 或者对应当前的安装方向：图像中心到物体的向量，需从相机原点反向推导。
        
        world_x = self.camera_x - cam_rel_y  
        world_y = self.camera_y - cam_rel_x  

        return world_x, world_y

    def detect_table(self, image):
        """检测黑色桌子并更新像素/米比例尺"""
        (b, g, r) = cv2.split(image)
        
        # 降噪处理
        r = cv2.medianBlur(r, 5)
        g = cv2.medianBlur(g, 5)

        # 黑色物体提取：R和G分量都极低
        mask = (r < 30) & (g < 30)
        
        binary = np.zeros_like(r)
        binary[mask] = 255
        
        # 形态学闭运算：填充物体内部孔洞
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None

        # 提取最大轮廓作为桌子
        largest_cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_cnt)

        # 尺寸过滤，防止噪点干扰
        if w > 50 and h > 50:
            # 动态更新比例尺 (Pixels per Meter)
            if self.table_length > 0:
                self.pixels_permm_y = h / self.table_length 
            if self.table_breadth > 0:
                self.pixels_permm_x = w / self.table_breadth

            # Debug: 在图像上绘制检测框
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(image, "Table", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            
            self.table_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            return (x, y, w, h)
        
        return None

    def detect_2d_object(self, image, table_rect):
        """在桌子ROI区域内检测绿色物体"""
        tx, ty, tw, th = table_rect
        
        # ROI (Region of Interest) 截取，只处理桌子内部
        roi = image[ty:ty+th, tx:tx+tw]
        (b, g, r) = cv2.split(roi)

        # 绿色识别逻辑 (增强版)：
        # 1. 绿色分量亮度足够 (g > 80)
        # 2. 红蓝分量被抑制 (r < 60, b < 60)
        # 3. 绿色必须显著强于红色 (g > r + 20) -> 避免识别白光/高光
        mask_green = (g > 80) & (r < 60) & (b < 60) & (g > r + 20)
        
        binary = np.zeros_like(g)
        binary[mask_green] = 255

        # 发布二值化掩膜供调试
        self.mask_debug_pub.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_cnt)
            
            if w > 5 and h > 5:
                # 将ROI局部坐标转换回全图坐标
                global_x = tx + x + w // 2
                global_y = ty + y + h // 2

                # 绘制识别结果
                cv2.rectangle(image, (tx+x, ty+y), (tx+x+w, ty+y+h), (0, 255, 0), 2)
                cv2.circle(image, (global_x, global_y), 5, (0, 0, 255), -1)
                
                self.object_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                return global_x, global_y
        
        return None, None

    def pixels_to_meters(self, x, y):
        if self.pixels_permm_x == 0 or self.pixels_permm_y == 0:
            return 0.0, 0.0
        
        # 计算像素点相对于图像中心的物理距离
        # 定义：向右为正 (x+), 向下为正 (y+)
        rel_x_m = (x - self.img_centre_x) / self.pixels_permm_x
        rel_y_m = (y - self.img_centre_y) / self.pixels_permm_y
        
        return rel_x_m, rel_y_m

    def print_debug_info(self, rel_x, rel_y, abs_x, abs_y):
        self.get_logger().info("="*30)
        self.get_logger().info(f"📸 视觉相对中心: dx={rel_x:.3f}m, dy={rel_y:.3f}m")
        self.get_logger().warn(f"🌍 Gazebo 绝对坐标: X={abs_x:.3f}, Y={abs_y:.3f}")
        self.get_logger().info(f"📍 目标误差参考: GreenCan实际应为 X=0.5")
        self.get_logger().info("="*30)

def main(args=None):
    rclpy.init(args=args)
    node = VisionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()