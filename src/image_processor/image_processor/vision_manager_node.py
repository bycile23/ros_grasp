import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class VisionManager(Node):
    def __init__(self):
        super().__init__('vision_manager')
        self.bridge = CvBridge()

        # ==========================================
        # 1. 物理环境与校准参数
        # ==========================================
        self.table_length = 0.3    
        self.table_breadth = 0.3   
        
        self.camera_x = 0.677  
        self.camera_y = -0.056
        self.fixed_target_z = 0.35 

        self.img_width = 640
        self.img_height = 480
        self.img_centre_x = self.img_width / 2
        self.img_centre_y = self.img_height / 2

        self.pixels_permm_x = 0.0
        self.pixels_permm_y = 0.0
        
        self.history_x = deque(maxlen=10)
        self.history_y = deque(maxlen=10)
        
        # 🌟 核心状态位
        self.target_locked = False
        self.final_x = 0.0
        self.final_y = 0.0

        # ==========================================
        # 3. 通信配置
        # ==========================================
        self.image_sub = self.create_subscription(
            Image, '/narrow_stereo/image_raw', self.image_callback, 10)
        self.target_pub = self.create_publisher(PointStamped, '/vision/target_point', 10)
        
        self.table_detect_pub = self.create_publisher(Image, '/vision/table_debug', 10)
        self.object_detect_pub = self.create_publisher(Image, '/vision/object_debug', 10)
        self.mask_debug_pub = self.create_publisher(Image, '/vision/mask_debug', 10)

        self.get_logger().info("🚀 视觉节点已启动 | 正在疯狂燃烧 CPU 计算高精度坐标...")

    def image_callback(self, msg):
        # 如果已经锁定了目标，直接跳出（虽然很快订阅器就会被销毁）
        if self.target_locked:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        table_rect = self.detect_table(cv_image.copy())

        if table_rect is not None:
            obj_pixel_x, obj_pixel_y = self.detect_2d_object(cv_image.copy(), table_rect)

            if obj_pixel_x is not None:
                rel_x, rel_y = self.pixels_to_meters(obj_pixel_x, obj_pixel_y)
                abs_x, abs_y = self.calculate_world_pose(rel_x, rel_y)

                self.history_x.append(abs_x)
                self.history_y.append(abs_y)

                if len(self.history_x) == self.history_x.maxlen:
                    avg_x = sum(self.history_x) / len(self.history_x)
                    avg_y = sum(self.history_y) / len(self.history_y)

                    self.final_x = round(avg_x, 2)
                    self.final_y = round(avg_y, 2)

                    # 🌟 触发变形机制
                    self.target_locked = True
                    
                    self.get_logger().warn(f"🌍 最终锁定坐标: X={self.final_x}, Y={self.final_y}")
                    self.get_logger().info("🎉 坐标锁定！正在销毁摄像头进程，变身 simple_vision 模式以释放物理引擎算力...")
                    
                    # 🌟 核心杀手锏：彻底销毁图像订阅，不再接受任何视频帧，归还所有 CPU
                    self.destroy_subscription(self.image_sub)
                    
                    # 创建 1Hz 定时器，完全复刻 simple_vision
                    self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """🌟 这个函数和 simple_vision 的行为绝对一模一样"""
        target_msg = PointStamped()
        target_msg.header.frame_id = "base_link"
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.point.x = float(self.final_x)
        target_msg.point.y = float(self.final_y)
        target_msg.point.z = float(self.fixed_target_z) 
        
        self.target_pub.publish(target_msg)
        self.get_logger().info(f"💤 伪 simple_vision 广播中... (X={self.final_x}, Y={self.final_y})")

    def calculate_world_pose(self, cam_rel_x, cam_rel_y):
        world_x = self.camera_x - cam_rel_y  
        world_y = self.camera_y - cam_rel_x  
        return world_x, world_y

    def detect_table(self, image):
        (b, g, r) = cv2.split(image)
        r = cv2.medianBlur(r, 5)
        g = cv2.medianBlur(g, 5)

        mask = (r < 30) & (g < 30)
        binary = np.zeros_like(r)
        binary[mask] = 255
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_cnt)

        if w > 50 and h > 50:
            self.pixels_permm_y = h / self.table_length 
            self.pixels_permm_x = w / self.table_breadth

            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            self.table_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            return (x, y, w, h)
        return None

    def detect_2d_object(self, image, table_rect):
        tx, ty, tw, th = table_rect
        roi = image[ty:ty+th, tx:tx+tw]
        (b, g, r) = cv2.split(roi)

        mask_green = (g > 80) & (r < 60) & (b < 60) & (g > r + 20)
        binary = np.zeros_like(g)
        binary[mask_green] = 255

        self.mask_debug_pub.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_cnt)
            if w > 5 and h > 5:
                global_x = tx + x + w // 2
                global_y = ty + y + h // 2

                cv2.rectangle(image, (tx+x, ty+y), (tx+x+w, ty+y+h), (0, 255, 0), 2)
                cv2.circle(image, (global_x, global_y), 5, (0, 0, 255), -1)
                self.object_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                return global_x, global_y
        return None, None

    def pixels_to_meters(self, x, y):
        if self.pixels_permm_x == 0 or self.pixels_permm_y == 0:
            return 0.0, 0.0
        rel_x_m = (x - self.img_centre_x) / self.pixels_permm_x
        rel_y_m = (y - self.img_centre_y) / self.pixels_permm_y
        return rel_x_m, rel_y_m

def main(args=None):
    rclpy.init(args=args)
    node = VisionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()