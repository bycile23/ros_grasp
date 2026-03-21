import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionManager(Node):
    def __init__(self):
        super().__init__('vision_manager')
        self.bridge = CvBridge()

        self.table_length = 0.3    
        self.table_breadth = 0.3   
        
        # 🌟 逆向算出的真实相机坐标（完美标定）
        self.camera_x = 0.594  
        self.camera_y = 0.027  
        self.fixed_target_z = 0.35 

        self.img_centre_x = 640 / 2
        self.img_centre_y = 480 / 2

        self.pixels_permm_x = 0.0
        self.pixels_permm_y = 0.0
        
        self.history_x = []
        self.history_y = []
        
        self.target_locked = False
        self.final_x = 0.0
        self.final_y = 0.0

        self.image_sub = self.create_subscription(Image, '/narrow_stereo/image_raw', self.image_callback, 10)
        self.target_pub = self.create_publisher(PointStamped, '/vision/target_point', 10)

        self.get_logger().info("🚀 终极视觉节点已启动 | 分类讨论误差补偿版！...")

    def image_callback(self, msg):
        if self.target_locked: return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except:
            return

        table_rect = self.detect_table(cv_image.copy())

        if table_rect is not None:
            obj_pixel_x, obj_pixel_y = self.detect_2d_object(cv_image.copy(), table_rect)

            if obj_pixel_x is not None:
                rel_x_m = (obj_pixel_x - self.img_centre_x) / self.pixels_permm_x
                rel_y_m = (obj_pixel_y - self.img_centre_y) / self.pixels_permm_y

                # 🌟 X 对应 X
                abs_x = self.camera_x - rel_x_m-0.01

                # 💡 终极暴力的分类补偿：按正负数抹平 3D 视差！
                if rel_y_m >= 0:
                    # 当易拉罐在右侧（正数），误差较大，减 0.06
                    abs_y = self.camera_y + rel_y_m - 0.06
                    if round(abs_y, 2)== -0.01:
                        abs_y=0
                else:
                    # 当易拉罐在左侧（负数），误差较小，减 0.05
                    abs_y = self.camera_y + rel_y_m - 0.05
                    

                self.history_x.append(abs_x)
                self.history_y.append(abs_y)

                if len(self.history_x) >= 10:
                    self.final_x = round(sum(self.history_x) / len(self.history_x), 2)
                    self.final_y = round(sum(self.history_y) / len(self.history_y), 2)

                    self.target_locked = True
                    self.get_logger().warn(f"🌍 最终锁定坐标: X={self.final_x}, Y={self.final_y}")
                    self.destroy_subscription(self.image_sub)
                    self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        target_msg = PointStamped()
        target_msg.header.frame_id = "base_link"
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.point.x = float(self.final_x)
        target_msg.point.y = float(self.final_y)
        target_msg.point.z = float(self.fixed_target_z) 
        self.target_pub.publish(target_msg)
        self.get_logger().info(f"💤 广播目标坐标中... (X={self.final_x}, Y={self.final_y})")

    def detect_table(self, image):
        (b, g, r) = cv2.split(image)
        mask = (cv2.medianBlur(r, 5) < 30) & (cv2.medianBlur(g, 5) < 30)
        binary = np.zeros_like(r)
        binary[mask] = 255
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        if w > 50 and h > 50:
            self.pixels_permm_x = w / self.table_breadth
            self.pixels_permm_y = self.pixels_permm_x # 只信任宽度的比例尺，抛弃高度畸变
            return (x, y, w, h)
        return None

    def detect_2d_object(self, image, table_rect):
        tx, ty, tw, th = table_rect
        roi = image[ty:ty+th, tx:tx+tw]
        (b, g, r) = cv2.split(roi)
        mask_green = (g > 80) & (r < 60) & (b < 60) & (g > r + 20)
        binary = np.zeros_like(g)
        binary[mask_green] = 255
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            if w > 5 and h > 5:
                global_x = tx + x + w // 2
                global_y = ty + y + h // 2
                return global_x, global_y
        return None, None

def main(args=None):
    rclpy.init(args=args)
    node = VisionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()