import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.bridge = CvBridge()

        # 订阅原始图像
        self.image_sub = self.create_subscription(
            Image,
            '/narrow_stereo/image_raw', 
            self.image_callback,
            10
        )
        
        # 发布 1：彩色画框图
        self.image_pub = self.create_publisher(Image, '/processed_image', 10)
        
        # 发布 2：黑白二值化图 (论文配图专用)
        self.binary_pub = self.create_publisher(Image, '/processed_binary', 10)

        self.get_logger().info("👀 影子视觉观察者已启动，支持 [彩色标记图] 和 [黑白二值图] 双路输出！")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"转换错误: {e}")
            return
        
        # 1. 检测桌面
        table_rect = self.detect_table(cv_image.copy())
        
        # 创建一张与原图等大的全黑图像，用于装载黑白二值化结果
        full_binary = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        if table_rect is not None:
            tx, ty, tw, th = table_rect
            cv2.rectangle(cv_image, (tx, ty), (tx+tw, ty+th), (255, 0, 0), 2)
            cv2.putText(cv_image, "Table ROI", (tx, ty - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            # 2. 提取绿色易拉罐
            roi = cv_image[ty:ty+th, tx:tx+tw]
            (b, g, r) = cv2.split(roi)
            mask_green = (g > 80) & (r < 60) & (b < 60) & (g > r + 20)
            binary = np.zeros_like(g)
            binary[mask_green] = 255
            
            # 将 ROI 区域的黑白结果贴到全黑背景对应的位置上
            full_binary[ty:ty+th, tx:tx+tw] = binary
            
            # 3. 寻找轮廓并画圈
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
                if w > 5 and h > 5:
                    global_x = tx + x + w // 2
                    global_y = ty + y + h // 2
                    radius = max(w, h) // 2
                    cv2.circle(cv_image, (global_x, global_y), radius, (0, 0, 255), 3)
                    cv2.circle(cv_image, (global_x, global_y), 4, (0, 255, 0), -1)

        # ================= 发布环节 =================
        # 发布黑白二值图 (单通道，编码为 mono8)
        try:
            binary_ros = self.bridge.cv2_to_imgmsg(full_binary, encoding="mono8")
            self.binary_pub.publish(binary_ros)
        except Exception as e:
            self.get_logger().error(f"发布黑白图错误: {e}")

        # 发布彩色标记图 (三通道，编码为 bgr8)
        try:
            color_ros = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(color_ros)
        except Exception as e:
            self.get_logger().error(f"发布彩色图错误: {e}")

    # 桌面提取函数
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
            return (x, y, w, h)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()