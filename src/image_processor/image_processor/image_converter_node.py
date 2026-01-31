import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.bridge = CvBridge()

        # 订阅话题：这里订阅的是 usb_cam 发出的原始图像
        # 如果你的 launch 文件使用了 namespace，这里可能需要改为 /narrow_stereo/image_raw
        # 现在的代码默认订阅 /image_raw
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        # 发布话题
        self.image_pub = self.create_publisher(
            Image,
            '/processed_image',
            10
        )
        self.get_logger().info("图像转换节点已启动，正在等待图像...")

    def image_callback(self, msg):
        try:
            # 1. 将 ROS 图像转为 OpenCV 格式 (bgr8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"转换错误: {e}")
            return
        
        # 2. 在图像上画一个红圈 (示例处理)
        # 参数：图像, 圆心坐标, 半径, 颜色(BGR), 线宽(-1表示填充)
        cv2.circle(cv_image, (100, 100), 50, (0, 0, 255), 3)

        # 3. 显示图像 (可选，用于调试)
        cv2.imshow("OpenCV Window", cv_image)
        cv2.waitKey(1)

        # 4. 将处理后的图像转回 ROS 格式并发布
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"发布错误: {e}")

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
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()