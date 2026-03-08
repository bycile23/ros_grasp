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
        
        # 我引入了 CvBridge，这是我用来连接 ROS2 图像消息和 OpenCV 处理库的桥梁。
        self.bridge = CvBridge()

        # ==========================================
        # 1. 物理环境与校准参数设定
        # ==========================================
        # 我在 Gazebo 的 world 文件里量过了，目标桌子的真实物理尺寸是 0.3米 x 0.3米。
        # 这个尺寸非常重要，这是我建立“像素 -> 米”比例尺的基石。
        self.table_length = 0.3    
        self.table_breadth = 0.3   
        
        # 经过我前几次的误差推导和系统标定，我计算出了我这颗摄像头在 base_link 坐标系下的真实投影中心。
        self.camera_x = 0.677  
        self.camera_y = -0.056
        
        # 易拉罐的抓取高度我硬编码为 0.35米，因为我要确保夹爪恰好在物体上半部分，绝对不能碰到桌子。
        self.fixed_target_z = 0.35 

        # ==========================================
        # 2. 图像参数与防抖容器初始化
        # ==========================================
        # 我使用的相机分辨率是 640x480。
        self.img_width = 640
        self.img_height = 480
        # 算出图像的绝对中心坐标（像素）。
        self.img_centre_x = self.img_width / 2
        self.img_centre_y = self.img_height / 2

        # 这两个变量用来存储我后续动态计算出来的“比例尺”（每像素代表多少米）。
        self.pixels_permm_x = 0.0
        self.pixels_permm_y = 0.0
        
        # 为了对抗视觉算法固有的“像素跳变（Jitter）”，我设计了一个滑动平均滤波器。
        # 我开了两个长度为 10 的队列，用来缓存最近 10 帧算出来的物理坐标。
        self.history_x = deque(maxlen=10)
        self.history_y = deque(maxlen=10)
        
        # 🌟 这是我这个节点的核心状态机：
        # False 代表我还在努力用 OpenCV 找目标；True 代表我已经找到了，准备变形（休眠）。
        self.target_locked = False
        
        # 用来存储我经过防抖滤波、截断处理后的最终纯净坐标。
        self.final_x = 0.0
        self.final_y = 0.0

        # ==========================================
        # 3. ROS2 通信接口配置
        # ==========================================
        # 我订阅了 Gazebo 发出的原始彩色图像话题。
        self.image_sub = self.create_subscription(
            Image, '/narrow_stereo/image_raw', self.image_callback, 10)
            
        # 这是我用来指挥机械臂的话题，我会把算好的点发到这里。
        self.target_pub = self.create_publisher(PointStamped, '/vision/target_point', 10)
        
        # 方便我调试的三个可视化话题，可以在 RViz 里看我的处理过程。
        self.table_detect_pub = self.create_publisher(Image, '/vision/table_debug', 10)
        self.object_detect_pub = self.create_publisher(Image, '/vision/object_debug', 10)
        self.mask_debug_pub = self.create_publisher(Image, '/vision/mask_debug', 10)

        self.get_logger().info("🚀 视觉节点已启动 | 正在疯狂燃烧 CPU 计算高精度坐标...")

    def image_callback(self, msg):
        """这是我处理每一帧画面的核心回调函数"""
        
        # 💡 【核心性能优化逻辑】：
        # 如果我已经算好了坐标，我就直接 return 跳出，再也不跑下面那些极其耗费 CPU 的 OpenCV 算法了。
        if self.target_locked:
            return
            
        try:
            # 我把 ROS 传过来的怪异数据流，转换成了 OpenCV 能认的 BGR 彩色矩阵。
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # 1️⃣ 第一步：我先调用检测桌子的函数，它不仅能找到桌子，还能顺便帮我更新全局比例尺。
        table_rect = self.detect_table(cv_image.copy())

        if table_rect is not None:
            # 2️⃣ 第二步：基于桌子的范围（作为 ROI 掩码），我再去寻找绿色的易拉罐。
            obj_pixel_x, obj_pixel_y = self.detect_2d_object(cv_image.copy(), table_rect)

            if obj_pixel_x is not None:
                # 3️⃣ 第三步：我把目标的像素坐标，转换成了相对中心的“米”。
                rel_x, rel_y = self.pixels_to_meters(obj_pixel_x, obj_pixel_y)
                
                # 4️⃣ 第四步：结合相机外参，算出易拉罐在机器人 base_link 下的绝对物理坐标。
                abs_x, abs_y = self.calculate_world_pose(rel_x, rel_y)

                # 把这帧算出来的粗糙坐标塞进我的防抖队列里。
                self.history_x.append(abs_x)
                self.history_y.append(abs_y)

                # 💡 当我的队列收集满 10 帧数据后，我就认为当前环境稳定了，可以开始结算了！
                if len(self.history_x) == self.history_x.maxlen:
                    
                    # 算出 10 帧数据的平均值，抹平噪点。
                    avg_x = sum(self.history_x) / len(self.history_x)
                    avg_y = sum(self.history_y) / len(self.history_y)

                    # 🌟 【我的绝杀】：工业级截断！
                    # 因为 Gazebo 对零点几毫米的误差极其敏感（会导致夹爪推飞物体），
                    # 我直接大刀阔斧，把它强行四舍五入到厘米级（2位小数），彻底斩断物理引擎的“推移效应”。
                    self.final_x = round(avg_x, 2)
                    self.final_y = round(avg_y, 2)

                    # 拨动状态机开关，告诉系统：“我算完了！”
                    self.target_locked = True
                    
                    self.get_logger().warn(f"🌍 最终锁定坐标: X={self.final_x}, Y={self.final_y}")
                    self.get_logger().info("🎉 坐标锁定！正在销毁摄像头进程，变身 simple_vision 模式以释放物理引擎算力...")
                    
                    # 💡 【高能架构设计】：
                    # 既然已经算出了绝对精准的死坐标，留着图像处理纯属浪费电脑算力。
                    # 我果断销毁（Destroy）了图像话题的订阅器，从底层切断了视频流！
                    # 这样就能把 100% 的 CPU 算力还给 Gazebo，让它能安心去算那个脆弱的“吸附插件力矩”。
                    self.destroy_subscription(self.image_sub)
                    
                    # 同时，为了防止机械臂因为还没启动而错过消息，
                    # 我启动了一个极低消耗的 1Hz 定时器，开始傻瓜式地广播这个最终坐标。
                    self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """🌟 伪 simple_vision 模式：这是在销毁图像流后接管广播的轻量级函数"""
        target_msg = PointStamped()
        target_msg.header.frame_id = "base_link"
        target_msg.header.stamp = self.get_clock().now().to_msg()
        # 把我之前存好的“纯净坐标”打包。
        target_msg.point.x = float(self.final_x)
        target_msg.point.y = float(self.final_y)
        target_msg.point.z = float(self.fixed_target_z) 
        
        # 孜孜不倦地发布，直到机械臂收到并执行为止。
        self.target_pub.publish(target_msg)
        self.get_logger().info(f"💤 伪 simple_vision 广播中... (X={self.final_x}, Y={self.final_y})")

    def calculate_world_pose(self, cam_rel_x, cam_rel_y):
        """我写的坐标系转换矩阵（平移法）"""
        world_x = self.camera_x - cam_rel_y  
        world_y = self.camera_y - cam_rel_x  
        return world_x, world_y

    def detect_table(self, image):
        """我用来提取黑色桌子轮廓并建立坐标比例尺的函数"""
        (b, g, r) = cv2.split(image)
        # 用中值滤波稍微去个噪
        r = cv2.medianBlur(r, 5)
        g = cv2.medianBlur(g, 5)

        # 桌子是黑色的，所以我设定 RGB 阈值都很低
        mask = (r < 30) & (g < 30)
        binary = np.zeros_like(r)
        binary[mask] = 255
        
        # 形态学闭运算，补全桌子中间的破洞
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 假设画面里最大的黑色方块就是那张桌子
        largest_cnt = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_cnt)

        # 如果桌子足够大，我就用它的像素宽高和真实物理宽高相除，得出宝贵的“比例尺”
        if w > 50 and h > 50:
            self.pixels_permm_y = h / self.table_length 
            self.pixels_permm_x = w / self.table_breadth

            # 画个框，发出去看看效果
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            self.table_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            return (x, y, w, h)
        return None

    def detect_2d_object(self, image, table_rect):
        """我用来在桌子范围内寻找绿色易拉罐的函数"""
        tx, ty, tw, th = table_rect
        # 我把画面切成了只有桌子大小的 ROI，这样能极大减少背景干扰
        roi = image[ty:ty+th, tx:tx+tw]
        (b, g, r) = cv2.split(roi)

        # 易拉罐是绿色的，我写了一个比较严苛的绿色通道过滤条件
        mask_green = (g > 80) & (r < 60) & (b < 60) & (g > r + 20)
        binary = np.zeros_like(g)
        binary[mask_green] = 255

        self.mask_debug_pub.publish(self.bridge.cv2_to_imgmsg(binary, "mono8"))

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # 找到最大的绿色色块
            largest_cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_cnt)
            
            if w > 5 and h > 5:
                # 算出它在原图（而不是 ROI）中的绝对像素坐标
                global_x = tx + x + w // 2
                global_y = ty + y + h // 2

                cv2.rectangle(image, (tx+x, ty+y), (tx+x+w, ty+y+h), (0, 255, 0), 2)
                cv2.circle(image, (global_x, global_y), 5, (0, 0, 255), -1)
                self.object_detect_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
                
                # 满载而归，返回坐标
                return global_x, global_y
        return None, None

    def pixels_to_meters(self, x, y):
        """我用来把像素距离转成物理真实米数的数学工具"""
        if self.pixels_permm_x == 0 or self.pixels_permm_y == 0:
            return 0.0, 0.0
        # 偏离中心的像素差 / 比例尺 = 真实的物理米数
        rel_x_m = (x - self.img_centre_x) / self.pixels_permm_x
        rel_y_m = (y - self.img_centre_y) / self.pixels_permm_y
        return rel_x_m, rel_y_m

def main(args=None):
    rclpy.init(args=args)
    # 实例化我写的这个骄傲的节点
    node = VisionManager()
    rclpy.spin(node)
    # 功成身退
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()