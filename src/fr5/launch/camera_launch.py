import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 动态获取标定文件路径
    calib_path = os.path.expanduser('~/.ros/camera_info/narrow_stereo.yaml')
    calib_url = 'file://' + calib_path

    return LaunchDescription([
        # --- 节点 1: USB 摄像头驱动 ---
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video2', 
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',  # 保持你满意的 mjpeg
                'framerate': 30.0,            # 保持 30 FPS
                'camera_frame_id': 'usb_cam',
                'camera_info_url': calib_url,
                'camera_name': 'narrow_stereo'
            }]
        ),

        # --- 节点 2: 图像处理 ---
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc_node',
            output='screen',
            # 【这里加回这个参数，防止刷屏报错】
            parameters=[{
                'approximate_sync': True,
                'queue_size': 20
            }],
            remappings=[
                ('image', '/image_raw'),
                ('camera_info', '/camera_info') 
            ]
        ),

        # --- 节点 3: 自动弹出 RQT 查看器 ---
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_viewer',
            arguments=['/image_rect_color'] 
        )
    ])