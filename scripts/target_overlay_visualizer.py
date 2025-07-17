#!/usr/bin/env python3
"""
目标可视化叠加器
订阅visualization_msgs/MarkerArray消息和图像，在图像上叠加圆形目标标记
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

# ROS2 消息类型
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

class TargetOverlayVisualizer(Node):
    def __init__(self):
        super().__init__('target_overlay_visualizer')
        
        # CV Bridge用于图像转换
        self.bridge = CvBridge()
        
        # 存储当前的目标信息
        self.current_targets = []
        
        # 相机参数（需要根据实际情况调整）
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = 1280
        self.image_height = 720
        
        # 创建订阅器
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'visualization_targets',  # 匹配C++代码中的topic名称
            self.marker_callback,
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            'image_topic',  # 输入图像topic
            self.image_callback,
            10
        )
        
        self.get_logger().info('目标叠加可视化器已启动')

    def marker_callback(self, msg: MarkerArray):
        """处理接收到的标记数组消息"""
        self.current_targets = []
        
        for marker in msg.markers:
            if marker.type == Marker.CYLINDER and marker.action == Marker.ADD:
                target_info = {
                    'id': marker.id,
                    'category': marker.ns,
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z,
                    'radius': marker.scale.x / 2.0,  # 从直径转换为半径
                    'color': {
                        'r': int(marker.color.r * 255),
                        'g': int(marker.color.g * 255),
                        'b': int(marker.color.b * 255),
                        'a': marker.color.a
                    }
                }
                self.current_targets.append(target_info)
        
        self.get_logger().debug(f'收到 {len(self.current_targets)} 个目标标记')

    def world_to_image(self, world_x, world_y, world_z):
        """
        将世界坐标转换为图像像素坐标
        这里需要根据实际的相机标定参数和坐标系变换来实现
        目前使用简化的投影模型
        """
        # 简化的透视投影（需要根据实际情况调整）
        # 假设相机位于原点，朝向z轴正方向
        
        if world_z <= 0:
            return None, None
        
        # 简化的针孔相机模型
        focal_length = 500  # 像素单位的焦距，需要标定
        
        # 投影到图像平面
        image_x = int((world_x / world_z) * focal_length + self.image_width / 2)
        image_y = int((world_y / world_z) * focal_length + self.image_height / 2)
        
        # 检查是否在图像范围内
        if 0 <= image_x < self.image_width and 0 <= image_y < self.image_height:
            return image_x, image_y
        else:
            return None, None

    def calculate_pixel_radius(self, world_radius, world_z):
        """计算目标在图像中的像素半径"""
        if world_z <= 0:
            return 0
        
        focal_length = 500  # 与world_to_image中的一致
        pixel_radius = int((world_radius / world_z) * focal_length)
        return max(pixel_radius, 5)  # 最小半径为5像素

    def draw_target_overlay(self, image, target):
        """在图像上绘制单个目标"""
        # 转换世界坐标到图像坐标
        img_x, img_y = self.world_to_image(target['x'], target['y'], target['z'])
        
        if img_x is None or img_y is None:
            return  # 目标不在图像范围内
        
        # 计算像素半径
        pixel_radius = self.calculate_pixel_radius(target['radius'], target['z'])
        
        # 绘制圆形
        color = (target['color']['b'], target['color']['g'], target['color']['r'])  # BGR格式
        thickness = 2
        
        # 绘制外圆
        cv2.circle(image, (img_x, img_y), pixel_radius, color, thickness)
        
        # 绘制中心点
        cv2.circle(image, (img_x, img_y), 3, color, -1)
        
        # 绘制目标信息文本
        text_lines = [
            f"ID: {target['id']}",
            f"Cat: {target['category']}",
            f"Pos: ({target['x']:.1f}, {target['y']:.1f}, {target['z']:.1f})",
            f"R: {target['radius']:.1f}m"
        ]
        
        # 计算文本位置（在圆形上方）
        text_x = img_x - 50
        text_y = img_y - pixel_radius - 10
        
        # 绘制文本背景
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        text_thickness = 1
        
        for i, text in enumerate(text_lines):
            y_offset = text_y + i * 15
            
            # 获取文本尺寸
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, text_thickness)
            
            # 绘制文本背景
            cv2.rectangle(image, 
                         (text_x - 2, y_offset - text_height - 2),
                         (text_x + text_width + 2, y_offset + 2),
                         (0, 0, 0, 128), -1)
            
            # 绘制文本
            cv2.putText(image, text, (text_x, y_offset), 
                       font, font_scale, (255, 255, 255), text_thickness)

    def image_callback(self, msg: Image):
        """处理接收到的图像消息"""
        try:
            # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 更新图像尺寸
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # 复制图像用于叠加
            overlay_image = cv_image.copy()
            
            # 在图像上绘制所有目标
            for target in self.current_targets:
                self.draw_target_overlay(overlay_image, target)
            
            # 绘制状态信息
            status_text = f"Targets: {len(self.current_targets)}"
            cv2.putText(overlay_image, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        visualizer = TargetOverlayVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
