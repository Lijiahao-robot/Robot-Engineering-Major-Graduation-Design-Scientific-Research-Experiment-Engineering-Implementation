#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray, Detection2D
import cv2
import numpy as np
from ultralytics import YOLO
import yaml

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")
        # 加载配置参数
        with open("src/vision_detection/config/detection_params.yaml", "r") as f:
            self.params = yaml.safe_load(f)
        self.conf_thres = self.params["confidence_threshold"]
        self.iou_thres = self.params["iou_threshold"]
        self.target_classes = self.params["target_classes"]  # [0:person, 2:car, 39:bottle, 62:chair]

        # 加载 YOLOv8-nano 模型（轻量级，适配嵌入式）
        self.model = YOLO("src/vision_detection/models/yolov8n.pt")
        self.model.fuse()  # 模型融合，提升推理速度

        # 订阅相机图像，发布检测结果
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.detection_pub = self.create_publisher(Detection2DArray, "/vision_detections", 10)
        self.get_logger().info("YOLOv8 视觉检测节点已启动，支持人员/纸箱/货架识别")

    def image_callback(self, msg):
        # 解析 ROS 图像消息转为 OpenCV 格式
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # YOLO 检测（只保留目标类别）
        results = self.model(img, conf=self.conf_thres, iou=self.iou_thres, classes=self.target_classes)

        # 构建检测结果消息
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        for r in results:
            boxes = r.boxes
            for box in boxes:
                det = Detection2D()
                # 检测框坐标（归一化）
                x1, y1, x2, y2 = box.xyxy[0]
                det.bbox.center.x = (x1 + x2) / 2 / msg.width
                det.bbox.center.y = (y1 + y2) / 2 / msg.height
                det.bbox.size_x = (x2 - x1) / msg.width
                det.bbox.size_y = (y2 - y1) / msg.height
                # 类别和置信度
                det.results[0].id = int(box.cls[0])
                det.results[0].score = float(box.conf[0])
                detection_msg.detections.append(det)

        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
