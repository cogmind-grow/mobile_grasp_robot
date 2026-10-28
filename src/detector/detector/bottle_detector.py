#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloBottleDetector(Node):
    def __init__(self):
        super().__init__('yolo_bottle_detector')
        self.bridge = CvBridge()
        # YOLO 模型
        self.model = YOLO('yolo11n.pt')
        self.subscription = self.create_subscription(
            Image, '/growing_bot/rgb_camera', self.image_callback, 10)
        self.count = 0
        self.get_logger().info('YOLO bottle detector started')

    def image_callback(self, msg):
        self.count += 1
        # 不需要每一帧都跑 YOLO
        if self.count % 3 != 0:
            return
        try:
            # ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # YOLO
            results = self.model(frame, verbose=False)
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names[cls_id]
                    # 只关注 bottle
                    if class_name != 'bottle':
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # bbox 中心
                    u = int((x1 + x2) / 2)
                    v = int((y1 + y2) / 2)
                    self.get_logger().info(
                        f'Bottle detected: conf={confidence:.2f} '
                        f'bbox=({x1},{y1})-({x2},{y2}) center=({u},{v})')
                    # 画框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f'bottle {confidence:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 显示结果
            cv2.imshow('YOLO Bottle Detection', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'YOLO error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloBottleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
