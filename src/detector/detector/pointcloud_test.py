#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointCloudTest(Node):
    def __init__(self):
        super().__init__('pointcloud_test')
        self.subscription = self.create_subscription(
            PointCloud2, '/growing_bot/kinect_camera/points', self.pointcloud_callback, 10)
        self.count = 0
        self.get_logger().info('PointCloud test node started')

    def pointcloud_callback(self, msg):
        self.count += 1
        # 每 25 帧打印一次
        if self.count % 25 != 0:
            return
        # 相机分辨率
        width = msg.width
        height = msg.height
        # 测试图像中心
        u = width // 2
        v = height // 2
        try:
            points = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)
            # PointCloud2 是按行排列的：index = v * width + u
            index = v * width + u
            point = None
            for i, p in enumerate(points):
                if i == index:
                    point = p
                    break
            if point is None:
                self.get_logger().warning('Cannot get point')
                return
            x, y, z = point
            self.get_logger().info(
                f'pixel=({u},{v}) frame={msg.header.frame_id} '
                f'XYZ=({x:.3f}, {y:.3f}, {z:.3f})')
        except Exception as e:
            self.get_logger().error(f'PointCloud error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
