#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

class PointCloudTest(Node):
    def __init__(self):
        super().__init__('pointcloud_tf')
        self.subscription = self.create_subscription(
            PointCloud2, '/growing_bot/kinect_camera/points', self.pointcloud_callback, 10)
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.count = 0
        self.get_logger().info('PointCloud + TF test node started')

    def pointcloud_callback(self, msg):
        self.count += 1
        # 每 25 帧处理一次
        if self.count % 25 != 0:
            return
        width = msg.width
        height = msg.height
        # 图像中心
        u = width // 2
        v = height // 2
        try:
            # 1. 从 PointCloud2 获取 XYZ
            points = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)
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
            # 检查 NaN
            if not all(value == value for value in (x, y, z)):
                self.get_logger().warning('Invalid XYZ')
                return
            self.get_logger().info(f'Camera XYZ: ({x:.3f}, {y:.3f}, {z:.3f})')
            # 2. 创建 PointStamped
            point_camera = PointStamped()
            point_camera.header = msg.header
            point_camera.point.x = float(x)
            point_camera.point.y = float(y)
            point_camera.point.z = float(z)
            # 3. TF: kinect_camera -> base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time())
            point_base = do_transform_point(point_camera, transform)
            # 4. 输出 base_link 坐标
            self.get_logger().info(
                f'Base XYZ: ({point_base.point.x:.3f}, '
                f'{point_base.point.y:.3f}, {point_base.point.z:.3f})')
        except Exception as e:
            self.get_logger().warning(f'TF / PointCloud error: {e}')

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
