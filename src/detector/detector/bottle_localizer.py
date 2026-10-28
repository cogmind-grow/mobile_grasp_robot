#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

class YoloBottleLocalizer(Node):

    # 定位流程：
    # RGB → YOLO → bbox 中心 (u,v) → PointCloud2(最新一帧) → XYZ(kinect_camera)
    #     → TF2 → XYZ(base_link) → 打印
    def __init__(self):
        super().__init__('yolo_bottle_localizer')
        self.bridge = CvBridge()
        # 参数（conf_threshold=0.0 与 bottle_detector.py 行为一致，不过滤）
        self.model_path_ = self.declare_parameter('model_path', 'yolo11n.pt').value
        self.target_class_ = self.declare_parameter('target_class', 'bottle').value
        self.conf_threshold_ = self.declare_parameter('conf_threshold', 0.0).value
        self.detect_every_n_ = max(
            1, self.declare_parameter('detect_every_n', 3).value)
        self.target_frame_ = self.declare_parameter('target_frame', 'base_link').value
        # YOLO 模型
        self.model = YOLO(self.model_path_)
        # TF2: kinect_camera -> base_link 是固定变换，查最新 TF 即可
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        # image/points/timer 分属不同 callback group：YOLO 推理耗时可达数秒
        # （首次加载模型 ~17s），若共用默认 MutuallyExclusive group，即使
        # MultiThreadedExecutor 也会互相阻塞，点云停更导致定位用到过期深度
        self.image_group_ = MutuallyExclusiveCallbackGroup()
        self.points_group_ = MutuallyExclusiveCallbackGroup()
        self.diag_group_ = MutuallyExclusiveCallbackGroup()
        # RGB 驱动流水线；点云只缓存最新一帧（两个传感器各自 30Hz，
        # 最多相差一帧，对静止目标可忽略；比时间同步器不会出现配对失败）
        self.image_sub_ = self.create_subscription(
            Image, '/growing_bot/rgb_camera', self.ImageCallback, 10,
            callback_group=self.image_group_)
        self.points_sub_ = self.create_subscription(
            PointCloud2, '/growing_bot/kinect_camera/points', self.PointsCallback, 10,
            callback_group=self.points_group_)
        # 诊断定时器：5 秒后打印两个话题的收包数，定位"哪个话题没数据"
        self.diag_timer_ = self.create_timer(
            5.0, self.DiagCallback, callback_group=self.diag_group_)
        # 瓶子位置发布：状态机节点（pick_state_machine）订阅
        self.bottle_pos_pub_ = self.create_publisher(
            PointStamped, '/growing_bot/bottle/position', 10)
        self.image_count_ = 0
        self.points_count_ = 0
        self.latest_points_ = None
        self.get_logger().info('YOLO bottle localizer started')

    def detectBottle(self, frame):
        # RGB -> YOLO -> bbox 中心 (u, v)；boxes 按置信度降序，首个 bottle 即最优
        results = self.model(frame, verbose=False)
        for box in results[0].boxes:
            class_name = self.model.names[int(box.cls[0])]
            confidence = float(box.conf[0])
            if class_name != self.target_class_:
                continue
            if confidence < self.conf_threshold_:
                self.get_logger().info(
                    f'detectBottle: bottle conf={confidence:.2f} '
                    f'below threshold={self.conf_threshold_:.2f}, skipped')
                return None
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            self.get_logger().info(
                f'detectBottle: conf={confidence:.2f} '
                f'bbox=({x1},{y1})-({x2},{y2}) center=({u},{v})')
            return u, v
        return None

    def lookupCameraXyz(self, points_msg, u, v):
        # (u, v) -> XYZ(kinect_camera)；点云与图像同为 640x480、行优先
        if u >= points_msg.width or v >= points_msg.height:
            self.get_logger().warning(
                f'lookupCameraXyz: ({u}, {v}) out of cloud range '
                f'{points_msg.width}x{points_msg.height}')
            return None
        index = v * points_msg.width + u
        point = point_cloud2.read_points(
            points_msg, field_names=('x', 'y', 'z'), skip_nans=False)[index]
        x = float(point['x'])
        y = float(point['y'])
        z = float(point['z'])
        # NaN 判定 (NaN != NaN)
        if x != x or y != y or z != z:
            self.get_logger().warning(f'lookupCameraXyz: point ({u}, {v}) is NaN')
            return None
        self.get_logger().info(
            f'lookupCameraXyz: ({x:.3f}, {y:.3f}, {z:.3f}) [kinect_camera]')
        return x, y, z

    def transformToBase(self, points_msg, x, y, z):
        # TF2: XYZ(kinect_camera) -> XYZ(base_link)
        point_camera = PointStamped()
        point_camera.header = points_msg.header
        point_camera.point.x = x
        point_camera.point.y = y
        point_camera.point.z = z
        transform = self.tf_buffer_.lookup_transform(
            self.target_frame_, points_msg.header.frame_id, Time())
        return do_transform_point(point_camera, transform)

    def PointsCallback(self, msg):
        # 只缓存最新一帧，不处理
        self.latest_points_ = msg
        self.points_count_ += 1

    def ImageCallback(self, image_msg):
        self.image_count_ += 1
        if self.image_count_ == 1:
            self.get_logger().info('ImageCallback: first image received')
        # 不需要每一帧都跑 YOLO
        if self.image_count_ % self.detect_every_n_ != 0:
            return
        try:
            # 0. 点云必须已就绪
            if self.latest_points_ is None:
                self.get_logger().warning(
                    'ImageCallback: no PointCloud2 received yet',
                    throttle_duration_sec=5.0)
                return
            # 1. RGB -> YOLO -> bbox 中心 (u, v)
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            detection = self.detectBottle(frame)
            if detection is None:
                self.get_logger().info(
                    'ImageCallback: no bottle in frame',
                    throttle_duration_sec=5.0)
                return
            u, v = detection
            # 2. PointCloud2 -> XYZ(kinect_camera)
            camera_xyz = self.lookupCameraXyz(self.latest_points_, u, v)
            if camera_xyz is None:
                return
            # 3. TF2 -> XYZ(base_link)
            point_base = self.transformToBase(self.latest_points_, *camera_xyz)
            # 4. 打印
            self.get_logger().info(
                f'ImageCallback: bottle XYZ(base_link) '
                f'({point_base.point.x:.3f}, '
                f'{point_base.point.y:.3f}, '
                f'{point_base.point.z:.3f})')
            # 5. 发布瓶子位置（base_link 坐标系），供状态机节点使用
            point_base.header.frame_id = 'base_link'
            point_base.header.stamp = image_msg.header.stamp
            self.bottle_pos_pub_.publish(point_base)
        except Exception as e:
            self.get_logger().warning(f'ImageCallback failed: {e}')

    def DiagCallback(self):
        # 周期打印收包统计，两个话题都有数据后自动停止
        # 注意：use_sim_time 下首帧 /clock 会让仿真时间从 0 跳变，
        # 定时器可能在启动瞬间就触发一次，那次统计为 0 属正常
        if self.image_count_ > 0 and self.points_count_ > 0:
            self.diag_timer_.cancel()
            return
        self.get_logger().info(
            f'DiagCallback: images={self.image_count_} '
            f'points={self.points_count_} '
            f'(zero count means that topic has no data)')

def main(args=None):
    rclpy.init(args=args)
    node = YoloBottleLocalizer()
    # YOLO 推理耗时，多线程执行器避免阻塞 /tf
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
