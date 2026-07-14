#!/usr/bin/env python3
"""
camera_bridge.py — AirSim 相机图像 + 激光雷达 → ROS 2 消息转换

功能：
  - 从 AirSim API 获取多路相机图像, 转为 sensor_msgs/Image
  - 从 AirSim API 获取激光雷达数据, 转为 sensor_msgs/PointCloud2
  - 发布 camera_info（内参校准信息）

对应 AirSim settings.json 中的相机:
  - front_center:  RGB (ImageType=0) + Depth (3) + Segmentation (5)
  - bottom_center: RGB (0)
  - front_left:    RGB (0)
  - front_right:   RGB (0)

对应激光雷达:
  - LidarSensor1:  32线, 20Hz, 80000点/秒, 60m
"""

import struct
import time
import numpy as np

try:
    import airsim  # type: ignore
    HAS_AIRSIM = True
except ImportError:
    HAS_AIRSIM = False

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster


def create_camera_info(width: int, height: int, fov_degrees: float,
                       frame_id: str = "") -> CameraInfo:
    """根据图像尺寸和 FOV 生成 CameraInfo（针孔模型）

    fx = fy = (width/2) / tan(fov/2)
    cx = width/2, cy = height/2
    """
    fov_rad = fov_degrees * 3.14159265 / 180.0
    fx = (width / 2.0) / (np.tan(fov_rad / 2.0)) if fov_rad > 0 else 500.0
    fy = fx
    cx = width / 2.0
    cy = height / 2.0

    info = CameraInfo()
    info.header = Header(frame_id=frame_id)
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def airsim_image_to_ros(airsim_img, img_type: int, frame_id: str = "") -> Image:
    """AirSim ImageResponse → sensor_msgs/Image

    RGB 图像请求使用 compress=False，AirSim/Unreal 原始像素通常为 BGRA/BGR 顺序。
    DepthPerspective 请求使用 pixels_as_float=True，返回 float32 深度数组。
    """
    if isinstance(airsim_img, str):
        return Image()

    h = airsim_img.height
    w = airsim_img.width
    pixel_count = h * w

    img = Image()
    img.header = Header(frame_id=frame_id)
    img.height = h
    img.width = w
    img.is_bigendian = False

    if img_type == 2:  # DepthPerspective → 32FC1 (4 bytes per pixel)
        depth = np.asarray(airsim_img.image_data_float, dtype=np.float32)
        if depth.size < pixel_count:
            raise ValueError(
                f"Depth image data too short: {depth.size} < {pixel_count}"
            )
        img.data = depth[:pixel_count].tobytes()
        img.encoding = "32FC1"
        img.step = w * 4
    else:
        data = airsim_img.image_data_uint8
        if isinstance(data, (bytes, bytearray)):
            raw = np.frombuffer(data, dtype=np.uint8)
        else:
            raw = np.asarray(data, dtype=np.uint8)

        rgba_size = pixel_count * 4
        rgb_size = pixel_count * 3
        if raw.size >= rgba_size:
            img.data = raw[:rgba_size].tobytes()
            img.encoding = "bgra8"
            img.step = w * 4
        elif raw.size >= rgb_size:
            img.data = raw[:rgb_size].tobytes()
            img.encoding = "bgr8"
            img.step = w * 3
        else:
            raise ValueError(
                f"RGB image data too short: {raw.size} bytes for {w}x{h}"
            )

    return img


def airsim_lidar_to_pointcloud(lidar_data, frame_id: str = "") -> PointCloud2:
    """AirSim LidarData → sensor_msgs/PointCloud2"""
    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

    cloud = PointCloud2()
    cloud.header = Header(frame_id=frame_id)
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


class CameraBridge:
    """AirSim 相机 + LiDAR → ROS 2 发布器"""

    # AirSim ImageType 常量（airsim.ImageType 枚举: Scene=0, DepthPerspective=2）
    IMAGE_TYPE_SCENE = 0
    IMAGE_TYPE_DEPTH = 2  # DepthPerspective

    def __init__(self, node, airsim_client):
        """
        Args:
            node: rclpy Node
            airsim_client: AirSim MultirotorClient
        """
        self._node = node
        self._client = airsim_client
        self._logger = node.get_logger()

        # 相机配置: (AirSim 相机名, ROS topic 后缀, FOV, width, height, ImageType int)
        # 使用整数常量避免类定义时依赖 airsim 包
        self._camera_configs = [
            ("front_center", "rgb/front_center", 95, 640, 360, self.IMAGE_TYPE_SCENE),
            ("front_center", "depth/front_center", 95, 640, 360, self.IMAGE_TYPE_DEPTH),
            ("bottom_center", "rgb/bottom_center", 90, 1024, 768, self.IMAGE_TYPE_SCENE),
            ("front_left", "rgb/front_left", 95, 640, 360, self.IMAGE_TYPE_SCENE),
            ("front_right", "rgb/front_right", 95, 640, 360, self.IMAGE_TYPE_SCENE),
        ]

        # 为每个相机配置创建发布者
        self._img_publishers = {}   # topic → Image publisher
        self._info_pubs = {}        # camera_info topic → CameraInfo publisher
        self._parent_info_pubs = {} # parent camera_info topic → CameraInfo publisher
        self._info_data = {}        # camera_info topic → (CameraInfo_msg, w, h, fov)

        for cam_name, topic_postfix, fov, w, h, img_type in self._camera_configs:
            topic = f"/sensor/camera/{topic_postfix}"
            info_topic = f"{topic}/camera_info"
            parent_info_topic = f"{topic.rsplit('/', 1)[0]}/camera_info"
            frame_id = f"camera_{cam_name}_optical"

            if topic not in self._img_publishers:
                self._img_publishers[topic] = node.create_publisher(
                    Image, topic, 10
                )
            if info_topic not in self._info_pubs:
                self._info_pubs[info_topic] = node.create_publisher(
                    CameraInfo, info_topic, 10
                )
                self._info_data[info_topic] = (
                    create_camera_info(w, h, fov, frame_id), w, h, fov
                )
            if parent_info_topic not in self._parent_info_pubs:
                self._parent_info_pubs[parent_info_topic] = node.create_publisher(
                    CameraInfo, parent_info_topic, 10
                )

        # LiDAR 发布者
        self._lidar_pub = node.create_publisher(
            PointCloud2, "/sensor/lidar/points", 10
        )

        # 静态 TF 广播（传感器外参相对 base_link）
        self._tf_broadcaster = StaticTransformBroadcaster(node)
        self._publish_static_transforms()

        self._logger.info(f"相机桥接已初始化: {len(self._img_publishers)} 个图像话题, "
                          f"{len(self._info_pubs)} 个 camera_info, "
                          f"LiDAR 点云")

    def _publish_static_transforms(self):
        """发布相机和 LiDAR 相对机体的静态外参。"""
        now = self._node.get_clock().now().to_msg()
        transforms = []

        # 相机帧
        camera_frames = set()
        for _, topic_postfix, _, _, _, _ in self._camera_configs:
            cam_name = topic_postfix.split("/")[-1] if "/" in topic_postfix else topic_postfix
            body_frame_id = f"camera_{cam_name}_body"
            optical_frame_id = f"camera_{cam_name}_optical"
            if cam_name not in camera_frames:
                camera_frames.add(cam_name)

                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = "base_link"
                t.child_frame_id = body_frame_id
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.w = 1.0
                transforms.append(t)

                optical_t = TransformStamped()
                optical_t.header.stamp = now
                optical_t.header.frame_id = body_frame_id
                optical_t.child_frame_id = optical_frame_id
                optical_t.transform.translation.x = 0.0
                optical_t.transform.translation.y = 0.0
                optical_t.transform.translation.z = 0.0
                # REP-103: body X forward/Y left/Z up -> optical Z forward/X right/Y down.
                optical_t.transform.rotation.x = -0.5
                optical_t.transform.rotation.y = 0.5
                optical_t.transform.rotation.z = -0.5
                optical_t.transform.rotation.w = 0.5
                transforms.append(optical_t)

        # LiDAR 帧
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        self._tf_broadcaster.sendTransform(transforms)

    def publish_all(self):
        """获取并发布所有相机图像 + LiDAR 数据（在定时器中调用）"""
        if self._client is None:
            return

        now = self._node.get_clock().now().to_msg()

        # === 相机 ===
        for cam_name, topic_postfix, fov, w, h, img_type in self._camera_configs:
            try:
                topic = f"/sensor/camera/{topic_postfix}"
                info_topic = f"{topic}/camera_info"
                parent_info_topic = f"{topic.rsplit('/', 1)[0]}/camera_info"

                pixels_as_float = (img_type == self.IMAGE_TYPE_DEPTH)
                # compress=False 才会返回 ROS Image 需要的原始像素/深度数组。
                responses = self._client.simGetImages([
                    airsim.ImageRequest(
                        cam_name,
                        img_type,
                        pixels_as_float=pixels_as_float,
                        compress=False,
                    )
                ])
                if responses and len(responses) > 0:
                    img_msg = airsim_image_to_ros(
                        responses[0], img_type,
                        frame_id=f"camera_{cam_name}_optical"
                    )
                    img_msg.header.stamp = now
                    self._img_publishers[topic].publish(img_msg)

                    # 发布 camera_info
                    if info_topic in self._info_data and info_topic in self._info_pubs:
                        info_msg = create_camera_info(
                            img_msg.width,
                            img_msg.height,
                            fov,
                            frame_id=f"camera_{cam_name}_optical",
                        )
                        info_msg.header.stamp = now
                        self._info_pubs[info_topic].publish(info_msg)
                        if parent_info_topic in self._parent_info_pubs:
                            self._parent_info_pubs[parent_info_topic].publish(info_msg)
            except Exception as e:
                self._logger.warn(f"相机 {cam_name} 获取失败: {e}")

        # === LiDAR ===
        try:
            lidar_data = self._client.getLidarData()
            if lidar_data and len(lidar_data.point_cloud) > 0:
                cloud_msg = airsim_lidar_to_pointcloud(
                    lidar_data, frame_id="lidar"
                )
                cloud_msg.header.stamp = now
                self._lidar_pub.publish(cloud_msg)
        except Exception as e:
            self._logger.warn(f"LiDAR 获取失败: {e}")
