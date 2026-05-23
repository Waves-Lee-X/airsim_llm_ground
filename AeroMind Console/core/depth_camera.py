from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DepthPointCloud:
    ok: bool
    points_world: list[tuple[float, float, float]]
    camera_position: tuple[float, float, float]
    camera_orientation: tuple[float, float, float, float]
    image_width: int
    image_height: int
    fov_deg: float
    raw_point_count: int
    sampled_point_count: int
    elapsed_ms: int
    error: str = ""

    def to_api(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "point_count": self.sampled_point_count,
            "camera_position": list(self.camera_position),
            "image_dimensions": f"{self.image_width}x{self.image_height}",
            "fov_deg": self.fov_deg,
            "elapsed_ms": self.elapsed_ms,
            "error": self.error,
        }


def _quaternion_to_rotation_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert a quaternion (w, x, y, z) to a 3x3 rotation matrix."""
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
    ], dtype=np.float64)
    return R


def world_to_ned_points(
    camera_pos: tuple[float, float, float],
    camera_ori_quat: tuple[float, float, float, float],
    points_cam: np.ndarray,
) -> np.ndarray:
    """Transform points from camera frame to AirSim world (NED) frame.

    Args:
        camera_pos: (x, y, z) in world NED
        camera_ori_quat: (w, x, y, z) quaternion
        points_cam: (N, 3) float array in camera frame
            Camera frame: X=right, Y=forward, Z=down (AirSim convention for cameras)

    Returns:
        (N, 3) float array in world NED frame
    """
    qw, qx, qy, qz = camera_ori_quat
    R = _quaternion_to_rotation_matrix(qw, qx, qy, qz)
    cx, cy, cz = float(camera_pos[0]), float(camera_pos[1]), float(camera_pos[2])
    world_pts = points_cam @ R.T + np.array([cx, cy, cz], dtype=np.float64)
    return world_pts


def depth_image_to_world_cloud(
    depth_image: np.ndarray,
    camera_pos: tuple[float, float, float],
    camera_ori_quat: tuple[float, float, float, float],
    fov_deg: float,
    max_range_m: float = 40.0,
    sample_step: int = 8,
    min_depth_m: float = 0.5,
) -> tuple[list[tuple[float, float, float]], int]:
    """Convert a depth image to a list of world-space 3D points.

    Args:
        depth_image: (H, W) float32 array with depth values in meters
        camera_pos: (x, y, z) world position of camera
        camera_ori_quat: (w, x, y, z) quaternion
        fov_deg: horizontal field of view in degrees
        max_range_m: ignore points beyond this range
        sample_step: sample every N pixels (step=1 → full resolution)
        min_depth_m: ignore points closer than this

    Returns:
        (list of (x, y, z) world points, raw_point_count)
    """
    H, W = depth_image.shape
    raw_points = 0

    fx = (W / 2.0) / math.tan(math.radians(fov_deg / 2.0))
    fy = fx
    cx, cy = W / 2.0, H / 2.0

    xs = np.arange(0, W, sample_step, dtype=np.float64) - cx
    ys = np.arange(0, H, sample_step, dtype=np.float64) - cy
    uu, vv = np.meshgrid(xs, ys)

    sampled_depth = depth_image[::sample_step, ::sample_step]
    if sampled_depth.shape != uu.shape:
        sampled_depth = sampled_depth[:uu.shape[0], :uu.shape[1]]

    valid = (sampled_depth > min_depth_m) & (sampled_depth < max_range_m) & np.isfinite(sampled_depth)
    raw_points = int(np.sum(valid))
    if raw_points == 0:
        return [], 0

    u_valid = uu[valid]
    v_valid = vv[valid]
    d_valid = sampled_depth[valid].astype(np.float64)

    norm = np.sqrt(u_valid**2 + v_valid**2 + fx**2)
    cam_x = (u_valid / fx) * d_valid
    cam_y = (v_valid / fy) * d_valid
    cam_z = d_valid

    points_cam = np.column_stack([cam_x, cam_y, cam_z])

    qw, qx, qy, qz = float(camera_ori_quat[0]), float(camera_ori_quat[1]), float(camera_ori_quat[2]), float(camera_ori_quat[3])
    R = _quaternion_to_rotation_matrix(qw, qx, qy, qz)
    cx_pos, cy_pos, cz_pos = float(camera_pos[0]), float(camera_pos[1]), float(camera_pos[2])
    world_pts = points_cam @ R.T + np.array([cx_pos, cy_pos, cz_pos])

    return [(float(p[0]), float(p[1]), float(p[2])) for p in world_pts], raw_points


class DepthCamera:
    """Captures depth images from AirSim and converts to world-space 3D point clouds."""

    def __init__(
        self,
        camera_name: str = "front_center",
        max_range_m: float = 40.0,
        sample_step: int = 8,
        min_depth_m: float = 0.5,
        max_points_per_frame: int = 3000,
    ) -> None:
        self.camera_name = camera_name
        self.max_range_m = max_range_m
        self.sample_step = sample_step
        self.min_depth_m = min_depth_m
        self.max_points_per_frame = max_points_per_frame
        self._last_fov: float = 90.0

    def capture(self, adapter: Any) -> DepthPointCloud:
        """Capture a depth image and convert to world point cloud.

        All AirSim RPC calls are routed through ``adapter._exec_rpc`` so
        they execute on the engine thread — no cross-thread client access.
        """
        t0 = time.time()
        camera_name = self.camera_name
        max_range_m = self.max_range_m
        sample_step = self.sample_step
        min_depth_m = self.min_depth_m
        max_points = self.max_points_per_frame

        try:
            if not adapter.is_connected():
                return DepthPointCloud(
                    False, [], (0, 0, 0), (1, 0, 0, 0),
                    0, 0, 0.0, 0, 0, 0,
                    error="AirSim not connected",
                )

            # -- fetch depth image on the engine thread -----------------------
            def _fetch(client: Any, airsim: Any) -> dict[str, Any]:
                vehicle = adapter.vehicle_name
                # Try to get camera FOV for calibration
                try:
                    cam_info = client.simGetCameraInfo(camera_name, vehicle_name=vehicle)
                    fov_val = float(getattr(cam_info, "fov", 90.0))
                except Exception:
                    fov_val = 90.0

                responses = client.simGetImages(
                    [
                        airsim.ImageRequest(
                            camera_name,
                            airsim.ImageType.DepthPerspective,
                            pixels_as_float=True,
                            compress=False,
                        )
                    ],
                    vehicle_name=vehicle,
                )
                if not responses:
                    return {"ok": False, "error": "No depth image response from AirSim", "fov": fov_val}

                resp = responses[0]
                depth_data = resp.image_data_float
                if depth_data is None or len(depth_data) == 0:
                    return {"ok": False, "error": "Empty depth image data", "fov": fov_val}

                return {
                    "ok": True,
                    "width": int(resp.width),
                    "height": int(resp.height),
                    "depth": list(depth_data),
                    "fov": fov_val,
                    "cam_pos": (
                        float(resp.camera_position.x_val),
                        float(resp.camera_position.y_val),
                        float(resp.camera_position.z_val),
                    ),
                    "cam_ori": (
                        float(resp.camera_orientation.w_val),
                        float(resp.camera_orientation.x_val),
                        float(resp.camera_orientation.y_val),
                        float(resp.camera_orientation.z_val),
                    ),
                }

            data = adapter._exec_rpc(_fetch, timeout_s=10.0)
            self._last_fov = float(data.get("fov", 90.0))

            if not data.get("ok"):
                elapsed = int((time.time() - t0) * 1000)
                return DepthPointCloud(
                    False, [], (0, 0, 0), (1, 0, 0, 0),
                    0, 0, self._last_fov, 0, 0, elapsed,
                    error=str(data.get("error", "Unknown error")),
                )

            W = data["width"]
            H = data["height"]
            depth_img = np.array(data["depth"], dtype=np.float32).reshape(H, W)
            cam_pos = data["cam_pos"]
            cam_ori = data["cam_ori"]

            # -- numpy processing stays on the calling thread -----------------
            points, raw_count = depth_image_to_world_cloud(
                depth_img, cam_pos, cam_ori, self._last_fov,
                max_range_m=max_range_m, sample_step=sample_step,
                min_depth_m=min_depth_m,
            )

            if len(points) > max_points:
                indices = np.random.choice(len(points), max_points, replace=False)
                points = [points[i] for i in indices]

            elapsed = int((time.time() - t0) * 1000)
            return DepthPointCloud(
                True, points, cam_pos, cam_ori,
                W, H, self._last_fov, raw_count, len(points), elapsed,
            )

        except Exception as exc:
            elapsed = int((time.time() - t0) * 1000)
            logger.warning("DepthCamera capture failed: %s", exc)
            return DepthPointCloud(
                False, [], (0, 0, 0), (1, 0, 0, 0),
                0, 0, self._last_fov, 0, 0, elapsed,
                error=str(exc),
            )
