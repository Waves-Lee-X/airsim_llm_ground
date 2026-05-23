from __future__ import annotations

import math

import numpy as np

from core.depth_camera import (
    _quaternion_to_rotation_matrix,
    depth_image_to_world_cloud,
    world_to_ned_points,
)


class TestQuaternionToRotationMatrix:
    """Verify quaternion → 3x3 rotation matrix conversion."""

    def test_identity_quaternion_gives_identity_matrix(self):
        R = _quaternion_to_rotation_matrix(1.0, 0.0, 0.0, 0.0)
        assert np.allclose(R, np.eye(3), atol=1e-10)

    def test_90_degree_yaw_gives_expected_rotation(self):
        cos45 = math.cos(math.radians(45))
        sin45 = math.sin(math.radians(45))
        R = _quaternion_to_rotation_matrix(cos45, 0.0, 0.0, sin45)
        expected = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        assert np.allclose(R, expected, atol=1e-10)

    def test_rotation_matrix_is_orthonormal(self):
        qw, qx, qy, qz = 0.7071, 0.0, 0.7071, 0.0
        R = _quaternion_to_rotation_matrix(qw, qx, qy, qz)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-4)
        assert abs(np.linalg.det(R) - 1.0) < 1e-4


class TestWorldToNEDPoints:
    """Verify camera-frame → world NED frame transformation."""

    def test_identity_transform_no_rotation_no_offset(self):
        cam_pos = (0.0, 0.0, 0.0)
        cam_ori = (1.0, 0.0, 0.0, 0.0)
        pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float64)
        result = world_to_ned_points(cam_pos, cam_ori, pts)
        assert np.allclose(result, pts)

    def test_translation_only(self):
        cam_pos = (10.0, -5.0, 3.0)
        cam_ori = (1.0, 0.0, 0.0, 0.0)
        pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
        result = world_to_ned_points(cam_pos, cam_ori, pts)
        assert np.allclose(result, np.array([[10.0, -5.0, 3.0]]))


class TestDepthImageToWorldCloud:
    """Depth image → world point cloud conversion."""

    @staticmethod
    def _sample_depth(w: int = 160, h: int = 90, fill_m: float = 10.0) -> np.ndarray:
        return np.full((h, w), fill_m, dtype=np.float32)

    def test_returns_empty_list_for_all_invalid_depth(self):
        img = np.zeros((10, 10), dtype=np.float32)
        points, raw = depth_image_to_world_cloud(
            img, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=40.0, sample_step=2, min_depth_m=0.5,
        )
        assert points == []
        assert raw == 0

    def test_returns_points_for_valid_depth(self):
        img = self._sample_depth(80, 45, fill_m=5.0)
        points, raw = depth_image_to_world_cloud(
            img, (0.0, 0.0, -8.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=40.0, sample_step=4, min_depth_m=0.5,
        )
        assert len(points) > 0
        assert raw > 0
        assert all(len(p) == 3 for p in points)

    def test_points_are_in_world_frame_identity_orientation(self):
        img = self._sample_depth(80, 45, fill_m=5.0)
        cam_pos = (0.0, 0.0, -8.0)
        cam_ori = (1.0, 0.0, 0.0, 0.0)
        points, _raw = depth_image_to_world_cloud(
            img, cam_pos, cam_ori, fov_deg=90.0,
            max_range_m=40.0, sample_step=4, min_depth_m=0.5,
        )
        for x, y, z in points:
            assert abs(x) < 40.0
            assert abs(y) < 40.0

    def test_clamps_to_max_range(self):
        img = self._sample_depth(80, 45, fill_m=100.0)
        points, raw = depth_image_to_world_cloud(
            img, (0.0, 0.0, -8.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=5.0, sample_step=4, min_depth_m=0.5,
        )
        assert points == []
        assert raw == 0

    def test_filters_below_min_depth(self):
        img = self._sample_depth(80, 45, fill_m=0.1)
        points, _raw = depth_image_to_world_cloud(
            img, (0.0, 0.0, -8.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=40.0, sample_step=4, min_depth_m=0.5,
        )
        assert points == []

    def test_respects_sample_step(self):
        img = self._sample_depth(80, 45, fill_m=5.0)
        points_full, _ = depth_image_to_world_cloud(
            img, (0.0, 0.0, -8.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=40.0, sample_step=2, min_depth_m=0.5,
        )
        points_sparse, _ = depth_image_to_world_cloud(
            img, (0.0, 0.0, -8.0), (1.0, 0.0, 0.0, 0.0),
            fov_deg=90.0, max_range_m=40.0, sample_step=8, min_depth_m=0.5,
        )
        assert len(points_full) > len(points_sparse)
