#!/usr/bin/env python3
"""
geodesic_utils.py — WGS84 测地线计算工具

来源：整合自 px4-ros2-interface-lib 的 geodesic.hpp + airsim_ros_pkgs 的 geodetic_conv.hpp
用途：GPS 坐标 ↔ 本地 ENU 坐标转换、两点距离、方位角计算

坐标系：
  - WGS84: 纬度(deg), 经度(deg), 海拔(m)
  - ENU:   x=东(m), y=北(m), z=上(m), 原点在参考点
"""

import math

# WGS84 椭球参数
WGS84_A = 6378137.0           # 长半轴 (m)
WGS84_B = 6356752.314245      # 短半轴 (m)
WGS84_F = 1.0 / 298.257223563 # 扁率
WGS84_E2 = 0.00669437999014   # 第一偏心率平方


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wrap_pi(angle: float) -> float:
    """角度归一化到 [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def geodetic_to_enu(lat: float, lon: float, alt: float,
                    ref_lat: float, ref_lon: float, ref_alt: float):
    """WGS84 经纬高 → 本地 ENU 坐标

    Args:
        lat, lon, alt: 目标点 (deg, deg, m)
        ref_lat, ref_lon, ref_alt: 参考原点 (deg, deg, m)

    Returns:
        (east, north, up): ENU 坐标 (m)
    """
    lat_r = deg2rad(lat)
    lon_r = deg2rad(lon)
    ref_lat_r = deg2rad(ref_lat)
    ref_lon_r = deg2rad(ref_lon)

    # 参考点曲率半径
    sin_ref_lat = math.sin(ref_lat_r)
    n_ref = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_ref_lat * sin_ref_lat)

    # 参考点 ECEF
    x_ref = (n_ref + ref_alt) * math.cos(ref_lat_r) * math.cos(ref_lon_r)
    y_ref = (n_ref + ref_alt) * math.cos(ref_lat_r) * math.sin(ref_lon_r)
    z_ref = (n_ref * (1.0 - WGS84_E2) + ref_alt) * math.sin(ref_lat_r)

    # 目标点曲率半径
    sin_lat = math.sin(lat_r)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    # 目标点 ECEF
    x = (n + alt) * math.cos(lat_r) * math.cos(lon_r)
    y = (n + alt) * math.cos(lat_r) * math.sin(lon_r)
    z = (n * (1.0 - WGS84_E2) + alt) * math.sin(lat_r)

    # ECEF 差 → ENU
    dx = x - x_ref
    dy = y - y_ref
    dz = z - z_ref

    east = -math.sin(ref_lon_r) * dx + math.cos(ref_lon_r) * dy
    north = (-math.sin(ref_lat_r) * math.cos(ref_lon_r) * dx
             - math.sin(ref_lat_r) * math.sin(ref_lon_r) * dy
             + math.cos(ref_lat_r) * dz)
    up = (math.cos(ref_lat_r) * math.cos(ref_lon_r) * dx
          + math.cos(ref_lat_r) * math.sin(ref_lon_r) * dy
          + math.sin(ref_lat_r) * dz)

    return (east, north, up)


def enu_to_geodetic(east: float, north: float, up: float,
                    ref_lat: float, ref_lon: float, ref_alt: float):
    """本地 ENU 坐标 → WGS84 经纬高

    Args:
        east, north, up: ENU 坐标 (m)
        ref_lat, ref_lon, ref_alt: 参考原点 (deg, deg, m)

    Returns:
        (lat, lon, alt): 经纬高 (deg, deg, m)
    """
    ref_lat_r = deg2rad(ref_lat)
    ref_lon_r = deg2rad(ref_lon)

    sin_ref_lat = math.sin(ref_lat_r)
    n_ref = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_ref_lat * sin_ref_lat)

    # 参考点 ECEF
    x_ref = (n_ref + ref_alt) * math.cos(ref_lat_r) * math.cos(ref_lon_r)
    y_ref = (n_ref + ref_alt) * math.cos(ref_lat_r) * math.sin(ref_lon_r)
    z_ref = (n_ref * (1.0 - WGS84_E2) + ref_alt) * math.sin(ref_lat_r)

    # ENU → ECEF
    x = x_ref - math.sin(ref_lon_r) * east \
        - math.sin(ref_lat_r) * math.cos(ref_lon_r) * north \
        + math.cos(ref_lat_r) * math.cos(ref_lon_r) * up
    y = y_ref + math.cos(ref_lon_r) * east \
        - math.sin(ref_lat_r) * math.sin(ref_lon_r) * north \
        + math.cos(ref_lat_r) * math.sin(ref_lon_r) * up
    z = z_ref + math.cos(ref_lat_r) * north + math.sin(ref_lat_r) * up

    # ECEF → 经纬高 (迭代)
    lon_r = math.atan2(y, x)
    p = math.sqrt(x * x + y * y)
    lat_r = math.atan2(z, p * (1.0 - WGS84_E2))

    for _ in range(5):
        sin_lat = math.sin(lat_r)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / math.cos(lat_r) - n
        lat_r = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))

    return (rad2deg(lat_r), rad2deg(lon_r), alt)


def distance_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine 公式计算两点间水平距离 (m)"""
    lat1_r = deg2rad(lat1)
    lat2_r = deg2rad(lat2)
    dlat = deg2rad(lat2 - lat1)
    dlon = deg2rad(lon2 - lon1)

    a = math.sin(dlat / 2) ** 2 \
        + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return WGS84_A * c


def heading_between(lat1: float, lon1: float,
                    lat2: float, lon2: float) -> float:
    """两 GPS 点之间的方位角 (rad, 从北顺时针)"""
    lat1_r = deg2rad(lat1)
    lat2_r = deg2rad(lat2)
    dlon = deg2rad(lon2 - lon1)

    y = math.sin(dlon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) \
        - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
    return math.atan2(y, x)
