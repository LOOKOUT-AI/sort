"""
Reverse transformation functions: world-space → image-space.

These are the inverse of the functions in image_to_world.py, used to back-project
world-space track positions into image-space pixel coordinates for unmatched tracks.
"""

from __future__ import annotations

import math
from typing import Tuple

from .image_to_world import ENU, LatLon, latlon_to_enu_m, _deg2rad


def enu_to_right_fwd_m(enu_rel_ego: ENU, heading_deg: float) -> Tuple[float, float]:
    """
    Reverse of right_fwd_to_enu_m: convert ENU (east, north) relative to ego
    back to boat-relative (right_m, forward_m) coordinates.

    This inverts the rotation applied by right_fwd_to_enu_m.

    heading_deg convention:
      0 = north, 90 = east, clockwise positive (standard navigation heading).

    Forward transform was:
      east  = right * cos(h) + forward * sin(h)
      north = right * (-sin(h)) + forward * cos(h)

    Inverse (solve for right, forward):
      right   = east * cos(h) + north * (-sin(h))
      forward = east * sin(h) + north * cos(h)
    """
    h = _deg2rad(heading_deg)
    cos_h = math.cos(h)
    sin_h = math.sin(h)

    east = enu_rel_ego.east_m
    north = enu_rel_ego.north_m

    right_m = east * cos_h - north * sin_h
    forward_m = east * sin_h + north * cos_h

    return right_m, forward_m


def right_fwd_to_distance_bearing(right_m: float, forward_m: float) -> Tuple[float, float]:
    """
    Convert boat-relative (right_m, forward_m) to (distance_m, bearing_rad).

    bearing_rad: angle from camera forward axis, positive = right.
    This is the inverse of the polar-to-cartesian conversion in camera_relative_xy_m.

    Returns:
      distance_m: range to target (always >= 0)
      bearing_rad: angle from forward axis (positive = right, negative = left)
    """
    distance_m = math.hypot(right_m, forward_m)
    if distance_m < 1e-9:
        return 0.0, 0.0

    # atan2(right, forward) gives bearing from forward axis
    bearing_rad = math.atan2(right_m, forward_m)
    return distance_m, bearing_rad


def bearing_rad_to_pixel_x(
    bearing_rad: float,
    image_width_px: float,
    fov_x_rad: float,
    camera_yaw_offset_deg: float = 0.0,
) -> float:
    """
    Reverse of pixel_x_to_bearing_rad: convert bearing angle to x pixel coordinate.

    bearing_rad: angle from camera forward axis (positive = right)
    camera_yaw_offset_deg: yaw offset between camera and boat forward (applied in forward transform)

    Original forward transform:
      fx = image_width / (2 * tan(fov/2))
      dx = x - cx
      bearing = atan2(dx, fx) + camera_yaw_offset

    Reverse:
      bearing_cam = bearing - camera_yaw_offset
      dx = fx * tan(bearing_cam)
      x = cx + dx

    Returns:
      x_px: pixel x coordinate (may be outside [0, image_width] if target is outside FOV)
    """
    # Remove camera yaw offset to get bearing relative to camera axis
    bearing_cam = bearing_rad - _deg2rad(camera_yaw_offset_deg)

    cx = image_width_px / 2.0
    fx = image_width_px / (2.0 * math.tan(fov_x_rad / 2.0))

    dx = fx * math.tan(bearing_cam)
    x_px = cx + dx

    return x_px


def world_to_image_space(
    *,
    world_east_m: float,
    world_north_m: float,
    ego_latlon: LatLon,
    ego_heading_deg: float,
    image_width_px: float,
    fov_x_rad: float,
    camera_yaw_offset_deg: float = 0.0,
    local_ref: LatLon,
) -> Tuple[float, float, ENU]:
    """
    Convert world-space ENU position (relative to local_ref) back to image-space.

    This is the reverse of detection_to_world_latlon in image_to_world.py.

    Args:
        world_east_m: target east position in meters (relative to local_ref)
        world_north_m: target north position in meters (relative to local_ref)
        ego_latlon: current ego boat lat/lon
        ego_heading_deg: current ego boat heading (0=north, 90=east, clockwise)
        image_width_px: camera image width in pixels
        fov_x_rad: camera horizontal field of view in radians
        camera_yaw_offset_deg: yaw offset between camera and boat forward
        local_ref: reference lat/lon for ENU coordinate system

    Returns:
        x_center_px: pixel x coordinate of target center
        distance_m: distance from ego to target in meters
        enu_rel_ego: ENU offset from ego to target (for world_rel_ego_east_m, world_rel_ego_north_m)
    """
    # Step 1: Get ego position in ENU coordinates (relative to local_ref)
    ego_enu_from_ref = latlon_to_enu_m(local_ref, ego_latlon)

    # Step 2: Compute target position relative to ego in ENU
    enu_rel_ego = ENU(
        east_m=world_east_m - ego_enu_from_ref.east_m,
        north_m=world_north_m - ego_enu_from_ref.north_m,
    )

    # Step 3: Convert ENU relative to ego → boat-relative (right, forward)
    right_m, forward_m = enu_to_right_fwd_m(enu_rel_ego, ego_heading_deg)

    # Step 4: Convert (right, forward) → (distance, bearing)
    distance_m, bearing_rad = right_fwd_to_distance_bearing(right_m, forward_m)

    # Step 5: Convert bearing → pixel x
    x_center_px = bearing_rad_to_pixel_x(
        bearing_rad=bearing_rad,
        image_width_px=image_width_px,
        fov_x_rad=fov_x_rad,
        camera_yaw_offset_deg=camera_yaw_offset_deg,
    )

    return x_center_px, distance_m, enu_rel_ego
