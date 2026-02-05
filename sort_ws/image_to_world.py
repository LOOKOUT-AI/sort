from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

EARTH_RADIUS_M = 6378137.0  # WGS84 spherical approximation


@dataclass(frozen=True)
class LatLon:
    lat: float
    lon: float


@dataclass(frozen=True)
class ENU:
    """
    Local tangent plane coordinates in meters.
    +east: +x
    +north: +y
    """

    east_m: float
    north_m: float


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def wrap_angle_deg_0_360(deg: float) -> float:
    return (deg + 360) % 360.0


def heading_diff_deg(a: float, b: float) -> float:
    """
    Smallest signed difference (a - b) in degrees in [-180, 180).
    """
    return (a - b + 180.0) % 360.0 - 180.0


def latlon_to_enu_m(ref: LatLon, p: LatLon) -> ENU:
    """
    Projects latitude/longitude coordinates from WGS84 to East-North-Up (ENU) meters
    relative to a reference point, using a small-area equirectangular approximation.

    - ref: The origin reference point in (lat, lon), defining ENU (0,0).
    - p:   The target point in (lat, lon) to convert.

    How it works:
      1. Converts lat/lon degrees to radians.
      2. Computes the northward (latitude) and eastward (longitude) offsets between
         point p and the reference point, in meters:
         - North (y): proportional to latitude difference * earth radius.
         - East (x): proportional to longitude difference * earth radius * cos(ref latitude).
    Approximation:
      - Accurate for small/local regions (up to a few km), since Earth's curvature is
        not explicitly corrected except for the cosine(lat) longitude scaling.
      - Assumes a spherical Earth (radius=6378137m), not an ellipsoid.
    """
    lat0_rad = _deg2rad(ref.lat)
    dlat_rad = _deg2rad(p.lat - ref.lat)
    dlon_rad = _deg2rad(p.lon - ref.lon)
    north_m = EARTH_RADIUS_M * dlat_rad
    east_m = EARTH_RADIUS_M * dlon_rad * math.cos(lat0_rad)
    return ENU(east_m=east_m, north_m=north_m)


def enu_to_latlon(ref: LatLon, enu: ENU) -> LatLon:
    lat0 = _deg2rad(ref.lat)
    dlat = enu.north_m / EARTH_RADIUS_M
    dlon = 0.0
    denom = EARTH_RADIUS_M * math.cos(lat0)
    if abs(denom) > 1e-9:
        dlon = enu.east_m / denom
    return LatLon(
        lat=ref.lat + _rad2deg(dlat),
        lon=ref.lon + _rad2deg(dlon),
    )


def pixel_x_to_bearing_rad(
    x_px: float,
    image_width_px: float,
    fov_x_rad: float,
) -> float:
    """
    Bearing of a pixel column relative to the camera forward axis (0 at center).
    Positive = to the right.

    Mirrors the JS logic:
      theta = atan2(|x-cx|, cx / tan(fov/2))
      sign by left/right of center
    """
    cx = image_width_px / 2.0
    # focal length in "pixels" for this pinhole model
    fx = image_width_px / (2.0 * math.tan(fov_x_rad / 2.0))
    dx = x_px - cx
    return math.atan2(dx, fx)


def camera_relative_xy_m(
    x_bottom_center_px: float,
    distance_m: float,
    image_width_px: float,
    fov_x_rad: float,
    camera_yaw_offset_deg: float = 0.0,
) -> Tuple[float, float]:
    """
    Compute target position relative to camera/boat in a 2D ground plane:
      right_m: + right of camera
      forward_m: + forward from camera

    We only use horizontal FOV and bottom-center x pixel, matching the frontend code.
    """
    if not (distance_m > 0.0):
        return 0.0, 0.0
    theta = pixel_x_to_bearing_rad(x_bottom_center_px, image_width_px, fov_x_rad)
    theta = theta + _deg2rad(camera_yaw_offset_deg)
    right_m = float(distance_m) * math.sin(theta)
    forward_m = float(distance_m) * math.cos(theta)
    return right_m, forward_m


def right_fwd_to_enu_m(
    right_m: float,
    forward_m: float,
    heading_deg: float,
) -> ENU:
    """
    Convert boat-relative (right, forward) meters into ENU (east, north) meters.

    heading_deg convention:
      0 = north, 90 = east, clockwise positive (standard navigation heading).
    """
    h = _deg2rad(heading_deg)
    # unit vectors in ENU
    f_e = math.sin(h)
    f_n = math.cos(h)
    r_e = math.cos(h)
    r_n = -math.sin(h)
    east = right_m * r_e + forward_m * f_e
    north = right_m * r_n + forward_m * f_n
    return ENU(east_m=east, north_m=north)


def detection_to_world_latlon(
    *,
    ego_latlon: LatLon,
    ego_heading_deg: float,
    x_bottom_center_px: float,
    distance_m: float,
    image_width_px: float,
    fov_x_rad: float,
    camera_yaw_offset_deg: float = 0.0,
    local_ref: Optional[LatLon] = None,
) -> Tuple[LatLon, ENU, ENU]:
    """
    Convert a single detection into world space.

    Returns:
      - world_latlon: absolute lat/lon
      - enu_from_ref: target ENU position relative to local_ref (or ego if local_ref is None)
      - enu_rel_ego: target ENU offset relative to ego position (east/north meters)
    """
    right_m, forward_m = camera_relative_xy_m(
        x_bottom_center_px=x_bottom_center_px,
        distance_m=distance_m,
        image_width_px=image_width_px,
        fov_x_rad=fov_x_rad,
        camera_yaw_offset_deg=camera_yaw_offset_deg,
    )
    enu_rel_ego = right_fwd_to_enu_m(right_m=right_m, forward_m=forward_m, heading_deg=ego_heading_deg)

    # If caller doesn't specify a local reference, treat ego as reference.
    ref = local_ref or ego_latlon
    ego_enu_from_ref = latlon_to_enu_m(ref, ego_latlon)
    target_enu_from_ref = ENU(
        east_m=ego_enu_from_ref.east_m + enu_rel_ego.east_m,
        north_m=ego_enu_from_ref.north_m + enu_rel_ego.north_m,
    )
    world_latlon = enu_to_latlon(ref, target_enu_from_ref)
    return world_latlon, target_enu_from_ref, enu_rel_ego

