from __future__ import annotations

import asyncio
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Optional

import numpy as np
from filterpy.kalman import KalmanFilter

from .image_to_world import ENU, LatLon, latlon_to_enu_m, wrap_angle_deg_0_360


def _parse_iso8601(ts: str) -> Optional[datetime]:
    # Accept "...Z" and timezone-aware strings.
    try:
        if ts.endswith("Z"):
            ts = ts[:-1] + "+00:00"
        return datetime.fromisoformat(ts)
    except Exception:
        return None


def _is_ego_message(obj: Dict[str, Any]) -> bool:
    # Anything with MMSI is for AIS boats (per your note).
    return "MMSI" not in obj and "mmsi" not in obj


@dataclass
class EgoObservation:
    timestamp: Optional[datetime] = None
    latlon: Optional[LatLon] = None
    heading_deg: Optional[float] = None


@dataclass
class EgoState:
    """
    Smoothed ego state in geodetic + local coordinates.
    """

    timestamp: Optional[datetime]
    latlon: Optional[LatLon]
    heading_deg: Optional[float]
    ref_latlon: Optional[LatLon]
    enu_from_ref: Optional[ENU]


class EgoSmoother:
    """
    Lightweight ego smoother:
      - position: 2D constant-velocity Kalman filter in local ENU meters (east,north)
      - heading: exponential smoothing on the unit circle (handles wrap-around)

    This is intentionally separate from target tracking (no association needed).
    """

    def __init__(
        self,
        heading_alpha: float = 0.9,
        pos_process_var: float = 1.0,
        pos_meas_var: float = 4.0,
    ):
        self._ref: Optional[LatLon] = None
        self._kf: Optional[KalmanFilter] = None
        self._last_ts: Optional[datetime] = None

        # heading smoothing state
        self._heading_alpha = min(1.0, max(0.0, float(heading_alpha)))
        self._hx: Optional[float] = None  # cos
        self._hy: Optional[float] = None  # sin

        self._pos_process_var = float(pos_process_var)
        self._pos_meas_var = float(pos_meas_var)

    @property
    def ref_latlon(self) -> Optional[LatLon]:
        return self._ref

    def update(self, obs: EgoObservation) -> EgoState:
        if obs.latlon and self._ref is None:
            self._ref = obs.latlon

        if obs.heading_deg is not None:
            h = wrap_angle_deg_0_360(float(obs.heading_deg))
            c = math.cos(math.radians(h))
            s = math.sin(math.radians(h))
            if self._hx is None or self._hy is None:
                self._hx, self._hy = c, s
            else:
                a = self._heading_alpha
                self._hx = (1.0 - a) * self._hx + a * c
                self._hy = (1.0 - a) * self._hy + a * s
                # renormalize
                n = math.hypot(self._hx, self._hy)
                if n > 1e-9:
                    self._hx /= n
                    self._hy /= n

        # Position KF in ENU meters relative to ref
        if obs.latlon and self._ref:
            # lat/lon is on a curvature & non-linear, not suitable for standard kalman filter, which assumes linear system dynamics and measurement models; 
            # therefore convert to local ENU coords, which approximates linear
            meas = latlon_to_enu_m(self._ref, obs.latlon)
            ts = obs.timestamp or self._last_ts
            dt = 1.0
            if ts and self._last_ts:
                dt = max(1e-3, (ts - self._last_ts).total_seconds())

            if self._kf is None:
                # state is 4D: [east, north, ve, vn], measurement is 2D: [eeast, north]
                kf = KalmanFilter(dim_x=4, dim_z=2)
                # kf.x: The initial state vector for the Kalman filter; here, it represents [east, north, ve, vn] (positions in ENU meters and their velocities), initialized with measurement and zero velocities.
                # kf.H: The measurement/observation matrix, mapping the state to the measurement space ([east, north] only).
                # kf.R: The measurement covariance matrix, representing expected observation noise, scaled by self._pos_meas_var.
                # kf.P: The initial state covariance matrix, representing initial estimation uncertainty; diagonals at 10.0 for moderate uncertainty.
                kf.x = np.array([[meas.east_m], [meas.north_m], [0.0], [0.0]], dtype=np.float32)
                kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
                kf.R = np.eye(2, dtype=np.float32) * self._pos_meas_var
                kf.P = np.eye(4, dtype=np.float32) * 10.0
                self._kf = kf

            # update F/Q with dt
            # kf.F: The state transition matrix, modeling how the state ideally (deterministically) evolves from one timestep to the next.
            # kf.Q: The process noise covariance matrix, representing uncertainty/randomness in the process/model dynamics.
            assert self._kf is not None
            self._kf.F = np.array(
                [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
                dtype=np.float32,
            )
            q = self._pos_process_var
            self._kf.Q = np.array(
                [[dt**4 / 4, 0, dt**3 / 2, 0], [0, dt**4 / 4, 0, dt**3 / 2], [dt**3 / 2, 0, dt**2, 0], [0, dt**3 / 2, 0, dt**2]],
                dtype=np.float32,
            ) * q

            self._kf.predict()
            self._kf.update(np.array([[meas.east_m], [meas.north_m]], dtype=np.float32))
            self._last_ts = ts

        heading_out: Optional[float] = None
        if self._hx is not None and self._hy is not None:
            heading_out = wrap_angle_deg_0_360(math.degrees(math.atan2(self._hy, self._hx)))

        latlon_out: Optional[LatLon] = obs.latlon
        enu_from_ref: Optional[ENU] = None
        if self._kf is not None and self._ref is not None:
            east = float(self._kf.x[0, 0])
            north = float(self._kf.x[1, 0])
            enu_from_ref = ENU(east_m=east, north_m=north)

        return EgoState(
            timestamp=obs.timestamp,
            latlon=latlon_out,
            heading_deg=heading_out if heading_out is not None else obs.heading_deg,
            ref_latlon=self._ref,
            enu_from_ref=enu_from_ref,
        )


class EgoStateStore:
    def __init__(self, smoother: EgoSmoother):
        self._lock = asyncio.Lock()
        self._smoother = smoother
        self._latest_obs = EgoObservation()
        self._latest_state = EgoState(
            timestamp=None, latlon=None, heading_deg=None, ref_latlon=None, enu_from_ref=None
        )

    async def update_from_nmea_json(self, msg_text: str) -> None:
        try:
            obj = json.loads(msg_text)
        except Exception:
            return
        if not isinstance(obj, dict):
            return
        if not _is_ego_message(obj):
            return

        ts = obj.get("timestamp")
        dt = _parse_iso8601(ts) if isinstance(ts, str) else None

        lat = obj.get("Latitude")
        lon = obj.get("Longitude")
        heading = obj.get("Heading")

        async with self._lock:
            changed = False
            if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                self._latest_obs.latlon = LatLon(lat=float(lat), lon=float(lon))
                self._latest_obs.timestamp = dt or self._latest_obs.timestamp
                changed = True
            if isinstance(heading, (int, float)):
                self._latest_obs.heading_deg = float(heading)
                self._latest_obs.timestamp = dt or self._latest_obs.timestamp
                changed = True

            if changed:
                self._latest_state = self._smoother.update(self._latest_obs)

    async def get(self) -> EgoState:
        async with self._lock:
            return self._latest_state

