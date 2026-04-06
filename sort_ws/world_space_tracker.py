from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from filterpy.kalman import KalmanFilter

from .image_space_tracker import linear_assignment  # reuse lap/scipy wrapper
from .image_to_world import heading_diff_deg

# Default dt (seconds) used on the very first assign() call when no previous
# timestamp exists.  The exact value doesn't matter much because there are
# typically no existing trackers to predict on the first frame.
_DEFAULT_DT_S: float = 1.0 / 30.0

# Maximum dt (seconds) allowed between consecutive assign() calls.  Caps the
# time step to prevent numerical instability and unrealistic covariance growth
# when there's a long pause (e.g., replay paused, reconnect, seek).
_MAX_DT_S: float = 2.0


def _matrix2_to_nested_list(mat: np.ndarray) -> List[List[float]]:
    """Convert a 2x2 numpy array into nested Python float lists for JSON transport."""
    return [
        [float(mat[0, 0]), float(mat[0, 1])],
        [float(mat[1, 0]), float(mat[1, 1])],
    ]


def _clamp_vector(vx: float, vy: float, vmax: float) -> Tuple[float, float]:
    """Clamp 2D vector magnitude to *vmax*, preserving direction.

    Always active.  *vmax* == 0 zeroes the vector; *vmax* > 0 caps
    magnitude while keeping direction.  Set a large value (e.g. 1000)
    if you effectively don't want a constraint.
    """
    if vmax <= 0.0:
        return 0.0, 0.0
    mag = math.hypot(vx, vy)
    if mag <= vmax or mag < 1e-12:
        return vx, vy
    s = vmax / mag
    return vx * s, vy * s


def _clamp(x: float, low: float, high: float) -> float:
    if high < low:
        low, high = high, low
    return max(low, min(high, float(x)))


def _remap01(x: float, low: float, high: float) -> float:
    if not math.isfinite(x):
        return 0.0
    low_f = float(low)
    high_f = float(high)
    if high_f <= low_f:
        return 0.0
    return _clamp((float(x) - low_f) / (high_f - low_f), 0.0, 1.0)


def _inverted_remap01(x: float, low: float, high: float) -> float:
    return 1.0 - _remap01(x, low, high)


def _safe_cosine_similarity(ax: float, ay: float, bx: float, by: float, *, near_zero_eps: float = 1e-6) -> float:
    amag = math.hypot(ax, ay)
    bmag = math.hypot(bx, by)
    if amag < near_zero_eps or bmag < near_zero_eps:
        return 1.0
    return _clamp((ax * bx + ay * by) / (amag * bmag), -1.0, 1.0)


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "t", "yes", "y", "on"):
            return True
        if lowered in ("0", "false", "f", "no", "n", "off", ""):
            return False
    raise ValueError(f"Cannot interpret {value!r} as bool")


def _measurement_covariance_from_rel_enu(
    rel_east_m: float,
    rel_north_m: float,
    cross_var_m2: float,
    radial_scale: float,
) -> np.ndarray:
    """Build a 2x2 ENU measurement covariance aligned to radial/cross-range axes."""
    cross_var = max(1e-6, float(cross_var_m2))
    radial_var = max(1e-6, cross_var * max(1e-6, float(radial_scale)))

    try:
        re = float(rel_east_m)
        rn = float(rel_north_m)
    except Exception:
        iso = max(cross_var, radial_var)
        return np.eye(2, dtype=np.float32) * iso

    norm = math.hypot(re, rn)
    if norm < 1e-6 or not math.isfinite(norm):
        iso = max(cross_var, radial_var)
        return np.eye(2, dtype=np.float32) * iso

    radial = np.array([re / norm, rn / norm], dtype=np.float32)
    cross = np.array([-radial[1], radial[0]], dtype=np.float32)
    rr_t = np.outer(radial, radial)
    cc_t = np.outer(cross, cross)
    return (radial_var * rr_t + cross_var * cc_t).astype(np.float32)


@dataclass
class WorldTrackExtras:
    confidence: float = float("nan")
    category: Optional[str] = None
    heading_deg: float = float("nan")  # if the detector provides a target heading estimate
    rel_east_m: float = float("nan")
    rel_north_m: float = float("nan")
    # Store last known bbox geometry for back-projection of unmatched tracks
    last_y_px: float = float("nan")
    last_width_px: float = float("nan")
    last_height_px: float = float("nan")


class KalmanCVPointTracker:
    """
    Track a single target in world space (local ENU meters), using a constant-velocity (CV) KF.

    State: [east, north, ve, vn]
    Measurement: [east, north]
    """

    count = 0

    def __init__(
        self,
        meas_enu: np.ndarray,
        extras: WorldTrackExtras,
        *,
        q_intensity: float = 0.0,
        measurement_noise_cross_var_m2: float = 1e6,
        max_speed_boat_mps: float = 50.0,
        max_speed_other_mps: float = 50.0,
        max_accel_boat_mps2: float = 20.0,   # accepted but unused (CV has no accel state)
        max_accel_other_mps2: float = 20.0,  # accepted but unused
        measurement_noise_radial_scale: float = 200.0,
    ):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)

        # Process noise intensity (continuous white-noise acceleration spectral
        # density).  F and Q are rebuilt every predict() call with the real dt.
        # Scaled up from the original 1.0 to compensate for dt now being in
        # seconds (~0.03–0.05) rather than the old hardcoded dt=1.  Q_vv scales
        # as q * dt², so q ≈ 1/dt² ≈ 400 restores comparable per-step noise.
        # self._q_intensity: float = 400.0
        self._q_intensity = max(0.0, float(q_intensity))

        # Initialise F and Q with a nominal dt so filterpy internals have the
        # correct shape/dtype.  They will be overwritten on the first predict().
        self._rebuild_dynamics(_DEFAULT_DT_S)

        # Measurement covariance. Cross-range keeps the legacy tuned variance,
        # while radial variance can be scaled independently to smooth range jitter.
        self.measurement_noise_cross_var_m2 = max(1e-6, float(measurement_noise_cross_var_m2))
        self.measurement_noise_radial_scale = max(1e-6, float(measurement_noise_radial_scale))
        self.kf.R = np.eye(2, dtype=np.float32) * self.measurement_noise_cross_var_m2
        self.kf.P = np.eye(4, dtype=np.float32) * 10.0
        self.kf.P[2:, 2:] *= 1000.0  # velocities initially very uncertain

        self.kf.x = np.array([[float(meas_enu[0])], [float(meas_enu[1])], [0.0], [0.0]], dtype=np.float32)

        self.time_since_update = 0
        self.id = KalmanCVPointTracker.count
        KalmanCVPointTracker.count += 1
        self.hits = 1
        self.hit_streak = 1
        self.age = 0
        self.extras = extras

        # Per-category speed caps (always active; use a large value to effectively uncap)
        self.max_speed_boat_mps = float(max_speed_boat_mps)
        self.max_speed_other_mps = float(max_speed_other_mps)
        self._update_measurement_covariance(extras)

    # ------------------------------------------------------------------
    # State clamping (applied after both predict and update)
    # ------------------------------------------------------------------

    def _resolve_max_speed(self) -> float:
        """Return the active speed cap based on the track's current category."""
        cat = (self.extras.category or "").lower()
        return self.max_speed_boat_mps if cat == "boat" else self.max_speed_other_mps

    def _clamp_state(self) -> None:
        """Clamp velocity to the category-specific max speed, preserving direction."""
        max_speed = self._resolve_max_speed()
        x = self.kf.x
        ve, vn = float(x[2, 0]), float(x[3, 0])
        ve_c, vn_c = _clamp_vector(ve, vn, max_speed)
        x[2, 0] = ve_c
        x[3, 0] = vn_c

    def _update_measurement_covariance(self, extras: WorldTrackExtras) -> None:
        self.kf.R = _measurement_covariance_from_rel_enu(
            extras.rel_east_m,
            extras.rel_north_m,
            self.measurement_noise_cross_var_m2,
            self.measurement_noise_radial_scale,
        )

    def set_measurement_noise_radial_scale(self, value: float) -> None:
        self.measurement_noise_radial_scale = max(1e-6, float(value))
        self._update_measurement_covariance(self.extras)

    def set_measurement_noise_cross_var_m2(self, value: float) -> None:
        self.measurement_noise_cross_var_m2 = max(1e-6, float(value))
        self._update_measurement_covariance(self.extras)

    def set_q_intensity(self, value: float) -> None:
        self._q_intensity = max(0.0, float(value))

    # ------------------------------------------------------------------
    # Dynamics rebuilding (called every predict with real dt in seconds)
    # ------------------------------------------------------------------

    def _rebuild_dynamics(self, dt: float) -> None:
        """Rebuild F and Q matrices for the given time step *dt* (seconds).

        CV state transition:
            east'  = east + ve * dt
            north' = north + vn * dt
            ve'    = ve
            vn'    = vn

        Q is the standard continuous white-noise acceleration (CWNA) model.
        """
        self.kf.F = np.array(
            [[1, 0, dt, 0],
             [0, 1, 0, dt],
             [0, 0, 1, 0],
             [0, 0, 0, 1]],
            dtype=np.float32,
        )
        q = self._q_intensity
        self.kf.Q = np.array(
            [[dt**4 / 4, 0, dt**3 / 2, 0],
             [0, dt**4 / 4, 0, dt**3 / 2],
             [dt**3 / 2, 0, dt**2, 0],
             [0, dt**3 / 2, 0, dt**2]],
            dtype=np.float32,
        ) * q

    def update(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        self._update_measurement_covariance(extras)
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()
        self.extras = extras

    def update_rewarming(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        """Mark a matched rewarming track as seen without correcting the KF state."""
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1

        self._update_measurement_covariance(extras)
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()

        self.extras = extras

    def predict(self, dt: float = _DEFAULT_DT_S) -> np.ndarray:
        """Predict one step forward by *dt* seconds."""
        self._rebuild_dynamics(dt)
        self.kf.predict()
        self._clamp_state()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        return self.get_state()

    def get_state(self) -> np.ndarray:
        # return [east, north]
        return self.kf.x[:2, 0].astype(np.float32)

    def get_position_covariances(self) -> Dict[str, List[List[float]]]:
        """Return 2x2 ENU position covariance blocks for state, process, and measurement."""
        return {
            "state_position_covariance_enu": _matrix2_to_nested_list(self.kf.P[:2, :2]),
            "process_position_covariance_enu": _matrix2_to_nested_list(self.kf.Q[:2, :2]),
            "measurement_position_covariance_enu": _matrix2_to_nested_list(self.kf.R[:2, :2]),
        }

    def get_measurement_noise_scalars(self) -> Dict[str, float]:
        """Return the live radial/cross measurement variances used to build R."""
        cross_var = max(1e-6, float(self.measurement_noise_cross_var_m2))
        radial_var = max(1e-6, cross_var * max(1e-6, float(self.measurement_noise_radial_scale)))
        return {
            "measurement_noise_cross_var_m2": cross_var,
            "measurement_noise_radial_var_m2": radial_var,
        }

    def get_full_state(self) -> Dict[str, Any]:
        """Return full KF state as a dict: position, velocity, and covariance summaries."""
        x = self.kf.x.flatten()
        east, north, ve, vn = float(x[0]), float(x[1]), float(x[2]), float(x[3])
        speed = math.hypot(ve, vn)
        course = math.degrees(math.atan2(ve, vn)) % 360.0  # 0=N, 90=E
        return {
            "east_m": east,
            "north_m": north,
            "vel_east_mps": ve,
            "vel_north_mps": vn,
            "speed_mps": speed,
            "course_deg": course,
            "accel_east_mps2": None,
            "accel_north_mps2": None,
            **self.get_measurement_noise_scalars(),
            **self.get_position_covariances(),
        }


class KalmanCAPointTracker:
    """
    Track a single target in world space (local ENU meters), using a constant-acceleration (CA) KF.

    State: [east, north, ve, vn, ae, an]
    Measurement: [east, north]
    """

    count = 0

    def __init__(
        self,
        meas_enu: np.ndarray,
        extras: WorldTrackExtras,
        *,
        q_intensity: float = 0.0,
        measurement_noise_cross_var_m2: float = 1e6,
        max_speed_boat_mps: float = 50.0,
        max_speed_other_mps: float = 50.0,
        max_accel_boat_mps2: float = 20.0,
        max_accel_other_mps2: float = 20.0,
        measurement_noise_radial_scale: float = 200.0,
    ):
        self.kf = KalmanFilter(dim_x=6, dim_z=2)
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0]],
            dtype=np.float32,
        )

        # Process noise intensity (continuous white-noise jerk spectral density).
        # F and Q are rebuilt every predict() call with the real dt.
        self._q_intensity = max(0.0, float(q_intensity))

        # Initialise F and Q with a nominal dt so filterpy internals have the
        # correct shape/dtype.  They will be overwritten on the first predict().
        self._rebuild_dynamics(_DEFAULT_DT_S)

        # Measurement covariance. Cross-range keeps the legacy tuned variance,
        # while radial variance can be scaled independently to smooth range jitter.
        self.measurement_noise_cross_var_m2 = max(1e-6, float(measurement_noise_cross_var_m2))
        self.measurement_noise_radial_scale = max(1e-6, float(measurement_noise_radial_scale))
        self.kf.R = np.eye(2, dtype=np.float32) * self.measurement_noise_cross_var_m2

        # Initial state covariance.
        self.kf.P = np.eye(6, dtype=np.float32) * 10.0
        self.kf.P[2:4, 2:4] *= 1000.0   # velocities initially very uncertain
        self.kf.P[4:6, 4:6] *= 10000.0   # accelerations even more uncertain

        self.kf.x = np.array(
            [[float(meas_enu[0])], [float(meas_enu[1])], [0.0], [0.0], [0.0], [0.0]],
            dtype=np.float32,
        )

        self.time_since_update = 0
        self.id = KalmanCAPointTracker.count
        KalmanCAPointTracker.count += 1
        self.hits = 1
        self.hit_streak = 1
        self.age = 0
        self.extras = extras

        # Per-category speed / acceleration caps (always active; use a large value to effectively uncap)
        self.max_speed_boat_mps = float(max_speed_boat_mps)
        self.max_speed_other_mps = float(max_speed_other_mps)
        self.max_accel_boat_mps2 = float(max_accel_boat_mps2)
        self.max_accel_other_mps2 = float(max_accel_other_mps2)
        self._update_measurement_covariance(extras)

    # ------------------------------------------------------------------
    # State clamping (applied after both predict and update)
    # ------------------------------------------------------------------

    def _resolve_limits(self) -> Tuple[float, float]:
        """Return (max_speed, max_accel) based on the track's current category."""
        cat = (self.extras.category or "").lower()
        if cat == "boat":
            return self.max_speed_boat_mps, self.max_accel_boat_mps2
        return self.max_speed_other_mps, self.max_accel_other_mps2

    def _clamp_state(self) -> None:
        """Clamp velocity and acceleration to category-specific maxima, preserving direction."""
        max_speed, max_accel = self._resolve_limits()
        x = self.kf.x
        ve, vn = float(x[2, 0]), float(x[3, 0])
        ve_c, vn_c = _clamp_vector(ve, vn, max_speed)
        x[2, 0] = ve_c
        x[3, 0] = vn_c
        ae, an = float(x[4, 0]), float(x[5, 0])
        ae_c, an_c = _clamp_vector(ae, an, max_accel)
        x[4, 0] = ae_c
        x[5, 0] = an_c

    def _update_measurement_covariance(self, extras: WorldTrackExtras) -> None:
        self.kf.R = _measurement_covariance_from_rel_enu(
            extras.rel_east_m,
            extras.rel_north_m,
            self.measurement_noise_cross_var_m2,
            self.measurement_noise_radial_scale,
        )

    def set_measurement_noise_radial_scale(self, value: float) -> None:
        self.measurement_noise_radial_scale = max(1e-6, float(value))
        self._update_measurement_covariance(self.extras)

    def set_measurement_noise_cross_var_m2(self, value: float) -> None:
        self.measurement_noise_cross_var_m2 = max(1e-6, float(value))
        self._update_measurement_covariance(self.extras)

    def set_q_intensity(self, value: float) -> None:
        self._q_intensity = max(0.0, float(value))

    # ------------------------------------------------------------------
    # Dynamics rebuilding (called every predict with real dt in seconds)
    # ------------------------------------------------------------------

    def _rebuild_dynamics(self, dt: float) -> None:
        """Rebuild F and Q matrices for the given time step *dt* (seconds).

        CA state transition:
            east'  = east + ve*dt + 0.5*ae*dt²
            north' = north + vn*dt + 0.5*an*dt²
            ve'    = ve + ae*dt
            vn'    = vn + an*dt
            ae'    = ae
            an'    = an

        Q is driven by jerk (white noise) as the unmodeled disturbance.
        Per-axis noise gain: G = [dt³/6, dt²/2, dt].
        """
        # fmt: off
        self.kf.F = np.array(
            [[1, 0, dt, 0,  dt**2/2, 0      ],
             [0, 1, 0,  dt, 0,       dt**2/2],
             [0, 0, 1,  0,  dt,      0      ],
             [0, 0, 0,  1,  0,       dt     ],
             [0, 0, 0,  0,  1,       0      ],
             [0, 0, 0,  0,  0,       1      ]],
            dtype=np.float32,
        )
        # fmt: on
        q_jerk = self._q_intensity
        g = np.array([dt**3 / 6.0, dt**2 / 2.0, dt], dtype=np.float32)
        q_axis = q_jerk * np.outer(g, g)  # 3×3
        # Assemble 6×6 block-diagonal Q for [e, n, ve, vn, ae, an] ordering
        # East indices: 0, 2, 4  |  North indices: 1, 3, 5
        self.kf.Q = np.zeros((6, 6), dtype=np.float32)
        east_idx = [0, 2, 4]
        north_idx = [1, 3, 5]
        for i_s, i_full in enumerate(east_idx):
            for j_s, j_full in enumerate(east_idx):
                self.kf.Q[i_full, j_full] = q_axis[i_s, j_s]
        for i_s, i_full in enumerate(north_idx):
            for j_s, j_full in enumerate(north_idx):
                self.kf.Q[i_full, j_full] = q_axis[i_s, j_s]

    def update(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        self._update_measurement_covariance(extras)
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()
        self.extras = extras

    def update_rewarming(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        """Mark a matched rewarming track as seen without correcting the KF state."""
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1

        self._update_measurement_covariance(extras)
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()

        self.extras = extras

    def predict(self, dt: float = _DEFAULT_DT_S) -> np.ndarray:
        """Predict one step forward by *dt* seconds."""
        self._rebuild_dynamics(dt)
        self.kf.predict()
        self._clamp_state()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        return self.get_state()

    def get_state(self) -> np.ndarray:
        # return [east, north]
        return self.kf.x[:2, 0].astype(np.float32)

    def get_position_covariances(self) -> Dict[str, List[List[float]]]:
        """Return 2x2 ENU position covariance blocks for state, process, and measurement."""
        return {
            "state_position_covariance_enu": _matrix2_to_nested_list(self.kf.P[:2, :2]),
            "process_position_covariance_enu": _matrix2_to_nested_list(self.kf.Q[:2, :2]),
            "measurement_position_covariance_enu": _matrix2_to_nested_list(self.kf.R[:2, :2]),
        }

    def get_measurement_noise_scalars(self) -> Dict[str, float]:
        """Return the live radial/cross measurement variances used to build R."""
        cross_var = max(1e-6, float(self.measurement_noise_cross_var_m2))
        radial_var = max(1e-6, cross_var * max(1e-6, float(self.measurement_noise_radial_scale)))
        return {
            "measurement_noise_cross_var_m2": cross_var,
            "measurement_noise_radial_var_m2": radial_var,
        }

    def get_full_state(self) -> Dict[str, Any]:
        """Return full KF state as a dict: position, velocity, acceleration, and covariance summaries."""
        x = self.kf.x.flatten()
        east, north = float(x[0]), float(x[1])
        ve, vn = float(x[2]), float(x[3])
        ae, an = float(x[4]), float(x[5])
        speed = math.hypot(ve, vn)
        course = math.degrees(math.atan2(ve, vn)) % 360.0  # 0=N, 90=E
        return {
            "east_m": east,
            "north_m": north,
            "vel_east_mps": ve,
            "vel_north_mps": vn,
            "speed_mps": speed,
            "course_deg": course,
            "accel_east_mps2": ae,
            "accel_north_mps2": an,
            **self.get_measurement_noise_scalars(),
            **self.get_position_covariances(),
        }


def _euclid_dist_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    a: (N,2), b: (M,2) -> (N,M) euclidean distances
    """
    if a.size == 0 or b.size == 0:
        return np.zeros((a.shape[0], b.shape[0]), dtype=np.float32)
    aa = a[:, None, :]  # (N,1,2)
    bb = b[None, :, :]  # (1,M,2)
    d = aa - bb
    return np.sqrt(np.sum(d * d, axis=2)).astype(np.float32)


def associate_world_detections_to_trackers(
    det_xy: np.ndarray,
    trk_xy: np.ndarray,
    det_heading: np.ndarray,
    trk_heading: np.ndarray,
    det_conf: np.ndarray,
    *,
    max_distance_m: float = 30.0,
    beta_heading: float = 0.0,
    gamma_confidence: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Association in world space.

    Gating: distance <= max_distance_m
    Cost (Hungarian): distance + beta*heading_cost + gamma*(1-confidence)
    """
    if trk_xy.shape[0] == 0:
        return (
            np.empty((0, 2), dtype=int),
            np.arange(det_xy.shape[0], dtype=int),
            np.empty((0,), dtype=int),
        )

    dist = _euclid_dist_matrix(det_xy, trk_xy)  # (N,M)
    cost = dist.copy()

    if beta_heading and det_heading.size and trk_heading.size:
        heading_cost = np.zeros_like(cost, dtype=np.float32)
        for i in range(det_xy.shape[0]):
            if np.isnan(det_heading[i]):
                continue
            for j in range(trk_xy.shape[0]):
                if np.isnan(trk_heading[j]):
                    continue
                heading_cost[i, j] = abs(heading_diff_deg(float(det_heading[i]), float(trk_heading[j]))) / 180.0
        cost = cost + float(beta_heading) * heading_cost

    if gamma_confidence and det_conf.size:
        det_conf_clipped = np.clip(det_conf.astype(np.float32), 0.0, 1.0)
        conf_penalty = 1.0 - det_conf_clipped
        cost = cost + float(gamma_confidence) * conf_penalty.reshape((-1, 1))

    gated = dist <= float(max_distance_m)
    if not np.any(gated):
        return (
            np.empty((0, 2), dtype=int),
            np.arange(det_xy.shape[0], dtype=int),
            np.arange(trk_xy.shape[0], dtype=int),
        )

    huge = 1e6
    cost_gated = np.where(gated, cost, huge)
    matched_indices = linear_assignment(cost_gated)

    unmatched_dets: List[int] = []
    for d in range(det_xy.shape[0]):
        if d not in matched_indices[:, 0]:
            unmatched_dets.append(d)
    unmatched_trks: List[int] = []
    for t in range(trk_xy.shape[0]):
        if t not in matched_indices[:, 1]:
            unmatched_trks.append(t)

    matches: List[np.ndarray] = []
    for m in matched_indices:
        if not gated[m[0], m[1]]:
            unmatched_dets.append(int(m[0]))
            unmatched_trks.append(int(m[1]))
        else:
            matches.append(m.reshape(1, 2))

    if len(matches) == 0:
        matches_arr = np.empty((0, 2), dtype=int)
    else:
        matches_arr = np.concatenate(matches, axis=0).astype(int)
    return matches_arr, np.array(unmatched_dets, dtype=int), np.array(unmatched_trks, dtype=int)


class WorldSpaceSort:
    """
    SORT-like tracker operating on world-space (ENU) point measurements.

    Input detections: dicts containing at least:
      - world_east_m, world_north_m (meters, local tangent plane)
    Optionally:
      - confidence, category, heading

    Output: list of assigned IDs aligned with input order (None if unconfirmed).
    IDs are 1-based (MOT-style).
    """

    def __init__(
        self,
        *,
        # World-space tracker parameters (prefixed for clarity at call sites / CLI wiring).
        world_space_max_age: int = 20,
        world_space_min_hits: int = 5,
        world_space_max_distance_m: float = 100.0,
        world_space_beta_heading: float = 0.0,
        world_space_gamma_confidence: float = 0.0,
        world_space_kf_model: str = "cv",
        world_space_q_intensity: Optional[float] = None,
        world_space_measurement_noise_cross_var_m2: Optional[float] = None,
        world_space_measurement_noise_radial_scale: float = 200.0,
        # Per-category kinematic caps (always active; use a large value to effectively uncap).
        world_space_max_speed_boat_mps: float = 50.0,
        world_space_max_speed_other_mps: float = 50.0,
        world_space_max_accel_boat_mps2: float = 20.0,
        world_space_max_accel_other_mps2: float = 20.0,
        world_space_enable_track_prefilter: bool = False,
        world_space_enable_detection_prefilter: bool = False,
        world_space_enable_track_dedup: bool = False,
        world_space_enable_detection_dedup: bool = False,
        world_space_prefilter_max_range_m: float = 300.0,
        world_space_deduplication_min_distance_m: float = 0.0,
        world_space_deduplication_max_distance_m: float = 100.0,
        world_space_deduplication_min_speed_diff_mps: float = 0.0,
        world_space_deduplication_max_speed_diff_mps: float = 10.0,
        world_space_track_similarity_position_weight: float = 1.0,
        world_space_track_similarity_direction_weight: float = 1.0,
        world_space_track_similarity_speed_weight: float = 1.0,
        world_space_track_similarity_score_threshold: float = 0.9,
        world_space_detection_similarity_score_threshold: float = 0.9,
    ):
        # Keep attribute names short/stable for runtime introspection/logging.
        self.max_age = int(world_space_max_age)
        self.min_hits = int(world_space_min_hits)
        self.max_distance_m = float(world_space_max_distance_m)
        self.beta_heading = float(world_space_beta_heading)
        self.gamma_confidence = float(world_space_gamma_confidence)
        self.enable_track_prefilter = bool(world_space_enable_track_prefilter)
        self.enable_detection_prefilter = bool(world_space_enable_detection_prefilter)
        self.enable_track_dedup = bool(world_space_enable_track_dedup)
        self.enable_detection_dedup = bool(world_space_enable_detection_dedup)
        self.prefilter_max_range_m = max(0.0, float(world_space_prefilter_max_range_m))
        self.deduplication_min_distance_m = max(0.0, float(world_space_deduplication_min_distance_m))
        self.deduplication_max_distance_m = max(
            self.deduplication_min_distance_m + 1e-6,
            float(world_space_deduplication_max_distance_m),
        )
        self.deduplication_min_speed_diff_mps = max(0.0, float(world_space_deduplication_min_speed_diff_mps))
        self.deduplication_max_speed_diff_mps = max(
            self.deduplication_min_speed_diff_mps + 1e-6,
            float(world_space_deduplication_max_speed_diff_mps),
        )
        self.track_similarity_position_weight = max(0.0, float(world_space_track_similarity_position_weight))
        self.track_similarity_direction_weight = max(0.0, float(world_space_track_similarity_direction_weight))
        self.track_similarity_speed_weight = max(0.0, float(world_space_track_similarity_speed_weight))
        self.track_similarity_score_threshold = _clamp(float(world_space_track_similarity_score_threshold), 0.0, 1.0)
        self.detection_similarity_score_threshold = _clamp(
            float(world_space_detection_similarity_score_threshold),
            0.0,
            1.0,
        )

        kf_model = str(world_space_kf_model).strip().lower()
        if kf_model not in ("cv", "ca"):
            raise ValueError(f"Unknown KF model '{world_space_kf_model}', expected 'cv' or 'ca'")
        self.kf_model = kf_model
        self._tracker_class = KalmanCAPointTracker if kf_model == "ca" else KalmanCVPointTracker

        default_q_intensity = 0.0
        default_measurement_noise_cross_var_m2 = 1e6

        # Kinematic caps forwarded to each tracker instance.
        self._tracker_kwargs: Dict[str, float] = {
            "q_intensity": float(default_q_intensity if world_space_q_intensity is None else world_space_q_intensity),
            "measurement_noise_cross_var_m2": float(
                default_measurement_noise_cross_var_m2
                if world_space_measurement_noise_cross_var_m2 is None
                else world_space_measurement_noise_cross_var_m2
            ),
            "measurement_noise_radial_scale": float(world_space_measurement_noise_radial_scale),
            "max_speed_boat_mps": float(world_space_max_speed_boat_mps),
            "max_speed_other_mps": float(world_space_max_speed_other_mps),
            "max_accel_boat_mps2": float(world_space_max_accel_boat_mps2),
            "max_accel_other_mps2": float(world_space_max_accel_other_mps2),
        }

        self.trackers: list = []
        self.frame_count = 0
        self._last_assign_time: Optional[float] = None

    def _det_to_xy(self, det: Dict[str, Any]) -> Tuple[np.ndarray, WorldTrackExtras]:
        e = float(det.get("world_east_m"))
        n = float(det.get("world_north_m"))
        conf = det.get("confidence", det.get("score", float("nan")))
        try:
            conf_f = float(conf)
        except Exception:
            conf_f = float("nan")
        heading = det.get("heading", det.get("target_heading", float("nan")))
        try:
            heading_f = float(heading)
        except Exception:
            heading_f = float("nan")
        try:
            rel_east_m = float(det.get("world_rel_ego_east_m", float("nan")))
        except Exception:
            rel_east_m = float("nan")
        try:
            rel_north_m = float(det.get("world_rel_ego_north_m", float("nan")))
        except Exception:
            rel_north_m = float("nan")
        # Extract bbox geometry for back-projection of unmatched tracks
        try:
            y_px = float(det.get("y", float("nan")))
        except Exception:
            y_px = float("nan")
        try:
            width_px = float(det.get("width", det.get("w", float("nan"))))
        except Exception:
            width_px = float("nan")
        try:
            height_px = float(det.get("height", det.get("h", float("nan"))))
        except Exception:
            height_px = float("nan")
        extras = WorldTrackExtras(
            confidence=conf_f,
            category=det.get("category"),
            heading_deg=heading_f,
            rel_east_m=rel_east_m,
            rel_north_m=rel_north_m,
            last_y_px=y_px,
            last_width_px=width_px,
            last_height_px=height_px,
        )
        return np.array([e, n], dtype=np.float32), extras

    @staticmethod
    def _connected_keep_first(indices: List[int], adjacency: np.ndarray) -> Tuple[List[int], List[int]]:
        if not indices:
            return [], []
        if adjacency.shape[0] != len(indices):
            return list(indices), []

        keep: List[int] = []
        duplicates: List[int] = []
        seen: set[int] = set()
        for start_local_idx, original_idx in enumerate(indices):
            if start_local_idx in seen:
                continue
            keep.append(original_idx)
            stack = [start_local_idx]
            component: List[int] = []
            while stack:
                current = stack.pop()
                if current in seen:
                    continue
                seen.add(current)
                component.append(current)
                neighbors = np.where(adjacency[current])[0]
                for neighbor in neighbors:
                    neighbor_i = int(neighbor)
                    if neighbor_i not in seen:
                        stack.append(neighbor_i)
            for local_idx in component:
                if local_idx == start_local_idx:
                    continue
                duplicates.append(indices[local_idx])
        return keep, duplicates

    @staticmethod
    def _extract_ego_xy(ego_enu_from_ref: Optional[Any]) -> Optional[np.ndarray]:
        if ego_enu_from_ref is None:
            return None
        try:
            if isinstance(ego_enu_from_ref, (tuple, list)) and len(ego_enu_from_ref) >= 2:
                east_m = float(ego_enu_from_ref[0])
                north_m = float(ego_enu_from_ref[1])
            else:
                east_m = float(getattr(ego_enu_from_ref, "east_m"))
                north_m = float(getattr(ego_enu_from_ref, "north_m"))
        except Exception:
            return None
        return np.array([east_m, north_m], dtype=np.float32)

    def _prefilter_indices(
        self,
        det_xy: np.ndarray,
        trk_xy: np.ndarray,
        *,
        ego_enu_from_ref: Optional[Any],
    ) -> Tuple[List[int], List[int]]:
        det_indices = list(range(det_xy.shape[0]))
        trk_indices = list(range(trk_xy.shape[0]))

        ego_xy = self._extract_ego_xy(ego_enu_from_ref)
        if ego_xy is None or self.prefilter_max_range_m <= 0.0:
            return det_indices, trk_indices

        if self.enable_detection_prefilter and det_xy.shape[0] > 0:
            det_dist = np.sqrt(np.sum((det_xy - ego_xy.reshape((1, 2))) ** 2, axis=1))
            det_indices = [idx for idx in det_indices if float(det_dist[idx]) <= self.prefilter_max_range_m]

        if self.enable_track_prefilter and trk_xy.shape[0] > 0:
            trk_dist = np.sqrt(np.sum((trk_xy - ego_xy.reshape((1, 2))) ** 2, axis=1))
            trk_indices = [idx for idx in trk_indices if float(trk_dist[idx]) <= self.prefilter_max_range_m]

        return det_indices, trk_indices

    def _dedup_detection_indices(self, candidate_indices: List[int], det_xy: np.ndarray) -> Tuple[List[int], List[int]]:
        if not self.enable_detection_dedup or len(candidate_indices) <= 1:
            return list(candidate_indices), []

        subset_xy = det_xy[candidate_indices, :]
        dist = _euclid_dist_matrix(subset_xy, subset_xy)
        similarity = np.vectorize(
            lambda x: _inverted_remap01(
                x,
                self.deduplication_min_distance_m,
                self.deduplication_max_distance_m,
            )
        )(dist).astype(np.float32)
        adjacency = similarity >= float(self.detection_similarity_score_threshold)
        np.fill_diagonal(adjacency, False)
        return self._connected_keep_first(candidate_indices, adjacency)

    def _dedup_track_indices(
        self,
        candidate_indices: List[int],
        trk_xy: np.ndarray,
        trk_vel_xy: np.ndarray,
        trk_speed: np.ndarray,
    ) -> Tuple[List[int], List[int]]:
        if not self.enable_track_dedup or len(candidate_indices) <= 1:
            return list(candidate_indices), []

        subset_xy = trk_xy[candidate_indices, :]
        subset_vel_xy = trk_vel_xy[candidate_indices, :]
        subset_speed = trk_speed[candidate_indices]
        dist = _euclid_dist_matrix(subset_xy, subset_xy)
        n_tracks = len(candidate_indices)
        similarity = np.zeros((n_tracks, n_tracks), dtype=np.float32)

        weights = np.array(
            [
                max(0.0, self.track_similarity_position_weight),
                max(0.0, self.track_similarity_direction_weight),
                max(0.0, self.track_similarity_speed_weight),
            ],
            dtype=np.float32,
        )
        weight_sum = float(np.sum(weights))
        if weight_sum <= 1e-6:
            weights[:] = 1.0 / 3.0
        else:
            weights /= weight_sum

        for i in range(n_tracks):
            for j in range(i + 1, n_tracks):
                position_similarity = _inverted_remap01(
                    float(dist[i, j]),
                    self.deduplication_min_distance_m,
                    self.deduplication_max_distance_m,
                )
                cosine_similarity = _safe_cosine_similarity(
                    float(subset_vel_xy[i, 0]),
                    float(subset_vel_xy[i, 1]),
                    float(subset_vel_xy[j, 0]),
                    float(subset_vel_xy[j, 1]),
                )
                direction_similarity = _remap01(cosine_similarity, -1.0, 1.0)
                speed_similarity = _inverted_remap01(
                    abs(float(subset_speed[i]) - float(subset_speed[j])),
                    self.deduplication_min_speed_diff_mps,
                    self.deduplication_max_speed_diff_mps,
                )
                score = (
                    float(weights[0]) * position_similarity
                    + float(weights[1]) * direction_similarity
                    + float(weights[2]) * speed_similarity
                )
                similarity[i, j] = score
                similarity[j, i] = score

        adjacency = similarity >= float(self.track_similarity_score_threshold)
        np.fill_diagonal(adjacency, False)
        return self._connected_keep_first(candidate_indices, adjacency)

    def _build_unmatched_track_output(self, trk: Any, full_state: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "track_id": int(trk.id) + 1,
            "world_east_m": full_state["east_m"],
            "world_north_m": full_state["north_m"],
            "confidence": float(trk.extras.confidence) if not np.isnan(trk.extras.confidence) else 0.5,
            "heading": float(trk.extras.heading_deg) if not np.isnan(trk.extras.heading_deg) else None,
            "category": trk.extras.category,
            "last_y_px": float(trk.extras.last_y_px) if not np.isnan(trk.extras.last_y_px) else None,
            "last_width_px": float(trk.extras.last_width_px) if not np.isnan(trk.extras.last_width_px) else None,
            "last_height_px": float(trk.extras.last_height_px) if not np.isnan(trk.extras.last_height_px) else None,
            "vel_east_mps": full_state["vel_east_mps"],
            "vel_north_mps": full_state["vel_north_mps"],
            "speed_mps": full_state["speed_mps"],
            "course_deg": full_state["course_deg"],
            "accel_east_mps2": full_state["accel_east_mps2"],
            "accel_north_mps2": full_state["accel_north_mps2"],
            "measurement_noise_cross_var_m2": full_state["measurement_noise_cross_var_m2"],
            "measurement_noise_radial_var_m2": full_state["measurement_noise_radial_var_m2"],
            "state_position_covariance_enu": full_state["state_position_covariance_enu"],
            "process_position_covariance_enu": full_state["process_position_covariance_enu"],
            "measurement_position_covariance_enu": full_state["measurement_position_covariance_enu"],
            "unmatched_frames": int(trk.time_since_update),
            "tracker_max_age": int(self.max_age),
        }

    def assign(
        self,
        detections: List[Dict[str, Any]],
        *,
        ego_enu_from_ref: Optional[Any] = None,
    ) -> Tuple[List[Optional[int]], List[Dict[str, Any]], Dict[int, Dict[str, Any]]]:
        """
        Returns a tuple of:
          - assigned_ids: track IDs for each detection (1-based IDs), or None if not assigned
          - unmatched_tracks: list of dicts for confirmed tracks that weren't matched this frame,
                              containing predicted world position, full KF state, and track metadata
          - matched_track_states: dict mapping track_id -> full KF state dict (from get_full_state())
                                  for every matched and confirmed track this frame
        """
        self.frame_count += 1

        # Compute real dt (seconds) from wall-clock time.
        now = time.monotonic()
        if self._last_assign_time is not None:
            dt = max(1e-6, min(now - self._last_assign_time, _MAX_DT_S))
        else:
            dt = _DEFAULT_DT_S
        self._last_assign_time = now

        det_xy = np.zeros((len(detections), 2), dtype=np.float32)
        det_heading = np.full((len(detections),), np.nan, dtype=np.float32)
        det_conf = np.full((len(detections),), np.nan, dtype=np.float32)
        det_extras: List[WorldTrackExtras] = []
        for i, d in enumerate(detections):
            xy, ex = self._det_to_xy(d)
            det_xy[i, :] = xy
            det_extras.append(ex)
            det_heading[i] = ex.heading_deg
            det_conf[i] = ex.confidence

        trk_xy = np.zeros((len(self.trackers), 2), dtype=np.float32)
        trk_heading = np.full((len(self.trackers),), np.nan, dtype=np.float32)
        trk_vel_xy = np.zeros((len(self.trackers), 2), dtype=np.float32)
        trk_speed = np.zeros((len(self.trackers),), dtype=np.float32)
        to_del: List[int] = []
        for t in range(len(self.trackers)):
            pos = self.trackers[t].predict(dt)
            trk_xy[t, :] = pos.astype(np.float32)
            if np.any(np.isnan(pos)):
                to_del.append(t)
                continue
            trk = self.trackers[t]
            trk_heading[t] = float(trk.extras.heading_deg)
            full_state = trk.get_full_state()
            trk_vel_xy[t, 0] = float(full_state["vel_east_mps"])
            trk_vel_xy[t, 1] = float(full_state["vel_north_mps"])
            trk_speed[t] = float(full_state["speed_mps"])
        for t in reversed(to_del):
            self.trackers.pop(t)
            trk_xy = np.delete(trk_xy, t, axis=0)
            trk_heading = np.delete(trk_heading, t, axis=0)
            trk_vel_xy = np.delete(trk_vel_xy, t, axis=0)
            trk_speed = np.delete(trk_speed, t, axis=0)

        filtered_det_indices, filtered_trk_indices = self._prefilter_indices(
            det_xy,
            trk_xy,
            ego_enu_from_ref=ego_enu_from_ref,
        )
        unique_det_indices, _duplicate_det_indices = self._dedup_detection_indices(filtered_det_indices, det_xy)
        unique_trk_indices, _duplicate_trk_indices = self._dedup_track_indices(
            filtered_trk_indices,
            trk_xy,
            trk_vel_xy,
            trk_speed,
        )

        assoc_det_xy = det_xy[unique_det_indices, :] if unique_det_indices else np.zeros((0, 2), dtype=np.float32)
        assoc_det_heading = (
            det_heading[unique_det_indices] if unique_det_indices else np.empty((0,), dtype=np.float32)
        )
        assoc_det_conf = det_conf[unique_det_indices] if unique_det_indices else np.empty((0,), dtype=np.float32)
        assoc_trk_xy = trk_xy[unique_trk_indices, :] if unique_trk_indices else np.zeros((0, 2), dtype=np.float32)
        assoc_trk_heading = (
            trk_heading[unique_trk_indices] if unique_trk_indices else np.empty((0,), dtype=np.float32)
        )

        matches, unmatched_dets_subset, unmatched_trk_indices_subset = associate_world_detections_to_trackers(
            assoc_det_xy,
            assoc_trk_xy,
            assoc_det_heading,
            assoc_trk_heading,
            assoc_det_conf,
            max_distance_m=self.max_distance_m,
            beta_heading=self.beta_heading,
            gamma_confidence=self.gamma_confidence,
        )

        assigned: List[Optional[int]] = [None] * len(detections)
        matched_track_states: Dict[int, Dict[str, Any]] = {}
        
        # Tracks that matched a detection but are still "re-warming" (hit_streak < min_hits)
        # These will be output as unmatched/ghost if they were previously confirmed (hits >= min_hits)
        rewarming_tracks: List[Dict[str, Any]] = []

        for det_subset_idx, trk_subset_idx in matches:
            trk_idx_i = int(unique_trk_indices[int(trk_subset_idx)])
            det_idx_i = int(unique_det_indices[int(det_subset_idx)])
            trk = self.trackers[trk_idx_i]
            was_confirmed = trk.hits >= self.min_hits
            will_recover_this_frame = (trk.hit_streak + 1) >= self.min_hits

            if was_confirmed and not will_recover_this_frame:
                trk.update_rewarming(det_xy[det_idx_i, :], det_extras[det_idx_i])
            else:
                trk.update(det_xy[det_idx_i, :], det_extras[det_idx_i])

            track_id = int(trk.id) + 1
            
            if trk.hit_streak >= self.min_hits:
                # Fully confirmed with consecutive matches → output as matched
                assigned[det_idx_i] = track_id
                matched_track_states[track_id] = trk.get_full_state()
            elif trk.hits > self.min_hits:
                # Previously confirmed but re-warming (hit_streak < min_hits).
                # Keep detection image-space fields (bbox, etc.), but force world
                # position to the KF prediction from this frame (pre-update).
                det = detections[det_idx_i]
                rewarming_entry = dict(det)  # Keep detection fields (x, y, w, h, distance, confidence, obj_id, etc.)
                rewarming_entry["world_east_m"] = float(trk_xy[trk_idx_i, 0])
                rewarming_entry["world_north_m"] = float(trk_xy[trk_idx_i, 1])
                rewarming_entry["track_id"] = track_id
                rewarming_entry["_is_rewarming"] = True  # Marker for bridge to skip back-projection
                rewarming_entry["unmatched_frames"] = 0
                rewarming_entry["tracker_max_age"] = int(self.max_age)
                # Include full KF state for rewarming tracks
                full_state = trk.get_full_state()
                rewarming_entry["vel_east_mps"] = full_state["vel_east_mps"]
                rewarming_entry["vel_north_mps"] = full_state["vel_north_mps"]
                rewarming_entry["speed_mps"] = full_state["speed_mps"]
                rewarming_entry["course_deg"] = full_state["course_deg"]
                rewarming_entry["accel_east_mps2"] = full_state["accel_east_mps2"]
                rewarming_entry["accel_north_mps2"] = full_state["accel_north_mps2"]
                rewarming_entry["measurement_noise_cross_var_m2"] = full_state["measurement_noise_cross_var_m2"]
                rewarming_entry["measurement_noise_radial_var_m2"] = full_state["measurement_noise_radial_var_m2"]
                rewarming_entry["state_position_covariance_enu"] = full_state["state_position_covariance_enu"]
                rewarming_entry["process_position_covariance_enu"] = full_state["process_position_covariance_enu"]
                rewarming_entry["measurement_position_covariance_enu"] = full_state["measurement_position_covariance_enu"]
                rewarming_tracks.append(rewarming_entry)
            # else: never confirmed (hits < min_hits), no output

        for det_subset_idx in unmatched_dets_subset:
            det_idx_i = int(unique_det_indices[int(det_subset_idx)])
            trk = self._tracker_class(det_xy[det_idx_i, :], det_extras[det_idx_i], **self._tracker_kwargs)
            self.trackers.append(trk)
            track_id = int(trk.id) + 1
            if trk.hit_streak >= self.min_hits:
                assigned[det_idx_i] = track_id

        # Collect unmatched but confirmed tracks (predicted state)
        unmatched_tracks: List[Dict[str, Any]] = []
        for trk_subset_idx in unmatched_trk_indices_subset:
            trk_idx = int(unique_trk_indices[int(trk_subset_idx)])
            trk = self.trackers[trk_idx]
            # Only output tracks that have been confirmed (seen enough times)
            if trk.hits >= self.min_hits:
                full_state = trk.get_full_state()
                unmatched_tracks.append(self._build_unmatched_track_output(trk, full_state))
        
        # Add re-warming tracks to unmatched output
        unmatched_tracks.extend(rewarming_tracks)

        # remove dead
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            i -= 1
            if trk.time_since_update > self.max_age:
                self.trackers.pop(i)

        return assigned, unmatched_tracks, matched_track_states

    # ------------------------------------------------------------------
    # Runtime-tunable parameter API (for live UI adjustment via control WS)
    # ------------------------------------------------------------------

    # The keys that can be changed at runtime, mapped to their types.
    _TUNABLE_KEYS: Dict[str, Any] = {
        "max_age": int,
        "min_hits": int,
        "max_distance_m": float,
        "q_intensity": float,
        "measurement_noise_cross_var_m2": float,
        "measurement_noise_radial_scale": float,
        "max_speed_boat_mps": float,
        "max_speed_other_mps": float,
        "max_accel_boat_mps2": float,
        "max_accel_other_mps2": float,
        "enable_track_prefilter": _coerce_bool,
        "enable_detection_prefilter": _coerce_bool,
        "enable_track_dedup": _coerce_bool,
        "enable_detection_dedup": _coerce_bool,
        "prefilter_max_range_m": float,
        "world_space_deduplication_min_distance_m": float,
        "world_space_deduplication_max_distance_m": float,
        "world_space_deduplication_min_speed_diff_mps": float,
        "world_space_deduplication_max_speed_diff_mps": float,
        "world_space_track_similarity_position_weight": float,
        "world_space_track_similarity_direction_weight": float,
        "world_space_track_similarity_speed_weight": float,
        "world_space_track_similarity_score_threshold": float,
        "world_space_detection_similarity_score_threshold": float,
    }

    def get_tunable_params(self) -> Dict[str, Any]:
        """Return the current values of all runtime-tunable parameters."""
        return {
            "max_age": self.max_age,
            "min_hits": self.min_hits,
            "max_distance_m": self.max_distance_m,
            "q_intensity": self._tracker_kwargs["q_intensity"],
            "measurement_noise_cross_var_m2": self._tracker_kwargs["measurement_noise_cross_var_m2"],
            "measurement_noise_radial_scale": self._tracker_kwargs["measurement_noise_radial_scale"],
            "max_speed_boat_mps": self._tracker_kwargs["max_speed_boat_mps"],
            "max_speed_other_mps": self._tracker_kwargs["max_speed_other_mps"],
            "max_accel_boat_mps2": self._tracker_kwargs["max_accel_boat_mps2"],
            "max_accel_other_mps2": self._tracker_kwargs["max_accel_other_mps2"],
            "enable_track_prefilter": self.enable_track_prefilter,
            "enable_detection_prefilter": self.enable_detection_prefilter,
            "enable_track_dedup": self.enable_track_dedup,
            "enable_detection_dedup": self.enable_detection_dedup,
            "prefilter_max_range_m": self.prefilter_max_range_m,
            "world_space_deduplication_min_distance_m": self.deduplication_min_distance_m,
            "world_space_deduplication_max_distance_m": self.deduplication_max_distance_m,
            "world_space_deduplication_min_speed_diff_mps": self.deduplication_min_speed_diff_mps,
            "world_space_deduplication_max_speed_diff_mps": self.deduplication_max_speed_diff_mps,
            "world_space_track_similarity_position_weight": self.track_similarity_position_weight,
            "world_space_track_similarity_direction_weight": self.track_similarity_direction_weight,
            "world_space_track_similarity_speed_weight": self.track_similarity_speed_weight,
            "world_space_track_similarity_score_threshold": self.track_similarity_score_threshold,
            "world_space_detection_similarity_score_threshold": self.detection_similarity_score_threshold,
        }

    def set_tunable_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """Update one or more tunable params.  Returns the full updated dict.

        *params* is a partial dict — only keys present are changed.
        Unrecognised keys are silently ignored.
        """
        for key, cast in self._TUNABLE_KEYS.items():
            if key not in params:
                continue
            try:
                val = cast(params[key])
            except (TypeError, ValueError):
                continue

            if key == "max_age":
                self.max_age = val
            elif key == "min_hits":
                self.min_hits = val
            elif key == "max_distance_m":
                self.max_distance_m = val
            elif key == "enable_track_prefilter":
                self.enable_track_prefilter = bool(val)
            elif key == "enable_detection_prefilter":
                self.enable_detection_prefilter = bool(val)
            elif key == "enable_track_dedup":
                self.enable_track_dedup = bool(val)
            elif key == "enable_detection_dedup":
                self.enable_detection_dedup = bool(val)
            elif key == "prefilter_max_range_m":
                self.prefilter_max_range_m = max(0.0, float(val))
            elif key == "world_space_deduplication_min_distance_m":
                self.deduplication_min_distance_m = max(0.0, float(val))
                self.deduplication_max_distance_m = max(
                    self.deduplication_max_distance_m,
                    self.deduplication_min_distance_m + 1e-6,
                )
            elif key == "world_space_deduplication_max_distance_m":
                self.deduplication_max_distance_m = max(
                    self.deduplication_min_distance_m + 1e-6,
                    float(val),
                )
            elif key == "world_space_deduplication_min_speed_diff_mps":
                self.deduplication_min_speed_diff_mps = max(0.0, float(val))
                self.deduplication_max_speed_diff_mps = max(
                    self.deduplication_max_speed_diff_mps,
                    self.deduplication_min_speed_diff_mps + 1e-6,
                )
            elif key == "world_space_deduplication_max_speed_diff_mps":
                self.deduplication_max_speed_diff_mps = max(
                    self.deduplication_min_speed_diff_mps + 1e-6,
                    float(val),
                )
            elif key == "world_space_track_similarity_position_weight":
                self.track_similarity_position_weight = max(0.0, float(val))
            elif key == "world_space_track_similarity_direction_weight":
                self.track_similarity_direction_weight = max(0.0, float(val))
            elif key == "world_space_track_similarity_speed_weight":
                self.track_similarity_speed_weight = max(0.0, float(val))
            elif key == "world_space_track_similarity_score_threshold":
                self.track_similarity_score_threshold = _clamp(float(val), 0.0, 1.0)
            elif key == "world_space_detection_similarity_score_threshold":
                self.detection_similarity_score_threshold = _clamp(float(val), 0.0, 1.0)
            elif key in self._tracker_kwargs:
                # Update for future trackers.
                self._tracker_kwargs[key] = val
                # Update all existing tracker instances so the new cap
                # takes effect immediately.
                for trk in self.trackers:
                    setter_name = f"set_{key}"
                    if hasattr(trk, setter_name):
                        getattr(trk, setter_name)(val)
                    elif hasattr(trk, key):
                        setattr(trk, key, val)

        return self.get_tunable_params()

