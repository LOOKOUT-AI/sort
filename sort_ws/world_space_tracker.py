from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from filterpy.kalman import KalmanFilter

from .image_space_tracker import linear_assignment  # reuse lap/scipy wrapper
from .image_to_world import heading_diff_deg


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


@dataclass
class WorldTrackExtras:
    confidence: float = float("nan")
    category: Optional[str] = None
    heading_deg: float = float("nan")  # if the detector provides a target heading estimate
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
        max_speed_boat_mps: float = 50.0,
        max_speed_other_mps: float = 50.0,
        max_accel_boat_mps2: float = 20.0,   # accepted but unused (CV has no accel state)
        max_accel_other_mps2: float = 20.0,  # accepted but unused
    ):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        dt = 1.0
        self.kf.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)

        # covariances: tuned for stability (similar spirit to SORT)
        self.kf.R = np.eye(2, dtype=np.float32) * 4.0
        self.kf.P = np.eye(4, dtype=np.float32) * 10.0
        self.kf.P[2:, 2:] *= 1000.0  # velocities initially very uncertain

        q = 1.0
        self.kf.Q = np.array(
            [[dt**4 / 4, 0, dt**3 / 2, 0], [0, dt**4 / 4, 0, dt**3 / 2], [dt**3 / 2, 0, dt**2, 0], [0, dt**3 / 2, 0, dt**2]],
            dtype=np.float32,
        ) * q

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

    def update(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()
        self.extras = extras

    def predict(self) -> np.ndarray:
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

    def get_full_state(self) -> Dict[str, float]:
        """Return full KF state as a dict: position, velocity, and convenience scalars."""
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
        max_speed_boat_mps: float = 50.0,
        max_speed_other_mps: float = 50.0,
        max_accel_boat_mps2: float = 20.0,
        max_accel_other_mps2: float = 20.0,
    ):
        self.kf = KalmanFilter(dim_x=6, dim_z=2)
        dt = 1.0
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
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0]],
            dtype=np.float32,
        )
        # fmt: on

        # Measurement noise: same sensor as CV, same R.
        self.kf.R = np.eye(2, dtype=np.float32) * 4.0

        # Initial state covariance.
        self.kf.P = np.eye(6, dtype=np.float32) * 10.0
        self.kf.P[2:4, 2:4] *= 1000.0   # velocities initially very uncertain
        self.kf.P[4:6, 4:6] *= 10000.0   # accelerations even more uncertain

        # Process noise: driven by jerk (white noise) as the unmodeled disturbance.
        # Per-axis noise gain: G = [dt^3/6, dt^2/2, dt]
        # Q_axis = q_jerk * G @ G^T, then interleaved for (east, north).
        q_jerk = 1.0
        g = np.array([dt**3 / 6.0, dt**2 / 2.0, dt], dtype=np.float32)
        q_axis = q_jerk * np.outer(g, g)  # 3x3
        # Assemble 6x6 block-diagonal Q for [e, n, ve, vn, ae, an] ordering
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

    def update(self, meas_enu: np.ndarray, extras: WorldTrackExtras) -> None:
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        z = np.array([[float(meas_enu[0])], [float(meas_enu[1])]], dtype=np.float32)
        self.kf.update(z)
        self._clamp_state()
        self.extras = extras

    def predict(self) -> np.ndarray:
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

    def get_full_state(self) -> Dict[str, float]:
        """Return full KF state as a dict: position, velocity, acceleration, and convenience scalars."""
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
        world_space_max_distance_m: float = 30.0,
        world_space_beta_heading: float = 0.0,
        world_space_gamma_confidence: float = 0.0,
        world_space_new_track_min_confidence: float = 0.0,
        world_space_kf_model: str = "cv",
        # Per-category kinematic caps (always active; use a large value to effectively uncap).
        world_space_max_speed_boat_mps: float = 50.0,
        world_space_max_speed_other_mps: float = 50.0,
        world_space_max_accel_boat_mps2: float = 20.0,
        world_space_max_accel_other_mps2: float = 20.0,
    ):
        # Keep attribute names short/stable for runtime introspection/logging.
        self.max_age = int(world_space_max_age)
        self.min_hits = int(world_space_min_hits)
        self.max_distance_m = float(world_space_max_distance_m)
        self.beta_heading = float(world_space_beta_heading)
        self.gamma_confidence = float(world_space_gamma_confidence)
        self.new_track_min_confidence = float(world_space_new_track_min_confidence)

        kf_model = str(world_space_kf_model).strip().lower()
        if kf_model not in ("cv", "ca"):
            raise ValueError(f"Unknown KF model '{world_space_kf_model}', expected 'cv' or 'ca'")
        self.kf_model = kf_model
        self._tracker_class = KalmanCAPointTracker if kf_model == "ca" else KalmanCVPointTracker

        # Kinematic caps forwarded to each tracker instance.
        self._tracker_kwargs: Dict[str, float] = {
            "max_speed_boat_mps": float(world_space_max_speed_boat_mps),
            "max_speed_other_mps": float(world_space_max_speed_other_mps),
            "max_accel_boat_mps2": float(world_space_max_accel_boat_mps2),
            "max_accel_other_mps2": float(world_space_max_accel_other_mps2),
        }

        self.trackers: list = []
        self.frame_count = 0

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
            last_y_px=y_px,
            last_width_px=width_px,
            last_height_px=height_px,
        )
        return np.array([e, n], dtype=np.float32), extras

    def assign(self, detections: List[Dict[str, Any]]) -> Tuple[List[Optional[int]], List[Dict[str, Any]], Dict[int, Dict[str, Any]]]:
        """
        Returns a tuple of:
          - assigned_ids: track IDs for each detection (1-based IDs), or None if not assigned
          - unmatched_tracks: list of dicts for confirmed tracks that weren't matched this frame,
                              containing predicted world position, full KF state, and track metadata
          - matched_track_states: dict mapping track_id -> full KF state dict (from get_full_state())
                                  for every matched and confirmed track this frame
        """
        self.frame_count += 1

        if len(detections) == 0:
            # age/predict existing trackers
            unmatched_tracks: List[Dict[str, Any]] = []
            to_del: List[int] = []
            for t in range(len(self.trackers)):
                pos = self.trackers[t].predict()
                if np.any(np.isnan(pos)):
                    to_del.append(t)
                else:
                    trk = self.trackers[t]
                    # Output confirmed tracks that are now unmatched
                    if trk.hits >= self.min_hits:
                        full_state = trk.get_full_state()
                        entry: Dict[str, Any] = {
                            "track_id": int(trk.id) + 1,
                            "world_east_m": full_state["east_m"],
                            "world_north_m": full_state["north_m"],
                            "confidence": float(trk.extras.confidence) if not np.isnan(trk.extras.confidence) else 0.5,
                            "heading": float(trk.extras.heading_deg) if not np.isnan(trk.extras.heading_deg) else None,
                            "category": trk.extras.category,
                            # Bbox geometry for back-projection
                            "last_y_px": float(trk.extras.last_y_px) if not np.isnan(trk.extras.last_y_px) else None,
                            "last_width_px": float(trk.extras.last_width_px) if not np.isnan(trk.extras.last_width_px) else None,
                            "last_height_px": float(trk.extras.last_height_px) if not np.isnan(trk.extras.last_height_px) else None,
                            # Full KF kinematic state
                            "vel_east_mps": full_state["vel_east_mps"],
                            "vel_north_mps": full_state["vel_north_mps"],
                            "speed_mps": full_state["speed_mps"],
                            "course_deg": full_state["course_deg"],
                            "accel_east_mps2": full_state["accel_east_mps2"],
                            "accel_north_mps2": full_state["accel_north_mps2"],
                        }
                        unmatched_tracks.append(entry)
            for t in reversed(to_del):
                self.trackers.pop(t)
            # remove dead
            i = len(self.trackers)
            for trk in reversed(self.trackers):
                i -= 1
                if trk.time_since_update > self.max_age:
                    self.trackers.pop(i)
            return [], unmatched_tracks, {}

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
        trk_conf = np.full((len(self.trackers),), np.nan, dtype=np.float32)
        to_del: List[int] = []
        for t in range(len(self.trackers)):
            pos = self.trackers[t].predict()
            trk_xy[t, :] = pos.astype(np.float32)
            if np.any(np.isnan(pos)):
                to_del.append(t)
            trk_heading[t] = float(self.trackers[t].extras.heading_deg)
            trk_conf[t] = float(self.trackers[t].extras.confidence)
        for t in reversed(to_del):
            self.trackers.pop(t)
            trk_xy = np.delete(trk_xy, t, axis=0)
            trk_heading = np.delete(trk_heading, t, axis=0)
            trk_conf = np.delete(trk_conf, t, axis=0)

        matches, unmatched_dets, unmatched_trk_indices = associate_world_detections_to_trackers(
            det_xy,
            trk_xy,
            det_heading,
            trk_heading,
            det_conf,
            max_distance_m=self.max_distance_m,
            beta_heading=self.beta_heading,
            gamma_confidence=self.gamma_confidence,
        )

        assigned: List[Optional[int]] = [None] * len(detections)
        matched_trk_indices = set()
        matched_track_states: Dict[int, Dict[str, Any]] = {}
        
        # Tracks that matched a detection but are still "re-warming" (hit_streak < min_hits)
        # These will be output as unmatched/ghost if they were previously confirmed (hits >= min_hits)
        rewarming_tracks: List[Dict[str, Any]] = []

        for det_idx, trk_idx in matches:
            matched_trk_indices.add(int(trk_idx))
            self.trackers[int(trk_idx)].update(det_xy[int(det_idx), :], det_extras[int(det_idx)])
            trk = self.trackers[int(trk_idx)]
            track_id = int(trk.id) + 1
            
            if trk.hit_streak >= self.min_hits:
                # Fully confirmed with consecutive matches → output as matched
                assigned[int(det_idx)] = track_id
                matched_track_states[track_id] = trk.get_full_state()
            elif trk.hits > self.min_hits:
                # Previously confirmed but re-warming (hit_streak < min_hits)
                # → pass through the FULL DETECTION dict directly (no back-projection needed)
                det = detections[int(det_idx)]
                rewarming_entry = dict(det)  # Copy all detection fields (x, y, w, h, distance, confidence, obj_id, etc.)
                rewarming_entry["track_id"] = track_id
                rewarming_entry["_is_rewarming"] = True  # Marker for bridge to skip back-projection
                # Include full KF state for rewarming tracks
                full_state = trk.get_full_state()
                rewarming_entry["vel_east_mps"] = full_state["vel_east_mps"]
                rewarming_entry["vel_north_mps"] = full_state["vel_north_mps"]
                rewarming_entry["speed_mps"] = full_state["speed_mps"]
                rewarming_entry["course_deg"] = full_state["course_deg"]
                rewarming_entry["accel_east_mps2"] = full_state["accel_east_mps2"]
                rewarming_entry["accel_north_mps2"] = full_state["accel_north_mps2"]
                rewarming_tracks.append(rewarming_entry)
            # else: never confirmed (hits < min_hits), no output

        for det_idx in unmatched_dets:
            conf = float(det_extras[int(det_idx)].confidence)
            if not np.isnan(conf) and conf < self.new_track_min_confidence:
                continue
            trk = self._tracker_class(det_xy[int(det_idx), :], det_extras[int(det_idx)], **self._tracker_kwargs)
            self.trackers.append(trk)
            track_id = int(trk.id) + 1
            if trk.hit_streak >= self.min_hits:
                assigned[int(det_idx)] = track_id

        # Collect unmatched but confirmed tracks (predicted state)
        unmatched_tracks: List[Dict[str, Any]] = []
        for trk_idx in unmatched_trk_indices:
            trk = self.trackers[int(trk_idx)]
            # Only output tracks that have been confirmed (seen enough times)
            if trk.hits >= self.min_hits:
                full_state = trk.get_full_state()
                unmatched_tracks.append({
                    "track_id": int(trk.id) + 1,
                    "world_east_m": full_state["east_m"],
                    "world_north_m": full_state["north_m"],
                    "confidence": float(trk.extras.confidence) if not np.isnan(trk.extras.confidence) else 0.5,
                    "heading": float(trk.extras.heading_deg) if not np.isnan(trk.extras.heading_deg) else None,
                    "category": trk.extras.category,
                    # Bbox geometry for back-projection
                    "last_y_px": float(trk.extras.last_y_px) if not np.isnan(trk.extras.last_y_px) else None,
                    "last_width_px": float(trk.extras.last_width_px) if not np.isnan(trk.extras.last_width_px) else None,
                    "last_height_px": float(trk.extras.last_height_px) if not np.isnan(trk.extras.last_height_px) else None,
                    # Full KF kinematic state
                    "vel_east_mps": full_state["vel_east_mps"],
                    "vel_north_mps": full_state["vel_north_mps"],
                    "speed_mps": full_state["speed_mps"],
                    "course_deg": full_state["course_deg"],
                    "accel_east_mps2": full_state["accel_east_mps2"],
                    "accel_north_mps2": full_state["accel_north_mps2"],
                })
        
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
    _TUNABLE_KEYS: Dict[str, type] = {
        "max_age": int,
        "min_hits": int,
        "max_distance_m": float,
        "max_speed_boat_mps": float,
        "max_speed_other_mps": float,
        "max_accel_boat_mps2": float,
        "max_accel_other_mps2": float,
    }

    def get_tunable_params(self) -> Dict[str, Any]:
        """Return the current values of all runtime-tunable parameters."""
        return {
            "max_age": self.max_age,
            "min_hits": self.min_hits,
            "max_distance_m": self.max_distance_m,
            "max_speed_boat_mps": self._tracker_kwargs["max_speed_boat_mps"],
            "max_speed_other_mps": self._tracker_kwargs["max_speed_other_mps"],
            "max_accel_boat_mps2": self._tracker_kwargs["max_accel_boat_mps2"],
            "max_accel_other_mps2": self._tracker_kwargs["max_accel_other_mps2"],
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
            elif key in self._tracker_kwargs:
                # Update for future trackers.
                self._tracker_kwargs[key] = val
                # Update all existing tracker instances so the new cap
                # takes effect immediately.
                for trk in self.trackers:
                    if hasattr(trk, key):
                        setattr(trk, key, val)

        return self.get_tunable_params()

