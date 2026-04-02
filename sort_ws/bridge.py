from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
from dataclasses import dataclass
from typing import Optional, Set, List, Dict, Any

import websockets
from websockets.exceptions import ConnectionClosed
from websockets.server import WebSocketServerProtocol

from .codec import decode_video_message, encode_video_message
from .ego import EgoSmoother, EgoStateStore
from .image_space_tracker import ImageSpaceSort
from .world_space_tracker import WorldSpaceSort
from .image_to_world import detection_to_world_latlon, enu_to_latlon, latlon_to_enu_m, ENU
from .path_planner import PathPlannerRuntime
from .world_to_image import world_to_image_space


@dataclass(frozen=True)
class BridgeConfig:
    upstream_host: str
    upstream_video_port: int
    upstream_nmea_port: int
    upstream_control_port: int

    downstream_bind: str
    downstream_video_port: int
    downstream_nmea_port: int
    downstream_control_port: int

    reconnect_delay_s: float = 1.0

    # Tracking mode: "auto", "world_space", or "image_space"
    mode: str = "auto"

    # World-tracking / camera params (used when mode == "world_space")
    world_space_cv_width_px: int = 1920
    world_space_fov_x_deg: float = 90.0
    world_space_camera_yaw_offset_deg: float = 0.0

    # Simulation ingestion ports (bridge acts as server, frontend connects as client)
    sim_video_port: int = 0
    sim_nmea_port: int = 0

    # Logging
    # If > 0, print a simple progress line every N upstream video frames.
    log_every_n_frames: int = 0
    input_min_confidence: float = 0.0


@dataclass
class SharedTrackerRuntimeState:
    # Drop raw detections below this confidence before either tracker sees them.
    input_min_confidence: float = 0.0


class BroadcastHub:
    def __init__(self, name: str):
        self.name = name
        self._clients: Set[WebSocketServerProtocol] = set()
        self._lock = asyncio.Lock()

    async def register(self, ws: WebSocketServerProtocol) -> None:
        async with self._lock:
            self._clients.add(ws)

    async def unregister(self, ws: WebSocketServerProtocol) -> None:
        async with self._lock:
            self._clients.discard(ws)

    async def broadcast(self, message) -> None:
        async with self._lock:
            clients = list(self._clients)
        if not clients:
            return
        dead: list[WebSocketServerProtocol] = []
        for ws in clients:
            try:
                await ws.send(message)
            except ConnectionClosed:
                dead.append(ws)
            except Exception:
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.discard(ws)


class TrackingModeState:
    """
    Shared, runtime-selectable tracking mode.

    Modes:
      - "auto": prefer world-space outputs when available, else fall back to image-space
      - "world_space": output world-space tracked bboxes (may be empty until ego arrives)
      - "image_space": output image-space tracked bboxes
    """

    def __init__(self, initial_mode: str = "auto"):
        self._lock = asyncio.Lock()
        self._mode = self._normalize(initial_mode)

    @staticmethod
    def _normalize(mode: str) -> str:
        m = str(mode or "").strip().lower()
        if m in ("auto", "world_space", "image_space"):
            return m
        return "auto"

    async def get(self) -> str:
        async with self._lock:
            return self._mode

    async def set(self, mode: str) -> str:
        async with self._lock:
            self._mode = self._normalize(mode)
            return self._mode


class PlaybackInfoCache:
    """
    Cache for playback_info from upstream, so newly connected clients can receive it.
    """

    def __init__(self):
        self._lock = asyncio.Lock()
        self._playback_info: Optional[Dict[str, Any]] = None

    async def update(self, playback_info: Dict[str, Any]) -> None:
        async with self._lock:
            self._playback_info = playback_info

    async def get(self) -> Optional[Dict[str, Any]]:
        async with self._lock:
            return self._playback_info


def _ws_url(host: str, port: int) -> str:
    return f"ws://{host}:{port}"


def _summarize_tracker_states(tracker: Any) -> Dict[str, int]:
    counts = {
        "tracks": 0,
        "warming_unconfirmed": 0,
        "matched": 0,
        "rewarming": 0,
        "ghost": 0,
    }
    try:
        trackers = list(getattr(tracker, "trackers", []) or [])
        min_hits = int(getattr(tracker, "min_hits", 0))
    except Exception:
        return counts

    counts["tracks"] = len(trackers)
    for trk in trackers:
        try:
            hits = int(getattr(trk, "hits", 0))
            hit_streak = int(getattr(trk, "hit_streak", 0))
            time_since_update = int(getattr(trk, "time_since_update", 0))
        except Exception:
            counts["warming_unconfirmed"] += 1
            continue

        # Mirror the guide/current tracker behavior:
        # - matched: confirmed and matched this frame
        # - ghost: confirmed but unmatched this frame
        # - rewarming: previously confirmed, matched again, but not reconfirmed yet
        # - warming_unconfirmed: everything else still short of confirmed output
        if time_since_update > 0 and hits >= min_hits:
            counts["ghost"] += 1
        elif time_since_update == 0 and hit_streak >= min_hits:
            counts["matched"] += 1
        elif time_since_update == 0 and hits > min_hits:
            counts["rewarming"] += 1
        else:
            counts["warming_unconfirmed"] += 1
    return counts


def _filtered_bboxes_for_tracking(
    bboxes: List[Dict[str, Any]],
    runtime_state: SharedTrackerRuntimeState,
) -> List[Dict[str, Any]]:
    threshold = float(getattr(runtime_state, "input_min_confidence", 0.0) or 0.0)
    if threshold <= 0.0:
        return bboxes

    filtered: List[Dict[str, Any]] = []
    for bbox in bboxes:
        if not isinstance(bbox, dict):
            continue
        try:
            confidence = float(bbox.get("confidence", float("nan")))
        except Exception:
            confidence = float("nan")
        if not math.isnan(confidence) and confidence < threshold:
            continue
        filtered.append(bbox)
    return filtered


def _get_tracker_params_payload(
    world_tracker: Optional[WorldSpaceSort],
    runtime_state: SharedTrackerRuntimeState,
) -> Dict[str, Any]:
    params = world_tracker.get_tunable_params() if world_tracker is not None else {}
    params["input_min_confidence"] = float(getattr(runtime_state, "input_min_confidence", 0.0) or 0.0)
    return params


def _track_image_space(bboxes: list, tracker: ImageSpaceSort) -> List[Dict[str, Any]]:
    assigned_ids, unmatched_tracks = tracker.assign(bboxes)
    tracked_bboxes: List[Dict[str, Any]] = []
    
    # Process matched detections
    for det, track_id in zip(bboxes, assigned_ids):
        if not isinstance(det, dict):
            continue
        # If the detection isn't confirmed by the tracker yet, drop it entirely.
        # This avoids leaking upstream `obj_id` values downstream.
        if track_id is None:
            continue
        det2 = dict(det)
        # Add track_id from SORT tracker; keep original obj_id as-is
        det2["track_id"] = int(track_id)
        det2["tracked_space"] = "image_space"
        det2["tracked_status"] = "matched"
        tracked_bboxes.append(det2)
    
    # Process unmatched but confirmed tracks (predicted state)
    for unmatched in unmatched_tracks:
        det2 = dict(unmatched)
        is_rewarming = det2.pop("_is_rewarming", False)
        det2["tracked_space"] = "image_space"
        det2["tracked_status"] = "unmatched"
        det2["unmatched_status"] = "rewarming" if is_rewarming else "ghost"
        tracked_bboxes.append(det2)
    
    return tracked_bboxes


async def _track_world_space(
    *,
    bboxes: list,
    world_tracker: WorldSpaceSort,
    ego_store: EgoStateStore,
    cfg: BridgeConfig,
) -> List[Dict[str, Any]]:
    ego = await ego_store.get()
    if ego.latlon is None or ego.heading_deg is None or ego.ref_latlon is None:
        return []

    fov_x_rad = float(cfg.world_space_fov_x_deg) * 3.141592653589793 / 180.0
    local_ref = ego.ref_latlon

    world_dets: List[Dict[str, Any]] = []
    # keep mapping back to original list order if needed later
    for det in bboxes:
        if not isinstance(det, dict):
            continue
        try:
            x = float(det.get("x"))
            w = float(det.get("width", det.get("w")))
            distance_m = float(det.get("distance"))
        except Exception:
            continue
        if not (distance_m > 0.0) or not (w > 0.0):
            continue

        x_bottom_center = x + w / 2.0
        world_latlon, enu_from_ref, enu_rel_ego = detection_to_world_latlon(
            ego_latlon=ego.latlon,
            ego_heading_deg=float(ego.heading_deg),
            x_bottom_center_px=float(x_bottom_center),
            distance_m=float(distance_m),
            image_width_px=float(cfg.world_space_cv_width_px),
            fov_x_rad=fov_x_rad,
            camera_yaw_offset_deg=float(cfg.world_space_camera_yaw_offset_deg),
            local_ref=local_ref,
        )
        det2 = dict(det)
        det2["world_latitude"] = float(world_latlon.lat)
        det2["world_longitude"] = float(world_latlon.lon)
        det2["world_east_m"] = float(enu_from_ref.east_m)
        det2["world_north_m"] = float(enu_from_ref.north_m)
        det2["world_rel_ego_east_m"] = float(enu_rel_ego.east_m)
        det2["world_rel_ego_north_m"] = float(enu_rel_ego.north_m)
        world_dets.append(det2)

    assigned_ids, unmatched_tracks, matched_track_states = world_tracker.assign(world_dets)
    tracked_bboxes: List[Dict[str, Any]] = []

    # ENU reference point (same for all tracks, set once per session)
    enu_ref_lat = float(local_ref.lat)
    enu_ref_lon = float(local_ref.lon)
    ego_enu_from_ref = latlon_to_enu_m(local_ref, ego.latlon)
    
    # Process matched detections
    for det, track_id in zip(world_dets, assigned_ids):
        if track_id is None:
            continue
        det2 = dict(det)
        # Add track_id from SORT tracker; keep original obj_id as-is
        det2["track_id"] = int(track_id)
        det2["tracked_space"] = "world_space"
        det2["tracked_status"] = "matched"
        # ENU reference point
        det2["enu_ref_lat"] = enu_ref_lat
        det2["enu_ref_lon"] = enu_ref_lon
        # Merge full KF kinematic state (velocity, acceleration, speed, course)
        kf_state = matched_track_states.get(int(track_id))
        if kf_state:
            kf_enu = ENU(east_m=float(kf_state["east_m"]), north_m=float(kf_state["north_m"]))
            kf_latlon = enu_to_latlon(local_ref, kf_enu)
            kf_rel_ego = ENU(
                east_m=float(kf_enu.east_m - ego_enu_from_ref.east_m),
                north_m=float(kf_enu.north_m - ego_enu_from_ref.north_m),
            )
            det2["world_east_m"] = float(kf_enu.east_m)
            det2["world_north_m"] = float(kf_enu.north_m)
            det2["world_latitude"] = float(kf_latlon.lat)
            det2["world_longitude"] = float(kf_latlon.lon)
            det2["world_rel_ego_east_m"] = float(kf_rel_ego.east_m)
            det2["world_rel_ego_north_m"] = float(kf_rel_ego.north_m)
            det2["vel_east_mps"] = kf_state["vel_east_mps"]
            det2["vel_north_mps"] = kf_state["vel_north_mps"]
            det2["speed_mps"] = kf_state["speed_mps"]
            det2["course_deg"] = kf_state["course_deg"]
            det2["accel_east_mps2"] = kf_state["accel_east_mps2"]
            det2["accel_north_mps2"] = kf_state["accel_north_mps2"]
            det2["measurement_noise_cross_var_m2"] = kf_state["measurement_noise_cross_var_m2"]
            det2["measurement_noise_radial_var_m2"] = kf_state["measurement_noise_radial_var_m2"]
            det2["state_position_covariance_enu"] = kf_state["state_position_covariance_enu"]
            det2["process_position_covariance_enu"] = kf_state["process_position_covariance_enu"]
            det2["measurement_position_covariance_enu"] = kf_state["measurement_position_covariance_enu"]
        tracked_bboxes.append(det2)
    
    # Process unmatched but confirmed tracks (ghost tracks and re-warming tracks)
    for unmatched in unmatched_tracks:
        # Check if this is a re-warming track (has detection data) vs ghost track (needs back-projection)
        is_rewarming = unmatched.get("_is_rewarming", False)
        
        if is_rewarming:
            # Re-warming track: use detection data directly (no back-projection needed)
            # The detection already has x, y, width, height, distance, confidence, heading, category, obj_id, etc.
            # KF kinematic state fields (vel_*, speed_mps, etc.) already included by assign().
            det2 = dict(unmatched)
            det2.pop("_is_rewarming", None)  # Remove internal marker
            det2["tracked_space"] = "world_space"
            det2["tracked_status"] = "unmatched"
            det2["unmatched_status"] = "rewarming"
            det2["enu_ref_lat"] = enu_ref_lat
            det2["enu_ref_lon"] = enu_ref_lon
            tracked_bboxes.append(det2)
        else:
            # Ghost track: back-project world position to image space
            enu = ENU(east_m=unmatched["world_east_m"], north_m=unmatched["world_north_m"])
            latlon = enu_to_latlon(local_ref, enu)
            
            # Back-project world position to image space for AR view
            x_center_px, distance_m, enu_rel_ego = world_to_image_space(
                world_east_m=unmatched["world_east_m"],
                world_north_m=unmatched["world_north_m"],
                ego_latlon=ego.latlon,
                ego_heading_deg=float(ego.heading_deg),
                image_width_px=float(cfg.world_space_cv_width_px),
                fov_x_rad=fov_x_rad,
                camera_yaw_offset_deg=float(cfg.world_space_camera_yaw_offset_deg),
                local_ref=local_ref,
            )
            
            det2 = dict(unmatched)
            det2["world_latitude"] = float(latlon.lat)
            det2["world_longitude"] = float(latlon.lon)
            det2["tracked_space"] = "world_space"
            det2["tracked_status"] = "unmatched"
            det2["unmatched_status"] = "ghost"
            det2["enu_ref_lat"] = enu_ref_lat
            det2["enu_ref_lon"] = enu_ref_lon
            # KF kinematic state fields (vel_*, speed_mps, etc.) already included by assign().
            
            # Keep the historical ghost back-projection logic here for future reference,
            # but stop emitting image-space bbox fields so ghost tracks do not render on
            # the AR/video overlays.
            last_y = unmatched.get("last_y_px")
            last_width = unmatched.get("last_width_px")
            last_height = unmatched.get("last_height_px")
            
            # Use stored values or defaults
            y_px = float(last_y) if last_y is not None and not math.isnan(last_y) else 0.0
            width_px = float(last_width) if last_width is not None and not math.isnan(last_width) else 50.0
            height_px = float(last_height) if last_height is not None and not math.isnan(last_height) else 50.0
            
            # det2["x"] = float(x_center_px - width_px / 2.0)
            # det2["y"] = y_px
            # det2["width"] = width_px
            # det2["height"] = height_px
            det2["distance"] = float(distance_m)
            det2["world_rel_ego_east_m"] = float(enu_rel_ego.east_m)
            det2["world_rel_ego_north_m"] = float(enu_rel_ego.north_m)
            
            tracked_bboxes.append(det2)
    
    return tracked_bboxes

async def _process_video_payload(
    payload: bytes,
    *,
    cfg: BridgeConfig,
    mode_state: TrackingModeState,
    image_tracker: ImageSpaceSort,
    world_tracker: Optional[WorldSpaceSort],
    ego_store: Optional[EgoStateStore],
    path_planner: Optional[PathPlannerRuntime],
    video_hub: BroadcastHub,
    frame_counter: List[int],
    runtime_state: SharedTrackerRuntimeState,
    source_label: str = "upstream",
) -> None:
    """Shared tracking pipeline for both upstream replay and simulation ingestion."""
    frame_counter[0] += 1
    vm = decode_video_message(payload)
    bboxes = vm.metadata.get("bboxes", [])
    if not isinstance(bboxes, list):
        bboxes = []
    bboxes_for_tracking = _filtered_bboxes_for_tracking(bboxes, runtime_state)

    # Always compute image-space tracks.
    tracked_local = _track_image_space(bboxes_for_tracking, image_tracker)

    # Also compute world-space tracks when possible (may be empty until ego arrives).
    tracked_world: List[Dict[str, Any]] = []
    if world_tracker is not None and ego_store is not None:
        tracked_world = await _track_world_space(
            bboxes=bboxes_for_tracking,
            world_tracker=world_tracker,
            ego_store=ego_store,
            cfg=cfg,
        )

    path_planning_payload: Optional[Dict[str, Any]] = None
    if path_planner is not None and ego_store is not None:
        ego = await ego_store.get()
        if ego.heading_deg is not None and ego.enu_from_ref is not None and ego.ref_latlon is not None:
            result = await path_planner.compute(
                frame_number=frame_counter[0],
                ego_enu=ego.enu_from_ref,
                ego_ref_latlon=ego.ref_latlon,
                ego_heading_deg=float(ego.heading_deg),
                ego_vel_east_mps=float(ego.vel_east_mps or 0.0),
                ego_vel_north_mps=float(ego.vel_north_mps or 0.0),
                tracked_world=tracked_world,
            )
            if result is not None:
                path_planning_payload = result.to_payload(
                    enu_ref_lat=float(ego.ref_latlon.lat),
                    enu_ref_lon=float(ego.ref_latlon.lon),
                )

    # Choose which list goes into `bboxes`.
    mode = await mode_state.get()
    tracked_bboxes = tracked_local
    selected_space = "image_space"
    if mode == "world_space":
        tracked_bboxes = tracked_world
        selected_space = "world_space"
    elif mode == "auto":
        if tracked_world:
            tracked_bboxes = tracked_world
            selected_space = "world_space"

    vm.metadata["bboxes"] = tracked_bboxes
    vm.metadata["tracked"] = True
    if path_planning_payload is not None:
        vm.metadata["path_planning"] = path_planning_payload
    else:
        vm.metadata.pop("path_planning", None)
    out_payload = encode_video_message(vm.metadata, vm.jpeg_bytes)
    await video_hub.broadcast(out_payload)

    if cfg.log_every_n_frames and frame_counter[0] % int(cfg.log_every_n_frames) == 0:
        active = world_tracker if (selected_space == "world_space" and world_tracker is not None) else image_tracker
        active_counts = _summarize_tracker_states(active)
        try:
            active_frame_count = int(getattr(active, "frame_count", -1))
        except Exception:
            active_frame_count = -1
        try:
            active_max_age = int(getattr(active, "max_age", -1))
        except Exception:
            active_max_age = -1
        print(
            f"[sort-ws] [{source_label}] frames={frame_counter[0]} "
            f"tracker_frames={active_frame_count} "
            f"tracks={active_counts['tracks']} "
            f"warming_unconfirmed={active_counts['warming_unconfirmed']} "
            f"matched={active_counts['matched']} "
            f"rewarming={active_counts['rewarming']} "
            f"ghost={active_counts['ghost']} "
            f"max_age={active_max_age}"
        )


async def _upstream_video_loop(
    cfg: BridgeConfig,
    mode_state: TrackingModeState,
    image_tracker: ImageSpaceSort,
    world_tracker: Optional[WorldSpaceSort],
    ego_store: Optional[EgoStateStore],
    path_planner: Optional[PathPlannerRuntime],
    video_hub: BroadcastHub,
    runtime_state: SharedTrackerRuntimeState,
) -> None:
    url = _ws_url(cfg.upstream_host, cfg.upstream_video_port)
    frame_counter = [0]
    while True:
        try:
            async with websockets.connect(url, max_size=None) as upstream:
                print(f"[sort-ws] Connected upstream video: {url}")
                async for payload in upstream:
                    if not isinstance(payload, (bytes, bytearray)):
                        continue
                    await _process_video_payload(
                        payload,
                        cfg=cfg,
                        mode_state=mode_state,
                        image_tracker=image_tracker,
                        world_tracker=world_tracker,
                        ego_store=ego_store,
                        path_planner=path_planner,
                        video_hub=video_hub,
                        frame_counter=frame_counter,
                        runtime_state=runtime_state,
                        source_label="upstream",
                    )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[sort-ws] Upstream video error ({url}): {e}", file=sys.stderr)
            await asyncio.sleep(cfg.reconnect_delay_s)


async def _upstream_nmea_loop(
    cfg: BridgeConfig, nmea_hub: BroadcastHub, ego_store: Optional[EgoStateStore]
) -> None:
    url = _ws_url(cfg.upstream_host, cfg.upstream_nmea_port)
    while True:
        try:
            async with websockets.connect(url, max_size=None) as upstream:
                print(f"[sort-ws] Connected upstream NMEA: {url}")
                async for msg in upstream:
                    # NMEA is JSON text in replay; forward as-is.
                    if ego_store is not None and isinstance(msg, str):
                        await ego_store.update_from_nmea_json(msg)
                    await nmea_hub.broadcast(msg)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[sort-ws] Upstream NMEA error ({url}): {e}", file=sys.stderr)
            await asyncio.sleep(cfg.reconnect_delay_s)


async def _upstream_control_loop(
    cfg: BridgeConfig,
    control_hub: BroadcastHub,
    outbound_to_upstream: "asyncio.Queue[str]",
    playback_cache: PlaybackInfoCache,
) -> None:
    url = _ws_url(cfg.upstream_host, cfg.upstream_control_port)
    while True:
        try:
            async with websockets.connect(url, max_size=None) as upstream:
                print(f"[sort-ws] Connected upstream control: {url}")

                async def sender() -> None:
                    while True:
                        msg = await outbound_to_upstream.get()
                        await upstream.send(msg)

                sender_task = asyncio.create_task(sender())
                try:
                    async for msg in upstream:
                        # Cache playback_info for newly connecting clients
                        try:
                            obj = json.loads(msg)
                            if isinstance(obj, dict) and "playback_info" in obj:
                                await playback_cache.update(obj["playback_info"])
                        except Exception:
                            pass
                        await control_hub.broadcast(msg)
                finally:
                    sender_task.cancel()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[sort-ws] Upstream control error ({url}): {e}", file=sys.stderr)
            await asyncio.sleep(cfg.reconnect_delay_s)


async def _downstream_video_handler(ws: WebSocketServerProtocol, video_hub: BroadcastHub) -> None:
    await video_hub.register(ws)
    try:
        # Clients generally don't send anything on the video socket; drain if they do.
        async for _ in ws:
            pass
    finally:
        await video_hub.unregister(ws)


async def _downstream_nmea_handler(ws: WebSocketServerProtocol, nmea_hub: BroadcastHub) -> None:
    await nmea_hub.register(ws)
    try:
        async for _ in ws:
            pass
    finally:
        await nmea_hub.unregister(ws)


async def _downstream_control_handler(
    ws: WebSocketServerProtocol,
    control_hub: BroadcastHub,
    outbound_to_upstream: "asyncio.Queue[str]",
    mode_state: TrackingModeState,
    playback_cache: PlaybackInfoCache,
    world_tracker: Optional[WorldSpaceSort] = None,
    path_planner: Optional[PathPlannerRuntime] = None,
    runtime_state: Optional[SharedTrackerRuntimeState] = None,
) -> None:
    await control_hub.register(ws)
    try:
        # Send current mode to newly connected clients.
        try:
            await ws.send(json.dumps({"tracking_mode": await mode_state.get()}))
        except Exception:
            pass

        # Send cached playback_info to newly connected clients.
        try:
            cached_info = await playback_cache.get()
            if cached_info is not None:
                await ws.send(json.dumps({"playback_info": cached_info}))
        except Exception:
            pass

        # Send current tracker params to newly connected clients.
        try:
            if runtime_state is not None:
                await ws.send(json.dumps({"tracker_params": _get_tracker_params_payload(world_tracker, runtime_state)}))
        except Exception:
            pass

        try:
            if path_planner is not None:
                await ws.send(json.dumps({"path_planning_params": await path_planner.get_params()}))
        except Exception:
            pass

        async for msg in ws:
            # Forward downstream control commands upstream (pause/play/seek),
            # but intercept local tracking-mode and tracker-param changes.
            if isinstance(msg, (bytes, bytearray)):
                try:
                    msg = msg.decode("utf-8")
                except Exception:
                    continue
            msg_s = str(msg)
            try:
                obj = json.loads(msg_s)
            except Exception:
                obj = None

            if isinstance(obj, dict):
                cmd = str(obj.get("command") or obj.get("cmd") or "").strip().lower()
                if cmd in ("set_tracking_mode", "set_mode", "set_tracked_space_mode"):
                    requested = obj.get("mode") or obj.get("tracking_mode") or obj.get("tracked_space_mode") or "auto"
                    new_mode = await mode_state.set(str(requested))
                    notice = json.dumps({"tracking_mode": new_mode})
                    try:
                        await ws.send(json.dumps({"ack": "set_tracking_mode", "tracking_mode": new_mode}))
                    except Exception:
                        pass
                    await control_hub.broadcast(notice)
                    continue
                if cmd in ("get_tracking_mode", "get_mode", "get_tracked_space_mode"):
                    cur = await mode_state.get()
                    try:
                        await ws.send(json.dumps({"tracking_mode": cur}))
                    except Exception:
                        pass
                    continue
                if cmd == "get_playback_info":
                    # Always forward upstream so clients can refresh playback_info
                    # (cache may be stale, e.g. total_frames=0 before video loads).
                    await outbound_to_upstream.put(msg_s)
                    continue

                # --- Tracker param commands (intercepted, NOT forwarded upstream) ---
                if cmd == "get_tracker_params":
                    if runtime_state is not None:
                        try:
                            await ws.send(json.dumps({"tracker_params": _get_tracker_params_payload(world_tracker, runtime_state)}))
                        except Exception:
                            pass
                    continue
                if cmd == "set_tracker_params":
                    if runtime_state is not None:
                        new_params = {k: v for k, v in obj.items() if k not in ("command", "cmd")}
                        raw_input_min_conf = new_params.pop("input_min_confidence", None)
                        if raw_input_min_conf is not None:
                            try:
                                runtime_state.input_min_confidence = max(0.0, float(raw_input_min_conf))
                            except (TypeError, ValueError):
                                pass
                        updated = _get_tracker_params_payload(world_tracker, runtime_state)
                        if world_tracker is not None:
                            updated.update(world_tracker.set_tunable_params(new_params))
                        updated["input_min_confidence"] = float(runtime_state.input_min_confidence)
                        try:
                            await ws.send(json.dumps({"ack": "set_tracker_params", "tracker_params": updated}))
                        except Exception:
                            pass
                    continue
                if cmd == "get_path_planning_params":
                    if path_planner is not None:
                        try:
                            await ws.send(json.dumps({"path_planning_params": await path_planner.get_params()}))
                        except Exception:
                            pass
                    continue
                if cmd == "set_path_planning_params":
                    if path_planner is not None:
                        new_params = {k: v for k, v in obj.items() if k not in ("command", "cmd")}
                        updated = await path_planner.set_params(new_params)
                        try:
                            await ws.send(json.dumps({"ack": "set_path_planning_params", "path_planning_params": updated}))
                        except Exception:
                            pass
                    continue
                if cmd == "set_path_planning_route":
                    if path_planner is not None:
                        updated = await path_planner.set_route_points(obj.get("route_points"))
                        try:
                            await ws.send(json.dumps({"ack": "set_path_planning_route", **updated}))
                        except Exception:
                            pass
                    continue

            await outbound_to_upstream.put(msg_s)
    finally:
        await control_hub.unregister(ws)


async def _sim_video_handler(
    ws: WebSocketServerProtocol,
    *,
    cfg: BridgeConfig,
    mode_state: TrackingModeState,
    image_tracker: ImageSpaceSort,
    world_tracker: Optional[WorldSpaceSort],
    ego_store: Optional[EgoStateStore],
    path_planner: Optional[PathPlannerRuntime],
    video_hub: BroadcastHub,
    runtime_state: SharedTrackerRuntimeState,
) -> None:
    """Accept video+bbox messages from the frontend simulator."""
    print("[sort-ws] Simulation video client connected")
    frame_counter = [0]
    try:
        async for payload in ws:
            if not isinstance(payload, (bytes, bytearray)):
                continue
            await _process_video_payload(
                payload,
                cfg=cfg,
                mode_state=mode_state,
                image_tracker=image_tracker,
                world_tracker=world_tracker,
                ego_store=ego_store,
                path_planner=path_planner,
                video_hub=video_hub,
                frame_counter=frame_counter,
                        runtime_state=runtime_state,
                source_label="sim",
            )
    except ConnectionClosed:
        pass
    finally:
        print(f"[sort-ws] Simulation video client disconnected (processed {frame_counter[0]} frames)")


async def _sim_nmea_handler(
    ws: WebSocketServerProtocol,
    *,
    nmea_hub: BroadcastHub,
    ego_store: Optional[EgoStateStore],
) -> None:
    """Accept NMEA JSON messages from the frontend simulator."""
    print("[sort-ws] Simulation NMEA client connected")
    msg_count = 0
    try:
        async for msg in ws:
            if ego_store is not None and isinstance(msg, str):
                await ego_store.update_from_nmea_json(msg)
            await nmea_hub.broadcast(msg)
            msg_count += 1
    except ConnectionClosed:
        pass
    finally:
        print(f"[sort-ws] Simulation NMEA client disconnected ({msg_count} messages)")


async def run_bridge(
    cfg: BridgeConfig,
    image_tracker: ImageSpaceSort,
    world_tracker: Optional[WorldSpaceSort] = None,
    ego_store: Optional[EgoStateStore] = None,
) -> None:
    video_hub = BroadcastHub("video")
    nmea_hub = BroadcastHub("nmea")
    control_hub = BroadcastHub("control")
    outbound_to_upstream: asyncio.Queue[str] = asyncio.Queue()
    mode_state = TrackingModeState(cfg.mode)
    playback_cache = PlaybackInfoCache()
    path_planner = PathPlannerRuntime()
    runtime_state = SharedTrackerRuntimeState(input_min_confidence=float(cfg.input_min_confidence))

    # Start downstream servers and keep the returned server objects alive.
    video_server = await websockets.serve(
        lambda ws: _downstream_video_handler(ws, video_hub),
        cfg.downstream_bind,
        cfg.downstream_video_port,
        max_size=None,
    )
    nmea_server = await websockets.serve(
        lambda ws: _downstream_nmea_handler(ws, nmea_hub),
        cfg.downstream_bind,
        cfg.downstream_nmea_port,
        max_size=None,
    )
    control_server = await websockets.serve(
        lambda ws: _downstream_control_handler(
            ws,
            control_hub,
            outbound_to_upstream,
            mode_state,
            playback_cache,
            world_tracker,
            path_planner,
            runtime_state,
        ),
        cfg.downstream_bind,
        cfg.downstream_control_port,
        max_size=None,
    )

    # Simulation ingestion servers (frontend connects as client)
    sim_servers = []
    if cfg.sim_video_port:
        sim_video_server = await websockets.serve(
            lambda ws: _sim_video_handler(
                ws,
                cfg=cfg,
                mode_state=mode_state,
                image_tracker=image_tracker,
                world_tracker=world_tracker,
                ego_store=ego_store,
                path_planner=path_planner,
                video_hub=video_hub,
                runtime_state=runtime_state,
            ),
            cfg.downstream_bind,
            cfg.sim_video_port,
            max_size=None,
        )
        sim_servers.append(sim_video_server)

    if cfg.sim_nmea_port:
        sim_nmea_server = await websockets.serve(
            lambda ws: _sim_nmea_handler(ws, nmea_hub=nmea_hub, ego_store=ego_store),
            cfg.downstream_bind,
            cfg.sim_nmea_port,
            max_size=None,
        )
        sim_servers.append(sim_nmea_server)

    print("[sort-ws] Starting downstream websocket servers:")
    print(f"  video   ws://{cfg.downstream_bind}:{cfg.downstream_video_port}")
    print(f"  nmea    ws://{cfg.downstream_bind}:{cfg.downstream_nmea_port}")
    print(f"  control ws://{cfg.downstream_bind}:{cfg.downstream_control_port}")
    if cfg.sim_video_port:
        print(f"  sim-video ws://{cfg.downstream_bind}:{cfg.sim_video_port}")
    if cfg.sim_nmea_port:
        print(f"  sim-nmea  ws://{cfg.downstream_bind}:{cfg.sim_nmea_port}")
    print("[sort-ws] Connecting to upstream websocket servers:")
    print(f"  video   {_ws_url(cfg.upstream_host, cfg.upstream_video_port)}")
    print(f"  nmea    {_ws_url(cfg.upstream_host, cfg.upstream_nmea_port)}")
    print(f"  control {_ws_url(cfg.upstream_host, cfg.upstream_control_port)}")

    try:
        await asyncio.gather(
            _upstream_video_loop(cfg, mode_state, image_tracker, world_tracker, ego_store, path_planner, video_hub, runtime_state),
            _upstream_nmea_loop(cfg, nmea_hub, ego_store),
            _upstream_control_loop(cfg, control_hub, outbound_to_upstream, playback_cache),
        )
    finally:
        video_server.close()
        nmea_server.close()
        control_server.close()
        for s in sim_servers:
            s.close()
        await video_server.wait_closed()
        await nmea_server.wait_closed()
        await control_server.wait_closed()
        for s in sim_servers:
            await s.wait_closed()


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="SORT websocket bridge: consume replay streams, run SORT, re-broadcast with tracker IDs."
    )
    p.add_argument("--upstream-host", type=str, default="127.0.0.1")
    p.add_argument("--upstream-video-port", type=int, default=5001)
    p.add_argument("--upstream-nmea-port", type=int, default=3636)
    p.add_argument("--upstream-control-port", type=int, default=6001)

    p.add_argument("--downstream-bind", type=str, default="0.0.0.0")
    p.add_argument("--downstream-video-port", type=int, default=5002)
    p.add_argument("--downstream-nmea-port", type=int, default=3637)
    p.add_argument("--downstream-control-port", type=int, default=6002)

    # Image-space tracker params (shown in --help)
    p.add_argument("--image-space-max-age", dest="image_space_max_age", type=int, default=40)
    p.add_argument("--image-space-min-hits", dest="image_space_min_hits", type=int, default=300)
    p.add_argument("--image-space-iou-threshold", dest="image_space_iou_threshold", type=float, default=0.1)
    p.add_argument("--image-space-alpha-distance", dest="image_space_alpha_distance", type=float, default=0.15)
    p.add_argument("--image-space-beta-heading", dest="image_space_beta_heading", type=float, default=0.0)
    p.add_argument("--image-space-gamma-confidence", dest="image_space_gamma_confidence", type=float, default=0.0)
    p.add_argument("--reconnect-delay-s", type=float, default=1.0)

    # World-space tracking
    p.add_argument(
        "--mode",
        type=str,
        default="auto",
        choices=["auto", "world_space", "image_space"],
        help='Tracking mode. "auto" prefers world-space outputs when available, else falls back to image-space. "world_space" uses NMEA ego lat/lon/heading and tracks in ENU meters. "image_space" runs image-space SORT.',
    )
    p.add_argument(
        "--world-space-cv-width-px",
        dest="world_space_cv_width_px",
        type=int,
        default=1920,
        help="CV image width in pixels (used to compute bearing from x pixel).",
    )
    p.add_argument(
        "--world-space-fov-x-deg",
        dest="world_space_fov_x_deg",
        type=float,
        default=55.3,
        help="Camera horizontal field-of-view in degrees.",
    )
    p.add_argument(
        "--world-space-camera-yaw-offset-deg",
        dest="world_space_camera_yaw_offset_deg",
        type=float,
        default=0.0,
        help="Yaw offset between camera forward and boat forward (degrees).",
    )

    # World-space tracker params (shown in --help)
    p.add_argument("--world-space-max-age", dest="world_space_max_age", type=int, default=50)
    p.add_argument("--world-space-min-hits", dest="world_space_min_hits", type=int, default=5)
    p.add_argument(
        "--world-space-max-distance-m",
        dest="world_space_max_distance_m",
        type=float,
        default=50.0,
        help="Max world-space (meters) to allow a detection to match a track.",
    )
    p.add_argument(
        "--world-space-beta-heading",
        dest="world_space_beta_heading",
        type=float,
        default=0.0,
        help="Weight for target heading difference in world association cost.",
    )
    p.add_argument(
        "--world-space-gamma-confidence",
        dest="world_space_gamma_confidence",
        type=float,
        default=0.0,
        help="Weight for (1-confidence) in world association cost.",
    )
    p.add_argument(
        "--world-space-kf-model",
        dest="world_space_kf_model",
        type=str,
        default="cv",
        choices=["cv", "ca"],
        help='Kalman filter motion model for world-space tracker. "cv" = constant velocity (4-state), "ca" = constant acceleration (6-state).',
    )
    p.add_argument(
        "--world-space-q-intensity",
        dest="world_space_q_intensity",
        type=float,
        default=None,
        help="Process-noise intensity for the world-space KF motion model. If omitted, defaults to 0.0.",
    )
    p.add_argument(
        "--world-space-measurement-cross-var-m2",
        dest="world_space_measurement_noise_cross_var_m2",
        type=float,
        default=None,
        help="Base world-space measurement covariance across the radial line-of-sight direction, in square meters. If omitted, defaults to 1e6.",
    )
    p.add_argument(
        "--world-space-measurement-radial-scale",
        dest="world_space_measurement_noise_radial_scale",
        type=float,
        default=200.0,
        help="Scale factor applied to the measurement covariance along the radial ego-to-target direction. 200.0 is the default; values > 1 smooth radial jitter more aggressively.",
    )

    # Per-category kinematic caps (always active; use a large value to effectively uncap)
    p.add_argument(
        "--world-space-max-speed-boat-mps",
        dest="world_space_max_speed_boat_mps",
        type=float,
        default=20.0,
        help="Max speed cap for 'boat' category tracks (m/s). 0 = clamp velocity to zero. Use a large value (e.g. 1000) to effectively uncap.",
    )
    p.add_argument(
        "--world-space-max-speed-other-mps",
        dest="world_space_max_speed_other_mps",
        type=float,
        default=0.0,
        help="Max speed cap for non-boat category tracks (m/s). 0 = clamp velocity to zero. Use a large value (e.g. 1000) to effectively uncap.",
    )
    p.add_argument(
        "--world-space-max-accel-boat-mps2",
        dest="world_space_max_accel_boat_mps2",
        type=float,
        default=10.0,
        help="Max acceleration cap for 'boat' category tracks (m/s²). Only used with CA model. 0 = clamp acceleration to zero. Use a large value (e.g. 100) to effectively uncap.",
    )
    p.add_argument(
        "--world-space-max-accel-other-mps2",
        dest="world_space_max_accel_other_mps2",
        type=float,
        default=0.0,
        help="Max acceleration cap for non-boat category tracks (m/s²). Only used with CA model. 0 = clamp acceleration to zero. Use a large value (e.g. 100) to effectively uncap.",
    )

    # Simulation ingestion ports (bridge acts as server, frontend connects as client)
    p.add_argument(
        "--sim-video-port",
        dest="sim_video_port",
        type=int,
        default=5003,
        help="Port for simulation video ingestion (0 to disable). Frontend simulator connects here to send video+bbox data.",
    )
    p.add_argument(
        "--sim-nmea-port",
        dest="sim_nmea_port",
        type=int,
        default=3638,
        help="Port for simulation NMEA ingestion (0 to disable). Frontend simulator connects here to send NMEA data.",
    )

    p.add_argument(
        "--log-every-n-frames",
        type=int,
        default=0,
        help="If > 0, print a progress line every N upstream video frames (includes tracker frame_count / max_age).",
    )
    p.add_argument(
        "--input-min-confidence",
        type=float,
        default=0.0,
        help="Drop raw detections below this confidence before either tracker processes them.",
    )

    return p


def cli_main(argv: Optional[list[str]] = None) -> None:
    args = build_arg_parser().parse_args(argv)
    mode = str(args.mode)
    cfg = BridgeConfig(
        upstream_host=args.upstream_host,
        upstream_video_port=args.upstream_video_port,
        upstream_nmea_port=args.upstream_nmea_port,
        upstream_control_port=args.upstream_control_port,
        downstream_bind=args.downstream_bind,
        downstream_video_port=args.downstream_video_port,
        downstream_nmea_port=args.downstream_nmea_port,
        downstream_control_port=args.downstream_control_port,
        reconnect_delay_s=args.reconnect_delay_s,
        mode=mode,
        world_space_cv_width_px=int(args.world_space_cv_width_px),
        world_space_fov_x_deg=float(args.world_space_fov_x_deg),
        world_space_camera_yaw_offset_deg=float(args.world_space_camera_yaw_offset_deg),
        sim_video_port=int(args.sim_video_port),
        sim_nmea_port=int(args.sim_nmea_port),
        log_every_n_frames=int(args.log_every_n_frames),
        input_min_confidence=float(args.input_min_confidence),
    )

    image_tracker = ImageSpaceSort(
        image_space_max_age=args.image_space_max_age,
        image_space_min_hits=args.image_space_min_hits,
        image_space_iou_threshold=args.image_space_iou_threshold,
        image_space_alpha_distance=args.image_space_alpha_distance,
        image_space_beta_heading=args.image_space_beta_heading,
        image_space_gamma_confidence=args.image_space_gamma_confidence,
    )

    # Always initialize world-space tracking components so downstream can switch modes at runtime.
    ego_store: Optional[EgoStateStore] = EgoStateStore(EgoSmoother())
    world_tracker: Optional[WorldSpaceSort] = WorldSpaceSort(
        world_space_max_age=args.world_space_max_age,
        world_space_min_hits=args.world_space_min_hits,
        world_space_max_distance_m=args.world_space_max_distance_m,
        world_space_beta_heading=args.world_space_beta_heading,
        world_space_gamma_confidence=args.world_space_gamma_confidence,
        world_space_kf_model=args.world_space_kf_model,
        world_space_q_intensity=args.world_space_q_intensity,
        world_space_measurement_noise_cross_var_m2=args.world_space_measurement_noise_cross_var_m2,
        world_space_measurement_noise_radial_scale=args.world_space_measurement_noise_radial_scale,
        world_space_max_speed_boat_mps=args.world_space_max_speed_boat_mps,
        world_space_max_speed_other_mps=args.world_space_max_speed_other_mps,
        world_space_max_accel_boat_mps2=args.world_space_max_accel_boat_mps2,
        world_space_max_accel_other_mps2=args.world_space_max_accel_other_mps2,
    )
    print(f"[sort-ws] World-space KF model: {args.world_space_kf_model}")
    print(f"[sort-ws] Q intensity: {args.world_space_q_intensity}")
    print(f"[sort-ws] Measurement cross R var (m^2): {args.world_space_measurement_noise_cross_var_m2}")
    print(f"[sort-ws] Measurement radial R scale: {args.world_space_measurement_noise_radial_scale}")
    print(f"[sort-ws] Speed caps: boat={args.world_space_max_speed_boat_mps} m/s, other={args.world_space_max_speed_other_mps} m/s")
    print(f"[sort-ws] Accel caps: boat={args.world_space_max_accel_boat_mps2} m/s², other={args.world_space_max_accel_other_mps2} m/s²")

    try:
        asyncio.run(run_bridge(cfg, image_tracker, world_tracker, ego_store))
    except KeyboardInterrupt:
        pass


