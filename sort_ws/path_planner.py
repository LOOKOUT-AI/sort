from __future__ import annotations

import asyncio
import heapq
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .image_to_world import ENU, LatLon, latlon_to_enu_m, right_fwd_to_enu_m


GridCoord = Tuple[int, int]
CellCostMap = Dict[GridCoord, float]
PATH_PLANNING_MODES = (
    "astar_enu",
    "astar_ego_relative",
    "theta_enu",
    "theta_ego_relative",
)
VELOCITY_ARROW_PROJECTION_MODES = ("tcpa", "manual")
DEFAULT_EGO_OBSTACLE_HALF_ANGLE_DEG = 55.3 / 2.0
DEFAULT_COLREG_DANGER_RADIUS_M = 300.0
DEFAULT_COLREG_ANGLE_MIN_DEG = -90.0
DEFAULT_COLREG_ANGLE_MAX_DEG = 90.0
EGO_OBSTACLE_MARKER_COST = -1.0
COLREG_ZONE_MARKER_COST = -2.0


def _clamp(value: Any, lo: float, hi: float, *, integer: bool = False):
    try:
        num = float(value)
    except Exception:
        num = lo
    num = max(lo, min(hi, num))
    return int(round(num)) if integer else float(num)


def _coerce_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return bool(default)


def _coerce_mode(value: Any, default: str) -> str:
    mode = str(value or default).strip().lower()
    return mode if mode in PATH_PLANNING_MODES else default


def _coerce_velocity_arrow_mode(value: Any, default: str) -> str:
    mode = str(value or default).strip().lower()
    return mode if mode in VELOCITY_ARROW_PROJECTION_MODES else default


def _coerce_angle_range(
    min_value: Any,
    max_value: Any,
    *,
    default_min: float,
    default_max: float,
) -> Tuple[float, float]:
    angle_min = _clamp(min_value, -180.0, 180.0)
    angle_max = _clamp(max_value, -180.0, 180.0)
    if not math.isfinite(angle_min):
        angle_min = float(default_min)
    if not math.isfinite(angle_max):
        angle_max = float(default_max)
    if angle_min > angle_max:
        angle_min, angle_max = angle_max, angle_min
    return float(angle_min), float(angle_max)


@dataclass(frozen=True)
class PathPlanningParams:
    enabled: bool = True
    path_distance_m: float = 1000.0
    use_route_goal: bool = True
    route_goal_radius_m: float = 500.0
    route_attraction_enabled: bool = True
    route_attraction_corridor_radius_cells: int = 1
    route_attraction_cost_per_cell: float = 0.2
    route_attraction_max_cost: float = 2.0
    grid_size_m: float = 5.0
    cardinal_cost: float = 1.0
    diagonal_cost: float = 1.4
    run_every_n_frames: int = 5
    velocity_arrow_projection_mode: str = "tcpa"
    velocity_arrow_projection_time_s: float = 10.0
    track_current_position_cost: float = 1.0
    track_future_collision_cost: float = 3.0
    ego_current_position_cost: float = 2.0
    ego_future_collision_cost: float = 3.0
    track_cost_expansion_radius_cells: int = 2
    track_cost_expansion_decay: float = 0.5
    ego_obstacle_radius_m: float = 150.0
    ego_obstacle_angle_min_deg: float = -DEFAULT_EGO_OBSTACLE_HALF_ANGLE_DEG
    ego_obstacle_angle_max_deg: float = DEFAULT_EGO_OBSTACLE_HALF_ANGLE_DEG
    colreg_enabled: bool = True
    colreg_danger_radius_m: float = DEFAULT_COLREG_DANGER_RADIUS_M
    colreg_danger_angle_min_deg: float = DEFAULT_COLREG_ANGLE_MIN_DEG
    colreg_danger_angle_max_deg: float = DEFAULT_COLREG_ANGLE_MAX_DEG
    port_side_cost_decay: float = 0.1
    colreg_collision_goal_forward_m: float = 5.0
    mode: str = "astar_enu"
    show_debug_grid: bool = True

    @classmethod
    def from_mapping(cls, values: Optional[Dict[str, Any]] = None) -> "PathPlanningParams":
        raw = values or {}
        ego_obstacle_angle_min_deg, ego_obstacle_angle_max_deg = _coerce_angle_range(
            raw.get("ego_obstacle_angle_min_deg", cls.ego_obstacle_angle_min_deg),
            raw.get("ego_obstacle_angle_max_deg", cls.ego_obstacle_angle_max_deg),
            default_min=cls.ego_obstacle_angle_min_deg,
            default_max=cls.ego_obstacle_angle_max_deg,
        )
        colreg_danger_angle_min_deg, colreg_danger_angle_max_deg = _coerce_angle_range(
            raw.get("colreg_danger_angle_min_deg", cls.colreg_danger_angle_min_deg),
            raw.get("colreg_danger_angle_max_deg", cls.colreg_danger_angle_max_deg),
            default_min=cls.colreg_danger_angle_min_deg,
            default_max=cls.colreg_danger_angle_max_deg,
        )
        return cls(
            enabled=_coerce_bool(raw.get("enabled", cls.enabled), cls.enabled),
            path_distance_m=_clamp(raw.get("path_distance_m", cls.path_distance_m), 500.0, 1500.0),
            use_route_goal=_coerce_bool(raw.get("use_route_goal", cls.use_route_goal), cls.use_route_goal),
            route_goal_radius_m=_clamp(raw.get("route_goal_radius_m", cls.route_goal_radius_m), 0.0, 5000.0),
            route_attraction_enabled=_coerce_bool(
                raw.get("route_attraction_enabled", cls.route_attraction_enabled),
                cls.route_attraction_enabled,
            ),
            route_attraction_corridor_radius_cells=_clamp(
                raw.get("route_attraction_corridor_radius_cells", cls.route_attraction_corridor_radius_cells),
                1,
                3,
                integer=True,
            ),
            route_attraction_cost_per_cell=_clamp(
                raw.get("route_attraction_cost_per_cell", cls.route_attraction_cost_per_cell),
                0.0,
                10.0,
            ),
            route_attraction_max_cost=_clamp(
                raw.get("route_attraction_max_cost", cls.route_attraction_max_cost),
                0.0,
                20.0,
            ),
            grid_size_m=_clamp(raw.get("grid_size_m", cls.grid_size_m), 1.0, 20.0),
            cardinal_cost=_clamp(raw.get("cardinal_cost", cls.cardinal_cost), 0.1, 20.0),
            diagonal_cost=_clamp(raw.get("diagonal_cost", cls.diagonal_cost), 0.1, 30.0),
            run_every_n_frames=_clamp(raw.get("run_every_n_frames", cls.run_every_n_frames), 1, 20, integer=True),
            velocity_arrow_projection_mode=_coerce_velocity_arrow_mode(
                raw.get("velocity_arrow_projection_mode", cls.velocity_arrow_projection_mode),
                cls.velocity_arrow_projection_mode,
            ),
            velocity_arrow_projection_time_s=_clamp(
                raw.get("velocity_arrow_projection_time_s", cls.velocity_arrow_projection_time_s),
                0.0,
                30.0,
            ),
            track_current_position_cost=_clamp(
                raw.get("track_current_position_cost", cls.track_current_position_cost),
                0.0,
                20.0,
            ),
            track_future_collision_cost=_clamp(
                raw.get("track_future_collision_cost", cls.track_future_collision_cost),
                0.0,
                50.0,
            ),
            ego_current_position_cost=_clamp(
                raw.get("ego_current_position_cost", cls.ego_current_position_cost),
                0.0,
                20.0,
            ),
            ego_future_collision_cost=_clamp(
                raw.get("ego_future_collision_cost", cls.ego_future_collision_cost),
                0.0,
                50.0,
            ),
            track_cost_expansion_radius_cells=_clamp(
                raw.get("track_cost_expansion_radius_cells", cls.track_cost_expansion_radius_cells),
                0,
                10,
                integer=True,
            ),
            track_cost_expansion_decay=_clamp(
                raw.get("track_cost_expansion_decay", cls.track_cost_expansion_decay),
                0.0,
                1.0,
            ),
            ego_obstacle_radius_m=_clamp(
                raw.get("ego_obstacle_radius_m", cls.ego_obstacle_radius_m),
                100.0,
                300.0,
            ),
            ego_obstacle_angle_min_deg=ego_obstacle_angle_min_deg,
            ego_obstacle_angle_max_deg=ego_obstacle_angle_max_deg,
            colreg_enabled=_coerce_bool(raw.get("colreg_enabled", cls.colreg_enabled), cls.colreg_enabled),
            colreg_danger_radius_m=_clamp(raw.get("colreg_danger_radius_m", cls.colreg_danger_radius_m), 100.0, 300.0),
            colreg_danger_angle_min_deg=colreg_danger_angle_min_deg,
            colreg_danger_angle_max_deg=colreg_danger_angle_max_deg,
            port_side_cost_decay=_clamp(raw.get("port_side_cost_decay", cls.port_side_cost_decay), 0.0, 1.0),
            colreg_collision_goal_forward_m=_clamp(
                raw.get("colreg_collision_goal_forward_m", cls.colreg_collision_goal_forward_m),
                1.0,
                100.0,
            ),
            mode=_coerce_mode(raw.get("mode", cls.mode), cls.mode),
            show_debug_grid=_coerce_bool(raw.get("show_debug_grid", cls.show_debug_grid), cls.show_debug_grid),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "path_distance_m": float(self.path_distance_m),
            "use_route_goal": bool(self.use_route_goal),
            "route_goal_radius_m": float(self.route_goal_radius_m),
            "route_attraction_enabled": bool(self.route_attraction_enabled),
            "route_attraction_corridor_radius_cells": int(self.route_attraction_corridor_radius_cells),
            "route_attraction_cost_per_cell": float(self.route_attraction_cost_per_cell),
            "route_attraction_max_cost": float(self.route_attraction_max_cost),
            "grid_size_m": float(self.grid_size_m),
            "cardinal_cost": float(self.cardinal_cost),
            "diagonal_cost": float(self.diagonal_cost),
            "run_every_n_frames": int(self.run_every_n_frames),
            "velocity_arrow_projection_mode": str(self.velocity_arrow_projection_mode),
            "velocity_arrow_projection_time_s": float(self.velocity_arrow_projection_time_s),
            "track_current_position_cost": float(self.track_current_position_cost),
            "track_future_collision_cost": float(self.track_future_collision_cost),
            "ego_current_position_cost": float(self.ego_current_position_cost),
            "ego_future_collision_cost": float(self.ego_future_collision_cost),
            "track_cost_expansion_radius_cells": int(self.track_cost_expansion_radius_cells),
            "track_cost_expansion_decay": float(self.track_cost_expansion_decay),
            "ego_obstacle_radius_m": float(self.ego_obstacle_radius_m),
            "ego_obstacle_angle_min_deg": float(self.ego_obstacle_angle_min_deg),
            "ego_obstacle_angle_max_deg": float(self.ego_obstacle_angle_max_deg),
            "colreg_enabled": bool(self.colreg_enabled),
            "colreg_danger_radius_m": float(self.colreg_danger_radius_m),
            "colreg_danger_angle_min_deg": float(self.colreg_danger_angle_min_deg),
            "colreg_danger_angle_max_deg": float(self.colreg_danger_angle_max_deg),
            "port_side_cost_decay": float(self.port_side_cost_decay),
            "colreg_collision_goal_forward_m": float(self.colreg_collision_goal_forward_m),
            "mode": str(self.mode),
            "show_debug_grid": bool(self.show_debug_grid),
        }


@dataclass
class TrackedObstacle:
    position_enu: ENU
    vel_east_mps: float = 0.0
    vel_north_mps: float = 0.0
    speed_mps: float = 0.0
    track_id: Optional[int] = None
    source_record: Optional[Dict[str, Any]] = None
    tcpa_s: Optional[float] = None


@dataclass(frozen=True)
class GridFrame:
    space: str
    origin_enu: ENU
    heading_deg: float
    grid_size_m: float
    starboard_unit_enu: Tuple[float, float]
    forward_unit_enu: Tuple[float, float]

    def enu_to_grid(self, enu: ENU) -> GridCoord:
        inv = 1.0 / max(float(self.grid_size_m), 1e-6)
        delta_east = float(enu.east_m) - float(self.origin_enu.east_m)
        delta_north = float(enu.north_m) - float(self.origin_enu.north_m)
        grid_x_m = delta_east * self.starboard_unit_enu[0] + delta_north * self.starboard_unit_enu[1]
        grid_y_m = delta_east * self.forward_unit_enu[0] + delta_north * self.forward_unit_enu[1]
        return int(round(grid_x_m * inv)), int(round(grid_y_m * inv))

    def grid_to_enu(self, cell: GridCoord) -> ENU:
        grid_x_m = float(cell[0]) * float(self.grid_size_m)
        grid_y_m = float(cell[1]) * float(self.grid_size_m)
        return self.offset_to_enu(grid_x_m=grid_x_m, grid_y_m=grid_y_m)

    def offset_to_enu(self, *, grid_x_m: float, grid_y_m: float) -> ENU:
        east_m = (
            float(self.origin_enu.east_m)
            + float(grid_x_m) * self.starboard_unit_enu[0]
            + float(grid_y_m) * self.forward_unit_enu[0]
        )
        north_m = (
            float(self.origin_enu.north_m)
            + float(grid_x_m) * self.starboard_unit_enu[1]
            + float(grid_y_m) * self.forward_unit_enu[1]
        )
        return ENU(east_m=east_m, north_m=north_m)

    def to_payload(self) -> Dict[str, Any]:
        return {
            "space": str(self.space),
            "heading_deg": float(self.heading_deg),
            "origin_enu": {
                "east_m": float(self.origin_enu.east_m),
                "north_m": float(self.origin_enu.north_m),
            },
            "starboard_unit_enu": {
                "east_m": float(self.starboard_unit_enu[0]),
                "north_m": float(self.starboard_unit_enu[1]),
            },
            "forward_unit_enu": {
                "east_m": float(self.forward_unit_enu[0]),
                "north_m": float(self.forward_unit_enu[1]),
            },
        }


@dataclass(frozen=True)
class PathPlanningResult:
    frame_number: int
    params: PathPlanningParams
    start_enu: ENU
    display_goal_enu: ENU
    active_goal_enu: ENU
    goal_enu: ENU
    path_enu_points: List[Dict[str, float]]
    grid_frame: Dict[str, Any]
    grid_bounds: Dict[str, int]
    start_cell: List[int]
    display_goal_cell: List[int]
    active_goal_cell: List[int]
    goal_cell: List[int]
    obstacle_cells: List[List[int]]
    cost_cells: List[List[float]]
    max_cell_cost: float
    future_region_cells: List[List[float]]
    max_future_region_cost: float
    route_cells: List[List[int]]
    route_cost_cells: List[List[float]]
    max_route_attraction_cost: float
    path_cells: List[List[int]]
    obstacle_count: int
    used_cached_result: bool = False
    path_found: bool = False

    def to_payload(self, *, enu_ref_lat: Optional[float], enu_ref_lon: Optional[float]) -> Dict[str, Any]:
        payload = {
            "frame_number": int(self.frame_number),
            "params": self.params.to_dict(),
            "enu_ref_lat": float(enu_ref_lat) if enu_ref_lat is not None else None,
            "enu_ref_lon": float(enu_ref_lon) if enu_ref_lon is not None else None,
            "start_enu": {
                "east_m": float(self.start_enu.east_m),
                "north_m": float(self.start_enu.north_m),
            },
            "display_goal_enu": {
                "east_m": float(self.display_goal_enu.east_m),
                "north_m": float(self.display_goal_enu.north_m),
            },
            "active_goal_enu": {
                "east_m": float(self.active_goal_enu.east_m),
                "north_m": float(self.active_goal_enu.north_m),
            },
            "goal_enu": {
                "east_m": float(self.goal_enu.east_m),
                "north_m": float(self.goal_enu.north_m),
            },
            "path_enu_points": list(self.path_enu_points),
            "obstacle_count": int(self.obstacle_count),
            "path_found": bool(self.path_found),
            "used_cached_result": bool(self.used_cached_result),
        }
        if self.params.show_debug_grid:
            payload.update(
                {
                    "grid_frame": dict(self.grid_frame),
                    "grid_bounds": dict(self.grid_bounds),
                    "start_cell": list(self.start_cell),
                    "display_goal_cell": list(self.display_goal_cell),
                    "active_goal_cell": list(self.active_goal_cell),
                    "goal_cell": list(self.goal_cell),
                    "obstacle_cells": [list(cell) for cell in self.obstacle_cells],
                    "cost_cells": [list(cell) for cell in self.cost_cells],
                    "max_cell_cost": float(self.max_cell_cost),
                    "future_region_cells": [list(cell) for cell in self.future_region_cells],
                    "max_future_region_cost": float(self.max_future_region_cost),
                    "route_cells": [list(cell) for cell in self.route_cells],
                    "route_cost_cells": [list(cell) for cell in self.route_cost_cells],
                    "max_route_attraction_cost": float(self.max_route_attraction_cost),
                    "path_cells": [list(cell) for cell in self.path_cells],
                }
            )
        return payload


def _normalize_enu_basis(east_m: float, north_m: float) -> Tuple[float, float]:
    mag = math.hypot(float(east_m), float(north_m))
    if mag <= 1e-6:
        return (1.0, 0.0)
    return (float(east_m) / mag, float(north_m) / mag)


def _parse_route_latlon_points(values: Optional[Iterable[Any]]) -> List[LatLon]:
    route_points: List[LatLon] = []
    for value in values or []:
        lat_value: Any = None
        lon_value: Any = None
        if isinstance(value, dict):
            lat_value = value.get("lat")
            lon_value = value.get("lon")
        elif isinstance(value, (list, tuple)) and len(value) >= 2:
            lon_value = value[0]
            lat_value = value[1]
        try:
            lat = float(lat_value)
            lon = float(lon_value)
        except Exception:
            continue
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        route_points.append(LatLon(lat=lat, lon=lon))
    return route_points


def _route_latlon_points_to_enu(route_points: Sequence[LatLon], ref_latlon: Optional[LatLon]) -> List[ENU]:
    if ref_latlon is None:
        return []
    return [latlon_to_enu_m(ref_latlon, point) for point in route_points]


def _find_closest_route_point_within_radius(
    *,
    route_points_enu: Sequence[ENU],
    goal_enu: ENU,
    radius_m: float,
) -> Optional[ENU]:
    radius_sq = max(float(radius_m), 0.0) ** 2
    best_point: Optional[ENU] = None
    best_distance_sq = float("inf")
    for point in route_points_enu:
        delta_east = float(point.east_m) - float(goal_enu.east_m)
        delta_north = float(point.north_m) - float(goal_enu.north_m)
        distance_sq = delta_east * delta_east + delta_north * delta_north
        if distance_sq > radius_sq:
            continue
        if distance_sq >= best_distance_sq:
            continue
        best_distance_sq = distance_sq
        best_point = point
    return best_point


def _resolve_route_goal_enu(
    *,
    default_goal_enu: ENU,
    route_points_enu: Sequence[ENU],
    params: PathPlanningParams,
) -> ENU:
    if not params.use_route_goal:
        return default_goal_enu
    if not route_points_enu:
        return default_goal_enu
    route_goal = _find_closest_route_point_within_radius(
        route_points_enu=route_points_enu,
        goal_enu=default_goal_enu,
        radius_m=float(params.route_goal_radius_m),
    )
    return route_goal if route_goal is not None else default_goal_enu


def _build_grid_frame(*, params: PathPlanningParams, ego_enu: ENU, ego_heading_deg: float) -> GridFrame:
    mode = str(params.mode)
    if mode in {"astar_ego_relative", "theta_ego_relative"}:
        starboard = right_fwd_to_enu_m(right_m=1.0, forward_m=0.0, heading_deg=float(ego_heading_deg))
        forward = right_fwd_to_enu_m(right_m=0.0, forward_m=1.0, heading_deg=float(ego_heading_deg))
        return GridFrame(
            space="ego_relative",
            origin_enu=ENU(east_m=float(ego_enu.east_m), north_m=float(ego_enu.north_m)),
            heading_deg=float(ego_heading_deg),
            grid_size_m=float(params.grid_size_m),
            starboard_unit_enu=_normalize_enu_basis(starboard.east_m, starboard.north_m),
            forward_unit_enu=_normalize_enu_basis(forward.east_m, forward.north_m),
        )
    return GridFrame(
        space="enu",
        origin_enu=ENU(east_m=0.0, north_m=0.0),
        heading_deg=0.0,
        grid_size_m=float(params.grid_size_m),
        starboard_unit_enu=(1.0, 0.0),
        forward_unit_enu=(0.0, 1.0),
    )


def _heuristic(a: GridCoord, b: GridCoord, cardinal_cost: float, diagonal_cost: float) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    diag = min(dx, dy)
    straight = max(dx, dy) - diag
    return diag * diagonal_cost + straight * cardinal_cost


def _neighbors(cell: GridCoord, cardinal_cost: float, diagonal_cost: float) -> Sequence[Tuple[GridCoord, float]]:
    x, y = cell
    return (
        ((x + 1, y), cardinal_cost),
        ((x - 1, y), cardinal_cost),
        ((x, y + 1), cardinal_cost),
        ((x, y - 1), cardinal_cost),
        ((x + 1, y + 1), diagonal_cost),
        ((x + 1, y - 1), diagonal_cost),
        ((x - 1, y + 1), diagonal_cost),
        ((x - 1, y - 1), diagonal_cost),
    )


def _reconstruct_path(came_from: Dict[GridCoord, GridCoord], current: GridCoord) -> List[GridCoord]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _search_bounds(start: GridCoord, goal: GridCoord, padding_cells: int) -> Tuple[int, int, int, int]:
    xs = [start[0], goal[0]]
    ys = [start[1], goal[1]]
    return (
        min(xs) - padding_cells,
        max(xs) + padding_cells,
        min(ys) - padding_cells,
        max(ys) + padding_cells,
    )


def _in_bounds(cell: GridCoord, bounds: Tuple[int, int, int, int]) -> bool:
    min_x, max_x, min_y, max_y = bounds
    return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y


def _bounds_to_dict(bounds: Tuple[int, int, int, int]) -> Dict[str, int]:
    min_x, max_x, min_y, max_y = bounds
    return {
        "min_x": int(min_x),
        "max_x": int(max_x),
        "min_y": int(min_y),
        "max_y": int(max_y),
    }


@dataclass(frozen=True)
class AStarGridResult:
    start_cell: GridCoord
    display_goal_cell: GridCoord
    active_goal_cell: GridCoord
    goal_cell: GridCoord
    obstacle_cells: Set[GridCoord]
    ego_obstacle_cells: Set[GridCoord]
    colreg_zone_cells: Set[GridCoord]
    cell_costs: CellCostMap
    future_region_costs: CellCostMap
    route_costs: CellCostMap
    bounds: Tuple[int, int, int, int]
    route_cells: Set[GridCoord]
    path_nodes: List[GridCoord]
    path_cells: List[GridCoord]
    path_enu: List[ENU]


def _iter_cells_on_segment(start: GridCoord, end: GridCoord) -> Iterable[GridCoord]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        yield (x0, y0)
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def _expand_path_cells(path_nodes: Sequence[GridCoord]) -> List[GridCoord]:
    if not path_nodes:
        return []
    expanded: List[GridCoord] = [path_nodes[0]]
    for idx in range(1, len(path_nodes)):
        for cell in _iter_cells_on_segment(path_nodes[idx - 1], path_nodes[idx]):
            if expanded[-1] == cell:
                continue
            expanded.append(cell)
    return expanded


def _build_route_grid_cells(
    *,
    route_points_enu: Sequence[ENU],
    frame: GridFrame,
    bounds: Tuple[int, int, int, int],
) -> Set[GridCoord]:
    if not route_points_enu:
        return set()
    route_nodes = [frame.enu_to_grid(point) for point in route_points_enu]
    route_cells = _expand_path_cells(route_nodes)
    return {cell for cell in route_cells if _in_bounds(cell, bounds)}


def _iter_grid_neighbors_8(cell: GridCoord) -> Sequence[GridCoord]:
    x, y = cell
    return (
        (x - 1, y - 1),
        (x, y - 1),
        (x + 1, y - 1),
        (x - 1, y),
        (x + 1, y),
        (x - 1, y + 1),
        (x, y + 1),
        (x + 1, y + 1),
    )


def _build_route_corridor_cells(
    *,
    route_cells: Set[GridCoord],
    bounds: Tuple[int, int, int, int],
    corridor_radius_cells: int,
) -> Set[GridCoord]:
    if not route_cells:
        return set()
    radius = max(0, int(corridor_radius_cells))
    corridor_cells: Set[GridCoord] = set(route_cells)
    frontier: deque[Tuple[GridCoord, int]] = deque((cell, 0) for cell in route_cells)
    while frontier:
        cell, distance = frontier.popleft()
        if distance >= radius:
            continue
        for neighbor in _iter_grid_neighbors_8(cell):
            if not _in_bounds(neighbor, bounds) or neighbor in corridor_cells:
                continue
            corridor_cells.add(neighbor)
            frontier.append((neighbor, distance + 1))
    return corridor_cells


def _build_route_attraction_costs(
    *,
    route_cells: Set[GridCoord],
    bounds: Tuple[int, int, int, int],
    params: PathPlanningParams,
) -> CellCostMap:
    if not route_cells or not params.route_attraction_enabled:
        return {}
    per_layer_cost = float(params.route_attraction_cost_per_cell)
    max_cost = float(params.route_attraction_max_cost)
    if per_layer_cost <= 0.0 or max_cost <= 0.0:
        return {}
    corridor_cells = _build_route_corridor_cells(
        route_cells=route_cells,
        bounds=bounds,
        corridor_radius_cells=int(params.route_attraction_corridor_radius_cells),
    )
    min_x, max_x, min_y, max_y = bounds
    route_costs: CellCostMap = {}
    visited: Set[GridCoord] = set(corridor_cells)
    saturation_layer = max(1, int(math.ceil(max_cost / per_layer_cost)))
    frontier: deque[Tuple[GridCoord, int]] = deque((cell, 0) for cell in corridor_cells)
    while frontier:
        cell, distance = frontier.popleft()
        if distance >= saturation_layer:
            continue
        for neighbor in _iter_grid_neighbors_8(cell):
            if not _in_bounds(neighbor, bounds) or neighbor in visited:
                continue
            next_distance = distance + 1
            visited.add(neighbor)
            penalty = min(max_cost, next_distance * per_layer_cost)
            if penalty > 0.0:
                route_costs[neighbor] = penalty
            if penalty < max_cost:
                frontier.append((neighbor, next_distance))
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            cell = (x, y)
            if cell in visited:
                continue
            route_costs[cell] = max_cost
    return route_costs


@dataclass(frozen=True)
class SectorMaskCacheKey:
    grid_size_m: float
    radius_m: float
    angle_min_deg: float
    angle_max_deg: float


@dataclass(frozen=True)
class SectorMaskCache:
    key: SectorMaskCacheKey
    local_cells: Tuple[GridCoord, ...]


@dataclass
class ColregEvaluation:
    starboard_track_entered: bool = False
    starboard_presence_cells: Set[GridCoord] = field(default_factory=set)
    collision_cells: Set[GridCoord] = field(default_factory=set)


def _angle_in_range(angle_deg: float, angle_min_deg: float, angle_max_deg: float) -> bool:
    if angle_min_deg <= -180.0 and angle_max_deg >= 180.0:
        return True
    epsilon = 1e-6
    return float(angle_min_deg) - epsilon <= float(angle_deg) <= float(angle_max_deg) + epsilon


def _build_local_sector_cells(
    *,
    grid_size_m: float,
    radius_m: float,
    angle_min_deg: float,
    angle_max_deg: float,
) -> Tuple[GridCoord, ...]:
    safe_grid_size = max(float(grid_size_m), 1e-6)
    safe_radius = max(0.0, float(radius_m))
    radius_cells = int(math.ceil(safe_radius / safe_grid_size))
    local_cells: List[GridCoord] = []
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            right_m = float(dx) * safe_grid_size
            forward_m = float(dy) * safe_grid_size
            distance_m = math.hypot(right_m, forward_m)
            if distance_m > safe_radius + 1e-6:
                continue
            angle_deg = 0.0 if distance_m <= 1e-6 else math.degrees(math.atan2(right_m, forward_m))
            if not _angle_in_range(angle_deg, angle_min_deg, angle_max_deg):
                continue
            local_cells.append((dx, dy))
    return tuple(sorted(set(local_cells)))


def _build_local_ego_obstacle_cells(
    *,
    grid_size_m: float,
    radius_m: float,
    angle_min_deg: float,
    angle_max_deg: float,
) -> Tuple[GridCoord, ...]:
    return _build_local_sector_cells(
        grid_size_m=grid_size_m,
        radius_m=radius_m,
        angle_min_deg=angle_min_deg,
        angle_max_deg=angle_max_deg,
    )


def _project_local_sector_cells(
    *,
    local_cells: Sequence[GridCoord],
    frame: GridFrame,
    ego_enu: ENU,
    bounds: Tuple[int, int, int, int],
) -> Set[GridCoord]:
    if not local_cells:
        return set()
    ego_cell = frame.enu_to_grid(ego_enu)
    if frame.space == "ego_relative":
        projected_cells: Set[GridCoord] = set()
        for dx, dy in local_cells:
            cell = (ego_cell[0] + dx, ego_cell[1] + dy)
            if _in_bounds(cell, bounds):
                projected_cells.add(cell)
        return projected_cells
    projected_cells: Set[GridCoord] = set()
    for dx, dy in local_cells:
        offset_enu = right_fwd_to_enu_m(
            right_m=float(dx) * float(frame.grid_size_m),
            forward_m=float(dy) * float(frame.grid_size_m),
            heading_deg=float(frame.heading_deg),
        )
        projected_cell = frame.enu_to_grid(
            ENU(
                east_m=float(ego_enu.east_m) + float(offset_enu.east_m),
                north_m=float(ego_enu.north_m) + float(offset_enu.north_m),
            )
        )
        if _in_bounds(projected_cell, bounds):
            projected_cells.add(projected_cell)
    return projected_cells


def _project_local_ego_obstacle_cells(
    *,
    local_cells: Sequence[GridCoord],
    frame: GridFrame,
    ego_enu: ENU,
    bounds: Tuple[int, int, int, int],
) -> Set[GridCoord]:
    return _project_local_sector_cells(local_cells=local_cells, frame=frame, ego_enu=ego_enu, bounds=bounds)


def _track_region_overlaps_ego_obstacle(track_region: CellCostMap, ego_obstacle_cells: Set[GridCoord]) -> bool:
    if not track_region or not ego_obstacle_cells:
        return False
    if len(track_region) <= len(ego_obstacle_cells):
        for cell in track_region.keys():
            if cell in ego_obstacle_cells:
                return True
        return False
    for cell in ego_obstacle_cells:
        if cell in track_region:
            return True
    return False


def _cell_costs_in_mask(cell_costs: CellCostMap, mask_cells: Set[GridCoord]) -> Set[GridCoord]:
    if not cell_costs or not mask_cells:
        return set()
    if len(cell_costs) <= len(mask_cells):
        return {cell for cell in cell_costs.keys() if cell in mask_cells}
    return {cell for cell in mask_cells if cell in cell_costs}


def _relative_bearing_deg(*, ego_enu: ENU, target_enu: ENU, heading_deg: float) -> float:
    starboard = right_fwd_to_enu_m(right_m=1.0, forward_m=0.0, heading_deg=float(heading_deg))
    forward = right_fwd_to_enu_m(right_m=0.0, forward_m=1.0, heading_deg=float(heading_deg))
    starboard_unit = _normalize_enu_basis(starboard.east_m, starboard.north_m)
    forward_unit = _normalize_enu_basis(forward.east_m, forward.north_m)
    delta_east = float(target_enu.east_m) - float(ego_enu.east_m)
    delta_north = float(target_enu.north_m) - float(ego_enu.north_m)
    right_m = delta_east * starboard_unit[0] + delta_north * starboard_unit[1]
    forward_m = delta_east * forward_unit[0] + delta_north * forward_unit[1]
    if abs(right_m) <= 1e-6 and abs(forward_m) <= 1e-6:
        return 0.0
    return math.degrees(math.atan2(right_m, forward_m))


def _classify_colreg_side(*, ego_enu: ENU, target_enu: ENU, heading_deg: float) -> str:
    bearing_deg = _relative_bearing_deg(ego_enu=ego_enu, target_enu=target_enu, heading_deg=heading_deg)
    if bearing_deg > 1e-6:
        return "starboard"
    if bearing_deg < -1e-6:
        return "port"
    return "ahead"


def _scale_cell_costs(cell_costs: CellCostMap, factor: float) -> CellCostMap:
    safe_factor = max(0.0, float(factor))
    if safe_factor >= 1.0:
        return dict(cell_costs)
    if safe_factor <= 0.0:
        return {}
    scaled: CellCostMap = {}
    for cell, cost in cell_costs.items():
        scaled_cost = float(cost) * safe_factor
        if scaled_cost > 0.0 and math.isfinite(scaled_cost):
            scaled[cell] = scaled_cost
    return scaled


def _extract_obstacles(tracked_world: Iterable[Dict[str, Any]]) -> List[TrackedObstacle]:
    obstacles: List[TrackedObstacle] = []
    for det in tracked_world:
        if not isinstance(det, dict):
            continue
        try:
            east_m = float(det.get("world_east_m"))
            north_m = float(det.get("world_north_m"))
        except Exception:
            continue
        if not (math.isfinite(east_m) and math.isfinite(north_m)):
            continue
        vel_east_mps = det.get("vel_east_mps", 0.0)
        vel_north_mps = det.get("vel_north_mps", 0.0)
        speed_mps = det.get("speed_mps")
        try:
            vel_east_mps = float(vel_east_mps)
        except Exception:
            vel_east_mps = 0.0
        try:
            vel_north_mps = float(vel_north_mps)
        except Exception:
            vel_north_mps = 0.0
        if speed_mps is None:
            speed_mps = math.hypot(vel_east_mps, vel_north_mps)
        else:
            try:
                speed_mps = float(speed_mps)
            except Exception:
                speed_mps = math.hypot(vel_east_mps, vel_north_mps)
        if not math.isfinite(vel_east_mps):
            vel_east_mps = 0.0
        if not math.isfinite(vel_north_mps):
            vel_north_mps = 0.0
        if not math.isfinite(speed_mps):
            speed_mps = math.hypot(vel_east_mps, vel_north_mps)
        obstacles.append(
            TrackedObstacle(
                position_enu=ENU(east_m=east_m, north_m=north_m),
                vel_east_mps=vel_east_mps,
                vel_north_mps=vel_north_mps,
                speed_mps=float(speed_mps),
                track_id=int(det["track_id"]) if det.get("track_id") is not None else None,
                source_record=det,
            )
        )
    return obstacles


def _set_track_tcpa_metadata(record: Optional[Dict[str, Any]], tcpa_s: Optional[float]) -> None:
    if not isinstance(record, dict):
        return
    if tcpa_s is None or not math.isfinite(tcpa_s):
        record["tcpa_s"] = None
        record["tcpa_is_infinite"] = True
        return
    record["tcpa_s"] = max(0.0, float(tcpa_s))
    record["tcpa_is_infinite"] = False


def _compute_tcpa_s(
    obstacle: TrackedObstacle,
    *,
    ego_enu: ENU,
    ego_vel_east_mps: float,
    ego_vel_north_mps: float,
) -> Optional[float]:
    rel_pos_east = float(ego_enu.east_m) - float(obstacle.position_enu.east_m)
    rel_pos_north = float(ego_enu.north_m) - float(obstacle.position_enu.north_m)
    distance_m = math.hypot(rel_pos_east, rel_pos_north)
    if distance_m <= 1e-6:
        return 0.0

    unit_rel_east = rel_pos_east / distance_m
    unit_rel_north = rel_pos_north / distance_m
    rel_vel_east = float(obstacle.vel_east_mps) - float(ego_vel_east_mps)
    rel_vel_north = float(obstacle.vel_north_mps) - float(ego_vel_north_mps)
    closing_speed_mps = rel_vel_east * unit_rel_east + rel_vel_north * unit_rel_north
    if not math.isfinite(closing_speed_mps) or closing_speed_mps <= 1e-6:
        return None

    tcpa_s = distance_m / closing_speed_mps
    if not math.isfinite(tcpa_s) or tcpa_s < 0.0:
        return None
    return tcpa_s


def _cell_cost(cell_costs: CellCostMap, cell: GridCoord) -> float:
    value = cell_costs.get(cell, 0.0)
    if not math.isfinite(value):
        return 0.0
    return max(0.0, float(value))


def _add_cell_cost(
    cell_costs: CellCostMap,
    cell: GridCoord,
    amount: float,
    *,
    bounds: Optional[Tuple[int, int, int, int]] = None,
) -> None:
    if not math.isfinite(amount) or amount <= 0.0:
        return
    if bounds is not None and not _in_bounds(cell, bounds):
        return
    cell_costs[cell] = _cell_cost(cell_costs, cell) + float(amount)


def _apply_expanded_cost(
    cell_costs: CellCostMap,
    center: GridCoord,
    base_cost: float,
    *,
    radius_cells: int,
    decay: float,
    bounds: Optional[Tuple[int, int, int, int]] = None,
) -> None:
    radius = max(0, int(radius_cells))
    decay = max(0.0, min(1.0, float(decay)))
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            ring = max(abs(dx), abs(dy))
            ring_cost = float(base_cost) * (decay**ring)
            _add_cell_cost(cell_costs, (center[0] + dx, center[1] + dy), ring_cost, bounds=bounds)


def _apply_segment_cost_ramp(
    cell_costs: CellCostMap,
    *,
    start: GridCoord,
    end: GridCoord,
    start_cost: float,
    end_cost: float,
    expansion_radius_cells: int,
    expansion_decay: float,
    bounds: Optional[Tuple[int, int, int, int]] = None,
) -> None:
    segment_cells = list(_iter_cells_on_segment(start, end))
    if not segment_cells:
        return
    if len(segment_cells) == 1:
        _apply_expanded_cost(
            cell_costs,
            segment_cells[0],
            max(float(start_cost), float(end_cost)),
            radius_cells=expansion_radius_cells,
            decay=expansion_decay,
            bounds=bounds,
        )
        return

    denom = max(1, len(segment_cells) - 1)
    for idx, cell in enumerate(segment_cells):
        interp = float(idx) / float(denom)
        base_cost = float(start_cost) + (float(end_cost) - float(start_cost)) * interp
        _apply_expanded_cost(
            cell_costs,
            cell,
            base_cost,
            radius_cells=expansion_radius_cells,
            decay=expansion_decay,
            bounds=bounds,
        )


def _has_valid_tcpa(tcpa_s: Optional[float]) -> bool:
    return tcpa_s is not None and math.isfinite(tcpa_s) and tcpa_s > 0.0


def _project_enu(enu: ENU, *, vel_east_mps: float, vel_north_mps: float, duration_s: float) -> ENU:
    return ENU(
        east_m=float(enu.east_m) + float(vel_east_mps) * float(duration_s),
        north_m=float(enu.north_m) + float(vel_north_mps) * float(duration_s),
    )


def _build_static_region_costs(
    *,
    frame: GridFrame,
    center_enu: ENU,
    base_cost: float,
    expansion_radius_cells: int,
    expansion_decay: float,
    bounds: Tuple[int, int, int, int],
) -> CellCostMap:
    region_costs: CellCostMap = {}
    _apply_expanded_cost(
        region_costs,
        frame.enu_to_grid(center_enu),
        float(base_cost),
        radius_cells=expansion_radius_cells,
        decay=expansion_decay,
        bounds=bounds,
    )
    return region_costs


def _build_motion_region_costs(
    *,
    frame: GridFrame,
    start_enu: ENU,
    end_enu: ENU,
    start_cost: float,
    end_cost: float,
    expansion_radius_cells: int,
    expansion_decay: float,
    bounds: Tuple[int, int, int, int],
) -> CellCostMap:
    region_costs: CellCostMap = {}
    _apply_segment_cost_ramp(
        region_costs,
        start=frame.enu_to_grid(start_enu),
        end=frame.enu_to_grid(end_enu),
        start_cost=float(start_cost),
        end_cost=float(end_cost),
        expansion_radius_cells=expansion_radius_cells,
        expansion_decay=expansion_decay,
        bounds=bounds,
    )
    return region_costs


def _multiply_cell_costs(a: CellCostMap, b: CellCostMap) -> CellCostMap:
    if not a or not b:
        return {}
    shared_cells = a.keys() & b.keys()
    multiplied: CellCostMap = {}
    for cell in shared_cells:
        value = _cell_cost(a, cell) * _cell_cost(b, cell)
        if value > 0.0 and math.isfinite(value):
            multiplied[cell] = float(value)
    return multiplied


def _accumulate_cell_costs(
    target: CellCostMap,
    source: CellCostMap,
    *,
    bounds: Optional[Tuple[int, int, int, int]] = None,
) -> None:
    for cell, amount in source.items():
        _add_cell_cost(target, cell, amount, bounds=bounds)


def _build_forward_goal_cell(
    *,
    frame: GridFrame,
    ego_enu: ENU,
    ego_heading_deg: float,
    forward_cells: int,
) -> GridCoord:
    forward_distance_m = max(0, int(forward_cells)) * float(frame.grid_size_m)
    offset = right_fwd_to_enu_m(right_m=0.0, forward_m=forward_distance_m, heading_deg=float(ego_heading_deg))
    return frame.enu_to_grid(
        ENU(
            east_m=float(ego_enu.east_m) + float(offset.east_m),
            north_m=float(ego_enu.north_m) + float(offset.north_m),
        )
    )


def _shortest_grid_distance_to_target(
    *,
    seed_cells: Set[GridCoord],
    target_cell: GridCoord,
    bounds: Tuple[int, int, int, int],
) -> Optional[int]:
    if target_cell in seed_cells:
        return 0
    if not seed_cells:
        return None
    visited: Set[GridCoord] = set(seed_cells)
    frontier: deque[Tuple[GridCoord, int]] = deque((cell, 0) for cell in seed_cells if _in_bounds(cell, bounds))
    while frontier:
        cell, distance = frontier.popleft()
        for neighbor in _iter_grid_neighbors_8(cell):
            if not _in_bounds(neighbor, bounds) or neighbor in visited:
                continue
            next_distance = distance + 1
            if neighbor == target_cell:
                return next_distance
            visited.add(neighbor)
            frontier.append((neighbor, next_distance))
    return None


def _resolve_colreg_active_goal_cell(
    *,
    frame: GridFrame,
    ego_enu: ENU,
    ego_heading_deg: float,
    start_cell: GridCoord,
    display_goal_cell: GridCoord,
    bounds: Tuple[int, int, int, int],
    blocked_cells: Set[GridCoord],
    collision_cells: Set[GridCoord],
    starboard_presence_cells: Set[GridCoord],
    collision_goal_forward_m: float,
) -> GridCoord:
    if not starboard_presence_cells:
        return display_goal_cell
    if collision_cells:
        desired_steps = max(1, int(math.ceil(max(float(collision_goal_forward_m), 0.0) / max(float(frame.grid_size_m), 1e-6))))
    else:
        distance = _shortest_grid_distance_to_target(
            seed_cells=starboard_presence_cells,
            target_cell=start_cell,
            bounds=bounds,
        )
        if distance is None:
            return display_goal_cell
        desired_steps = max(1, int(math.floor(float(distance) / 2.0)))
    for forward_cells in range(desired_steps, 0, -1):
        candidate = _build_forward_goal_cell(
            frame=frame,
            ego_enu=ego_enu,
            ego_heading_deg=ego_heading_deg,
            forward_cells=forward_cells,
        )
        if not _in_bounds(candidate, bounds):
            continue
        if candidate in blocked_cells or candidate in collision_cells:
            continue
        return candidate
    return display_goal_cell


def _build_track_regions_and_overlap(
    obstacle: TrackedObstacle,
    *,
    ego_enu: ENU,
    ego_vel_east_mps: float,
    ego_vel_north_mps: float,
    frame: GridFrame,
    params: PathPlanningParams,
    bounds: Tuple[int, int, int, int],
) -> Tuple[CellCostMap, CellCostMap, CellCostMap]:
    expansion_radius = int(params.track_cost_expansion_radius_cells)
    expansion_decay = float(params.track_cost_expansion_decay)
    if _has_valid_tcpa(obstacle.tcpa_s):
        assert obstacle.tcpa_s is not None
        horizon_s = float(obstacle.tcpa_s)
        track_region = _build_motion_region_costs(
            frame=frame,
            start_enu=obstacle.position_enu,
            end_enu=_project_enu(
                obstacle.position_enu,
                vel_east_mps=float(obstacle.vel_east_mps),
                vel_north_mps=float(obstacle.vel_north_mps),
                duration_s=horizon_s,
            ),
            start_cost=float(params.track_current_position_cost),
            end_cost=float(params.track_future_collision_cost),
            expansion_radius_cells=expansion_radius,
            expansion_decay=expansion_decay,
            bounds=bounds,
        )
        ego_region = _build_motion_region_costs(
            frame=frame,
            start_enu=ego_enu,
            end_enu=_project_enu(
                ego_enu,
                vel_east_mps=float(ego_vel_east_mps),
                vel_north_mps=float(ego_vel_north_mps),
                duration_s=horizon_s,
            ),
            start_cost=float(params.ego_current_position_cost),
            end_cost=float(params.ego_future_collision_cost),
            expansion_radius_cells=expansion_radius,
            expansion_decay=expansion_decay,
            bounds=bounds,
        )
        return track_region, ego_region, _multiply_cell_costs(track_region, ego_region)

    track_region = _build_static_region_costs(
        frame=frame,
        center_enu=obstacle.position_enu,
        base_cost=float(params.track_current_position_cost),
        expansion_radius_cells=expansion_radius,
        expansion_decay=expansion_decay,
        bounds=bounds,
    )
    ego_region = _build_static_region_costs(
        frame=frame,
        center_enu=ego_enu,
        base_cost=float(params.ego_current_position_cost),
        expansion_radius_cells=expansion_radius,
        expansion_decay=expansion_decay,
        bounds=bounds,
    )
    return track_region, ego_region, _multiply_cell_costs(track_region, ego_region)


def _annotate_obstacles_tcpa(
    obstacles: Sequence[TrackedObstacle],
    *,
    ego_enu: ENU,
    ego_vel_east_mps: float,
    ego_vel_north_mps: float,
) -> None:
    for obstacle in obstacles:
        obstacle.tcpa_s = _compute_tcpa_s(
            obstacle,
            ego_enu=ego_enu,
            ego_vel_east_mps=ego_vel_east_mps,
            ego_vel_north_mps=ego_vel_north_mps,
        )
        _set_track_tcpa_metadata(obstacle.source_record, obstacle.tcpa_s)


def _build_cost_field(
    obstacles: Sequence[TrackedObstacle],
    *,
    ego_enu: ENU,
    ego_heading_deg: float,
    ego_vel_east_mps: float,
    ego_vel_north_mps: float,
    frame: GridFrame,
    params: PathPlanningParams,
    bounds: Tuple[int, int, int, int],
    ego_obstacle_cells: Set[GridCoord],
    colreg_zone_cells: Set[GridCoord],
) -> Tuple[CellCostMap, CellCostMap, bool, ColregEvaluation]:
    cell_costs: CellCostMap = {}
    future_region_costs: CellCostMap = {}
    ego_obstacle_active = False
    colreg_evaluation = ColregEvaluation()
    for obstacle in obstacles:
        track_region, ego_region, overlap_costs = _build_track_regions_and_overlap(
            obstacle,
            ego_enu=ego_enu,
            ego_vel_east_mps=ego_vel_east_mps,
            ego_vel_north_mps=ego_vel_north_mps,
            frame=frame,
            params=params,
            bounds=bounds,
        )
        track_side = _classify_colreg_side(
            ego_enu=ego_enu,
            target_enu=obstacle.position_enu,
            heading_deg=ego_heading_deg,
        )
        track_zone_cells = _cell_costs_in_mask(track_region, colreg_zone_cells)
        entered_colreg_zone = bool(track_zone_cells)
        if entered_colreg_zone and params.colreg_enabled and track_side == "port":
            track_region = _scale_cell_costs(track_region, float(params.port_side_cost_decay))
            overlap_costs = _multiply_cell_costs(track_region, ego_region)
            track_zone_cells = _cell_costs_in_mask(track_region, colreg_zone_cells)
        if not ego_obstacle_active and _track_region_overlaps_ego_obstacle(track_region, ego_obstacle_cells):
            ego_obstacle_active = True
        if entered_colreg_zone and params.colreg_enabled and track_side == "starboard":
            colreg_evaluation.starboard_track_entered = True
            colreg_evaluation.starboard_presence_cells.update(track_zone_cells)
            colreg_evaluation.collision_cells.update(_cell_costs_in_mask(overlap_costs, colreg_zone_cells))
        _accumulate_cell_costs(future_region_costs, track_region, bounds=bounds)
        _accumulate_cell_costs(future_region_costs, ego_region, bounds=bounds)
        _accumulate_cell_costs(cell_costs, overlap_costs, bounds=bounds)
    return cell_costs, future_region_costs, ego_obstacle_active, colreg_evaluation


def _segment_step_cost(a: GridCoord, b: GridCoord, cardinal_cost: float, diagonal_cost: float) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if dx == 1 and dy == 1:
        return diagonal_cost
    return cardinal_cost


def _score_theta_shortcut(
    *,
    parent: GridCoord,
    neighbor: GridCoord,
    parent_g: float,
    baseline_cost: float,
    cell_costs: CellCostMap,
    blocked_cells: Set[GridCoord],
    cardinal_cost: float,
    diagonal_cost: float,
) -> float:
    running_cost = float(parent_g)
    prev = parent
    first = True
    for cell in _iter_cells_on_segment(parent, neighbor):
        if first:
            first = False
            continue
        if cell in blocked_cells:
            return float("inf")
        running_cost += _segment_step_cost(prev, cell, cardinal_cost, diagonal_cost)
        running_cost += _cell_cost(cell_costs, cell)
        if running_cost >= baseline_cost:
            return float("inf")
        prev = cell
    return running_cost


def astar_path(
    *,
    frame: GridFrame,
    start_enu: ENU,
    display_goal_enu: ENU,
    route_points_enu: Sequence[ENU],
    obstacles: Sequence[TrackedObstacle],
    ego_enu: ENU,
    ego_heading_deg: float,
    ego_vel_east_mps: float,
    ego_vel_north_mps: float,
    params: PathPlanningParams,
    ego_obstacle_local_cells: Sequence[GridCoord] = (),
    colreg_danger_local_cells: Sequence[GridCoord] = (),
) -> AStarGridResult:
    start = frame.enu_to_grid(start_enu)
    display_goal = frame.enu_to_grid(display_goal_enu)
    padding_cells = max(10, int(math.ceil(params.path_distance_m / max(params.grid_size_m, 1e-6) * 0.2)))
    bounds = _search_bounds(start, display_goal, padding_cells)
    projected_ego_obstacle_cells = _project_local_sector_cells(
        local_cells=ego_obstacle_local_cells,
        frame=frame,
        ego_enu=ego_enu,
        bounds=bounds,
    )
    projected_colreg_zone_cells = _project_local_sector_cells(
        local_cells=colreg_danger_local_cells,
        frame=frame,
        ego_enu=ego_enu,
        bounds=bounds,
    )

    debug_cell_costs, debug_future_region_costs, ego_obstacle_active, colreg_evaluation = _build_cost_field(
        obstacles,
        ego_enu=ego_enu,
        ego_heading_deg=ego_heading_deg,
        ego_vel_east_mps=ego_vel_east_mps,
        ego_vel_north_mps=ego_vel_north_mps,
        frame=frame,
        params=params,
        bounds=bounds,
        ego_obstacle_cells=projected_ego_obstacle_cells,
        colreg_zone_cells=projected_colreg_zone_cells,
    )
    route_cells = _build_route_grid_cells(route_points_enu=route_points_enu, frame=frame, bounds=bounds)
    debug_route_costs = _build_route_attraction_costs(route_cells=route_cells, bounds=bounds, params=params)
    active_goal = _resolve_colreg_active_goal_cell(
        frame=frame,
        ego_enu=ego_enu,
        ego_heading_deg=ego_heading_deg,
        start_cell=start,
        display_goal_cell=display_goal,
        bounds=bounds,
        blocked_cells=set(projected_ego_obstacle_cells) if ego_obstacle_active else set(),
        collision_cells=set(colreg_evaluation.collision_cells),
        starboard_presence_cells=set(colreg_evaluation.starboard_presence_cells),
        collision_goal_forward_m=float(params.colreg_collision_goal_forward_m),
    )
    search_cell_costs: CellCostMap = dict(debug_cell_costs)
    _accumulate_cell_costs(search_cell_costs, debug_route_costs, bounds=bounds)
    search_cell_costs.pop(start, None)
    search_cell_costs.pop(active_goal, None)
    ego_obstacle_cells = set(projected_ego_obstacle_cells) if ego_obstacle_active else set()
    ego_obstacle_cells.discard(start)
    ego_obstacle_cells.discard(active_goal)
    occupied_cells = {cell for cell, cost in debug_cell_costs.items() if cost > 0.0}

    open_heap: List[Tuple[float, int, GridCoord]] = []
    heapq.heappush(open_heap, (0.0, 0, start))
    came_from: Dict[GridCoord, GridCoord] = {}
    g_score: Dict[GridCoord, float] = {start: 0.0}
    visited: Set[GridCoord] = set()
    push_count = 0
    use_theta = str(params.mode).startswith("theta_")

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == active_goal:
            path_nodes = _reconstruct_path(came_from, current)
            path_cells = _expand_path_cells(path_nodes)
            return AStarGridResult(
                start_cell=start,
                display_goal_cell=display_goal,
                active_goal_cell=active_goal,
                goal_cell=active_goal,
                obstacle_cells=occupied_cells,
                ego_obstacle_cells=ego_obstacle_cells,
                colreg_zone_cells=projected_colreg_zone_cells,
                cell_costs=debug_cell_costs,
                future_region_costs=debug_future_region_costs,
                route_costs=debug_route_costs,
                bounds=bounds,
                route_cells=route_cells,
                path_nodes=path_nodes,
                path_cells=path_cells,
                path_enu=[frame.grid_to_enu(cell) for cell in path_nodes],
            )

        current_g = g_score.get(current, float("inf"))
        for neighbor, step_cost in _neighbors(current, params.cardinal_cost, params.diagonal_cost):
            if not _in_bounds(neighbor, bounds):
                continue
            if neighbor in ego_obstacle_cells:
                continue

            normal_candidate = current_g + step_cost + _cell_cost(search_cell_costs, neighbor)
            best_parent = current
            best_cost = normal_candidate
            parent_key = came_from.get(current)
            if use_theta and parent_key is not None:
                parent_g = g_score.get(parent_key, float("inf"))
                shortcut_candidate = _score_theta_shortcut(
                    parent=parent_key,
                    neighbor=neighbor,
                    parent_g=parent_g,
                    baseline_cost=normal_candidate,
                    cell_costs=search_cell_costs,
                    blocked_cells=ego_obstacle_cells,
                    cardinal_cost=float(params.cardinal_cost),
                    diagonal_cost=float(params.diagonal_cost),
                )
                if shortcut_candidate < best_cost:
                    best_parent = parent_key
                    best_cost = shortcut_candidate

            if best_cost >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = best_parent
            g_score[neighbor] = best_cost
            push_count += 1
            f_score = best_cost + _heuristic(neighbor, active_goal, params.cardinal_cost, params.diagonal_cost)
            heapq.heappush(open_heap, (f_score, push_count, neighbor))

    return AStarGridResult(
        start_cell=start,
        display_goal_cell=display_goal,
        active_goal_cell=active_goal,
        goal_cell=active_goal,
        obstacle_cells=occupied_cells,
        ego_obstacle_cells=ego_obstacle_cells,
        colreg_zone_cells=projected_colreg_zone_cells,
        cell_costs=debug_cell_costs,
        future_region_costs=debug_future_region_costs,
        route_costs=debug_route_costs,
        bounds=bounds,
        route_cells=route_cells,
        path_nodes=[],
        path_cells=[],
        path_enu=[],
    )


class PathPlannerRuntime:
    def __init__(self, initial_params: Optional[Dict[str, Any]] = None):
        self._lock = asyncio.Lock()
        self._params = PathPlanningParams.from_mapping(initial_params)
        self._last_result: Optional[PathPlanningResult] = None
        self._last_run_frame: Optional[int] = None
        self._route_points_latlon: List[LatLon] = []
        self._ego_obstacle_mask_cache: Optional[SectorMaskCache] = None
        self._colreg_danger_mask_cache: Optional[SectorMaskCache] = None

    def _get_sector_mask_cache(
        self,
        cache: Optional[SectorMaskCache],
        *,
        grid_size_m: float,
        radius_m: float,
        angle_min_deg: float,
        angle_max_deg: float,
    ) -> SectorMaskCache:
        key = SectorMaskCacheKey(
            grid_size_m=float(grid_size_m),
            radius_m=float(radius_m),
            angle_min_deg=float(angle_min_deg),
            angle_max_deg=float(angle_max_deg),
        )
        if cache is not None and cache.key == key:
            return cache
        return SectorMaskCache(
            key=key,
            local_cells=_build_local_sector_cells(
                grid_size_m=key.grid_size_m,
                radius_m=key.radius_m,
                angle_min_deg=key.angle_min_deg,
                angle_max_deg=key.angle_max_deg,
            ),
        )

    def _get_ego_obstacle_mask_cache(self, params: PathPlanningParams) -> SectorMaskCache:
        self._ego_obstacle_mask_cache = self._get_sector_mask_cache(
            self._ego_obstacle_mask_cache,
            grid_size_m=float(params.grid_size_m),
            radius_m=float(params.ego_obstacle_radius_m),
            angle_min_deg=float(params.ego_obstacle_angle_min_deg),
            angle_max_deg=float(params.ego_obstacle_angle_max_deg),
        )
        return self._ego_obstacle_mask_cache

    def _get_colreg_danger_mask_cache(self, params: PathPlanningParams) -> SectorMaskCache:
        self._colreg_danger_mask_cache = self._get_sector_mask_cache(
            self._colreg_danger_mask_cache,
            grid_size_m=float(params.grid_size_m),
            radius_m=float(params.colreg_danger_radius_m),
            angle_min_deg=float(params.colreg_danger_angle_min_deg),
            angle_max_deg=float(params.colreg_danger_angle_max_deg),
        )
        return self._colreg_danger_mask_cache

    async def get_params(self) -> Dict[str, Any]:
        async with self._lock:
            return self._params.to_dict()

    async def set_params(self, new_values: Dict[str, Any]) -> Dict[str, Any]:
        async with self._lock:
            merged = self._params.to_dict()
            for key, value in (new_values or {}).items():
                if key in merged:
                    merged[key] = value
            self._params = PathPlanningParams.from_mapping(merged)
            self._last_result = None
            self._last_run_frame = None
            return self._params.to_dict()

    async def set_route_points(self, route_points: Optional[Iterable[Any]]) -> Dict[str, Any]:
        async with self._lock:
            self._route_points_latlon = _parse_route_latlon_points(route_points)
            self._last_result = None
            self._last_run_frame = None
            return {"route_point_count": len(self._route_points_latlon)}

    async def compute(
        self,
        *,
        frame_number: int,
        ego_enu: ENU,
        ego_ref_latlon: Optional[LatLon],
        ego_heading_deg: float,
        ego_vel_east_mps: float = 0.0,
        ego_vel_north_mps: float = 0.0,
        tracked_world: Sequence[Dict[str, Any]],
    ) -> Optional[PathPlanningResult]:
        async with self._lock:
            params = self._params
            obstacles = _extract_obstacles(tracked_world)
            _annotate_obstacles_tcpa(
                obstacles,
                ego_enu=ego_enu,
                ego_vel_east_mps=ego_vel_east_mps,
                ego_vel_north_mps=ego_vel_north_mps,
            )
            if not params.enabled:
                self._last_result = None
                self._last_run_frame = None
                return None

            should_run = (
                self._last_result is None
                or self._last_run_frame is None
                or (frame_number - self._last_run_frame) >= int(params.run_every_n_frames)
            )

            if not should_run:
                if self._last_result is None:
                    return None
                return PathPlanningResult(
                    frame_number=frame_number,
                    params=self._last_result.params,
                    start_enu=self._last_result.start_enu,
                    display_goal_enu=self._last_result.display_goal_enu,
                    active_goal_enu=self._last_result.active_goal_enu,
                    goal_enu=self._last_result.goal_enu,
                    path_enu_points=list(self._last_result.path_enu_points),
                    grid_frame=dict(self._last_result.grid_frame),
                    grid_bounds=dict(self._last_result.grid_bounds),
                    start_cell=list(self._last_result.start_cell),
                    display_goal_cell=list(self._last_result.display_goal_cell),
                    active_goal_cell=list(self._last_result.active_goal_cell),
                    goal_cell=list(self._last_result.goal_cell),
                    obstacle_cells=[list(cell) for cell in self._last_result.obstacle_cells],
                    cost_cells=[list(cell) for cell in self._last_result.cost_cells],
                    max_cell_cost=float(self._last_result.max_cell_cost),
                    future_region_cells=[list(cell) for cell in self._last_result.future_region_cells],
                    max_future_region_cost=float(self._last_result.max_future_region_cost),
                    route_cells=[list(cell) for cell in self._last_result.route_cells],
                    route_cost_cells=[list(cell) for cell in self._last_result.route_cost_cells],
                    max_route_attraction_cost=float(self._last_result.max_route_attraction_cost),
                    path_cells=[list(cell) for cell in self._last_result.path_cells],
                    obstacle_count=self._last_result.obstacle_count,
                    used_cached_result=True,
                    path_found=self._last_result.path_found,
                )

            goal_offset = right_fwd_to_enu_m(
                right_m=0.0,
                forward_m=float(params.path_distance_m),
                heading_deg=float(ego_heading_deg),
            )
            display_goal_enu = ENU(
                east_m=float(ego_enu.east_m) + float(goal_offset.east_m),
                north_m=float(ego_enu.north_m) + float(goal_offset.north_m),
            )
            route_points_enu = _route_latlon_points_to_enu(self._route_points_latlon, ego_ref_latlon)
            display_goal_enu = _resolve_route_goal_enu(
                default_goal_enu=display_goal_enu,
                route_points_enu=route_points_enu,
                params=params,
            )

            grid_frame = _build_grid_frame(params=params, ego_enu=ego_enu, ego_heading_deg=float(ego_heading_deg))
            ego_obstacle_mask_cache = self._get_ego_obstacle_mask_cache(params)
            colreg_danger_mask_cache = self._get_colreg_danger_mask_cache(params)
            grid_result = astar_path(
                frame=grid_frame,
                start_enu=ego_enu,
                display_goal_enu=display_goal_enu,
                route_points_enu=route_points_enu,
                obstacles=obstacles,
                ego_enu=ego_enu,
                ego_heading_deg=float(ego_heading_deg),
                ego_vel_east_mps=ego_vel_east_mps,
                ego_vel_north_mps=ego_vel_north_mps,
                params=params,
                ego_obstacle_local_cells=ego_obstacle_mask_cache.local_cells,
                colreg_danger_local_cells=colreg_danger_mask_cache.local_cells,
            )
            path_points = [{"east_m": float(p.east_m), "north_m": float(p.north_m)} for p in grid_result.path_enu]
            sorted_cost_cells = sorted(
                [
                    [int(cell[0]), int(cell[1]), float(cost)]
                    for cell, cost in grid_result.cell_costs.items()
                    if cost > 0.0 and cell not in grid_result.ego_obstacle_cells
                ]
                + [[int(cell[0]), int(cell[1]), EGO_OBSTACLE_MARKER_COST] for cell in grid_result.ego_obstacle_cells]
                + [[int(cell[0]), int(cell[1]), COLREG_ZONE_MARKER_COST] for cell in grid_result.colreg_zone_cells],
                key=lambda item: (item[0], item[1], item[2]),
            )
            max_cell_cost = max((float(item[2]) for item in sorted_cost_cells if float(item[2]) > 0.0), default=0.0)
            sorted_future_region_cells = sorted(
                [
                    [int(cell[0]), int(cell[1]), float(cost)]
                    for cell, cost in grid_result.future_region_costs.items()
                    if cost > 0.0
                ],
                key=lambda item: (item[0], item[1]),
            )
            max_future_region_cost = max((float(item[2]) for item in sorted_future_region_cells), default=0.0)
            sorted_route_cells = sorted([[int(cell[0]), int(cell[1])] for cell in grid_result.route_cells])
            sorted_route_cost_cells = sorted(
                [
                    [int(cell[0]), int(cell[1]), float(cost)]
                    for cell, cost in grid_result.route_costs.items()
                    if cost > 0.0
                ],
                key=lambda item: (item[0], item[1]),
            )
            max_route_attraction_cost = max((float(item[2]) for item in sorted_route_cost_cells), default=0.0)
            result = PathPlanningResult(
                frame_number=frame_number,
                params=params,
                start_enu=ego_enu,
                display_goal_enu=grid_frame.grid_to_enu(grid_result.display_goal_cell),
                active_goal_enu=grid_frame.grid_to_enu(grid_result.active_goal_cell),
                goal_enu=grid_frame.grid_to_enu(grid_result.goal_cell),
                path_enu_points=path_points,
                grid_frame=grid_frame.to_payload(),
                grid_bounds=_bounds_to_dict(grid_result.bounds),
                start_cell=[int(grid_result.start_cell[0]), int(grid_result.start_cell[1])],
                display_goal_cell=[int(grid_result.display_goal_cell[0]), int(grid_result.display_goal_cell[1])],
                active_goal_cell=[int(grid_result.active_goal_cell[0]), int(grid_result.active_goal_cell[1])],
                goal_cell=[int(grid_result.goal_cell[0]), int(grid_result.goal_cell[1])],
                obstacle_cells=sorted([[int(cell[0]), int(cell[1])] for cell in grid_result.obstacle_cells]),
                cost_cells=sorted_cost_cells,
                max_cell_cost=max_cell_cost,
                future_region_cells=sorted_future_region_cells,
                max_future_region_cost=max_future_region_cost,
                route_cells=sorted_route_cells,
                route_cost_cells=sorted_route_cost_cells,
                max_route_attraction_cost=max_route_attraction_cost,
                path_cells=[[int(cell[0]), int(cell[1])] for cell in grid_result.path_cells],
                obstacle_count=len(obstacles),
                used_cached_result=False,
                path_found=len(path_points) >= 2,
            )
            self._last_result = result
            self._last_run_frame = frame_number
            return result
