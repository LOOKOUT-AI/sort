from __future__ import annotations

import asyncio
import heapq
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .image_to_world import ENU, right_fwd_to_enu_m


GridCoord = Tuple[int, int]
PATH_FINDING_MODES = (
    "astar_enu",
    "astar_ego_relative",
    "theta_enu",
    "theta_ego_relative",
)


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
    return mode if mode in PATH_FINDING_MODES else default


@dataclass(frozen=True)
class PathFindingParams:
    enabled: bool = True
    path_distance_m: float = 1000.0
    grid_size_m: float = 5.0
    cardinal_cost: float = 1.0
    diagonal_cost: float = 1.4
    run_every_n_frames: int = 5
    obstacle_projection_time_s: float = 10.0
    obstacle_expansion_radius_cells: int = 0
    mode: str = "astar_enu"
    show_debug_grid: bool = True

    @classmethod
    def from_mapping(cls, values: Optional[Dict[str, Any]] = None) -> "PathFindingParams":
        raw = values or {}
        return cls(
            enabled=_coerce_bool(raw.get("enabled", cls.enabled), cls.enabled),
            path_distance_m=_clamp(raw.get("path_distance_m", cls.path_distance_m), 500.0, 1500.0),
            grid_size_m=_clamp(raw.get("grid_size_m", cls.grid_size_m), 1.0, 20.0),
            cardinal_cost=_clamp(raw.get("cardinal_cost", cls.cardinal_cost), 0.1, 20.0),
            diagonal_cost=_clamp(raw.get("diagonal_cost", cls.diagonal_cost), 0.1, 30.0),
            run_every_n_frames=_clamp(raw.get("run_every_n_frames", cls.run_every_n_frames), 1, 20, integer=True),
            obstacle_projection_time_s=_clamp(
                raw.get("obstacle_projection_time_s", cls.obstacle_projection_time_s),
                0.0,
                30.0,
            ),
            obstacle_expansion_radius_cells=_clamp(
                raw.get("obstacle_expansion_radius_cells", cls.obstacle_expansion_radius_cells),
                0,
                10,
                integer=True,
            ),
            mode=_coerce_mode(raw.get("mode", cls.mode), cls.mode),
            show_debug_grid=_coerce_bool(raw.get("show_debug_grid", cls.show_debug_grid), cls.show_debug_grid),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "path_distance_m": float(self.path_distance_m),
            "grid_size_m": float(self.grid_size_m),
            "cardinal_cost": float(self.cardinal_cost),
            "diagonal_cost": float(self.diagonal_cost),
            "run_every_n_frames": int(self.run_every_n_frames),
            "obstacle_projection_time_s": float(self.obstacle_projection_time_s),
            "obstacle_expansion_radius_cells": int(self.obstacle_expansion_radius_cells),
            "mode": str(self.mode),
            "show_debug_grid": bool(self.show_debug_grid),
        }


@dataclass(frozen=True)
class TrackedObstacle:
    position_enu: ENU
    vel_east_mps: float = 0.0
    vel_north_mps: float = 0.0
    speed_mps: float = 0.0


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
class PathFindingResult:
    frame_number: int
    params: PathFindingParams
    start_enu: ENU
    goal_enu: ENU
    path_enu_points: List[Dict[str, float]]
    grid_frame: Dict[str, Any]
    grid_bounds: Dict[str, int]
    start_cell: List[int]
    goal_cell: List[int]
    obstacle_cells: List[List[int]]
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
                    "goal_cell": list(self.goal_cell),
                    "obstacle_cells": [list(cell) for cell in self.obstacle_cells],
                    "path_cells": [list(cell) for cell in self.path_cells],
                }
            )
        return payload

def _normalize_enu_basis(east_m: float, north_m: float) -> Tuple[float, float]:
    mag = math.hypot(float(east_m), float(north_m))
    if mag <= 1e-6:
        return (1.0, 0.0)
    return (float(east_m) / mag, float(north_m) / mag)


def _build_grid_frame(*, params: PathFindingParams, ego_enu: ENU, ego_heading_deg: float) -> GridFrame:
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


def _expand_cells(cells: Iterable[GridCoord], radius_cells: int) -> Set[GridCoord]:
    radius = max(0, int(radius_cells))
    expanded: Set[GridCoord] = set()
    for cell_x, cell_y in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                expanded.add((cell_x + dx, cell_y + dy))
    return expanded


def _build_obstacle_cells(obstacles: Iterable[TrackedObstacle], frame: GridFrame, params: PathFindingParams) -> Set[GridCoord]:
    obstacle_cells: Set[GridCoord] = set()
    projection_time_s = max(0.0, float(params.obstacle_projection_time_s))
    expansion_radius = int(params.obstacle_expansion_radius_cells)

    for obstacle in obstacles:
        start_cell = frame.enu_to_grid(obstacle.position_enu)
        projected_cells: Set[GridCoord] = {start_cell}
        speed = float(obstacle.speed_mps)
        if projection_time_s > 0.0 and math.isfinite(speed) and speed > 0.0:
            projected_enu = ENU(
                east_m=float(obstacle.position_enu.east_m) + float(obstacle.vel_east_mps) * projection_time_s,
                north_m=float(obstacle.position_enu.north_m) + float(obstacle.vel_north_mps) * projection_time_s,
            )
            end_cell = frame.enu_to_grid(projected_enu)
            for cell in _iter_cells_on_segment(start_cell, end_cell):
                projected_cells.add(cell)

        obstacle_cells.update(_expand_cells(projected_cells, expansion_radius))

    return obstacle_cells


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
            )
        )
    return obstacles


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
    goal_cell: GridCoord
    obstacle_cells: Set[GridCoord]
    bounds: Tuple[int, int, int, int]
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


def _line_of_sight(start: GridCoord, end: GridCoord, obstacle_cells: Set[GridCoord]) -> bool:
    for cell in _iter_cells_on_segment(start, end):
        if cell in obstacle_cells and cell != start and cell != end:
            return False
    return True


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


def astar_path(
    *,
    frame: GridFrame,
    start_enu: ENU,
    goal_enu: ENU,
    obstacles: Iterable[TrackedObstacle],
    params: PathFindingParams,
) -> AStarGridResult:
    start = frame.enu_to_grid(start_enu)
    goal = frame.enu_to_grid(goal_enu)
    obstacle_cells = _build_obstacle_cells(obstacles, frame, params)
    obstacle_cells.discard(start)
    obstacle_cells.discard(goal)

    padding_cells = max(10, int(math.ceil(params.path_distance_m / max(params.grid_size_m, 1e-6) * 0.2)))
    bounds = _search_bounds(start, goal, padding_cells)

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
        if current == goal:
            path_nodes = _reconstruct_path(came_from, current)
            path_cells = _expand_path_cells(path_nodes)
            return AStarGridResult(
                start_cell=start,
                goal_cell=goal,
                obstacle_cells=set(obstacle_cells),
                bounds=bounds,
                path_nodes=path_nodes,
                path_cells=path_cells,
                path_enu=[frame.grid_to_enu(cell) for cell in path_nodes],
            )

        current_g = g_score.get(current, float("inf"))
        for neighbor, step_cost in _neighbors(current, params.cardinal_cost, params.diagonal_cost):
            if neighbor in obstacle_cells or not _in_bounds(neighbor, bounds):
                continue
            actual_parent = current
            actual_parent_key = current
            actual_step_cost = step_cost
            parent_key = came_from.get(current)
            if use_theta and parent_key is not None and _line_of_sight(parent_key, neighbor, obstacle_cells):
                actual_parent = parent_key
                actual_parent_key = parent_key
                actual_step_cost = _heuristic(
                    actual_parent,
                    neighbor,
                    params.cardinal_cost,
                    params.diagonal_cost,
                )
            tentative_g = g_score.get(actual_parent_key, float("inf")) + actual_step_cost
            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = actual_parent
            g_score[neighbor] = tentative_g
            push_count += 1
            f_score = tentative_g + _heuristic(neighbor, goal, params.cardinal_cost, params.diagonal_cost)
            heapq.heappush(open_heap, (f_score, push_count, neighbor))

    return AStarGridResult(
        start_cell=start,
        goal_cell=goal,
        obstacle_cells=set(obstacle_cells),
        bounds=bounds,
        path_nodes=[],
        path_cells=[],
        path_enu=[],
    )


class PathFinderRuntime:
    def __init__(self, initial_params: Optional[Dict[str, Any]] = None):
        self._lock = asyncio.Lock()
        self._params = PathFindingParams.from_mapping(initial_params)
        self._last_result: Optional[PathFindingResult] = None
        self._last_run_frame: Optional[int] = None

    async def get_params(self) -> Dict[str, Any]:
        async with self._lock:
            return self._params.to_dict()

    async def set_params(self, new_values: Dict[str, Any]) -> Dict[str, Any]:
        async with self._lock:
            merged = self._params.to_dict()
            for key, value in (new_values or {}).items():
                if key in merged:
                    merged[key] = value
            self._params = PathFindingParams.from_mapping(merged)
            self._last_result = None
            self._last_run_frame = None
            return self._params.to_dict()

    async def compute(
        self,
        *,
        frame_number: int,
        ego_enu: ENU,
        ego_heading_deg: float,
        tracked_world: Sequence[Dict[str, Any]],
    ) -> Optional[PathFindingResult]:
        async with self._lock:
            params = self._params
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
                return PathFindingResult(
                    frame_number=frame_number,
                    params=self._last_result.params,
                    start_enu=self._last_result.start_enu,
                    goal_enu=self._last_result.goal_enu,
                    path_enu_points=list(self._last_result.path_enu_points),
                    grid_frame=dict(self._last_result.grid_frame),
                    grid_bounds=dict(self._last_result.grid_bounds),
                    start_cell=list(self._last_result.start_cell),
                    goal_cell=list(self._last_result.goal_cell),
                    obstacle_cells=[list(cell) for cell in self._last_result.obstacle_cells],
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
            goal_enu = ENU(
                east_m=float(ego_enu.east_m) + float(goal_offset.east_m),
                north_m=float(ego_enu.north_m) + float(goal_offset.north_m),
            )

            obstacles = _extract_obstacles(tracked_world)
            grid_frame = _build_grid_frame(params=params, ego_enu=ego_enu, ego_heading_deg=float(ego_heading_deg))
            grid_result = astar_path(
                frame=grid_frame,
                start_enu=ego_enu,
                goal_enu=goal_enu,
                obstacles=obstacles,
                params=params,
            )
            path_points = [{"east_m": float(p.east_m), "north_m": float(p.north_m)} for p in grid_result.path_enu]
            result = PathFindingResult(
                frame_number=frame_number,
                params=params,
                start_enu=ego_enu,
                goal_enu=goal_enu,
                path_enu_points=path_points,
                grid_frame=grid_frame.to_payload(),
                grid_bounds=_bounds_to_dict(grid_result.bounds),
                start_cell=[int(grid_result.start_cell[0]), int(grid_result.start_cell[1])],
                goal_cell=[int(grid_result.goal_cell[0]), int(grid_result.goal_cell[1])],
                obstacle_cells=sorted([[int(cell[0]), int(cell[1])] for cell in grid_result.obstacle_cells]),
                path_cells=[[int(cell[0]), int(cell[1])] for cell in grid_result.path_cells],
                obstacle_count=len(obstacles),
                used_cached_result=False,
                path_found=len(path_points) >= 2,
            )
            self._last_result = result
            self._last_run_frame = frame_number
            return result
