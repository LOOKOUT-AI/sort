from __future__ import annotations

import asyncio
import heapq
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from .image_to_world import ENU, right_fwd_to_enu_m


GridCoord = Tuple[int, int]


def _clamp(value: Any, lo: float, hi: float, *, integer: bool = False):
    try:
        num = float(value)
    except Exception:
        num = lo
    num = max(lo, min(hi, num))
    return int(round(num)) if integer else float(num)


@dataclass(frozen=True)
class PathFindingParams:
    path_distance_m: float = 1000.0
    grid_size_m: float = 5.0
    cardinal_cost: float = 1.0
    diagonal_cost: float = 1.4
    run_every_n_frames: int = 5

    @classmethod
    def from_mapping(cls, values: Optional[Dict[str, Any]] = None) -> "PathFindingParams":
        raw = values or {}
        return cls(
            path_distance_m=_clamp(raw.get("path_distance_m", cls.path_distance_m), 500.0, 1500.0),
            grid_size_m=_clamp(raw.get("grid_size_m", cls.grid_size_m), 1.0, 20.0),
            cardinal_cost=_clamp(raw.get("cardinal_cost", cls.cardinal_cost), 0.1, 20.0),
            diagonal_cost=_clamp(raw.get("diagonal_cost", cls.diagonal_cost), 0.1, 30.0),
            run_every_n_frames=_clamp(raw.get("run_every_n_frames", cls.run_every_n_frames), 1, 20, integer=True),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "path_distance_m": float(self.path_distance_m),
            "grid_size_m": float(self.grid_size_m),
            "cardinal_cost": float(self.cardinal_cost),
            "diagonal_cost": float(self.diagonal_cost),
            "run_every_n_frames": int(self.run_every_n_frames),
        }


@dataclass(frozen=True)
class PathFindingResult:
    frame_number: int
    params: PathFindingParams
    start_enu: ENU
    goal_enu: ENU
    path_enu_points: List[Dict[str, float]]
    grid_bounds: Dict[str, int]
    start_cell: List[int]
    goal_cell: List[int]
    obstacle_cells: List[List[int]]
    path_cells: List[List[int]]
    obstacle_count: int
    used_cached_result: bool = False
    path_found: bool = False

    def to_payload(self, *, enu_ref_lat: Optional[float], enu_ref_lon: Optional[float]) -> Dict[str, Any]:
        return {
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
            "grid_bounds": dict(self.grid_bounds),
            "start_cell": list(self.start_cell),
            "goal_cell": list(self.goal_cell),
            "obstacle_cells": [list(cell) for cell in self.obstacle_cells],
            "path_cells": [list(cell) for cell in self.path_cells],
            "obstacle_count": int(self.obstacle_count),
            "path_found": bool(self.path_found),
            "used_cached_result": bool(self.used_cached_result),
        }


def enu_to_grid(enu: ENU, grid_size_m: float) -> GridCoord:
    inv = 1.0 / max(float(grid_size_m), 1e-6)
    return int(round(float(enu.east_m) * inv)), int(round(float(enu.north_m) * inv))


def grid_to_enu(cell: GridCoord, grid_size_m: float) -> ENU:
    return ENU(east_m=float(cell[0]) * float(grid_size_m), north_m=float(cell[1]) * float(grid_size_m))


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


def _build_obstacle_cells(obstacles_enu: Iterable[ENU], grid_size_m: float) -> Set[GridCoord]:
    return {enu_to_grid(obs, grid_size_m) for obs in obstacles_enu}


def _extract_obstacles(tracked_world: Iterable[Dict[str, Any]]) -> List[ENU]:
    obstacles: List[ENU] = []
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
        obstacles.append(ENU(east_m=east_m, north_m=north_m))
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
    path_cells: List[GridCoord]
    path_enu: List[ENU]


def astar_path(
    *,
    start_enu: ENU,
    goal_enu: ENU,
    obstacles_enu: Iterable[ENU],
    params: PathFindingParams,
) -> AStarGridResult:
    start = enu_to_grid(start_enu, params.grid_size_m)
    goal = enu_to_grid(goal_enu, params.grid_size_m)
    obstacle_cells = _build_obstacle_cells(obstacles_enu, params.grid_size_m)
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

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path_cells = _reconstruct_path(came_from, current)
            return AStarGridResult(
                start_cell=start,
                goal_cell=goal,
                obstacle_cells=set(obstacle_cells),
                bounds=bounds,
                path_cells=path_cells,
                path_enu=[grid_to_enu(cell, params.grid_size_m) for cell in path_cells],
            )

        current_g = g_score.get(current, float("inf"))
        for neighbor, step_cost in _neighbors(current, params.cardinal_cost, params.diagonal_cost):
            if neighbor in obstacle_cells or not _in_bounds(neighbor, bounds):
                continue
            tentative_g = current_g + step_cost
            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            push_count += 1
            f_score = tentative_g + _heuristic(neighbor, goal, params.cardinal_cost, params.diagonal_cost)
            heapq.heappush(open_heap, (f_score, push_count, neighbor))

    return AStarGridResult(
        start_cell=start,
        goal_cell=goal,
        obstacle_cells=set(obstacle_cells),
        bounds=bounds,
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
            grid_result = astar_path(
                start_enu=ego_enu,
                goal_enu=goal_enu,
                obstacles_enu=obstacles,
                params=params,
            )
            path_points = [{"east_m": float(p.east_m), "north_m": float(p.north_m)} for p in grid_result.path_enu]
            result = PathFindingResult(
                frame_number=frame_number,
                params=params,
                start_enu=ego_enu,
                goal_enu=goal_enu,
                path_enu_points=path_points,
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
