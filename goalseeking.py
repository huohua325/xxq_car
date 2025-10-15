"""
===============================================================================
Goal-Seeking Navigation App/Pipeline (Real-Robot Ready)
Author: Jonathan Loo
Version: 1.0
===============================================================================

Purpose
- Synchronous Sense→Think→Act loop: read pose+LiDAR (sim), optionally refine pose
  via ICP + gated fusion, update the occupancy grid (OGM), compute a GOALSEEKING
  pose setpoint, and advance a simple kinematic simulator.
- Demonstrates a practical “SLAM-lite” loop: ICP-aided localisation + OGM mapping
  in real time while driving via pose setpoints.
- Real-robot ready: only replace get_robot_pose(), get_lidar_scan(), send_setpoint()
  (no other changes).

Execution Model (Blocking & Synchronous)
- Single-threaded while-loop; each tick runs end-to-end:
  SENSE → (ICP) → FUSE → MAP/VIZ → DECIDE → ACT → LOG → return.
- No background threads/async by default; each tick is blocking.

SIM vs REAL in this model
- SIM mode:
  * send_setpoint(...) immediately advances the simulated kinematics before the tick ends.
  * On-screen motion comes from the simulator's updated pose.
- REAL mode:
  * send_setpoint(...) transmits the setpoint to the robot; no simulated motion occurs.
  * The display updates only from the robot's reported pose/LiDAR.
  * If get_robot_pose()/get_lidar_scan() block waiting for data, the whole loop stalls.

I/O Guidance for REAL mode (brief)
- Use non-blocking serial (or a small RX thread/queue) with timeouts.
- Validate packets (length + checksum/CRC); discard partial frames.
- Adjust input rates suitable for your robot: Pose 20-30 Hz, LiDAR 5-10 Hz. Units: metres/radians.

Modes
- MANUAL: key-steered; ICP/mapping optional for logging.
- GOALSEEKING: follow global A* path with lookahead + near-field LiDAR repulsion.

High-Level Goal-Seeking Pipeline (each tick)
1) SENSE
   pose_raw = get_robot_pose();  scan_now = get_lidar_scan()
2) LOCALISE (ICP: scan-to-scan)
   estimate Δ = (Δx, Δy, Δθ) from prev_scan → scan_now
   icp_pose = prev_pose ⊕ Δ; keep RMSE / #pairs / |Δ| for gating
3) FUSE (state update)
   if gates pass: pose_used = fused(prev=pose_raw, target=icp_pose); state.pose = pose_used
   else:          pose_used = pose_raw
4) MAPPING
   integrate scan_now into OGM using pose_used; render dashboard
5) GOAL CHECK
   stop if distance(pose_used, goal) ≤ arrival_tol
6) DECIDE (GOALSEEKING)
   lookahead on A* path + LiDAR repulsion → PoseSetpoint(x*, y*, θ*)
7) ACT
   send_setpoint(setpoint):
     SIM_MODE=True  → step simple unicycle/diff-drive kinematics (v/ω-limited); update state.pose
     SIM_MODE=False → publish to hardware (no simulator step)
8) LOG
   record time, pose_raw, icp_pose (if any), pose_used, LiDAR, target

Maze / World Settings (sim ↔ real)
- Match these to a real maze:
  • World size (size_m)     - outer square size in metres
  • Cell size (cell_size_m) - grid resolution (metres per cell)
  • Wall thickness (m)      - physical wall width
  • Entrance & Goal cells   - integer grid coords (cx, cy)
- Typical parameters (edit to your arena):
  WORLD:
    size_m                = 1.80        # arena side length
    cell_size_m           = 0.45        # grid pitch
    wall_half_thickness_m = 0.005       # wall_thickness = 2 * this
    border_thickness_m    = 0.01        # outer frame width
  MAP (choose one):
    # SNAKE
    num_walls   = 4
    gap_cells   = 1
    entrance_cell   = (0, 0)
    snake_goal_cell = None   # defaults to bottom-right if None
    # RANDOM
    random_wall_count = 12
    entrance_cell     = (0, 0)
    random_goal_cell  = (3, 3)

===============================================================================
"""

from __future__ import annotations

import csv
import logging
import math
import os
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path as _Path
from typing import Iterable, List, Optional, Protocol, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, Circle

# -----------------------------------------------------------------------------
# Type aliases
# -----------------------------------------------------------------------------
Cell = Tuple[int, int]  # (cx, cy) grid
Wall = Tuple[float, float, float, float]  # (minx, maxx, miny, maxy) meters

def cell_center_m(cell: Cell, cell_size_m: float) -> Tuple[float, float]:
    cx, cy = cell
    return (cx * cell_size_m + cell_size_m / 2.0, cy * cell_size_m + cell_size_m / 2.0)

# -----------------------------------------------------------------------------
# Config dataclasses (tunable block)
# -----------------------------------------------------------------------------
@dataclass(slots=True)
class WorldConfig:
    wall_half_thickness_m: float = 0.005
    border_thickness_m: float = 0.01

@dataclass(slots=True)
class SnakeMazeConfig:
    size_m: float = 1.80
    cell_size_m: float = 0.45
    num_walls: int = 4
    gap_cells: int = 1

@dataclass(slots=True)
class RandomMazeConfig:
    size_m: float = 1.80
    cell_size_m: float = 0.45
    random_wall_count: int = 12
    random_seed: Optional[int] = None
    candidates_to_list: int = 5
    seed_scan_start: int = 0
    seed_scan_stride: int = 1
    max_attempts_per_page: int = 10000
    segment_len_cells_min: int = 1
    segment_len_cells_max: int = 2
    orientation_bias: float = 0.5  # prob of H vs V

@dataclass(slots=True)
class PlanningConfig:
    sample_step_m: float = 0.03
    resample_ds_m: float = 0.05
    equal_eps: float = 1e-6
    seg_eps: float = 1e-9

@dataclass(slots=True)
class LidarConfig:
    num_rays: int = 360
    max_range_m: float = 3.0
    raycast_eps: float = 1e-6

@dataclass(slots=True)
class OGMConfig:
    xyreso_m: float = 0.03
    l_free: float = -0.4
    l_occ: float = +0.85
    l_min: float = -4.0
    l_max: float = +4.0
    hit_margin_m: float = 1e-3
    prob_free_max: float = 0.35
    prob_occ_min: float = 0.65
    size_eps: float = 1e-9
    gray_free: float = 0.9
    gray_occ: float = 0.0
    gray_unk: float = 1.0

@dataclass(slots=True)
class RobotConfig:
    robot_radius_m: float = 0.15            # used as A* inflation radius
    turn_angle_rad: float = math.radians(18)
    v_max_mps: float = 0.35
    dt_s: float = 0.1
    dt_guard_s: float = 1e-3
    manual_forward_cone_rad: float = math.pi / 6
    manual_step_m: float = 0.10

@dataclass(slots=True)
class SimpleControllerConfig:
    lookahead_m: float = 0.30
    k_ang: float = 1.8
    k_avoid: float = 1.2
    avoid_radius_m: float = 0.35
    v_max_mps: float = 0.30
    repulse_gain: float = 1.0
    manual_preview_lookahead_m: float = 0.25

@dataclass(slots=True)
class PoseFusionConfig:
    enabled: bool = True
    alpha: float = 0.1
    max_trans_m: float = 0.20
    max_rot_deg: float = 20.0
    min_points: int = 50
    max_rmse_m: float = 0.05
    snap_trans_m: float = 0.02
    snap_rot_deg: float = 2.0

@dataclass(slots=True)
class VizConfig:
    main_figsize_in: Tuple[float, float] = (14, 10)
    robot_arrow_len_m: float = 0.10
    robot_arrow_head_m: Tuple[float, float] = (0.03, 0.03)
    ogm_arrow_len_m: float = 0.10
    ogm_arrow_head_m: Tuple[float, float] = (0.03, 0.03)
    lidar_alpha: float = 0.2
    lidar_lw: float = 0.5
    thumb_size_in: Tuple[float, float] = (3, 3)
    pause_s: float = 0.01

@dataclass(slots=True)
class LoggingConfig:
    level: int = logging.INFO
    format: str = "[%(levelname)s] %(message)s"
    pose_csv: str = "pose.csv"
    lidar_csv: str = "lidar.csv"

@dataclass(slots=True)
class AppConfig:
    arrival_tol_m: float = 0.10
    mode: str = "GOALSEEKING"   # GOALSEEKING | MANUAL
    map_type: str = "RANDOM"    # RANDOM | SNAKE
    entrance_cell: Cell = (0, 0)
    snake_goal_cell: Optional[Cell] = None
    random_goal_cell: Cell = (3, 3)

@dataclass(slots=True)
class ProjectConfig:
    world: WorldConfig = field(default_factory=WorldConfig)
    snake: SnakeMazeConfig = field(default_factory=SnakeMazeConfig)
    random: RandomMazeConfig = field(default_factory=RandomMazeConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    ogm: OGMConfig = field(default_factory=OGMConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    controller: SimpleControllerConfig = field(default_factory=SimpleControllerConfig)
    fusion: PoseFusionConfig = field(default_factory=PoseFusionConfig)
    viz: VizConfig = field(default_factory=VizConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    app: AppConfig = field(default_factory=AppConfig)

DEFAULTS = ProjectConfig()

# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------
logging.basicConfig(level=DEFAULTS.logging.level, format=DEFAULTS.logging.format)
logger = logging.getLogger("maze_app")

# -----------------------------------------------------------------------------
# PythonRobotics imports
# -----------------------------------------------------------------------------
PR_PATH = os.environ.get("PYTHONROBOTICS_PATH", None)
if PR_PATH and PR_PATH not in sys.path:
    sys.path.insert(0, PR_PATH)

PR_HAS_ASTAR = False
PR_HAS_ICP = False
try:
    from PathPlanning.AStar.a_star import AStarPlanner  # type: ignore
    PR_HAS_ASTAR = True
except Exception as e:
    logger.debug("A* import failed: %s", e)
try:
    from SLAM.ICPMatching.icp_matching import icp_matching as PR_icp_matching  # type: ignore
    PR_HAS_ICP = True
except Exception as e:
    logger.debug("ICP import failed: %s", e)

# -----------------------------------------------------------------------------
# Protocols
# -----------------------------------------------------------------------------
class Planner(Protocol):
    def plan(self, start_cell: Cell, goal_cell: Cell) -> List[Tuple[float, float]]: ...

class AutoControllerProto(Protocol):
    def compute(self, pose: "Pose", scan: "Scan") -> "PoseSetpoint": ...

class ManualControllerProto(Protocol):
    def decide(self, pose: "Pose", scan: "Scan", world: "_MazeBase", goal: "Goal") -> "Pose": ...

# -----------------------------------------------------------------------------
# Data models
# -----------------------------------------------------------------------------
@dataclass(slots=True)
class Pose:
    x: float
    y: float
    theta: float  # rad

@dataclass(slots=True)
class PoseSetpoint:
    x: float
    y: float
    theta: float

@dataclass(slots=True)
class Scan:
    angles: List[float]
    ranges: List[float]

@dataclass(slots=True)
class Goal:
    cell: Cell

# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------
def sample_rect_perimeter(minx: float, maxx: float, miny: float, maxy: float, step: float) -> Iterable[Tuple[float, float]]:
    x = minx
    while x <= maxx:
        yield (x, miny)
        yield (x, maxy)
        x += step
    y = miny
    while y <= maxy:
        yield (minx, y)
        yield (maxx, y)
        y += step

def walls_to_obstacles_m(walls: List[Wall], step_m: float) -> Tuple[List[float], List[float]]:
    ox: List[float] = []
    oy: List[float] = []
    for (minx, maxx, miny, maxy) in walls:
        for x, y in sample_rect_perimeter(minx, maxx, miny, maxy, step_m):
            ox.append(x); oy.append(y)
    return ox, oy

def resample_path_m(path_xy_m: List[Tuple[float, float]], ds_m: float, equal_eps: float, seg_eps: float) -> List[Tuple[float, float]]:
    if not path_xy_m:
        return []
    if len(path_xy_m) == 1:
        return path_xy_m[:]
    cleaned = [path_xy_m[0]]
    for p in path_xy_m[1:]:
        if abs(p[0] - cleaned[-1][0]) > equal_eps or abs(p[1] - cleaned[-1][1]) > equal_eps:
            cleaned.append(p)
    if len(cleaned) < 2:
        return cleaned
    arc = [0.0]
    for i in range(1, len(cleaned)):
        dx = cleaned[i][0] - cleaned[i - 1][0]
        dy = cleaned[i][1] - cleaned[i - 1][1]
        arc.append(arc[-1] + math.hypot(dx, dy))
    total = arc[-1]
    if total < seg_eps:
        return cleaned
    out: List[Tuple[float, float]] = []
    s = 0.0
    i = 0
    while s <= total and i < len(cleaned) - 1:
        while arc[i + 1] < s and i + 1 < len(arc) - 1:
            i += 1
        seg_len = arc[i + 1] - arc[i]
        if seg_len <= seg_eps:
            out.append(cleaned[i])
            s += ds_m
            continue
        t = (s - arc[i]) / seg_len
        x = cleaned[i][0] * (1 - t) + cleaned[i + 1][0] * t
        y = cleaned[i][1] * (1 - t) + cleaned[i + 1][1] * t
        out.append((x, y))
        s += ds_m
    out.append(cleaned[-1])
    return out

# -----------------------------------------------------------------------------
# Maze
# -----------------------------------------------------------------------------
class _MazeBase:
    def __init__(self, cfg_world: WorldConfig, size_m: float, cell_size_m: float):
        self.size_m = float(size_m)
        self.cell_size_m = float(cell_size_m)
        self.grid_size = int(round(self.size_m / self.cell_size_m))
        self.barriers: List[dict] = []
        self.walls: List[Wall] = []
        self.w_half = cfg_world.wall_half_thickness_m
        self.border = cfg_world.border_thickness_m

    def _build_walls_rects(self) -> None:
        walls: List[Wall] = []
        wh = self.w_half
        for b in self.barriers:
            if b.get("orientation") == "H":
                miny = b["y"] - wh; maxy = b["y"] + wh
                walls.append((b["minx"], b["maxx"], miny, maxy))
            else:
                minx = b["x"] - wh; maxx = b["x"] + wh
                walls.append((minx, maxx, b["miny"], b["maxy"]))
        s = self.size_m; bt = self.border
        walls.extend([(0, bt, 0, s), (s - bt, s, 0, s), (0, s, 0, bt), (0, s, s - bt, s)])
        self.walls = walls

    def _is_vertical_open(self, row_from: int, row_to: int, col: int) -> bool:
        if abs(row_to - row_from) != 1:
            return True
        interval_y = (min(row_from, row_to) + 1) * self.cell_size_m
        col_start = col * self.cell_size_m
        col_end = col_start + self.cell_size_m
        for b in self.barriers:
            if b.get("orientation") == "H" and abs(b["y"] - interval_y) < 1e-9:
                if b["minx"] < col_end and b["maxx"] > col_start:
                    return False
        return True

    def _is_horizontal_open(self, col_from: int, col_to: int, row: int) -> bool:
        if abs(col_to - col_from) != 1:
            return True
        interval_x = (min(col_from, col_to) + 1) * self.cell_size_m
        row_start = row * self.cell_size_m
        row_end = row_start + self.cell_size_m
        for b in self.barriers:
            if b.get("orientation") == "V" and abs(b["x"] - interval_x) < 1e-9:
                if b["miny"] < row_end and b["maxy"] > row_start:
                    return False
        return True

    def neighbors4(self, row: int, col: int) -> Iterable[Tuple[int, int]]:
        n = self.grid_size
        if col + 1 < n and self._is_horizontal_open(col, col + 1, row):
            yield row, col + 1
        if col - 1 >= 0 and self._is_horizontal_open(col - 1, col, row):
            yield row, col - 1
        if row + 1 < n and self._is_vertical_open(row, row + 1, col):
            yield row + 1, col
        if row - 1 >= 0 and self._is_vertical_open(row - 1, row, col):
            yield row - 1, col

    def path_exists(self, start: Cell, goal: Cell) -> bool:
        sr, sc = start[1], start[0]
        gr, gc = goal[1], goal[0]
        stack = [(sr, sc)]
        visited = set(stack)
        while stack:
            row, col = stack.pop()
            if (row, col) == (gr, gc):
                return True
            for nr, nc in self.neighbors4(row, col):
                if (nr, nc) not in visited:
                    visited.add((nr, nc))
                    stack.append((nr, nc))
        return False

class MazeSnake(_MazeBase):
    def __init__(self, cfg_world: WorldConfig, cfg: SnakeMazeConfig):
        super().__init__(cfg_world, cfg.size_m, cfg.cell_size_m)
        gap_w = max(1, int(cfg.gap_cells)) * cfg.cell_size_m
        barriers: List[dict] = []
        for i in range(1, int(cfg.num_walls) + 1):
            y = i * cfg.cell_size_m
            if y >= cfg.size_m:
                break
            if i % 2 == 1:
                minx, maxx = 0.0, max(0.0, cfg.size_m - gap_w)
            else:
                minx, maxx = min(cfg.size_m, gap_w), cfg.size_m
            barriers.append({"orientation": "H", "y": float(y), "minx": float(minx), "maxx": float(maxx)})
        self.barriers = barriers
        self._build_walls_rects()

class MazeRandom(_MazeBase):
    def __init__(self, cfg_world: WorldConfig, cfg: RandomMazeConfig):
        super().__init__(cfg_world, cfg.size_m, cfg.cell_size_m)
        if cfg.random_seed is not None:
            random.seed(cfg.random_seed)
        barriers: List[dict] = []
        min_len = max(1, int(cfg.segment_len_cells_min))
        max_len = max(min_len, int(cfg.segment_len_cells_max))
        for _ in range(cfg.random_wall_count):
            ori = "H" if random.random() < cfg.orientation_bias else "V"
            if ori == "H":
                y = random.randint(1, self.grid_size - 1) * cfg.cell_size_m
                span_cells = random.randint(min_len, max_len)
                minx = random.randint(0, self.grid_size - 1 - span_cells) * cfg.cell_size_m
                maxx = minx + span_cells * cfg.cell_size_m
                barriers.append({"orientation": "H", "y": float(y), "minx": float(minx), "maxx": float(maxx)})
            else:
                x = random.randint(1, self.grid_size - 1) * cfg.cell_size_m
                span_cells = random.randint(min_len, max_len)
                miny = random.randint(0, self.grid_size - 1 - span_cells) * cfg.cell_size_m
                maxy = miny + span_cells * cfg.cell_size_m
                barriers.append({"orientation": "V", "x": float(x), "miny": float(miny), "maxy": float(maxy)})
        self.barriers = barriers
        self._build_walls_rects()

# -----------------------------------------------------------------------------
# LiDAR
# -----------------------------------------------------------------------------
class LidarSimulator:
    def __init__(self, cfg: LidarConfig):
        self.cfg = cfg
        self._angles = np.linspace(0, 2 * np.pi, cfg.num_rays, endpoint=False)
        self._cos = np.cos(self._angles)
        self._sin = np.sin(self._angles)

    def _ray_rect_intersect(self, px, py, dx, dy, minx, maxx, miny, maxy) -> List[float]:
        ts: List[float] = []
        eps = self.cfg.raycast_eps
        if abs(dx) > eps:
            t = (minx - px) / dx
            if t > 0:
                yy = py + t * dy
                if miny <= yy <= maxy:
                    ts.append(t)
            t = (maxx - px) / dx
            if t > 0:
                yy = py + t * dy
                if miny <= yy <= maxy:
                    ts.append(t)
        if abs(dy) > eps:
            t = (miny - py) / dy
            if t > 0:
                xx = px + t * dx
                if minx <= xx <= maxx:
                    ts.append(t)
            t = (maxy - py) / dy
            if t > 0:
                xx = px + t * dx
                if minx <= xx <= maxx:
                    ts.append(t)
        return ts

    def scan(self, pose: Pose, world: _MazeBase) -> Scan:
        cth, sth = math.cos(pose.theta), math.sin(pose.theta)
        ranges: List[float] = []
        for i in range(len(self._angles)):
            dx = cth * self._cos[i] - sth * self._sin[i]
            dy = sth * self._cos[i] + cth * self._sin[i]
            min_t = self.cfg.max_range_m
            for w in world.walls:
                ts = self._ray_rect_intersect(pose.x, pose.y, dx, dy, *w)
                if ts:
                    t = min(ts)
                    if t > 0:
                        min_t = min(min_t, t)
            d = min_t if min_t < self.cfg.max_range_m else self.cfg.max_range_m
            ranges.append(d)
        return Scan(angles=list(self._angles), ranges=ranges)

# -----------------------------------------------------------------------------
# OGM
# -----------------------------------------------------------------------------
class OccupancyGridMap:
    def __init__(self, cfg: OGMConfig, minx: float, miny: float, maxx: float, maxy: float):
        self.cfg = cfg
        self.minx, self.miny, self.maxx, self.maxy = minx, miny, maxx, maxy
        self.xyreso = cfg.xyreso_m
        h = int((maxy - miny) / self.xyreso + self.cfg.size_eps)
        w = int((maxx - minx) / self.xyreso + self.cfg.size_eps)
        self.grid = np.zeros((h, w), dtype=float)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        ix = int((x - self.minx) / self.xyreso)
        iy = int((y - self.miny) / self.xyreso)
        return ix, iy

    def integrate(self, scan: Scan, pose: Pose) -> None:
        h, w = self.grid.shape
        max_range = max(scan.ranges) if scan.ranges else 0.0
        margin = self.cfg.hit_margin_m
        for rel, dist in zip(scan.angles, scan.ranges):
            rng = min(dist, max_range)
            endx = pose.x + rng * math.cos(pose.theta + rel)
            endy = pose.y + rng * math.sin(pose.theta + rel)
            ix0, iy0 = self.world_to_grid(pose.x, pose.y)
            ix1, iy1 = self.world_to_grid(endx, endy)
            if not (0 <= ix0 < w and 0 <= iy0 < h):
                continue
            # Bresenham
            dx = abs(ix1 - ix0); dy = -abs(iy1 - iy0)
            sx = 1 if ix0 < ix1 else -1
            sy = 1 if iy0 < iy1 else -1
            err = dx + dy
            x, y = ix0, iy0
            cells = []
            while True:
                cells.append((x, y))
                if x == ix1 and y == iy1:
                    break
                e2 = 2 * err
                if e2 >= dy:
                    err += dy; x += sx
                if e2 <= dx:
                    err += dx; y += sy
            if not cells:
                continue
            hit_occ = (dist < max_range - margin)
            upto = cells[:-1] if hit_occ else cells
            for cx, cy in upto:
                if 0 <= cx < w and 0 <= cy < h:
                    self.grid[cy, cx] = np.clip(self.grid[cy, cx] + self.cfg.l_free, self.cfg.l_min, self.cfg.l_max)
            if hit_occ:
                ex, ey = cells[-1]
                if 0 <= ex < w and 0 <= ey < h:
                    self.grid[ey, ex] = np.clip(self.grid[ey, ex] + self.cfg.l_occ, self.cfg.l_min, self.cfg.l_max)

    def overlay_image(self) -> np.ndarray:
        prob = 1.0 / (1.0 + np.exp(-self.grid))
        img = np.full_like(prob, self.cfg.gray_unk, dtype=float)
        img = np.where(prob <= self.cfg.prob_free_max, self.cfg.gray_free, img)
        img = np.where(prob >= self.cfg.prob_occ_min, self.cfg.gray_occ, img)
        return img

# -----------------------------------------------------------------------------
# Global Planner (PythonRobotics A*)
# -----------------------------------------------------------------------------
class GlobalAStar:
    def __init__(self, world: _MazeBase, grid_step_m: float, robot_radius_m: float):
        self.world = world
        self.grid_step_m = grid_step_m
        self.robot_radius_m = robot_radius_m
        self.obstacles_xy: Tuple[List[float], List[float]] = ([], [])  # raw obstacle samples
        self.cspace_pts: Tuple[List[float], List[float]] = ([], [])     # C-space occupied cell centers
        if not PR_HAS_ASTAR:
            raise ImportError("PythonRobotics A* not found. Set PYTHONROBOTICS_PATH to repo root.")

    def plan(self, start_cell: Cell, goal_cell: Cell, cfg_planning: PlanningConfig) -> List[Tuple[float, float]]:
        try:
            ox_m, oy_m = walls_to_obstacles_m(self.world.walls, step_m=cfg_planning.sample_step_m)
            self.obstacles_xy = (ox_m, oy_m)
            sx_m, sy_m = cell_center_m(start_cell, self.world.cell_size_m)
            gx_m, gy_m = cell_center_m(goal_cell, self.world.cell_size_m)
            planner = AStarPlanner(ox_m, oy_m, cfg_planning.sample_step_m, self.robot_radius_m)
            rx_m, ry_m = planner.planning(sx_m, sy_m, gx_m, gy_m)
        except Exception as e:
            raise RuntimeError(f"A* planning failed: {e}")
        if rx_m is None or ry_m is None or len(rx_m) == 0:
            raise RuntimeError("A* returned empty path.")
        # Capture C-space obstacle grid (occupied cells) for viz
        try:
            xs: List[float] = []
            ys: List[float] = []
            for ix in range(planner.x_width):
                for iy in range(planner.y_width):
                    if planner.obstacle_map[ix][iy]:
                        wx = planner.calc_grid_position(ix, planner.min_x)
                        wy = planner.calc_grid_position(iy, planner.min_y)
                        xs.append(wx); ys.append(wy)
            self.cspace_pts = (xs, ys)
        except Exception:
            self.cspace_pts = ([], [])
        rx_m = list(rx_m)[::-1]; ry_m = list(ry_m)[::-1]
        path_xy_m = [(x, y) for x, y in zip(rx_m, ry_m)]
        path_xy_m = resample_path_m(path_xy_m, ds_m=cfg_planning.resample_ds_m,
                                    equal_eps=cfg_planning.equal_eps, seg_eps=cfg_planning.seg_eps)
        logger.debug("A* samples: %d", len(path_xy_m))
        return path_xy_m

# -----------------------------------------------------------------------------
# Simple Controller (stateless setpoint generator)
# -----------------------------------------------------------------------------
class SimpleController:
    def __init__(self, robot_cfg: RobotConfig, cfg: SimpleControllerConfig, world: _MazeBase, path_xy_m: List[Tuple[float, float]]):
        self.r = robot_cfg; self.c = cfg; self.world = world; self.path = path_xy_m[:]

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _lookahead_target(self, pose: Pose) -> Tuple[float, float]:
        if not self.path:
            return (pose.x, pose.y)
        dists = [math.hypot(px - pose.x, py - pose.y) for px, py in self.path]
        ind = int(np.argmin(dists))
        L = 0.0; i = ind
        while L < self.c.lookahead_m and i < len(self.path) - 1:
            dx = self.path[i + 1][0] - self.path[i][0]
            dy = self.path[i + 1][1] - self.path[i][1]
            L += math.hypot(dx, dy); i += 1
        return self.path[i]

    def _repulsive_heading(self, scan: Scan) -> Tuple[float, float]:
        R = self.c.avoid_radius_m; g = self.c.repulse_gain
        rx = ry = 0.0
        for a_rel, d in zip(scan.angles, scan.ranges):
            if d <= 0.0 or d > R: continue
            s = g * max(0.0, (1.0 / d - 1.0 / R))
            rx += -math.cos(a_rel) * s; ry += -math.sin(a_rel) * s
        return (rx, ry)

    def compute(self, pose: Pose, scan: Scan) -> PoseSetpoint:
        gx, gy = self._lookahead_target(pose)
        ax, ay = gx - pose.x, gy - pose.y
        ct, st = math.cos(pose.theta), math.sin(pose.theta)
        ahead_x_robot = ct * ax + st * ay
        ahead_y_robot = -st * ax + ct * ay
        rx, ry = self._repulsive_heading(scan)
        comb_x = ahead_x_robot + self.c.k_avoid * rx
        comb_y = ahead_y_robot + self.c.k_avoid * ry
        desired_heading_robot = math.atan2(comb_y, comb_x)
        desired_theta_world = (pose.theta + desired_heading_robot) % (2 * math.pi)
        return PoseSetpoint(x=gx, y=gy, theta=desired_theta_world)

# -----------------------------------------------------------------------------
# Visualization (single dashboard figure, 4 panels of equal size/aspect)
# -----------------------------------------------------------------------------
class MatplotlibViz:
    """TL: map+lidar+path, TR: OGM, BL: A* C-space obstacles (+ robot_radius), BR: ICP clouds.
    All four panels share identical world limits and square aspect.
    """
    def __init__(self, size_m: float, cell_size_m: float, cfg: VizConfig, robot_radius_m: float):
        from matplotlib.gridspec import GridSpec
        self.size = size_m; self.cell = cell_size_m; self.cfg = cfg; self.robot_radius = float(robot_radius_m)
        self.fig = plt.figure(figsize=self.cfg.main_figsize_in, constrained_layout=True)
        gs = GridSpec(2, 2, figure=self.fig)
        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_ogm = self.fig.add_subplot(gs[0, 1])
        self.ax_path = self.fig.add_subplot(gs[1, 0])
        self.ax_icp = self.fig.add_subplot(gs[1, 1])
        # enforce identical limits/aspect for all
        for ax in (self.ax_map, self.ax_ogm, self.ax_path, self.ax_icp):
            ax.set_xlim(0, self.size); ax.set_ylim(0, self.size)
            ax.set_box_aspect(1); ax.set_aspect('equal', adjustable='box')
            ax.grid(True)
        # major ticks on TL/TR only (grid lines)
        for ax in (self.ax_map, self.ax_ogm):
            ax.set_xticks(np.arange(0, self.size + 1e-9, self.cell))
            ax.set_yticks(np.arange(0, self.size + 1e-9, self.cell))
        self.ax_path.set_title("A* C-space Obstacles")
        self.ax_icp.set_title("ICP Clouds")
        # stateful ICP artists
        self._icp_prev = None
        self._icp_curr = None
        self._icp_xfrm = None

    def _draw_world(self, world: _MazeBase, entrance: Cell, goal: Cell):
        # TL/TR get refreshed; keep limits/aspect
        self.ax_map.clear(); self.ax_ogm.clear()
        for ax in (self.ax_map, self.ax_ogm):
            ax.set_xlim(0, self.size); ax.set_ylim(0, self.size)
            ax.set_box_aspect(1); ax.set_aspect('equal', adjustable='box')
            ax.set_xticks(np.arange(0, self.size + 1e-9, self.cell))
            ax.set_yticks(np.arange(0, self.size + 1e-9, self.cell))
            ax.grid(True)
        # Walls
        for w in world.walls:
            minx, maxx, miny, maxy = w
            self.ax_map.add_patch(Rectangle((minx, miny), maxx - minx, maxy - miny, facecolor="black"))
        # Entrance/Goal
        ex0, ey0 = entrance; ex1, ey1 = goal
        self.ax_map.add_patch(Rectangle((ex0 * self.cell, ey0 * self.cell), self.cell, self.cell, facecolor="green", alpha=0.3, label="Entrance"))
        self.ax_map.add_patch(Rectangle((ex1 * self.cell, ey1 * self.cell), self.cell, self.cell, facecolor="red", alpha=0.3, label="Goal"))
        h, l = self.ax_map.get_legend_handles_labels()
        by = dict(zip(l, h)); 
        if by: self.ax_map.legend(by.values(), by.keys())

    def render(
        self,
        world: _MazeBase,
        mapper: "OccupancyGridMap",
        pose: Pose,
        scan: Scan,
        goal: Goal,
        step: int,
        path_xy: Optional[List[Tuple[float, float]]] = None,
        entrance: Optional[Cell] = None,
        icp_prev_pts: Optional[np.ndarray] = None,
        icp_curr_pts: Optional[np.ndarray] = None,
        icp_prev_tf_pts: Optional[np.ndarray] = None,
        astar_pts: Optional[Tuple[List[float], List[float]]] = None,
    ) -> None:
        self._draw_world(world, entrance or (0, 0), goal.cell)
        # TL: map + lidar + path
        self.ax_map.plot(pose.x, pose.y, "bo", markersize=8)
        ral = self.cfg.robot_arrow_len_m; rah = self.cfg.robot_arrow_head_m
        self.ax_map.arrow(pose.x, pose.y, ral * math.cos(pose.theta), ral * math.sin(pose.theta), head_width=rah[0], head_length=rah[1], fc="blue", ec="blue")
        for a, d in zip(scan.angles, scan.ranges):
            ex = pose.x + d * math.cos(pose.theta + a)
            ey = pose.y + d * math.sin(pose.theta + a)
            self.ax_map.plot([pose.x, ex], [pose.y, ey], "r-", alpha=self.cfg.lidar_alpha, linewidth=self.cfg.lidar_lw)
        if path_xy:
            xs, ys = zip(*path_xy); self.ax_map.plot(xs, ys, "g--", linewidth=2, alpha=0.8)
        self.ax_map.set_title(f"Ground Truth Maze — Step {step}")
        self.ax_map.set_xlabel("X (m)"); self.ax_map.set_ylabel("Y (m)")
        # TR: OGM
        ogm_img = mapper.overlay_image()
        self.ax_ogm.imshow(ogm_img, extent=(0, self.size, 0, self.size), origin="lower", vmin=0.0, vmax=1.0, cmap="gray")
        self.ax_ogm.plot(pose.x, pose.y, "bo", markersize=6, label="Robot")
        oal = self.cfg.ogm_arrow_len_m; oah = self.cfg.ogm_arrow_head_m
        self.ax_ogm.arrow(pose.x, pose.y, oal * math.cos(pose.theta), oal * math.sin(pose.theta), head_width=oah[0], head_length=oah[1], fc="blue", ec="blue")
        self.ax_ogm.set_title("Occupancy Grid Map"); self.ax_ogm.set_xlabel("X (m)"); self.ax_ogm.set_ylabel("Y (m)")
        hh, ll = self.ax_ogm.get_legend_handles_labels()
        if hh: self.ax_ogm.legend(dict(zip(ll, hh)).values(), dict(zip(ll, hh)).keys())
        # BL: C-space obstacles from A* calc_obstacle_map + robot radius
        self.ax_path.clear()
        self.ax_path.set_xlim(0, self.size); self.ax_path.set_ylim(0, self.size)
        self.ax_path.set_box_aspect(1); self.ax_path.set_aspect('equal', adjustable='box')
        self.ax_path.set_title("A* C-space Obstacles"); self.ax_path.set_xlabel("X (m)"); self.ax_path.set_ylabel("Y (m)"); self.ax_path.grid(True)
        if astar_pts is not None and len(astar_pts[0]) > 0:
            self.ax_path.plot(astar_pts[0], astar_pts[1], 'x', ms=4, alpha=0.8, color='#12B2B2', label='occupied cells')
        if path_xy:
            xs, ys = zip(*path_xy); self.ax_path.plot(xs, ys, ".-", alpha=0.9, label='Goal path')
        # robot marker + C-space planning radius circle
        self.ax_path.plot(pose.x, pose.y, "ro", ms=4, label='robot')
        try:
            circ = Circle((pose.x, pose.y), radius=self.robot_radius, fill=False, linestyle='--', linewidth=1.2, color='#FF7F0E', label='robot_radius')
            self.ax_path.add_patch(circ)
        except Exception:
            pass
        if self.ax_path.get_legend_handles_labels()[0]: self.ax_path.legend()
        # BR: ICP clouds (persistent artists)
        self.ax_icp.set_xlim(0, self.size); self.ax_icp.set_ylim(0, self.size)
        self.ax_icp.set_box_aspect(1); self.ax_icp.set_aspect('equal', adjustable='box')
        self.ax_icp.set_title("ICP Clouds (prev=gray, curr=blue, xform=green)")
        self.ax_icp.set_xlabel("X (m)"); self.ax_icp.set_ylabel("Y (m)"); self.ax_icp.grid(True)
        if icp_prev_pts is not None and icp_prev_pts.size > 0:
            if self._icp_prev is None:
                self._icp_prev = self.ax_icp.scatter(icp_prev_pts[:,0], icp_prev_pts[:,1], s=3, c="#888888", label="prev")
            else:
                self._icp_prev.set_offsets(icp_prev_pts)
        if icp_curr_pts is not None and icp_curr_pts.size > 0:
            if self._icp_curr is None:
                self._icp_curr = self.ax_icp.scatter(icp_curr_pts[:,0], icp_curr_pts[:,1], s=3, label="curr")
            else:
                self._icp_curr.set_offsets(icp_curr_pts)
        if icp_prev_tf_pts is not None and icp_prev_tf_pts.size > 0:
            if self._icp_xfrm is None:
                self._icp_xfrm = self.ax_icp.scatter(icp_prev_tf_pts[:,0], icp_prev_tf_pts[:,1], s=3, c="#2ca02c", label="xformed prev")
            else:
                self._icp_xfrm.set_offsets(icp_prev_tf_pts)
        # Legend (once available)
        if self.ax_icp.get_legend_handles_labels()[0]:
            self.ax_icp.legend()
        # Draw
        plt.draw(); plt.pause(self.cfg.pause_s); self.fig.canvas.flush_events()

# -----------------------------------------------------------------------------
# CSV Logger
# -----------------------------------------------------------------------------
class CsvLogger:
    """CSV logger that writes robot pose and LiDAR scan each tick.
    On construction, any existing CSVs are deleted so each run starts fresh.
    """
    def __init__(self, lidar_num_rays: int, cfg: LoggingConfig):
        self.pose_path = _Path(cfg.pose_csv); self.lidar_path = _Path(cfg.lidar_csv)
        self.num_rays = lidar_num_rays

        # Ensure folders exist
        if self.pose_path.parent: self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        if self.lidar_path.parent: self.lidar_path.parent.mkdir(parents=True, exist_ok=True)

        # Always start fresh: delete old files (if present), then write headers
        for p in (self.pose_path, self.lidar_path):
            try:
                if p.exists():
                    p.unlink()
            except Exception:
                # Fallback: truncate if unlink fails (e.g., permissions/locks)
                with p.open("w", newline=""):
                    pass

        with self.pose_path.open("w", newline="") as f:
            csv.writer(f).writerow(["step", "x_m", "y_m", "theta_deg", "mode"])

        with self.lidar_path.open("w", newline="") as f:
            headers = ["step"] + [h for i in range(self.num_rays)
                                  for h in (f"angle_deg_{i}", f"dist_m_{i}")]
            csv.writer(f).writerow(headers)

    def write(self, step: int, pose: Pose, scan: Scan, mode: str) -> None:
        with self.pose_path.open("a", newline="") as f:
            csv.writer(f).writerow([step, pose.x, pose.y, math.degrees(pose.theta), mode])

        row = [step]
        for a, d in zip(scan.angles, scan.ranges):
            row.append(math.degrees(a)); row.append(d)
        with self.lidar_path.open("a", newline="") as f:
            csv.writer(f).writerow(row)

# -----------------------------------------------------------------------------
# Random seed paging (single figure, closed after selection)
# -----------------------------------------------------------------------------
THUMB_FIG: Optional[Figure] = None

def propose_random_mazes(cfg_world: WorldConfig, cfg: RandomMazeConfig, entrance: Cell, goal: Cell, viz_cfg: VizConfig) -> Tuple[List[int], int]:
    seeds: List[int] = []; walls_per_seed: List[List[Wall]] = []
    s = cfg.seed_scan_start; attempts = 0
    while len(seeds) < cfg.candidates_to_list and attempts < cfg.max_attempts_per_page:
        attempts += 1
        try:
            local = RandomMazeConfig(
                size_m=cfg.size_m, cell_size_m=cfg.cell_size_m, random_wall_count=cfg.random_wall_count, random_seed=s,
                candidates_to_list=cfg.candidates_to_list, seed_scan_start=s, seed_scan_stride=cfg.seed_scan_stride,
                max_attempts_per_page=cfg.max_attempts_per_page, segment_len_cells_min=cfg.segment_len_cells_min,
                segment_len_cells_max=cfg.segment_len_cells_max, orientation_bias=cfg.orientation_bias
            )
            maze = MazeRandom(cfg_world, local)
            if maze.path_exists(start=entrance, goal=goal):
                seeds.append(s); walls_per_seed.append(maze.walls)
        except Exception as e:
            logger.debug("Seed %d generation failed: %s", s, e)
        s += cfg.seed_scan_stride
    global THUMB_FIG
    if seeds:
        cols = len(seeds)
        if THUMB_FIG is None or not plt.fignum_exists(THUMB_FIG.number):
            THUMB_FIG = plt.figure(num="Random maze candidates", figsize=(viz_cfg.thumb_size_in[0] * cols, viz_cfg.thumb_size_in[1]))
        else:
            THUMB_FIG.clf(); THUMB_FIG.set_size_inches(viz_cfg.thumb_size_in[0] * cols, viz_cfg.thumb_size_in[1], forward=True)
        axs = THUMB_FIG.subplots(1, cols, squeeze=False)[0]
        for ax, seed, walls in zip(axs, seeds, walls_per_seed):
            ax.set_xlim(0, cfg.size_m); ax.set_ylim(0, cfg.size_m); ax.set_xticks([]); ax.set_yticks([]); ax.set_aspect("equal")
            for w in walls:
                minx, maxx, miny, maxy = w
                ax.add_patch(Rectangle((minx, miny), maxx - minx, maxy - miny, facecolor="black"))
            cell = cfg.cell_size_m
            ax.add_patch(Rectangle((entrance[0] * cell, entrance[1] * cell), cell, cell, facecolor="green", alpha=0.3))
            ax.add_patch(Rectangle((goal[0] * cell, goal[1] * cell), cell, cell, facecolor="red", alpha=0.3))
            ax.set_title(f"seed {seed}")
        THUMB_FIG.suptitle("Random maze candidates (path_exists=True)"); THUMB_FIG.tight_layout()
        plt.draw(); plt.pause(0.01)
        print("Candidate seeds:", ", ".join(map(str, seeds)))
    else:
        print("No valid random mazes found on this page.")
    return seeds, s

# -----------------------------------------------------------------------------
# Robot interface (SIM or REAL)
# -----------------------------------------------------------------------------
SIM_MODE = True

class _Context:
    world: Optional[_MazeBase] = None
    sensor: Optional["LidarSimulator"] = None
    pose: Optional[Pose] = None
    robot_cfg: Optional[RobotConfig] = None
    ctrl_cfg: Optional[SimpleControllerConfig] = None

CTX = _Context()

def simulate_kinematics_step(current: Pose, setpoint: PoseSetpoint, dt: float, v_max: float, k_ang: float, dt_guard: float) -> Pose:
    des_heading = math.atan2(setpoint.y - current.y, setpoint.x - current.x)
    heading_err = (des_heading - current.theta + math.pi) % (2 * math.pi) - math.pi
    omega = k_ang * heading_err
    dist = math.hypot(setpoint.x - current.x, setpoint.y - current.y)
    v = min(v_max, max(0.0, dist / max(dt_guard, dt)))
    nx = current.x + v * math.cos(current.theta) * dt
    ny = current.y + v * math.sin(current.theta) * dt
    nth = (current.theta + omega * dt) % (2 * math.pi)
    return Pose(nx, ny, nth)

def get_robot_pose() -> Pose:
    assert CTX.pose is not None, "CTX.pose not initialized"
    return CTX.pose

def get_lidar_scan() -> Scan:
    assert CTX.pose is not None and CTX.sensor is not None and CTX.world is not None, "Robot context not ready"
    return CTX.sensor.scan(CTX.pose, CTX.world)

def send_setpoint(sp: PoseSetpoint) -> None:
    if not SIM_MODE: return
    assert CTX.pose is not None and CTX.robot_cfg is not None and CTX.ctrl_cfg is not None, "CTX not initialized"
    CTX.pose = simulate_kinematics_step(
        CTX.pose, sp, CTX.robot_cfg.dt_s, min(CTX.robot_cfg.v_max_mps, CTX.ctrl_cfg.v_max_mps), CTX.ctrl_cfg.k_ang, CTX.robot_cfg.dt_guard_s
    )

# -----------------------------------------------------------------------------
# App Pipeline (with ICP + gated fusion; v8 fix to render true prev vs curr)
# -----------------------------------------------------------------------------
@dataclass(slots=True)
class AppPipeline:
    world: _MazeBase
    sensor: LidarSimulator
    mapper: OccupancyGridMap
    controller: object  # SimpleController or ManualController
    viz: MatplotlibViz
    logger: "CsvLogger"
    app_cfg: AppConfig
    goal: Goal
    entrance: Cell
    global_path_xy: List[Tuple[float, float]]
    astar_pts: Optional[Tuple[List[float], List[float]]]
    cfg: ProjectConfig
    icp_enabled: bool = False
    icp_prev_pts: Optional[np.ndarray] = None
    icp_prev_pose: Optional[Pose] = None
    step: int = 0

    def __post_init__(self):
        self.icp_enabled = bool(PR_HAS_ICP)

    def _scan_to_points_world(self, pose: Pose, scan: Scan) -> np.ndarray:
        xs = []; ys = []; maxr = self.cfg.lidar.max_range_m
        for a, d in zip(scan.angles, scan.ranges):
            if d <= 0.0 or not math.isfinite(d) or d >= maxr - 1e-6:
                continue
            x = pose.x + d * math.cos(pose.theta + a)
            y = pose.y + d * math.sin(pose.theta + a)
            xs.append(x); ys.append(y)
        if not xs: return np.zeros((0, 2), dtype=float)
        return np.column_stack([xs, ys])

    def _icp_estimate_pose(self, prev_pts: np.ndarray, curr_pts: np.ndarray, prev_pose: Pose) -> Tuple[Optional[Pose], Optional[float], int]:
        if not PR_HAS_ICP: return None, None, 0
        try:
            out = PR_icp_matching(prev_pts.T, curr_pts.T)
            if isinstance(out, tuple) and len(out) == 3 and all(isinstance(v, (int, float)) for v in out):
                yaw, tx, ty = float(out[0]), float(out[1]), float(out[2])
            elif isinstance(out, tuple) and len(out) in (2, 3):
                R, t = out[0], out[1]
                yaw = math.atan2(R[1, 0], R[0, 0]); tx, ty = float(t[0]), float(t[1])
            else:
                return None, None, 0
            x_est = prev_pose.x + tx; y_est = prev_pose.y + ty; th_est = (prev_pose.theta + yaw) % (2 * math.pi)
            # NN RMSE for gating
            try:
                c, s = math.cos(yaw), math.sin(yaw); Rm = np.array([[c, -s], [s, c]])
                prev_tf = prev_pts @ Rm.T + np.array([tx, ty])
                if curr_pts.shape[0] > 0 and prev_tf.shape[0] > 0:
                    d2 = ((prev_tf[:, None, :] - curr_pts[None, :, :]) ** 2).sum(axis=2)
                    min_d2 = d2.min(axis=1); rmse = float(np.sqrt(min_d2.mean())); n_pairs = int(prev_tf.shape[0])
                else:
                    rmse = None; n_pairs = 0
            except Exception:
                rmse = None; n_pairs = 0
            return Pose(x_est, y_est, th_est), rmse, n_pairs
        except Exception as e:
            logger.debug("ICP failed: %s", e); return None, None, 0

    def _ang_diff(self, a: float, b: float) -> float:
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _maybe_fuse_pose(self, pose: Pose, icp_pose: Optional[Pose], rmse: Optional[float], n_pts: int) -> Optional[Pose]:
        if not self.cfg.fusion.enabled or icp_pose is None: return None
        f = self.cfg.fusion
        if n_pts < f.min_points: return None
        if (rmse is None) or (rmse > f.max_rmse_m): return None
        dt = math.hypot(icp_pose.x - pose.x, icp_pose.y - pose.y); dr = abs(self._ang_diff(icp_pose.theta, pose.theta))
        if dt > f.max_trans_m or math.degrees(dr) > f.max_rot_deg: return None
        if dt < f.snap_trans_m and math.degrees(dr) < f.snap_rot_deg: return icp_pose
        a = max(0.0, min(1.0, f.alpha))
        x = (1 - a) * pose.x + a * icp_pose.x; y = (1 - a) * pose.y + a * icp_pose.y
        th = pose.theta + a * self._ang_diff(icp_pose.theta, pose.theta); th = (th + 2 * math.pi) % (2 * math.pi)
        return Pose(x, y, th)

    def _local_target_for_manual(self, pose: Pose) -> Tuple[float, float]:
        la = self.cfg.controller.manual_preview_lookahead_m
        if not self.global_path_xy: return (pose.x, pose.y)
        dists = [math.hypot(px - pose.x, py - pose.y) for px, py in self.global_path_xy]
        ind = int(np.argmin(dists))
        L = 0.0; i = ind
        while L < la and i < len(self.global_path_xy) - 1:
            dx = self.global_path_xy[i + 1][0] - self.global_path_xy[i][0]
            dy = self.global_path_xy[i + 1][1] - self.global_path_xy[i][1]
            L += math.hypot(dx, dy); i += 1
        return self.global_path_xy[i]

    def _print_step(self, pose: Pose, local_target_m: Tuple[float, float], icp_pose: Optional[Pose] = None):
        mode_label = self.app_cfg.mode
        icp_txt = (f" | icp_pose=({icp_pose.x:.3f},{icp_pose.y:.3f},{math.degrees(icp_pose.theta):.1f}°)") if icp_pose else " | icp_pose=(n/a)"
        msg = (f"Step {self.step:05d} | mode={mode_label} | "
               f"pose=({pose.x:.3f},{pose.y:.3f},{math.degrees(pose.theta):.1f}°)"
               f"{icp_txt} | local_target=({local_target_m[0]:.3f},{local_target_m[1]:.3f})")
        #print(msg); 
        logger.info(msg)

    def tick(self) -> bool:
        pose = get_robot_pose()
        scan = get_lidar_scan()
        # ICP: keep prev handles for plotting; compute with true prev->curr
        icp_pose = None; icp_rmse = None; icp_npts = 0
        curr_pts = None
        prev_pts_for_plot = self.icp_prev_pts
        prev_pose_for_tf  = self.icp_prev_pose
        if self.icp_enabled:
            curr_pts = self._scan_to_points_world(pose, scan)
            if prev_pts_for_plot is not None and curr_pts is not None and curr_pts.size > 0 and prev_pts_for_plot.size > 0 and prev_pose_for_tf is not None:
                icp_pose, icp_rmse, icp_npts = self._icp_estimate_pose(prev_pts_for_plot, curr_pts, prev_pose_for_tf)
        # Build ICP debug cloud (transform prev by ICP delta) for plotting
        icp_prev_tf_pts = None
        if icp_pose is not None and prev_pts_for_plot is not None and prev_pose_for_tf is not None:
            dyaw = (icp_pose.theta - prev_pose_for_tf.theta + math.pi) % (2 * math.pi) - math.pi
            tx = icp_pose.x - prev_pose_for_tf.x; ty = icp_pose.y - prev_pose_for_tf.y
            c, s = math.cos(dyaw), math.sin(dyaw); Rm = np.array([[c, -s], [s, c]])
            icp_prev_tf_pts = prev_pts_for_plot @ Rm.T + np.array([tx, ty])
        # Optional fusion
        fused = self._maybe_fuse_pose(pose, icp_pose, icp_rmse, icp_npts)
        if fused is not None: CTX.pose = fused; pose = fused
        # Map & draw
        self.mapper.integrate(scan, pose)
        self.viz.render(self.world, self.mapper, pose, scan, self.goal, self.step,
                        path_xy=self.global_path_xy, entrance=self.entrance,
                        icp_prev_pts=prev_pts_for_plot, icp_curr_pts=curr_pts if self.icp_enabled else None,
                        icp_prev_tf_pts=icp_prev_tf_pts, astar_pts=self.astar_pts)
        # Advance ICP history after plotting so prev/curr are distinct on-screen
        if self.icp_enabled:
            self.icp_prev_pts = curr_pts
            self.icp_prev_pose = pose
        # Done?
        if self._at_goal(pose):
            print("Simulation complete: Robot reached the goal."); logger.info("Reached goal; stopping.")
            plt.show(block=True); return False
        # Decide & act
        if self.app_cfg.mode == "GOALSEEKING":
            setpoint = self.controller.compute(pose, scan)  # type: ignore[attr-defined]
            send_setpoint(setpoint)
            new_pose = get_robot_pose()
            self.logger.write(self.step, pose=new_pose, scan=scan, mode=self.app_cfg.mode)
            self._print_step(new_pose, (setpoint.x, setpoint.y), icp_pose)
            self.step += 1; return True
        # Manual
        new_pose = self.controller.decide(pose, scan, self.world, self.goal)  # type: ignore[attr-defined]
        moved = (new_pose.x != pose.x) or (new_pose.y != pose.y) or (new_pose.theta != pose.theta)
        if not moved: plt.pause(self.cfg.viz.pause_s); return True
        CTX.pose = new_pose
        local_target_m = self._local_target_for_manual(new_pose)
        self.logger.write(self.step, pose=new_pose, scan=scan, mode=self.app_cfg.mode)
        self._print_step(new_pose, local_target_m, icp_pose)
        self.step += 1; return True

    def _at_goal(self, pose: Pose) -> bool:
        gx, gy = cell_center_m(self.goal.cell, self.world.cell_size_m)
        return math.hypot(gx - pose.x, gy - pose.y) <= self.app_cfg.arrival_tol_m

# -----------------------------------------------------------------------------
# Keyboard manual controller
# -----------------------------------------------------------------------------
_LAST_KEY = None
def _on_key_press(event):
    global _LAST_KEY; _LAST_KEY = event.key

class ManualController:
    def __init__(self, robot_cfg: RobotConfig):
        self.cfg = robot_cfg; self.last_key: Optional[str] = None
    def on_key(self, event):
        if event.key in ["up", "down", "left", "right", "q"]: self.last_key = event.key
    def decide(self, pose: Pose, scan: Scan, world: _MazeBase, goal: Goal) -> Pose:
        if self.last_key is None: return pose
        key = self.last_key; self.last_key = None
        if key == 'q': return pose
        alpha = self.cfg.manual_forward_cone_rad
        front = [d for a, d in zip(scan.angles, scan.ranges) if (a <= alpha or a >= 2 * math.pi - alpha)]
        can_move = (not front) or min(front) > self.cfg.robot_radius_m
        step_m = self.cfg.manual_step_m
        if key == "left": return Pose(pose.x, pose.y, (pose.theta + self.cfg.turn_angle_rad) % (2 * math.pi))
        if key == "right": return Pose(pose.x, pose.y, (pose.theta - self.cfg.turn_angle_rad) % (2 * math.pi))
        if key == "up" and can_move: return Pose(pose.x + step_m * math.cos(pose.theta), pose.y + step_m * math.sin(pose.theta), pose.theta)
        if key == "down" and can_move: return Pose(pose.x - step_m * math.cos(pose.theta), pose.y - step_m * math.sin(pose.theta), pose.theta)
        return pose

# -----------------------------------------------------------------------------
# Build / wiring
# -----------------------------------------------------------------------------
def ask_user_options(defaults: ProjectConfig) -> AppConfig:
    while True:
        m = input("Choose control mode: [G]OALSEEKING or [M]ANUAL? ").strip().lower()
        if m in ("g", "goal", "goalseeking", "auto", "a", ""): mode = "GOALSEEKING"; break
        if m in ("m", "manual"): mode = "MANUAL"; break
        print("Please enter G or M.")
    while True:
        t = input("Choose map: [R]ANDOM or [S]NAKE? ").strip().lower()
        if t in ("r", "random", ""): map_type = "RANDOM"; break
        if t in ("s", "snake"): map_type = "SNAKE"; break
        print("Please enter R or S.")
    return AppConfig(arrival_tol_m=defaults.app.arrival_tol_m, mode=mode, map_type=map_type,
                     entrance_cell=defaults.app.entrance_cell, snake_goal_cell=defaults.app.snake_goal_cell,
                     random_goal_cell=defaults.app.random_goal_cell)

def build_world_and_plan(cfg: ProjectConfig, app_cfg: AppConfig):
    entrance = app_cfg.entrance_cell
    if app_cfg.map_type == "SNAKE":
        world = MazeSnake(cfg.world, cfg.snake)
        goal_cell = app_cfg.snake_goal_cell or (world.grid_size - 1, world.grid_size - 1)
        if not world.path_exists(entrance, goal_cell):
            raise RuntimeError("Snake maze has no valid path with current params.")
    else:
        goal_cell = app_cfg.random_goal_cell
        local_cfg = RandomMazeConfig(
            size_m=cfg.random.size_m, cell_size_m=cfg.random.cell_size_m, random_wall_count=cfg.random.random_wall_count,
            random_seed=None, candidates_to_list=cfg.random.candidates_to_list, seed_scan_start=cfg.random.seed_scan_start,
            seed_scan_stride=cfg.random.seed_scan_stride, max_attempts_per_page=cfg.random.max_attempts_per_page,
            segment_len_cells_min=cfg.random.segment_len_cells_min, segment_len_cells_max=cfg.random.segment_len_cells_max,
            orientation_bias=cfg.random.orientation_bias
        )
        while local_cfg.random_seed is None:
            seeds, next_start = propose_random_mazes(cfg.world, local_cfg, entrance, goal_cell, cfg.viz)
            sel = input(f"Enter a seed from {seeds} to proceed (ENTER for next page): ").strip()
            if sel == "": local_cfg.seed_scan_start = next_start; continue
            try:
                chosen = int(sel)
                if chosen in seeds: local_cfg.random_seed = chosen
                else: print("[WARN] Seed not in current list.")
            except Exception as e:
                print(f"[WARN] {e}")
        # Close thumbnails window
        global THUMB_FIG
        try:
            import matplotlib.pyplot as _plt
            if THUMB_FIG is not None: _plt.close(THUMB_FIG); THUMB_FIG = None
        except Exception: pass
        world = MazeRandom(cfg.world, local_cfg)
        if not world.path_exists(entrance, goal_cell):
            raise RuntimeError("Selected random maze has no valid path; choose another seed.")
    if not PR_HAS_ASTAR:
        raise ImportError("PythonRobotics A* not found. Set PYTHONROBOTICS_PATH to repo root.")
    planner = GlobalAStar(world, grid_step_m=cfg.planning.sample_step_m, robot_radius_m=cfg.robot.robot_radius_m)
    global_path_xy = planner.plan(entrance, goal_cell, cfg_planning=cfg.planning)
    astar_pts = planner.cspace_pts if planner.cspace_pts else planner.obstacles_xy
    return world, entrance, goal_cell, global_path_xy, astar_pts

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main() -> None:
    all_cfg = DEFAULTS
    app_cfg = ask_user_options(all_cfg)
    robot_cfg = all_cfg.robot; ctrl_cfg = all_cfg.controller
    world, entrance, goal_cell, global_path_xy, astar_pts = build_world_and_plan(all_cfg, app_cfg)
    lidar_cfg = all_cfg.lidar; ogm_cfg = all_cfg.ogm
    sensor = LidarSimulator(lidar_cfg)
    mapper = OccupancyGridMap(ogm_cfg, 0.0, 0.0, float(world.size_m), float(world.size_m))
    viz = MatplotlibViz(world.size_m, world.cell_size_m, all_cfg.viz, robot_radius_m=all_cfg.robot.robot_radius_m)
    logger_csv = CsvLogger(lidar_num_rays=lidar_cfg.num_rays, cfg=all_cfg.logging)
    # start pose
    x0, y0 = cell_center_m(entrance, world.cell_size_m); theta0 = 0.0
    if len(global_path_xy) >= 2:
        x1, y1 = global_path_xy[1]; theta0 = math.atan2((y1 - y0), (x1 - x0))
    start_pose = Pose(x=x0, y=y0, theta=theta0)
    # controller
    controller = SimpleController(robot_cfg, ctrl_cfg, world, global_path_xy) if app_cfg.mode == "GOALSEEKING" else ManualController(robot_cfg)
    goal = Goal(cell=goal_cell)
    # CTX
    global CTX
    CTX.world = world; CTX.sensor = sensor; CTX.pose = start_pose; CTX.robot_cfg = robot_cfg; CTX.ctrl_cfg = ctrl_cfg
    # Keyboard
    viz.fig.canvas.mpl_connect('key_press_event', _on_key_press)
    if isinstance(controller, ManualController):
        viz.fig.canvas.mpl_connect('key_press_event', controller.on_key)
    # Pipeline
    pipeline = AppPipeline(world=world, sensor=sensor, mapper=mapper, controller=controller, viz=viz,
                           logger=logger_csv, app_cfg=app_cfg, goal=goal, entrance=entrance,
                           global_path_xy=global_path_xy, astar_pts=astar_pts, cfg=all_cfg)
    print("Controls:"); 
    if app_cfg.mode == "MANUAL":
        print("  Arrow keys: Up/Down move, Left/Right rotate.  q: quit")
    else:
        print("  GOALSEEKING running (stateless setpoint controller).  q: quit")
    plt.ion(); running = True
    try:
        while running:
            key = globals().get('_LAST_KEY', None); globals()['_LAST_KEY'] = None
            if key == 'q':
                print("Quit requested."); break
            running = pipeline.tick()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Done."); plt.close('all')

if __name__ == "__main__":
    main()
