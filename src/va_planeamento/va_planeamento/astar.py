"""Local planning with A* on an occupancy grid (no ROS, numpy only).

Steps:
  1. obstacles = occupied cells (and, optionally, the unknown ones);
  2. inflation: cells within `inflation_radius` of an obstacle become
     forbidden, so the car can be treated as a point;
  3. A* with 8 neighbors and Euclidean heuristic.
"""
import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .grid import UNKNOWN

NEIGHBORS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def inflate(obstacle, radius):
    """Mark as forbidden the cells within `radius` cells of an obstacle."""
    ny, nx = obstacle.shape
    forbidden = obstacle.copy()
    r = int(radius)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= radius * radius:
                # shift the obstacle mask by (dx, dy) cells
                forbidden[max(dy, 0):ny + min(dy, 0), max(dx, 0):nx + min(dx, 0)] |= \
                    obstacle[max(-dy, 0):ny + min(-dy, 0), max(-dx, 0):nx + min(-dx, 0)]
    return forbidden


def astar(forbidden, start, goal, heuristic_weight=1.0):
    """A* on a grid. start and goal are cells (ix, iy).

    Returns (path, expanded): path is the list of cells, or None.
    """
    ny, nx = forbidden.shape
    g = {start: 0.0}                  # cost from the start
    parent = {start: None}            # to rebuild the path
    open_list = [(0.0, start)]        # open list: (f, cell), ordered by f
    closed = set()                    # cells already expanded

    while open_list:
        _, current = heapq.heappop(open_list)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:           # reached: rebuild the path backwards
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            return path[::-1], len(closed)

        for dx, dy in NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < nx and 0 <= neighbor[1] < ny):
                continue
            if forbidden[neighbor[1], neighbor[0]] or neighbor in closed:
                continue
            new_g = g[current] + math.hypot(dx, dy)
            if new_g < g.get(neighbor, math.inf):
                g[neighbor] = new_g
                parent[neighbor] = current
                h = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(open_list, (new_g + heuristic_weight * h, neighbor))

    return None, len(closed)


@dataclass
class PlannerParams:
    inflation_radius: float = 1.0    # m: half the car width + margin
    heuristic_weight: float = 1.0    # 1 = optimal A*; > 1 = weighted A* (faster)
    unknown_is_free: bool = True     # do -1 cells count as free?
    corridor_y_min: float = 0.0      # m: road lateral limits, if known;
    corridor_y_max: float = 0.0      #    outside [y_min, y_max] is obstacle (equal = disabled)
    swerve_threshold: float = 0.3    # m: lateral deviation above which the status is DESVIO


@dataclass
class PlanResult:
    status: str                      # LIVRE, DESVIO or BLOQUEADO
    path_xy: list = field(default_factory=list)
    max_deviation: float = 0.0
    expanded: int = 0
    time_ms: float = 0.0
    forbidden: np.ndarray = None


def plan(grid, start_xy, goal_xy, p=PlannerParams()):
    """Plan from start_xy to goal_xy (m, in the grid frame)."""
    t0 = time.perf_counter()

    # 1. obstacles
    obstacles = grid.data >= 50
    if not p.unknown_is_free:
        obstacles |= grid.data == UNKNOWN
    if p.corridor_y_max > p.corridor_y_min:
        _, y = grid.to_meters(0, np.arange(grid.ny))
        obstacles |= ((y < p.corridor_y_min) | (y > p.corridor_y_max))[:, None]

    # 2. inflation
    forbidden = inflate(obstacles, p.inflation_radius / grid.res)

    # 3. A* from the car's cell to the free cell closest to the goal
    ix, iy, _ = grid.to_cell(*start_xy)
    start = (int(ix), int(iy))
    forbidden[start[1], start[0]] = False           # the car is there
    gx, gy, _ = grid.to_cell(*goal_xy)
    free = np.argwhere(~forbidden)                   # rows (iy, ix)
    k = np.argmin((free[:, 1] - gx) ** 2 + (free[:, 0] - gy) ** 2)
    goal = (int(free[k, 1]), int(free[k, 0]))

    path, expanded = astar(forbidden, start, goal, p.heuristic_weight)
    ms = (time.perf_counter() - t0) * 1000.0
    if path is None:
        return PlanResult('BLOQUEADO', [], 0.0, expanded, ms, forbidden)

    # 4. status: max lateral deviation from the straight line start -> goal
    xs, ys = grid.to_meters(*np.array(path).T)
    (ax, ay), (bx, by) = start_xy, grid.to_meters(*goal)
    length = math.hypot(bx - ax, by - ay) or 1.0
    deviation = float(np.max(np.abs((bx - ax) * (ay - ys) - (ax - xs) * (by - ay)) / length))
    status = 'DESVIO' if deviation > p.swerve_threshold else 'LIVRE'
    return PlanResult(status, list(zip(xs.tolist(), ys.tolist())),
                      deviation, expanded, ms, forbidden)
