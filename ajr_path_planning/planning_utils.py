import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class GridMap:
    width: int
    height: int
    resolution: float
    origin: Tuple[float, float]
    data: List[int] = field(default_factory=list)

    def index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return True

        return self.data[self.index(gx, gy)] > 50

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = self.origin[0] + (gx + 0.5) * self.resolution
        wy = self.origin[1] + (gy + 0.5) * self.resolution
        return wx, wy


@dataclass
class TreeNode:
    x: float
    y: float
    theta: float = 0.0
    parent: Optional["TreeNode"] = None
    cost: float = 0.0


NEIGHBORS = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (-1, -1, math.sqrt(2)),
]


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def can_move(grid: GridMap, current: Tuple[int, int], neighbor: Tuple[int, int]) -> bool:
    dx = neighbor[0] - current[0]
    dy = neighbor[1] - current[1]

    if grid.is_occupied(*neighbor):
        return False

    if dx != 0 and dy != 0:
        if grid.is_occupied(current[0] + dx, current[1]):
            return False

        if grid.is_occupied(current[0], current[1] + dy):
            return False

    return True


def reconstruct_path(
    came_from: dict, current: Tuple[int, int]
) -> List[Tuple[int, int]]:
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar_search(
    grid: GridMap, start: Tuple[int, int], goal: Tuple[int, int]
) -> Optional[List[Tuple[int, int]]]:
    if grid.is_occupied(*start) or grid.is_occupied(*goal):
        return None

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            return reconstruct_path(came_from, current)

        if current in closed:
            continue

        closed.add(current)

        for dx, dy, step_cost in NEIGHBORS:
            neighbor = (current[0] + dx, current[1] + dy)

            if not can_move(grid, current, neighbor):
                continue

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None


def rrt_search(
    grid: GridMap,
    start: TreeNode,
    goal: TreeNode,
    max_iter: int = 2000,
):
    raise NotImplementedError