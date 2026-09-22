import heapq
import math
import random
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

    def point_is_free(self, x: float, y: float) -> bool:
        gx, gy = self.world_to_grid(x, y)
        return not self.is_occupied(gx, gy)

    def segment_is_free(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
    ) -> bool:
        distance = math.dist(start, end)
        sample_count = max(2, int(distance / (self.resolution / 4.0)))

        for index in range(sample_count + 1):
            ratio = index / sample_count
            x = start[0] + (end[0] - start[0]) * ratio
            y = start[1] + (end[1] - start[1]) * ratio

            if not self.point_is_free(x, y):
                return False

        return True


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


def can_move(
    grid: GridMap,
    current: Tuple[int, int],
    neighbor: Tuple[int, int],
) -> bool:
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
    came_from: dict,
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar_search(
    grid: GridMap,
    start: Tuple[int, int],
    goal: Tuple[int, int],
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


def nearest_node(
    nodes: List[TreeNode],
    point: Tuple[float, float],
) -> TreeNode:
    return min(
        nodes,
        key=lambda node: math.dist((node.x, node.y), point),
    )


def steer(
    start_node: TreeNode,
    target: Tuple[float, float],
    step_size: float,
) -> TreeNode:
    dx = target[0] - start_node.x
    dy = target[1] - start_node.y
    distance = math.hypot(dx, dy)

    if distance == 0.0:
        return TreeNode(
            x=start_node.x,
            y=start_node.y,
            theta=start_node.theta,
            parent=start_node,
            cost=start_node.cost,
        )

    scale = min(step_size, distance) / distance
    new_x = start_node.x + dx * scale
    new_y = start_node.y + dy * scale
    new_theta = math.atan2(dy, dx)
    new_cost = start_node.cost + math.dist(
        (start_node.x, start_node.y),
        (new_x, new_y),
    )

    return TreeNode(
        x=new_x,
        y=new_y,
        theta=new_theta,
        parent=start_node,
        cost=new_cost,
    )


def random_free_point(grid: GridMap) -> Tuple[float, float]:
    while True:
        x = random.uniform(
            grid.origin[0] + grid.resolution,
            grid.origin[0] + (grid.width - 1) * grid.resolution,
        )
        y = random.uniform(
            grid.origin[1] + grid.resolution,
            grid.origin[1] + (grid.height - 1) * grid.resolution,
        )

        if grid.point_is_free(x, y):
            return x, y


def reconstruct_rrt_path(goal_node: TreeNode) -> List[TreeNode]:
    path = []
    current = goal_node

    while current is not None:
        path.append(current)
        current = current.parent

    path.reverse()
    return path


def rrt_search(
    grid: GridMap,
    start: TreeNode,
    goal: TreeNode,
    max_iter: int = 3000,
    step_size: float = 0.5,
    goal_sample_rate: float = 0.15,
    goal_threshold: float = 0.5,
) -> Tuple[Optional[List[TreeNode]], List[TreeNode]]:
    if not grid.point_is_free(start.x, start.y):
        return None, []

    if not grid.point_is_free(goal.x, goal.y):
        return None, []

    nodes = [start]

    for _ in range(max_iter):
        if random.random() < goal_sample_rate:
            sample = (goal.x, goal.y)
        else:
            sample = random_free_point(grid)

        nearest = nearest_node(nodes, sample)
        new_node = steer(nearest, sample, step_size)

        if not grid.segment_is_free(
            (nearest.x, nearest.y),
            (new_node.x, new_node.y),
        ):
            continue

        nodes.append(new_node)

        distance_to_goal = math.dist(
            (new_node.x, new_node.y),
            (goal.x, goal.y),
        )

        if distance_to_goal <= goal_threshold:
            if grid.segment_is_free(
                (new_node.x, new_node.y),
                (goal.x, goal.y),
            ):
                goal.parent = new_node
                goal.cost = new_node.cost + distance_to_goal
                nodes.append(goal)

                return reconstruct_rrt_path(goal), nodes

    return None, nodes