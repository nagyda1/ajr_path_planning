"""Kozos adatstrukturak es segedfuggvenyek az utvonaltervezo node-okhoz.

Ez a modul kulon fajlban tartja a fastruktura (RRT) es racs (A*) reprezentaciot,
kovetve a simple_random_trees mintaprojekt jo gyakorlatat.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class GridMap:
    """Egyszeru occupancy grid reprezentacio A* szamara."""

    width: int
    height: int
    resolution: float
    origin: Tuple[float, float]
    data: List[int] = field(default_factory=list)

    def is_occupied(self, x: int, y: int) -> bool:
        # TODO: implementalando - rack index alapjan occupancy lekerdezese
        raise NotImplementedError

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        # TODO: implementalando
        raise NotImplementedError

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        # TODO: implementalando
        raise NotImplementedError


@dataclass
class TreeNode:
    """RRT fa csomopont."""

    x: float
    y: float
    theta: float = 0.0
    parent: Optional["TreeNode"] = None
    cost: float = 0.0


def astar_search(grid: GridMap, start: Tuple[int, int], goal: Tuple[int, int]):
    """A* keresés a racson.

    TODO: implementalando - prioritasi sor (heapq) alapu A* kereses,
    Manhattan vagy euklideszi heurisztikaval.
    """
    raise NotImplementedError


def rrt_search(grid: GridMap, start: TreeNode, goal: TreeNode, max_iter: int = 2000):
    """RRT / RRT* kereses jarmu-kinematikai korlatokkal.

    TODO: implementalando - veletlen mintavetelezes, legkozelebbi csomopont
    kereses, kinematikailag ervenyes kapcsolat (pl. Dubins/Ackermann modell)
    ellenorzese, fa bovitese.
    """
    raise NotImplementedError
