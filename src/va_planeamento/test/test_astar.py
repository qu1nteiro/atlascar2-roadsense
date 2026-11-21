"""Tests for A* and the grid (run without ROS: python3 -m pytest src/va_planeamento)."""
import numpy as np

from va_planeamento.astar import PlannerParams, astar, plan
from va_planeamento.grid import FREE, OCCUPIED, LocalGrid


def free_grid():
    return LocalGrid(0.2, -2.0, 30.0, -6.0, 6.0, initial_value=FREE)


def test_straight_path_without_obstacles():
    r = plan(free_grid(), (0.0, 0.0), (20.0, 0.0))
    assert r.status == 'LIVRE'
    assert abs(r.path_xy[-1][0] - 20.0) < 0.3


def test_swerves_around_a_parked_car():
    g = free_grid()
    g.mark_polygon([(10, -1.9), (14.5, -1.9), (14.5, 0.2), (10, 0.2)], OCCUPIED)
    r = plan(g, (0.0, 0.0), (20.0, 0.0))
    assert r.status == 'DESVIO'
    ys = [y for x, y in r.path_xy if 10 <= x <= 14.5]
    assert min(ys) > 0.2 + 1.0 - 0.2      # passes on the left, with the inflation margin


def test_blocked_when_the_road_is_closed():
    g = free_grid()
    g.mark_polygon([(10, -6), (11, -6), (11, 6), (10, 6)], OCCUPIED)
    assert plan(g, (0.0, 0.0), (20.0, 0.0)).status == 'BLOQUEADO'


def test_corridor_prevents_going_around():
    g = free_grid()
    g.mark_polygon([(10, -3), (11, -3), (11, 3), (10, 3)], OCCUPIED)   # there is room outside
    assert plan(g, (0.0, 0.0), (20.0, 0.0)).status == 'DESVIO'
    p = PlannerParams(corridor_y_min=-3.5, corridor_y_max=3.5)
    assert plan(g, (0.0, 0.0), (20.0, 0.0), p).status == 'BLOQUEADO'


def test_inflated_heuristic_expands_less():
    g = free_grid()
    g.mark_polygon([(10, -3), (12, -3), (12, 3), (10, 3)], OCCUPIED)
    optimal = plan(g, (0.0, 0.0), (20.0, 0.0), PlannerParams(heuristic_weight=1.0))
    fast = plan(g, (0.0, 0.0), (20.0, 0.0), PlannerParams(heuristic_weight=3.0))
    assert fast.expanded < optimal.expanded


def test_astar_on_a_minimal_grid():
    forbidden = np.zeros((3, 3), dtype=bool)
    forbidden[1, 1] = True
    path, _ = astar(forbidden, (0, 1), (2, 1))
    assert path[0] == (0, 1) and path[-1] == (2, 1)
    assert (1, 1) not in path
