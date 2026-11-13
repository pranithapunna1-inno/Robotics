"""
navigation.py - pure-Python geometry, sensing and path-planning helpers.

Nothing in here depends on Pygame, so the logic can be unit-tested without a
window (see test_logic.py).
"""

import heapq
import math
from collections import deque

# ----------------------------------------------------------------------------
# Basic geometry
# ----------------------------------------------------------------------------


def clamp(value, low, high):
    """Limit value to the range [low, high]."""
    return max(low, min(high, value))


def wrap_angle(angle_deg):
    """Wrap an angle to the range [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def circle_hits_rect(cx, cy, radius, rect):
    """True if a circle overlaps an (x, y, w, h) rectangle.

    Find the point of the rectangle closest to the circle centre; the circle
    overlaps if that point is nearer than the radius.
    """
    x, y, w, h = rect
    nearest_x = clamp(cx, x, x + w)
    nearest_y = clamp(cy, y, y + h)
    return (cx - nearest_x) ** 2 + (cy - nearest_y) ** 2 < radius ** 2


def circle_hits_any(cx, cy, radius, rects):
    return any(circle_hits_rect(cx, cy, radius, r) for r in rects)


def point_in_rect(px, py, rect):
    x, y, w, h = rect
    return x <= px <= x + w and y <= py <= y + h


def cast_ray(x, y, angle_deg, max_range, obstacles, width, height, step=3.0):
    """Distance from (x, y) to the first obstacle / wall along a ray.

    The ray is marched forward in small steps. It stops at the first point that
    is inside an obstacle or outside the arena. Returns max_range if clear.
    """
    dx = math.cos(math.radians(angle_deg))
    dy = math.sin(math.radians(angle_deg))
    dist = step
    while dist <= max_range:
        px, py = x + dx * dist, y + dy * dist
        if px < 0 or py < 0 or px > width or py > height:
            return dist
        if any(point_in_rect(px, py, r) for r in obstacles):
            return dist
        dist += step
    return max_range


# ----------------------------------------------------------------------------
# Grid map + A* path planning
# ----------------------------------------------------------------------------


class GridMap:
    """Occupancy grid used for A* planning.

    Every obstacle is "inflated" by `clearance` (robot radius + safety margin):
    a cell is blocked if a robot standing at the cell centre would touch an
    obstacle or the arena wall. That lets us plan for the robot as a single
    point, which keeps A* simple.
    """

    def __init__(self, width, height, obstacles, clearance, cell=10):
        self.width = width
        self.height = height
        self.cell = cell
        self.cols = math.ceil(width / cell)
        self.rows = math.ceil(height / cell)
        self.blocked = [[self._cell_blocked(c, r, obstacles, clearance)
                         for c in range(self.cols)] for r in range(self.rows)]

    def _cell_blocked(self, col, row, obstacles, clearance):
        cx, cy = self.cell_center(col, row)
        if (cx < clearance or cy < clearance or
                cx > self.width - clearance or cy > self.height - clearance):
            return True
        return circle_hits_any(cx, cy, clearance, obstacles)

    # -- conversions ---------------------------------------------------------
    def cell_center(self, col, row):
        return ((col + 0.5) * self.cell, (row + 0.5) * self.cell)

    def to_cell(self, x, y):
        col = clamp(int(x // self.cell), 0, self.cols - 1)
        row = clamp(int(y // self.cell), 0, self.rows - 1)
        return col, row

    def is_free(self, col, row):
        return (0 <= col < self.cols and 0 <= row < self.rows
                and not self.blocked[row][col])

    def is_free_point(self, x, y):
        return self.is_free(*self.to_cell(x, y))

    # -- helpers ---------------------------------------------------------------
    def nearest_free(self, cell):
        """Breadth-first search for the closest unblocked cell."""
        if self.is_free(*cell):
            return cell
        seen = {cell}
        queue = deque([cell])
        while queue:
            col, row = queue.popleft()
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (col + dc, row + dr)
                if nxt in seen or not (0 <= nxt[0] < self.cols and 0 <= nxt[1] < self.rows):
                    continue
                if self.is_free(*nxt):
                    return nxt
                seen.add(nxt)
                queue.append(nxt)
        return None

    def line_of_sight(self, a, b):
        """True if the straight segment a->b stays on free cells."""
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(dist / (self.cell * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            if not self.is_free_point(a[0] + (b[0] - a[0]) * t,
                                      a[1] + (b[1] - a[1]) * t):
                return False
        return True

    # -- A* ----------------------------------------------------------------------
    def find_path(self, start_xy, goal_xy):
        """Return a list of (x, y) waypoints from start to goal, or None."""
        start = self.nearest_free(self.to_cell(*start_xy))
        goal = self.nearest_free(self.to_cell(*goal_xy))
        if start is None or goal is None:
            return None

        def heuristic(c):  # octile distance: exact for an empty 8-way grid
            dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

        open_heap = [(heuristic(start), 0.0, start)]
        came_from = {}
        best_cost = {start: 0.0}
        while open_heap:
            _, cost, current = heapq.heappop(open_heap)
            if current == goal:
                break
            if cost > best_cost.get(current, float("inf")):
                continue  # stale heap entry
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (current[0] + dc, current[1] + dr)
                if not self.is_free(*nxt):
                    continue
                if dc and dr and not (self.is_free(current[0] + dc, current[1]) and
                                      self.is_free(current[0], current[1] + dr)):
                    continue  # forbid cutting diagonally past a blocked corner
                new_cost = cost + (math.sqrt(2) if dc and dr else 1.0)
                if new_cost < best_cost.get(nxt, float("inf")):
                    best_cost[nxt] = new_cost
                    came_from[nxt] = current
                    heapq.heappush(open_heap, (new_cost + heuristic(nxt), new_cost, nxt))
        else:
            return None  # open list exhausted: goal unreachable

        cells = [goal]
        while cells[-1] != start:
            cells.append(came_from[cells[-1]])
        cells.reverse()
        points = [self.cell_center(*c) for c in cells]
        points[-1] = goal_xy  # finish exactly on the goal
        return self.simplify(points)

    def simplify(self, points):
        """String-pulling: skip waypoints that have a clear straight line."""
        if len(points) <= 2:
            return points[1:] or points
        result = []
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self.line_of_sight(points[i], points[j]):
                j -= 1
            result.append(points[j])
            i = j
        return result
