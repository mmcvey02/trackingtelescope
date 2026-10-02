"""Coverage path planning on the grid map.

The robot repeatedly drives to the nearest cell that still needs cleaning.
Neighbours are explored with the current heading first, so ties are broken
in favour of driving straight; on an open floor this yields the familiar
back-and-forth "lawnmower" pattern, and it naturally fills in gaps left
behind obstacles that are discovered on the way.
"""

from collections import deque

HEADINGS = ("N", "E", "S", "W")
DIRS = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}


def direction_between(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    for name, vec in DIRS.items():
        if vec == (dx, dy):
            return name
    raise ValueError(f"cells {a} and {b} are not adjacent")


def _ordered_dirs(heading):
    if heading not in DIRS:
        return HEADINGS
    i = HEADINGS.index(heading)
    # straight ahead, then the two sides, then reverse
    return (HEADINGS[i], HEADINGS[(i + 1) % 4], HEADINGS[(i + 3) % 4], HEADINGS[(i + 2) % 4])


def bfs(grid, start, is_goal, heading=None):
    """Shortest passable path from start to the first cell satisfying is_goal.

    Returns the list of cells to visit (excluding start), [] if start itself
    is a goal, or None if no goal is reachable.
    """
    start = tuple(start)
    if is_goal(*start):
        return []
    parents = {start: None}
    queue = deque([(start, heading)])
    while queue:
        cell, h = queue.popleft()
        for d in _ordered_dirs(h):
            dx, dy = DIRS[d]
            nxt = (cell[0] + dx, cell[1] + dy)
            if nxt in parents or not grid.passable(*nxt):
                continue
            parents[nxt] = cell
            if is_goal(*nxt):
                path = [nxt]
                while parents[path[-1]] != start:
                    path.append(parents[path[-1]])
                path.reverse()
                return path
            queue.append((nxt, d))
    return None


def path_to(grid, start, goal, heading=None):
    goal = tuple(goal)
    if not grid.passable(*goal):
        return None
    return bfs(grid, start, lambda x, y: (x, y) == goal, heading)


def coverage_path(grid, start, heading, now, stale_after=0):
    """Path to the nearest cell (other than start) that still needs cleaning."""
    start = tuple(start)
    return bfs(
        grid,
        start,
        lambda x, y: (x, y) != start and grid.needs_cleaning(x, y, now, stale_after),
        heading,
    )
