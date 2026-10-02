"""Plain-Python geometry used to turn robot trajectories into floor plans.

Conventions: world coordinates are metres with the dock at the origin,
x to the right and y up (the robot's own odometry frame). Raster cells are
integer pairs (i, j) covering [i*res, (i+1)*res) x [j*res, (j+1)*res).
"""

import math

# -- polygons ---------------------------------------------------------------


def polygon_area(pts):
    """Signed area (positive = counter-clockwise)."""
    a = 0.0
    n = len(pts)
    for k in range(n):
        x1, y1 = pts[k]
        x2, y2 = pts[(k + 1) % n]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def point_in_polygon(x, y, pts):
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def polygon_bounds(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def _point_segment_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_segment_distance(p, a, b):
    return _point_segment_dist(p, a, b)


def _dp(pts, tol):
    if len(pts) < 3:
        return list(pts)
    first, last = pts[0], pts[-1]
    idx, dmax = 0, -1.0
    for k in range(1, len(pts) - 1):
        d = _point_segment_dist(pts[k], first, last)
        if d > dmax:
            idx, dmax = k, d
    if dmax <= tol:
        return [first, last]
    left = _dp(pts[: idx + 1], tol)
    right = _dp(pts[idx:], tol)
    return left[:-1] + right


def simplify_closed(pts, tol):
    """Douglas-Peucker for a closed ring (no repeated closing point)."""
    pts = remove_collinear(pts)
    if len(pts) <= 4 or tol <= 0:
        return pts
    # split the ring at the two points farthest apart
    a = 0
    b = max(range(len(pts)), key=lambda k: (pts[k][0] - pts[a][0]) ** 2 + (pts[k][1] - pts[a][1]) ** 2)
    a = max(range(len(pts)), key=lambda k: (pts[k][0] - pts[b][0]) ** 2 + (pts[k][1] - pts[b][1]) ** 2)
    if a > b:
        a, b = b, a
    first = _dp(pts[a: b + 1], tol)
    second = _dp(pts[b:] + pts[: a + 1], tol)
    out = first[:-1] + second[:-1]
    return out if len(out) >= 3 else pts


def remove_collinear(pts):
    out = list(pts)
    changed = True
    while changed and len(out) > 3:
        changed = False
        keep = []
        n = len(out)
        for k in range(n):
            px, py = out[k - 1]
            cx, cy = out[k]
            nx, ny = out[(k + 1) % n]
            cross = (cx - px) * (ny - cy) - (cy - py) * (nx - cx)
            if abs(cross) < 1e-12 or (cx, cy) == (px, py):
                changed = True
                continue
            keep.append(out[k])
        if len(keep) < 3:
            break
        out = keep
    return out


def dominant_angle(pts):
    """Length-weighted dominant wall direction, in [0, pi/2)."""
    bins = [0.0] * 90
    n = len(pts)
    for k in range(n):
        (x1, y1), (x2, y2) = pts[k], pts[(k + 1) % n]
        length = math.hypot(x2 - x1, y2 - y1)
        deg = int(round(math.degrees(math.atan2(y2 - y1, x2 - x1)))) % 90
        bins[deg] += length
    # smooth over +-2 degrees so ragged walls still vote together
    best = max(range(90), key=lambda d: sum(bins[(d + o) % 90] for o in range(-2, 3)))
    # refine: length-weighted mean of the edges near that direction
    sx = sy = 0.0
    for k in range(n):
        (x1, y1), (x2, y2) = pts[k], pts[(k + 1) % n]
        length = math.hypot(x2 - x1, y2 - y1)
        a = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 90
        diff = (a - best + 45) % 90 - 45
        if abs(diff) <= 4:
            w = length
            sx += w * math.cos(math.radians(diff) * 4)
            sy += w * math.sin(math.radians(diff) * 4)
    if sx or sy:
        return math.radians((best + math.degrees(math.atan2(sy, sx)) / 4) % 90)
    return math.radians(best)


def _intersect(l1, l2):
    (p, d), (q, e) = l1, l2
    den = d[0] * e[1] - d[1] * e[0]
    if abs(den) < 1e-9:
        return None
    t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
    return p[0] + t * d[0], p[1] + t * d[1]


def orthogonalize(pts, angle_tol_deg=15.0, theta=None, chamfer_m=0.45):
    """Snap walls that are nearly parallel/perpendicular to the main direction.

    Rooms are mostly rectangular; the robot's swept area is not. Edges within
    angle_tol of the dominant axes become exactly axis-aligned (in the
    rotated frame) and corners are recomputed as line intersections.
    """
    if len(pts) < 4:
        return pts
    theta = dominant_angle(pts) if theta is None else theta
    c, s = math.cos(-theta), math.sin(-theta)
    rot = [(c * x - s * y, s * x + c * y) for x, y in pts]
    tol = math.radians(angle_tol_deg)
    n = len(rot)
    lines = []
    for k in range(n):
        (x1, y1), (x2, y2) = rot[k], rot[(k + 1) % n]
        ang = math.atan2(y2 - y1, x2 - x1) % math.pi
        if min(ang, math.pi - ang) <= tol:
            lines.append((((x1 + x2) / 2, (y1 + y2) / 2), (1.0, 0.0), "h"))
        elif abs(ang - math.pi / 2) <= tol:
            lines.append((((x1 + x2) / 2, (y1 + y2) / 2), (0.0, 1.0), "v"))
        else:
            lines.append(((x1, y1), (x2 - x1, y2 - y1), "d"))
    # The robot body can't reach into corners, so a corner shows up as a short
    # diagonal between two walls: drop those and let the walls meet.
    kinds = [l[2] for l in lines]
    keep = []
    for k in range(n):
        (x1, y1), (x2, y2) = rot[k], rot[(k + 1) % n]
        short = math.hypot(x2 - x1, y2 - y1) <= chamfer_m
        if kinds[k] == "d" and short and kinds[k - 1] in "hv" and kinds[(k + 1) % n] in "hv":
            continue
        keep.append(k)
    if len(keep) >= 3:
        lines = [lines[k] for k in keep]
        rot_starts = keep
    else:
        rot_starts = list(range(n))
    n_lines = len(lines)
    # merge runs of same-orientation axis edges into one wall
    merged = []
    def same_wall(a, b):
        if a[2] != b[2] or a[2] not in "hv":
            return False
        axis = 1 if a[2] == "h" else 0
        return abs(a[0][axis] - b[0][axis]) < 0.12

    for li in range(n_lines):
        line, k = lines[li], rot_starts[li]
        if merged and same_wall(merged[-1][0], line):
            merged[-1][1].append(k)
        else:
            merged.append((line, [k]))
    if len(merged) > 1 and same_wall(merged[0][0], merged[-1][0]):
        last = merged.pop()
        merged[0] = (merged[0][0], last[1] + merged[0][1])
    walls = []
    for line, idxs in merged:
        if line[2] == "d":
            walls.append((line[0], line[1], idxs))
            continue
        tot = 0.0
        acc = 0.0
        for k in idxs:
            (x1, y1), (x2, y2) = rot[k], rot[(k + 1) % n]
            w = math.hypot(x2 - x1, y2 - y1) + 1e-9
            acc += w * ((y1 + y2) / 2 if line[2] == "h" else (x1 + x2) / 2)
            tot += w
        v = acc / tot
        walls.append((((0.0, v) if line[2] == "h" else (v, 0.0)), line[1], idxs))
    if len(walls) < 3:
        return pts
    out = []
    m = len(walls)
    for k in range(m):
        a, b = walls[k - 1], walls[k]
        corner = rot[b[2][0]]  # original vertex where wall b starts
        p = _intersect((a[0], a[1]), (b[0], b[1]))
        if p is None or math.hypot(p[0] - corner[0], p[1] - corner[1]) > 0.5:
            # parallel walls with a step between them: connect with a short jog
            pa = _project(corner, a)
            pb = _project(corner, b)
            out.extend([pa, pb])
        else:
            out.append(p)
    c, s = math.cos(theta), math.sin(theta)
    res = [(round(c * x - s * y, 3), round(s * x + c * y, 3)) for x, y in out]
    res = remove_collinear(_dedupe(res))
    if len(res) < 3 or abs(polygon_area(res)) < 0.5 * abs(polygon_area(pts)):
        return pts
    return res


def _project(p, wall):
    (qx, qy), (dx, dy), _ = wall
    t = ((p[0] - qx) * dx + (p[1] - qy) * dy) / (dx * dx + dy * dy)
    return qx + t * dx, qy + t * dy


def _dedupe(pts, eps=0.02):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= eps:
        out.pop()
    return out


# -- rasters ----------------------------------------------------------------


def disc_offsets(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
            if di * di + dj * dj <= radius_cells * radius_cells + 1e-9]


def cells_in_disc(x, y, radius, res):
    """Raster cells whose centres lie within `radius` of (x, y)."""
    out = []
    i0, i1 = int(math.floor((x - radius) / res)), int(math.floor((x + radius) / res))
    j0, j1 = int(math.floor((y - radius) / res)), int(math.floor((y + radius) / res))
    r2 = radius * radius
    for i in range(i0, i1 + 1):
        cx = (i + 0.5) * res - x
        for j in range(j0, j1 + 1):
            cy = (j + 0.5) * res - y
            if cx * cx + cy * cy <= r2:
                out.append((i, j))
    return out


def cells_along(x0, y0, x1, y1, radius, res):
    """Cells swept by a disc moving in a straight line."""
    dist = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(math.ceil(dist / (res * 0.75))))
    out = set()
    for k in range(steps + 1):
        t = k / steps
        out.update(cells_in_disc(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, radius, res))
    return out


def dilate(cells, offsets):
    out = set()
    for i, j in cells:
        for di, dj in offsets:
            out.add((i + di, j + dj))
    return out


def erode(cells, offsets):
    return {(i, j) for i, j in cells if all((i + di, j + dj) in cells for di, dj in offsets)}


def components(cells, eight=True):
    nbrs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if eight:
        nbrs += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    seen = set()
    comps = []
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack, comp = [start], []
        while stack:
            i, j = stack.pop()
            comp.append((i, j))
            for di, dj in nbrs:
                n = (i + di, j + dj)
                if n in cells and n not in seen:
                    seen.add(n)
                    stack.append(n)
        comps.append(comp)
    return comps


def fill_small_holes(cells, max_hole_cells):
    """Fill enclosed gaps up to max_hole_cells; returns (cells, big_holes)."""
    if not cells:
        return set(cells), []
    i0 = min(i for i, _ in cells) - 1
    i1 = max(i for i, _ in cells) + 1
    j0 = min(j for _, j in cells) - 1
    j1 = max(j for _, j in cells) + 1
    background = {(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1) if (i, j) not in cells}
    filled = set(cells)
    holes = []
    for comp in components(background, eight=False):
        touches_edge = any(i in (i0, i1) or j in (j0, j1) for i, j in comp)
        if touches_edge:
            continue
        if len(comp) <= max_hole_cells:
            filled.update(comp)
        else:
            holes.append(comp)
    return filled, holes


_LEFT = {(1, 0): (0, 1), (0, 1): (-1, 0), (-1, 0): (0, -1), (0, -1): (1, 0)}
_RIGHT = {v: k for k, v in _LEFT.items()}


def trace_loops(cells):
    """Boundary loops of a cell set, in grid-vertex coordinates.

    Outer boundaries come out counter-clockwise, holes clockwise.
    """
    edges = {}
    for i, j in cells:
        if (i, j - 1) not in cells:
            edges.setdefault((i, j), []).append((i + 1, j))
        if (i + 1, j) not in cells:
            edges.setdefault((i + 1, j), []).append((i + 1, j + 1))
        if (i, j + 1) not in cells:
            edges.setdefault((i + 1, j + 1), []).append((i, j + 1))
        if (i - 1, j) not in cells:
            edges.setdefault((i, j + 1), []).append((i, j))
    loops = []
    while edges:
        start = next(iter(edges))
        loop = [start]
        cur = start
        prev_dir = None
        while True:
            outs = edges[cur]
            if len(outs) == 1 or prev_dir is None:
                nxt = outs[0]
            else:
                # at a pinch point turn left, keeping each loop simple
                wanted = [_LEFT[prev_dir], prev_dir, _RIGHT[prev_dir]]
                nxt = min(outs, key=lambda o: wanted.index((o[0] - cur[0], o[1] - cur[1]))
                          if (o[0] - cur[0], o[1] - cur[1]) in wanted else 9)
            outs.remove(nxt)
            if not outs:
                del edges[cur]
            prev_dir = (nxt[0] - cur[0], nxt[1] - cur[1])
            cur = nxt
            if cur == start:
                break
            loop.append(cur)
        loops.append(loop)
    return loops


def extract_polygons(cells, res, simplify_tol, min_area, hole_min_area, square=True):
    """Turn a raster into (outer polygons, hole polygons) in world metres."""
    outers, holes = [], []
    loops = [[(i * res, j * res) for i, j in loop] for loop in trace_loops(cells)]
    # one dominant direction for the whole home keeps walls of rooms parallel
    theta = None
    if square and loops:
        theta = dominant_angle(max(loops, key=lambda l: abs(polygon_area(l))))
    for pts in loops:
        area = polygon_area(pts)
        pts = simplify_closed(pts, simplify_tol)
        if square and len(pts) >= 4:
            pts = orthogonalize(pts, theta=theta)
        if len(pts) < 3:
            continue
        pts = [(round(x, 3), round(y, 3)) for x, y in pts]
        if area > 0 and area >= min_area:
            outers.append(pts)
        elif area < 0 and -area >= hole_min_area:
            holes.append(list(reversed(pts)))  # store every polygon CCW
    return outers, holes


# -- frames & alignment -------------------------------------------------------


def transform(x, y, tf):
    dx, dy, th = tf
    c, s = math.cos(th), math.sin(th)
    return c * x - s * y + dx, s * x + c * y + dy


def align(points, is_inside, max_shift=0.4, max_rot_deg=8.0, prefer_identity_margin=0.03):
    """Find the small rotation+shift that best puts `points` inside a known map.

    is_inside(x, y) -> bool tests the reference map. Returns (tf, score) where
    score is the fraction of points that land inside. The identity transform
    wins unless another is clearly better, because poses already share the
    dock-origin frame and should only need fixing for drift.
    """
    if not points:
        return (0.0, 0.0, 0.0), 0.0
    step = max(1, len(points) // 300)
    pts = points[::step]

    def score(tf):
        inside = sum(1 for x, y in pts if is_inside(*transform(x, y, tf))) / len(pts)
        # among equally good fits prefer the smallest correction
        return inside - 0.002 * (abs(tf[0]) + abs(tf[1]) + abs(math.degrees(tf[2])) / 10)

    identity = (0.0, 0.0, 0.0)
    base = score(identity)
    best_tf, best = identity, base

    def search(center, rot_span, rot_step, shift_span, shift_step):
        nonlocal best_tf, best
        cx, cy, cth = center
        nr = int(round(rot_span / rot_step))
        ns = int(round(shift_span / shift_step))
        for r in range(-nr, nr + 1):
            th = cth + r * rot_step
            for a in range(-ns, ns + 1):
                for b in range(-ns, ns + 1):
                    tf = (cx + a * shift_step, cy + b * shift_step, th)
                    s = score(tf)
                    if s > best + 1e-9:
                        best_tf, best = tf, s

    search(identity, math.radians(max_rot_deg), math.radians(2.0), max_shift, 0.1)
    search(best_tf, math.radians(2.0), math.radians(0.5), 0.1, 0.025)
    inside = lambda tf: sum(1 for x, y in pts if is_inside(*transform(x, y, tf))) / len(pts)
    if best - base < prefer_identity_margin:
        return identity, inside(identity)
    return best_tf, inside(best_tf)
