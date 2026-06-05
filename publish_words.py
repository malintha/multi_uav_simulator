#!/usr/bin/env python3.12
# ---------------------------------------------------------------------------
# Spell an arbitrary WORD with the 10 drones, one letter at a time.
#
#   python3.12 publish_kth.py HELLO
#   python3.12 publish_kth.py            # defaults to "KTH"
#
# How it works, per letter:
#   1. Build the letter as exactly 10 goal slots from a vector stroke-font
#      (resampled by arc length), with >= CLEARANCE between any two slots, and
#      the whole glyph tilted by TILT degrees so it is not edge-on.
#   2. Read the drones' current positions from TF.
#   3. Solve the optimal drone -> slot assignment with the Hungarian algorithm
#      (minimum total transit distance).
#   4. Publish the goals; wait TRANSIT_S for the drones to fly over.
#
# Goals are in the controller's NED frame (x = North, y = East, z = Down;
# negative z = up). A glyph point (h, v) maps to
#       x = GX + h*sin(TILT),   y = h*cos(TILT),   z = -(V_BASE + v)
#
# NOTE: ROS Jazzy's rclpy is built for Python 3.12 -- run with python3.12.
# ---------------------------------------------------------------------------

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

# ---- layout / behaviour knobs ---------------------------------------------
N          = 10               # drones (= slots per letter)
GX         = 2.0              # North/depth offset of the letter plane
V_BASE     = 1.5              # altitude of the bottom of a glyph (m)
TILT       = math.radians(30) # yaw tilt of the letter plane (so it has depth)
SCALE      = 2.5              # overall glyph size multiplier (bigger = more spacing)
CLEARANCE  = 0.75             # min spacing between slots BEFORE scaling (> safety R)
TRANSIT_S  = 10.0             # seconds to wait between letters (bigger -> longer transit)
STAGGER_S  = 0.2              # small interval between successive drone commands

# ---------------------------------------------------------------------------
# Vector stroke-font. Each glyph is a list of strokes; each stroke is a polyline
# of (h, v) vertices with h in [-1.5, 1.5] (width) and v in [0, 4.5] (height).
# These ARE the saved letter locations -- the 10 slots are sampled from them.
# ---------------------------------------------------------------------------
ALPHABET = {
    "A": [[(-1.5,0),(0,4.5),(1.5,0)], [(-0.85,1.9),(0.85,1.9)]],
    "B": [[(-1.5,0),(-1.5,4.5),(0.6,4.5),(1.3,3.6),(0.6,2.25),(-1.5,2.25)],
          [(0.6,2.25),(1.4,1.1),(0.5,0),(-1.5,0)]],
    "C": [[(1.3,3.7),(0.4,4.5),(-0.9,4.5),(-1.5,3.6),(-1.5,0.9),(-0.9,0),(0.4,0),(1.3,0.8)]],
    "D": [[(-1.5,0),(-1.5,4.5),(0.3,4.5),(1.3,3.4),(1.3,1.1),(0.3,0),(-1.5,0)]],
    "E": [[(1.3,4.5),(-1.5,4.5),(-1.5,0),(1.3,0)], [(-1.5,2.25),(0.8,2.25)]],
    "F": [[(1.3,4.5),(-1.5,4.5),(-1.5,0)], [(-1.5,2.25),(0.8,2.25)]],
    "G": [[(1.3,3.7),(0.4,4.5),(-0.9,4.5),(-1.5,3.6),(-1.5,0.9),(-0.9,0),
           (0.6,0),(1.3,0.8),(1.3,2.0),(0.4,2.0)]],
    "H": [[(-1.5,0),(-1.5,4.5)], [(1.5,0),(1.5,4.5)], [(-1.5,2.25),(1.5,2.25)]],
    "I": [[(-1.0,4.5),(1.0,4.5)], [(0,4.5),(0,0)], [(-1.0,0),(1.0,0)]],
    "J": [[(1.1,4.5),(1.1,1.0),(0.4,0),(-0.6,0),(-1.3,1.0)]],
    "K": [[(-1.5,0),(-1.5,4.5)], [(-1.5,2.1),(1.3,4.5)], [(-1.5,2.1),(1.3,0)]],
    "L": [[(-1.5,4.5),(-1.5,0),(1.3,0)]],
    "M": [[(-1.5,0),(-1.5,4.5),(0,2.1),(1.5,4.5),(1.5,0)]],
    "N": [[(-1.5,0),(-1.5,4.5),(1.5,0),(1.5,4.5)]],
    "O": [[(-1.5,1.0),(-1.5,3.5),(-0.7,4.5),(0.7,4.5),(1.5,3.5),(1.5,1.0),
           (0.7,0),(-0.7,0),(-1.5,1.0)]],
    "P": [[(-1.5,0),(-1.5,4.5),(0.6,4.5),(1.3,3.6),(0.6,2.5),(-1.5,2.5)]],
    "Q": [[(-1.5,1.0),(-1.5,3.5),(-0.7,4.5),(0.7,4.5),(1.5,3.5),(1.5,1.0),
           (0.7,0),(-0.7,0),(-1.5,1.0)], [(0.3,1.3),(1.5,0)]],
    "R": [[(-1.5,0),(-1.5,4.5),(0.6,4.5),(1.3,3.6),(0.6,2.5),(-1.5,2.5)],
          [(-0.1,2.5),(1.3,0)]],
    "S": [[(1.3,3.7),(0.4,4.5),(-0.9,4.5),(-1.5,3.7),(-0.9,2.4),(0.8,2.0),
           (1.3,1.0),(0.6,0),(-0.7,0),(-1.4,0.8)]],
    "T": [[(-1.5,4.5),(1.5,4.5)], [(0,4.5),(0,0)]],
    "U": [[(-1.5,4.5),(-1.5,1.0),(-0.8,0),(0.8,0),(1.5,1.0),(1.5,4.5)]],
    "V": [[(-1.5,4.5),(0,0),(1.5,4.5)]],
    "W": [[(-1.5,4.5),(-0.8,0),(0,2.4),(0.8,0),(1.5,4.5)]],
    "X": [[(-1.5,0),(1.5,4.5)], [(-1.5,4.5),(1.5,0)]],
    "Y": [[(-1.5,4.5),(0,2.3),(1.5,4.5)], [(0,2.3),(0,0)]],
    "Z": [[(-1.5,4.5),(1.5,4.5),(-1.5,0),(1.5,0)]],
}


# ---- glyph -> 10 slots ----------------------------------------------------
def _poly_len(poly):
    return sum(math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def _sample_polyline(poly, count):
    """`count` points evenly spaced by arc length along the polyline."""
    if count <= 1:
        return [poly[len(poly) // 2]]
    seg = [math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1)]
    total = sum(seg)
    if total < 1e-9:
        return [poly[0]] * count
    out = []
    for s in range(count):
        target = total * s / (count - 1)
        acc = 0.0
        for i, sl in enumerate(seg):
            if acc + sl >= target or i == len(seg) - 1:
                r = (target - acc) / sl if sl > 1e-9 else 0.0
                r = max(0.0, min(1.0, r))
                out.append((poly[i][0] + r * (poly[i + 1][0] - poly[i][0]),
                            poly[i][1] + r * (poly[i + 1][1] - poly[i][1])))
                break
            acc += sl
    return out


def _resample_glyph(strokes, n=N):
    """Distribute exactly n points across the strokes, by length (>=2 each)."""
    lens = [_poly_len(s) for s in strokes]
    total = sum(lens) or 1.0
    counts = [max(2, round(n * l / total)) for l in lens]
    while sum(counts) > n:
        cand = [k for k in range(len(counts)) if counts[k] > 2] or list(range(len(counts)))
        i = min(cand, key=lambda k: lens[k] / counts[k])
        counts[i] -= 1
    while sum(counts) < n:
        i = max(range(len(counts)), key=lambda k: lens[k] / counts[k])
        counts[i] += 1
    pts = []
    for s, c in zip(strokes, counts):
        pts += _sample_polyline(s, c)
    return pts


def _enforce_clearance(pts, clear=CLEARANCE, iters=60):
    """Push apart any pair closer than `clear` (in the glyph plane = 3D)."""
    p = [list(q) for q in pts]
    for _ in range(iters):
        moved = False
        for i in range(len(p)):
            for j in range(i + 1, len(p)):
                dx, dy = p[j][0] - p[i][0], p[j][1] - p[i][1]
                d = math.hypot(dx, dy)
                if d < clear:
                    if d < 1e-6:
                        dx, dy, d = 0.0, 1.0, 1.0
                    push = (clear - d) / 2.0 + 1e-3
                    ux, uy = dx / d, dy / d
                    p[i][0] -= ux * push; p[i][1] -= uy * push
                    p[j][0] += ux * push; p[j][1] += uy * push
                    moved = True
        if not moved:
            break
    return [(q[0], q[1]) for q in p]


def build_letter(ch):
    """Return 10 NED goals for character `ch`, or None if unknown."""
    strokes = ALPHABET.get(ch.upper())
    if strokes is None:
        return None
    glyph = _enforce_clearance(_resample_glyph(strokes, N), CLEARANCE)
    goals = []
    for (h, v) in glyph:
        hs, vs = h * SCALE, v * SCALE          # enlarge the glyph
        x = GX + hs * math.sin(TILT)
        y = hs * math.cos(TILT)
        z = -(V_BASE + vs)
        goals.append((x, y, z))
    return goals


# ---- Hungarian assignment (scipy if available, else built-in O(n^3)) ------
def hungarian(cost):
    try:
        from scipy.optimize import linear_sum_assignment
        r, c = linear_sum_assignment(cost)
        ans = [0] * len(cost)
        for ri, ci in zip(r, c):
            ans[ri] = ci
        return ans
    except Exception:
        pass
    n = len(cost)
    INF = float('inf')
    u = [0.0] * (n + 1); v = [0.0] * (n + 1)
    p = [0] * (n + 1); way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i; j0 = 0
        minv = [INF] * (n + 1); used = [False] * (n + 1)
        while True:
            used[j0] = True; i0 = p[j0]; delta = INF; j1 = -1
            for j in range(1, n + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur; way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]; j1 = j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta; v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]; p[j0] = p[j1]; j0 = j1
            if j0 == 0:
                break
    ans = [0] * n
    for j in range(1, n + 1):
        ans[p[j] - 1] = j - 1
    return ans


def _ned_to_map(p):
    return (p[0], -p[1], -p[2])      # NWU(map) <-> NED is the involution (x,-y,-z)


class WordSpeller(Node):
    def __init__(self):
        super().__init__('word_speller')
        self.pubs = [self.create_publisher(Point, f'/mavswarm2/robot_{i}/desired_state', 10)
                     for i in range(N)]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def current_positions_map(self):
        pos = [None] * N
        for i in range(N):
            try:
                t = self.tf_buffer.lookup_transform('map', f'robot_{i}/base_link', Time())
                tr = t.transform.translation
                pos[i] = (tr.x, tr.y, tr.z)
            except Exception:
                pos[i] = None
        return pos

    def assign_and_publish(self, ch, goals_ned):
        goals_map = [_ned_to_map(g) for g in goals_ned]
        cur = self.current_positions_map()
        n_known = sum(1 for c in cur if c is not None)
        cost = [[0.0] * N for _ in range(N)]
        for i in range(N):
            ci = cur[i] if cur[i] is not None else goals_map[i]
            for k in range(N):
                cost[i][k] = math.dist(ci, goals_map[k])
        assign = hungarian(cost)
        total = sum(cost[i][assign[i]] for i in range(N))
        self.get_logger().info(
            f"'{ch}': tf {n_known}/{N}, total dist {total:.2f} m, drone->slot {assign}")
        # issue commands one drone at a time with a small interval (not all at once)
        for i in range(N):
            g = goals_ned[assign[i]]
            m = Point(); m.x, m.y, m.z = g
            self.pubs[i].publish(m)
            self.spin_for(STAGGER_S)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    word = (sys.argv[1] if len(sys.argv) > 1 else "KTH")
    rclpy.init()
    node = WordSpeller()
    node.get_logger().info(f"Spelling: {word}")
    node.spin_for(3.0)                       # let TF fill / publishers match

    for ch in word:
        if ch == ' ':
            node.get_logger().info("(space)")
            node.spin_for(TRANSIT_S)
            continue
        goals = build_letter(ch)
        if goals is None:
            node.get_logger().warn(f"no glyph for '{ch}', skipping")
            continue
        node.assign_and_publish(ch, goals)
        node.spin_for(TRANSIT_S)

    node.get_logger().info(f"Done spelling '{word}'.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
