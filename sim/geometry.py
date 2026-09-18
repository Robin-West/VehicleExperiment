"""Intersection geometry: approaches, movement paths, and conflict points.

One intersection where two perpendicular two-lane roads cross. Right-hand
driving. Everything is built from a single "from the South" template that is
rotated by 90-degree increments to produce all four approaches, which keeps the
geometry symmetric by construction.

A Path is stored as a densely-sampled polyline (x, y, heading, cumulative
arc-length). Sampling once and interpolating avoids per-segment trig bugs and
makes it trivial to find where two paths cross.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# --- fixed dimensions (metres) ---
LANE_W = 3.5                 # one lane width
HALF_BOX = LANE_W            # cross road is 2 lanes wide -> box spans [-LANE_W, LANE_W]
APPROACH_LEN = 140.0         # length of each approach road we simulate
R_RIGHT = LANE_W / 2         # tight right-turn radius
R_LEFT = 3 * LANE_W / 2      # wider left-turn radius
DS = 0.25                    # path sampling resolution

# movements
THROUGH, LEFT, RIGHT = "through", "left", "right"
MOVES = (THROUGH, LEFT, RIGHT)

# approaches, keyed by the compass direction the car comes FROM
ORIGINS = ("S", "E", "N", "W")
ORIGIN_ANGLE = {"S": 0.0, "E": 90.0, "N": 180.0, "W": 270.0}  # rotation of S template
OPPOSITE = {"S": "N", "N": "S", "E": "W", "W": "E"}
# perpendicular ("crossing") approaches for each origin
CROSS = {"S": ("E", "W"), "N": ("E", "W"), "E": ("S", "N"), "W": ("S", "N")}

# where each movement exits to (the compass side of the outgoing lane). Cars from
# different approaches can share an exit lane, so they must car-follow there.
DEST = {
    "S": {THROUGH: "N", LEFT: "W", RIGHT: "E"},
    "N": {THROUGH: "S", LEFT: "E", RIGHT: "W"},
    "E": {THROUGH: "W", LEFT: "S", RIGHT: "N"},
    "W": {THROUGH: "E", LEFT: "N", RIGHT: "S"},
}


def exit_progress(dest: str, x: float, y: float) -> float:
    """Distance travelled along the (axis-aligned) outgoing lane."""
    return {"N": y, "S": -y, "E": x, "W": -x}[dest]


def in_exit_region(dest: str, x: float, y: float) -> bool:
    """True once the vehicle has cleared the intersection box onto its exit lane."""
    if dest == "N":
        return y > HALF_BOX
    if dest == "S":
        return y < -HALF_BOX
    if dest == "E":
        return x > HALF_BOX
    return x < -HALF_BOX


def _rot(points: np.ndarray, deg: float) -> np.ndarray:
    """Rotate an (N,2) array of points by deg degrees CCW about the origin."""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    m = np.array([[c, -s], [s, c]])
    return points @ m.T


def _sample_line(p0, heading_deg, length):
    n = max(2, int(length / DS) + 1)
    d = np.linspace(0.0, length, n)
    h = math.radians(heading_deg)
    xs = p0[0] + d * math.cos(h)
    ys = p0[1] + d * math.sin(h)
    hd = np.full(n, heading_deg)
    return np.column_stack([xs, ys]), hd


def _sample_arc(center, radius, start_deg, sweep_deg):
    length = abs(math.radians(sweep_deg)) * radius
    n = max(2, int(length / DS) + 1)
    a = np.linspace(start_deg, start_deg + sweep_deg, n)
    ar = np.radians(a)
    xs = center[0] + radius * np.cos(ar)
    ys = center[1] + radius * np.sin(ar)
    # tangent heading: +90 deg from radial for CCW, -90 for CW
    hd = a + (90.0 if sweep_deg >= 0 else -90.0)
    return np.column_stack([xs, ys]), hd


@dataclass
class Path:
    origin: str
    move: str
    xy: np.ndarray          # (N,2)
    heading: np.ndarray     # (N,) degrees
    s: np.ndarray           # (N,) cumulative arc length
    stop_s: float           # arc length at the stop line

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def __post_init__(self):
        # precompute for a fast manual interpolation in point_at
        hr = np.radians(self.heading)
        self._cos = np.cos(hr)
        self._sin = np.sin(hr)

    def point_at(self, s: float):
        """Return (x, y, heading_rad) at arc length s (clamped to the path)."""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.s, s))
        if i <= 0:
            i = 1
        elif i >= len(self.s):
            i = len(self.s) - 1
        s0, s1 = self.s[i - 1], self.s[i]
        f = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
        xy0, xy1 = self.xy[i - 1], self.xy[i]
        x = xy0[0] + f * (xy1[0] - xy0[0])
        y = xy0[1] + f * (xy1[1] - xy0[1])
        cx = self._cos[i - 1] + f * (self._cos[i] - self._cos[i - 1])
        cy = self._sin[i - 1] + f * (self._sin[i] - self._sin[i - 1])
        return float(x), float(y), math.atan2(cy, cx)


def _build_template(move: str):
    """Points/headings for a movement coming from the South (heading +y)."""
    w = LANE_W
    L = APPROACH_LEN
    start = np.array([w / 2, -L])
    approach_len = L - w  # straight part up to the stop line at y = -w

    if move == THROUGH:
        xy, hd = _sample_line(start, 90.0, 2 * L)
    elif move == RIGHT:
        l1_xy, l1_h = _sample_line(start, 90.0, approach_len)          # up to stop line
        a_xy, a_h = _sample_arc((w, -w), R_RIGHT, 180.0, -90.0)        # clockwise
        l2_xy, l2_h = _sample_line((w, -w / 2), 0.0, L - w)            # exit east
        xy = np.vstack([l1_xy, a_xy, l2_xy])
        hd = np.concatenate([l1_h, a_h, l2_h])
    elif move == LEFT:
        l1_xy, l1_h = _sample_line(start, 90.0, approach_len)
        a_xy, a_h = _sample_arc((-w, -w), R_LEFT, 0.0, 90.0)           # counter-clockwise
        l2_xy, l2_h = _sample_line((-w, w / 2), 180.0, L - w)          # exit west
        xy = np.vstack([l1_xy, a_xy, l2_xy])
        hd = np.concatenate([l1_h, a_h, l2_h])
    else:
        raise ValueError(move)
    return xy, hd


def _cumlen(xy: np.ndarray) -> np.ndarray:
    d = np.sqrt((np.diff(xy, axis=0) ** 2).sum(axis=1))
    return np.concatenate([[0.0], np.cumsum(d)])


def build_paths() -> dict[tuple[str, str], Path]:
    """All 12 paths, keyed by (origin, move)."""
    paths: dict[tuple[str, str], Path] = {}
    for origin in ORIGINS:
        ang = ORIGIN_ANGLE[origin]
        for move in MOVES:
            xy, hd = _build_template(move)
            xy = _rot(xy, ang)
            hd = hd + ang
            s = _cumlen(xy)
            stop_s = float(np.interp(APPROACH_LEN - LANE_W, s, s))  # = approach_len arc
            # stop line is at arc length (APPROACH_LEN - LANE_W) on every path
            paths[(origin, move)] = Path(origin, move, xy, hd, s, APPROACH_LEN - LANE_W)
    return paths


def crossing_point(a: Path, b: Path):
    """Find where two paths cross inside the box.

    Returns (s_a, s_b, (x, y), gap) for the point of closest approach, or None
    if they never come within ~half a lane of each other (i.e. do not conflict).
    Only the portion of each path near the box is considered.
    """
    def mask(p: Path):
        near = (np.abs(p.xy[:, 0]) < 2 * LANE_W) & (np.abs(p.xy[:, 1]) < 2 * LANE_W)
        return near
    ma, mb = mask(a), mask(b)
    ia = np.where(ma)[0]
    ib = np.where(mb)[0]
    if len(ia) == 0 or len(ib) == 0:
        return None
    # brute-force nearest pair (paths are short near the box)
    best = None
    for i in ia:
        d = np.sqrt(((b.xy[ib] - a.xy[i]) ** 2).sum(axis=1))
        j_rel = int(np.argmin(d))
        dist = float(d[j_rel])
        if best is None or dist < best[3]:
            j = ib[j_rel]
            best = (float(a.s[i]), float(b.s[j]),
                    ((a.xy[i] + b.xy[j]) / 2), dist)
    if best is None or best[3] > LANE_W:
        return None
    return best[0], best[1], (float(best[2][0]), float(best[2][1])), best[3]


# --------------------------------------------------------------------------
# Roundabout geometry
# --------------------------------------------------------------------------
# A single-lane roundabout: four radial approaches meet a circle and all traffic
# circulates one way (counter-clockwise, right-hand driving). A movement is just
# how far around you go before exiting: right = quarter turn, through = half,
# left = three-quarter. Because everything circulates the same way, there are no
# crossing paths -- only entry merges and same-direction following.

R_CIRC = 12.0                # circulating lane centreline radius (m)
_LEG_DEG = {"S": 270.0, "E": 0.0, "N": 90.0, "W": 180.0}   # where each leg meets the circle


def build_roundabout_paths() -> dict[tuple[str, str], Path]:
    """All 12 roundabout paths, keyed by (origin, move).

    Each path = radial approach in -> CCW arc around the circle -> radial exit.
    Extra attributes are attached: entry_s / exit_s (arc lengths where the path
    joins and leaves the circle), phi0 (entry angle, radians) and R.
    """
    paths: dict[tuple[str, str], Path] = {}
    La = APPROACH_LEN
    R = R_CIRC
    off = LANE_W                      # inbound / outbound lanes to opposite sides
    r_give = R + 4.0                  # give-way radius: a waiting car sits clear of the ring
    rho_g = math.sqrt(r_give ** 2 - off ** 2)   # radial coord of the give-way point

    def uv(fdeg):
        r = math.radians(fdeg)
        return (math.cos(r), math.sin(r)), (-math.sin(r), math.cos(r))

    def ang(vec):
        return math.degrees(math.atan2(vec[1], vec[0]))

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    for origin in ORIGINS:
        for move in MOVES:
            dest = DEST[origin][move]
            f0, f1 = _LEG_DEG[origin], _LEG_DEG[dest]
            dphi = (f1 - f0) % 360.0
            if dphi == 0.0:
                dphi = 360.0
            u0, t0 = uv(f0)
            u1, t1 = uv(f1)
            ap_start = ((R + La) * u0[0] + off * t0[0], (R + La) * u0[1] + off * t0[1])
            G0 = (rho_g * u0[0] + off * t0[0], rho_g * u0[1] + off * t0[1])   # give-way in
            J0 = (R * u0[0], R * u0[1])                                       # ring join in
            J1 = (R * u1[0], R * u1[1])                                       # ring join out
            G1 = (rho_g * u1[0] - off * t1[0], rho_g * u1[1] - off * t1[1])   # give-way out
            ex_end = ((R + La) * u1[0] - off * t1[0], (R + La) * u1[1] - off * t1[1])

            ap_xy, ap_h = _sample_line(ap_start, f0 + 180.0, (R + La) - rho_g)   # approach in
            me_xy, me_h = _sample_line(G0, ang((J0[0] - G0[0], J0[1] - G0[1])),
                                       dist(G0, J0))                            # merge onto ring
            ar_xy, ar_h = _sample_arc((0.0, 0.0), R, f0, dphi)                  # circulate CCW
            mo_xy, mo_h = _sample_line(J1, ang((G1[0] - J1[0], G1[1] - J1[1])),
                                       dist(J1, G1))                            # merge off ring
            ex_xy, ex_h = _sample_line(G1, f1, (R + La) - rho_g)               # exit out

            xy = np.vstack([ap_xy, me_xy, ar_xy, mo_xy, ex_xy])
            hd = np.concatenate([ap_h, me_h, ar_h, mo_h, ex_h])
            s = _cumlen(xy)
            i_give = len(ap_xy) - 1
            i_ring = len(ap_xy) + len(me_xy) - 1
            i_exitring = len(ap_xy) + len(me_xy) + len(ar_xy) - 1
            p = Path(origin, move, xy, hd, s, float(s[i_give]))
            p.entry_s = float(s[i_give])          # give-way / yield line
            p.ring_s = float(s[i_ring])           # reaches the ring centreline
            p.exit_ring_s = float(s[i_exitring])  # leaves the ring
            p.phi0 = math.radians(f0)
            p.R = R
            paths[(origin, move)] = p
    return paths
