"""Safety and performance metrics.

Real crashes are rare, so alongside actual collisions we count near-misses using
the two standard surrogate measures:

  * TTC  - time-to-collision. A conflict event is logged when a pair of cars on
           a collision course drops below ~1.5 s.
  * PET  - post-encroachment time. The gap between one car leaving a spot and the
           next arriving; a small PET is a close shave.

Cross-stream measures only consider pairs from *different* approaches; cars from
the same approach queue behind one another (that is following, not a conflict,
and rear-ends among them are detected separately via the car-following gap).
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field

from .geometry import OPPOSITE

TTC_THRESH = 1.0     # s -- tight enough to flag genuine near-misses, not normal following
PET_THRESH = 1.0     # s
PET_CELL = 1.0       # m
NEAR_RANGE = 25.0    # m, only look at reasonably close pairs


def _vel_vec(veh):
    x, y, h = veh.pose()
    return (x, y, veh.v * math.cos(h), veh.v * math.sin(h))


@dataclass
class Metrics:
    spawned: int = 0
    cleared: int = 0
    total_delay: float = 0.0

    collisions: list = field(default_factory=list)   # dicts: type, severity, t, xy
    rear_end: int = 0
    right_angle: int = 0
    left_turn: int = 0
    entry: int = 0            # roundabout entry-merge collision

    ttc_conflicts: int = 0
    min_ttc: float = float("inf")
    pet_events: int = 0
    min_pet: float = float("inf")

    _ttc_active: set = field(default_factory=set)
    _cells: dict = field(default_factory=dict)       # (ix,iy) -> (veh_id, t)

    # --- collisions ---
    def register_collision(self, kind, severity, xy, t):
        self.collisions.append({"type": kind, "severity": severity, "t": t, "xy": xy})
        setattr(self, kind, getattr(self, kind) + 1)

    def check_collisions(self, vehicles, t):
        """Footprint-overlap collisions between cars from different approaches."""
        act = [v for v in vehicles if not v.crashed]
        fps = [v.footprint() for v in act]
        crashed_now = set()
        for i in range(len(act)):
            a = act[i]
            if a.id in crashed_now:
                continue
            fa = fps[i]
            for j in range(i + 1, len(act)):
                b = act[j]
                if b.id in crashed_now or a.origin == b.origin:
                    continue
                if self._overlap(fa, fps[j]):
                    kind = self._classify(a, b)
                    ax, ay, avx, avy = _vel_vec(a)
                    bx, by, bvx, bvy = _vel_vec(b)
                    sev = math.hypot(avx - bvx, avy - bvy)
                    xy = ((ax + bx) / 2, (ay + by) / 2)
                    ev = {"type": kind, "severity": sev, "t": t, "xy": xy,
                          "va": a, "vb": b,
                          "a": f"{a.origin}-{a.move} s={a.s:.1f} v={a.v:.1f}",
                          "b": f"{b.origin}-{b.move} s={b.s:.1f} v={b.v:.1f}"}
                    self.collisions.append(ev)
                    setattr(self, kind, getattr(self, kind) + 1)
                    a.crashed = b.crashed = True
                    crashed_now.add(a.id)
                    crashed_now.add(b.id)
                    break

    @staticmethod
    def _overlap(fa, fb):
        for (x1, y1, r1) in fa:
            for (x2, y2, r2) in fb:
                if (x1 - x2) ** 2 + (y1 - y2) ** 2 < (r1 + r2) ** 2:
                    return True
        return False

    @staticmethod
    def _classify(a, b):
        opposed = OPPOSITE[a.origin] == b.origin
        if opposed and (a.move == "left" or b.move == "left"):
            return "left_turn"
        return "right_angle"

    def register_rear_end(self, closing_speed, xy, t, va=None, vb=None):
        self.collisions.append({"type": "rear_end", "severity": max(0.0, closing_speed),
                                "t": t, "xy": xy, "va": va, "vb": vb,
                                "a": f"{va.origin}-{va.move}" if va else None,
                                "b": f"{vb.origin}-{vb.move}" if vb else None})
        self.rear_end += 1

    # --- TTC ---
    def update_ttc(self, vehicles, t):
        act = [v for v in vehicles if not v.crashed]
        vv = [_vel_vec(v) for v in act]
        still = set()
        for i in range(len(act)):
            a = act[i]
            ax, ay, avx, avy = vv[i]
            for j in range(i + 1, len(act)):
                b = act[j]
                if a.origin == b.origin or a.dest == b.dest:
                    continue                       # same lane or merging = following
                bx, by, bvx, bvy = vv[j]
                rx, ry = bx - ax, by - ay
                dist = math.hypot(rx, ry)
                if dist > NEAR_RANGE or dist < 1e-6:
                    continue
                rvx, rvy = bvx - avx, bvy - avy
                closing = -(rx * rvx + ry * rvy) / dist   # >0 means approaching
                if closing <= 0.3:
                    continue
                ttc = dist / closing
                if ttc < TTC_THRESH:
                    key = (a.id, b.id)
                    still.add(key)
                    if key not in self._ttc_active:
                        self.ttc_conflicts += 1
                    self.min_ttc = min(self.min_ttc, ttc)
        self._ttc_active = still

    # --- PET ---
    def update_pet(self, vehicles, t):
        for v in vehicles:
            if v.crashed:
                continue
            for (cx, cy, _r) in v.footprint():
                if abs(cx) > 3 * 3.5 or abs(cy) > 3 * 3.5:
                    continue
                key = (int(math.floor(cx / PET_CELL)), int(math.floor(cy / PET_CELL)))
                prev = self._cells.get(key)
                if prev is not None and prev[0] != v.id:
                    pet = t - prev[1]
                    if 0.0 < pet < PET_THRESH:
                        self.pet_events += 1
                        self.min_pet = min(self.min_pet, pet)
                self._cells[key] = (v.id, t)

    # --- reporting ---
    def summary(self):
        total_collisions = (self.rear_end + self.right_angle + self.left_turn
                            + self.entry)
        avg_delay = self.total_delay / self.cleared if self.cleared else 0.0
        return {
            "spawned": self.spawned,
            "cleared": self.cleared,
            "collisions_total": total_collisions,
            "rear_end": self.rear_end,
            "right_angle": self.right_angle,
            "left_turn": self.left_turn,
            "entry": self.entry,
            "ttc_conflicts": self.ttc_conflicts,
            "min_ttc": None if math.isinf(self.min_ttc) else round(self.min_ttc, 2),
            "pet_events": self.pet_events,
            "min_pet": None if math.isinf(self.min_pet) else round(self.min_pet, 2),
            "avg_delay_s": round(avg_delay, 2),
        }
