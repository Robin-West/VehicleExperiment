"""Vehicles: longitudinal dynamics (IDM) plus the per-driver imperfections.

Motion is one-dimensional along a fixed path (the geometry decides the lateral
line). Acceleration each step is the most restrictive of several constraints
(car ahead, signal, yield point). Reaction delay is applied by acting on the
acceleration the driver *planned* tau seconds ago.
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

from . import geometry
from .geometry import Path

DELTA = 4.0            # IDM acceleration exponent
BASE_REACTION = 0.6    # baseline human perception-reaction lag (s); the
                       # reaction_delay knob adds to this


@dataclass
class DriverConfig:
    """Population-level parameters. Per-vehicle values are sampled around these."""
    v0: float = 13.5           # desired speed ~49 km/h
    T: float = 1.2             # safe time headway (s)
    a_max: float = 1.8         # max acceleration (m/s^2)
    b: float = 2.6             # comfortable deceleration (m/s^2)
    s0: float = 2.0            # minimum standstill gap (m)
    length: float = 4.6        # vehicle length (m)
    width: float = 1.9         # vehicle width (m)

    # --- failure knobs ---
    reaction_delay: float = 0.0     # mean *extra* reaction lag (s) above baseline
    gap_error: float = 0.0          # std-dev of multiplicative error in judged oncoming gap
    p_run: float = 0.0              # prob. of running the light when caught by yellow
    sight_distance: float = 1e9     # max distance conflicting traffic is visible (m)
    p_speed: float = 0.0            # fraction of drivers who speed
    speed_excess: float = 0.30      # how far over the limit a speeder desires (fraction)

    critical_gap: float = 4.5       # accepted time gap for a permissive left (s)
    turn_speed: float = 5.0         # target speed through a turn (m/s)


def idm_accel(v, v0, dv, gap, T, a_max, b, s0):
    """Intelligent Driver Model acceleration.

    dv  = v - v_lead (positive when closing on the leader).
    gap = bumper-to-bumper distance to the leader/obstacle (m).
    """
    gap_eff = max(gap, 0.1)
    s_star = s0 + max(0.0, v * T + v * dv / (2 * math.sqrt(a_max * b)))
    return a_max * (1 - (v / v0) ** DELTA - (s_star / gap_eff) ** 2)


class Vehicle:
    _next_id = 0

    def __init__(self, path: Path, cfg: DriverConfig, dt: float, rng: random.Random,
                 spawn_time: float):
        self.id = Vehicle._next_id
        Vehicle._next_id += 1
        self.path = path
        self.origin = path.origin
        self.move = path.move
        self.dest = geometry.DEST[path.origin][path.move]
        self.dt = dt
        self.spawn_time = spawn_time

        # sample per-vehicle parameters around the population means
        j = lambda x, frac=0.12: max(0.0, x * (1 + rng.gauss(0, frac)))
        self.speed_limit = cfg.v0
        self.is_speeding = rng.random() < cfg.p_speed
        if self.is_speeding:
            self.v0 = cfg.v0 * (1 + cfg.speed_excess) * (1 + rng.gauss(0, 0.05))
        else:
            self.v0 = j(cfg.v0)
        self.T = j(cfg.T)
        self.a_max = j(cfg.a_max)
        self.b = j(cfg.b)
        self.s0 = j(cfg.s0)
        self.length = cfg.length
        self.width = cfg.width
        self.turn_speed = cfg.turn_speed
        self.critical_gap = cfg.critical_gap

        self.reaction_delay = BASE_REACTION + max(0.0, rng.gauss(
            cfg.reaction_delay, 0.3 * cfg.reaction_delay))
        self.gap_error = cfg.gap_error
        self.p_run = cfg.p_run
        self.sight_distance = cfg.sight_distance
        self.gap_bias = 1.0          # set by the simulation at spawn

        self.s = 0.0
        self.v = self.v0 * 0.9
        self.a = 0.0

        # reaction-delay buffer of planned accelerations
        n = max(1, int(round(self.reaction_delay / dt)))
        self.accel_buf = deque([0.0] * n, maxlen=n)

        self.decided_run = None      # None until a yellow decision is made
        self.violated = False        # True only for a deliberate red-run (not a dilemma)
        self.crashed = False
        self._pcache = None
        self.hist = deque(maxlen=90)  # recent (t, s, v) for counterfactual replay

    # --- kinematics ---
    def pose(self):
        if self._pcache is None:
            self._pcache = self.path.point_at(self.s)
        return self._pcache

    def front_s(self):
        return self.s + self.length / 2

    def footprint(self):
        """Two circles (front, rear) approximating the vehicle body."""
        x, y, h = self.pose()
        r = self.width / 2 * 1.05
        off = self.length / 2 - r
        cx, cy = math.cos(h), math.sin(h)
        return [(x + off * cx, y + off * cy, r),
                (x - off * cx, y - off * cy, r)]

    def desired_speed_here(self):
        """Slow down for the turn while inside the curved part of the path."""
        if self.move in ("left", "right"):
            if self.path.stop_s <= self.s <= self.path.length - (self.path.length
                                                                 - self.path.stop_s) * 0.4:
                return self.turn_speed
        return self.v0

    def apply(self, planned_accel: float):
        """Buffer the planned accel, act on the delayed one, integrate."""
        self.accel_buf.append(planned_accel)
        a = self.accel_buf[0]
        a = max(-8.0, min(self.a_max, a))
        self.a = a
        self.v = max(0.0, self.v + a * self.dt)
        self.s += self.v * self.dt
        self._pcache = None

    @property
    def done(self):
        return self.s >= self.path.length
