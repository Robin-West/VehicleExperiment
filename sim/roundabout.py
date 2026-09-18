"""Single-lane roundabout variant of the intersection.

Same vehicles, failure knobs, metrics, and counterfactual fault attribution as
the signalized intersection (it subclasses Simulation) -- only the geometry and
coordination change:

  * No signals -> red-running does not exist.
  * All traffic circulates one way -> no crossing conflicts (no T-bones, no
    left-turn-across-traffic). Only two conflict kinds remain:
        - rear-end   (same-direction following, on approach / ring / exit)
        - entry      (an entering car vs a circulating car it should yield to)
  * Tight radius caps circulating speed, so speeding matters mainly on approach.

Entering cars yield to circulating traffic (gap acceptance, corrupted by the
same knobs); circulating cars have right of way but brake defensively for a car
merging in front if they can see it in time.
"""
from __future__ import annotations

import math

from . import geometry as geo
from .geometry import ORIGINS, MOVES, build_roundabout_paths
from .metrics import Metrics, _vel_vec
from .simulation import Simulation, V_REF, _lane_group
from .signal import SignalController, SignalConfig
from .vehicle import Vehicle, idm_accel

A_LAT = 3.0             # comfortable lateral acceleration -> caps circulating speed
CLEAR_TIME = 2.5        # time an entrant needs to reach and clear the merge point (s)
RING_LO = geo.R_CIRC - 2.0
RING_HI = geo.R_CIRC + 5.0   # radius band counted as "on the ring" (includes merge)
TWO_PI = 2 * math.pi


class Roundabout(Simulation):
    def __init__(self, cfg):
        # deliberately do not call super().__init__ (it builds signalized geometry)
        import random
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)
        self.paths = build_roundabout_paths()
        self.signal = SignalController(SignalConfig())   # unused, kept for interface
        self.metrics = Metrics()
        self.vehicles: list[Vehicle] = []
        self.t = 0.0
        self._eff_t = 0.0
        self._ext = 0.0
        self._ext_run = 0.0
        self._prev_state = {}
        self._replay = False
        self.v_circ = math.sqrt(A_LAT * geo.R_CIRC)

    # ---------- spawning: one shared entry lane per approach ----------
    def _spawn(self):
        for o in ORIGINS:
            if self.rng.random() < self.cfg.arrival_rate * self.cfg.dt:
                move = self.rng.choices(MOVES, weights=self.cfg.turn_fracs)[0]
                path = self.paths[(o, move)]
                tail = min((v.s for v in self.vehicles if v.origin == o), default=1e9)
                if tail < self.cfg.driver.length + self.cfg.driver.s0 + 3.0:
                    continue
                v = Vehicle(path, self.cfg.driver, self.cfg.dt, self.rng, self.t)
                v.gap_bias = 1.0 + (self.rng.gauss(0, v.gap_error) if v.gap_error else 0.0)
                self.vehicles.append(v)
                self.metrics.spawned += 1

    # ---------- helpers ----------
    def _region(self, v):
        s = v.s
        if s < v.path.entry_s:
            return "approach"       # before the give-way line (may be waiting)
        if s < v.path.ring_s:
            return "merge"          # crossing from give-way onto the ring
        if s <= v.path.exit_ring_s:
            return "circle"         # on the ring
        return "exit"               # leaving the ring

    def _theta(self, v):
        return v.path.phi0 + (v.s - v.path.ring_s) / v.path.R

    def _theta_pos(self, v):
        x, y, _ = v.pose()
        return math.atan2(y, x)

    def _radius(self, v):
        x, y, _ = v.pose()
        return math.hypot(x, y)

    # ---------- car-following ----------
    def _leaders(self, ego):
        reg = self._region(ego)
        best_gap = best_lead = None
        if reg == "approach":
            cand = None
            for v in self.vehicles:
                if v is ego or v.origin != ego.origin or v.s <= ego.s:
                    continue
                if self._region(v) != "approach":
                    continue
                if cand is None or v.s < cand.s:
                    cand = v
            if cand is not None:
                best_gap = (cand.s - cand.length / 2) - (ego.s + ego.length / 2)
                best_lead = cand
        elif reg in ("merge", "circle"):
            # follow the nearest car ahead around the ring by angular position,
            # including merging entrants and cars leaving via the exit -- so a
            # circulator always sees a car that has committed onto the ring
            R = ego.path.R
            te = self._theta_pos(ego)
            for v in self.vehicles:
                if v is ego or self._region(v) not in ("merge", "circle", "exit"):
                    continue
                rv = self._radius(v)
                if rv < RING_LO or rv > RING_HI:
                    continue
                dtheta = (self._theta_pos(v) - te) % TWO_PI
                if dtheta <= 1e-6 or dtheta > math.pi:
                    continue
                gap = R * dtheta - (v.length / 2 + ego.length / 2)
                if best_gap is None or gap < best_gap:
                    best_gap, best_lead = gap, v
        else:  # exit
            ep = ego.s - ego.path.exit_ring_s
            for v in self.vehicles:
                if v is ego or v.dest != ego.dest or self._region(v) != "exit":
                    continue
                vp = v.s - v.path.exit_ring_s
                if vp <= ep:
                    continue
                gap = (vp - v.length / 2) - (ep + ego.length / 2)
                if best_gap is None or gap < best_gap:
                    best_gap, best_lead = gap, v
        return best_gap, best_lead, best_gap  # same_lane_gap == gap (all following here)

    # ---------- entry gap acceptance ----------
    def _entry_yield_s(self, ego):
        """Hold at the give-way line unless the entrant can reach the merge point
        and clear it before any circulating car arrives there."""
        if ego.front_s() >= ego.path.entry_s:
            return None                       # committed -- past the give-way line
        phi0 = ego.path.phi0
        R = ego.path.R
        ex, ey, _ = ego.pose()
        d_ego = ego.path.ring_s - ego.front_s()
        t_ego = max(0.0, d_ego) / max(ego.v, V_REF)        # time to reach the merge point
        need = t_ego + CLEAR_TIME
        for v in self.vehicles:
            if v is ego or self._region(v) not in ("merge", "circle"):
                continue
            vx, vy, _ = v.pose()
            if math.hypot(vx - ex, vy - ey) > ego.sight_distance:
                continue                      # can't see it -> won't yield to it
            dtheta = (phi0 - self._theta_pos(v)) % TWO_PI  # how far v is upstream of the merge
            if dtheta > math.pi:
                continue                      # already past the merge point
            if R * dtheta < 6.0:
                return ego.path.entry_s       # a car is right at the merge -> wait
            t_v = (R * dtheta) / max(v.v, V_REF)
            if t_v * getattr(ego, "gap_bias", 1.0) < need:
                return ego.path.entry_s       # would not clear in time -> wait
        return None

    # ---------- per-vehicle decision ----------
    def _plan(self, ego, by_key, in_box):
        reg = self._region(ego)
        v0_eff = ego.v0
        if reg in ("merge", "circle"):
            v0_eff = min(ego.v0, self.v_circ)
        elif reg == "approach" and ego.front_s() > ego.path.entry_s - 20.0:
            v0_eff = min(ego.v0, self.v_circ)            # slow to circulating speed near entry
        v0_eff = max(1.0, v0_eff)

        def obstacle_accel(gap):
            return idm_accel(ego.v, v0_eff, ego.v, gap, ego.T, ego.a_max, ego.b, ego.s0)

        a = ego.a_max * (1 - (ego.v / v0_eff) ** 4)

        gap, lead, same_lane_gap = self._leaders(ego)
        if same_lane_gap is not None and same_lane_gap < -0.2 and lead is not None \
                and (ego.v - lead.v) > 0.5 and not ego.crashed and not lead.crashed:
            if not self._replay:
                x, y, _ = ego.pose()
                self.metrics.register_rear_end(ego.v - lead.v, (x, y), self.t, ego, lead)
                ego.crashed = lead.crashed = True
            return a
        if lead is not None:
            a = min(a, idm_accel(ego.v, v0_eff, ego.v - lead.v, gap,
                                 ego.T, ego.a_max, ego.b, ego.s0))

        if reg == "approach":
            sy = self._entry_yield_s(ego)
            if sy is not None:
                a = min(a, obstacle_accel(sy - ego.front_s()))

        return a

    # ---------- collision detection / classification ----------
    def _detect_collisions(self):
        act = [v for v in self.vehicles if not v.crashed]
        fps = [v.footprint() for v in act]
        crashed_now = set()
        for i in range(len(act)):
            a = act[i]
            if a.id in crashed_now:
                continue
            for j in range(i + 1, len(act)):
                b = act[j]
                if b.id in crashed_now:
                    continue
                if not Metrics._overlap(fps[i], fps[j]):
                    continue
                ra, rb = self._region(a), self._region(b)
                onring = {"merge", "circle", "exit"}
                if a.origin == b.origin:
                    kind = "rear_end"          # same approach queue
                elif "merge" in (ra, rb) and ra in onring and rb in onring:
                    kind = "entry"             # a merging car meeting ring traffic
                elif ra in onring and rb in onring:
                    kind = "rear_end"          # two established ring cars following
                else:
                    kind = "entry"
                ax, ay, avx, avy = _vel_vec(a)
                bx, by, bvx, bvy = _vel_vec(b)
                sev = math.hypot(avx - bvx, avy - bvy)
                if sev < 2.0:
                    continue          # bumper-to-bumper queue contact, not an impact
                self.metrics.collisions.append(
                    {"type": kind, "severity": sev, "t": self.t,
                     "xy": ((ax + bx) / 2, (ay + by) / 2), "va": a, "vb": b,
                     "a": f"{a.origin}-{a.move}", "b": f"{b.origin}-{b.move}"})
                setattr(self.metrics, kind, getattr(self.metrics, kind) + 1)
                a.crashed = b.crashed = True
                crashed_now.add(a.id)
                crashed_now.add(b.id)
                break
