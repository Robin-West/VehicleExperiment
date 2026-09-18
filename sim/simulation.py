"""The simulation: spawning, per-vehicle decisions, integration, metrics.

Every crossing interaction runs through one mechanism: paths that cross have a
precomputed *conflict point*, and a vehicle performs gap acceptance there --
it proceeds only if no higher-priority conflicting vehicle will reach the point
within a critical time gap. Right-of-way ranks through/right movements above the
opposing left; equal-rank ties (left vs left) break deterministically.

With careful drivers this produces clean, collision-free coordination. The
failure knobs corrupt exactly this decision:
  * reaction delay   -> the yield/brake is acted on tau seconds late,
  * gap misjudgement -> the perceived arrival time of oncoming cars is wrong,
  * limited sight     -> conflicting cars beyond sight are not yielded to,
  * rule violations  -> a red-runner ignores the signal and forces through.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from . import geometry as geo
from .geometry import THROUGH, LEFT, RIGHT, ORIGINS, OPPOSITE, CROSS, crossing_point
from .metrics import Metrics
from .signal import SignalController, SignalConfig
from collections import deque

from .vehicle import Vehicle, DriverConfig, idm_accel, BASE_REACTION

REWIND = 4.0            # seconds before a crash to start a counterfactual replay

V_REF = 3.0             # speed floor used when estimating arrival times (m/s)
CRIT_GAP = 4.0          # accept a crossing gap only if larger than this (s)
CONFLICT_SETBACK = 5.0  # stop this far short of a conflict point when yielding (m)
COMMIT_MARGIN = 1.0     # once within this of the setback, the crossing is committed


def _row_rank(move):
    return 2 if move in (THROUGH, RIGHT) else 1


def _lane_group(move):
    """Approaches have a left-turn pocket separate from the through/right lane,
    so a left-turner waiting for a gap does not block through traffic."""
    return "L" if move == LEFT else "TR"


@dataclass
class SimConfig:
    duration: float = 600.0
    dt: float = 0.1
    seed: int = 0
    arrival_rate: float = 0.12          # veh/s per approach (Poisson)
    turn_fracs: tuple = (0.2, 0.55, 0.25)   # left, through, right
    driver: DriverConfig = field(default_factory=DriverConfig)
    signal: SignalConfig = field(default_factory=SignalConfig)
    speed_govern: bool = False          # cap speed to the limit near the intersection
    govern_dist: float = 80.0           # how far out the governing zone begins (m)
    interlock: bool = False             # hold a green while a conflicting car is in the box
    green_extend: bool = False          # extend green for a car caught in the dilemma zone
    max_green_ext: float = 4.0          # cap on green extension (s)
    stop_setback: float = 0.0           # move the stop line back this many metres
    setback_sight: bool = False         # model the setback improving cross-traffic sight


class Simulation:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)
        self.paths = geo.build_paths()
        self.signal = SignalController(cfg.signal)
        self.metrics = Metrics()
        self.vehicles: list[Vehicle] = []
        self.t = 0.0
        self._eff_t = 0.0           # effective signal time (green extension shifts it)
        self._ext = 0.0             # accumulated green extension
        self._ext_run = 0.0         # current consecutive extension
        self._prev_state: dict[int, str] = {}
        self._replay = False        # True while running a counterfactual replay

        # precompute conflict points between every pair of crossing paths from
        # different approaches: for each path, a list of (s_self, other_key, s_other)
        self._conflicts = {k: [] for k in self.paths}
        keys = list(self.paths)
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                ka, kb = keys[i], keys[j]
                if ka[0] == kb[0]:
                    continue                       # same approach -> lane logic
                cp = crossing_point(self.paths[ka], self.paths[kb])
                if cp is None:
                    continue
                s_a, s_b, _xy, _gap = cp
                self._conflicts[ka].append((s_a, kb, s_b))
                self._conflicts[kb].append((s_b, ka, s_a))

        # samples of each path in/around the intersection box, for emergency
        # collision-avoidance braking (a car already on my path ahead)
        self._box = {}
        for key, p in self.paths.items():
            m = (np.abs(p.xy[:, 0]) < 2.0 * geo.LANE_W) & (np.abs(p.xy[:, 1]) < 2.0 * geo.LANE_W)
            idx = np.where(m)[0]
            self._box[key] = (p.xy[idx, 0], p.xy[idx, 1], p.s[idx])

    def _sig(self, origin, move):
        return self.signal.state(origin, move, self._eff_t)

    def _eff_stop(self, path):
        """Effective stop-line arc length (moved back by the setback)."""
        return path.stop_s - self.cfg.stop_setback

    def _sight(self, ego):
        """Sight distance for conflicting traffic; a set-back stop line is
        modelled as improving it (the driver's hypothesised visibility gain)."""
        if self.cfg.setback_sight:
            return ego.sight_distance + self.cfg.stop_setback
        return ego.sight_distance

    # ---------- spawning ----------
    def _spawn(self):
        moves = (LEFT, THROUGH, RIGHT)
        for o in ORIGINS:
            if self.rng.random() < self.cfg.arrival_rate * self.cfg.dt:
                move = self.rng.choices(moves, weights=self.cfg.turn_fracs)[0]
                path = self.paths[(o, move)]
                grp = _lane_group(move)
                tail = min((v.s for v in self.vehicles
                            if v.origin == o and _lane_group(v.move) == grp), default=1e9)
                if tail < self.cfg.driver.length + self.cfg.driver.s0 + 3.0:
                    continue
                v = Vehicle(path, self.cfg.driver, self.cfg.dt, self.rng, self.t)
                v.gap_bias = 1.0 + (self.rng.gauss(0, v.gap_error) if v.gap_error else 0.0)
                self.vehicles.append(v)
                self.metrics.spawned += 1

    # ---------- car-following leaders ----------
    def _leaders(self, ego: Vehicle):
        """Return (idm_gap, idm_lead, same_lane_gap).

        Two kinds of car ahead: the same-lane leader (same approach + lane group;
        a true rear-end risk) and the shared-exit-lane leader (a merging car,
        used only to slow down -- it is laterally offset until fully merged, so a
        negative "gap" there is not a rear-end). idm_gap is the binding one for
        deceleration; same_lane_gap is returned separately for rear-end logging.
        """
        idm_gap, idm_lead, same_lane_gap = None, None, None

        cand = None
        egrp = _lane_group(ego.move)
        for v in self.vehicles:
            if v is ego or v.origin != ego.origin or v.s <= ego.s:
                continue
            if _lane_group(v.move) != egrp:
                continue
            if cand is None or v.s < cand.s:
                cand = v
        if cand is not None:
            same_lane_gap = (cand.s - cand.length / 2) - (ego.s + ego.length / 2)
            idm_gap, idm_lead = same_lane_gap, cand

        if ego.front_s() >= ego.path.stop_s:
            ex, ey, _ = ego.pose()
            ep = geo.exit_progress(ego.dest, ex, ey)
            for v in self.vehicles:
                if v is ego or v.dest != ego.dest or v.front_s() < v.path.stop_s:
                    continue
                vx, vy, _ = v.pose()
                vp = geo.exit_progress(v.dest, vx, vy)
                if vp <= ep:
                    continue
                gap = (vp - v.length / 2) - (ep + ego.length / 2)
                if idm_gap is None or gap < idm_gap:
                    idm_gap, idm_lead = gap, v
        return idm_gap, idm_lead, same_lane_gap

    # ---------- conflict-point gap acceptance ----------
    def _will_proceed(self, v: Vehicle):
        """Is v actually going to enter the intersection (not sitting at a red)?"""
        if v.front_s() >= v.path.stop_s - 1.0:
            return True
        if v.decided_run:
            return True
        return self._sig(v.origin, v.move) == "G"

    def _conflict_obstacle_s(self, ego: Vehicle, by_key):
        """Nearest conflict point ego must yield at (as an arclength), or None."""
        if ego.decided_run:
            return None                       # a runner forces through conflicts
        # once in the box, clear it when the phase ends instead of stranding there
        if ego.front_s() >= ego.path.stop_s \
                and self._sig(ego.origin, ego.move) != "G":
            return None
        ego_front = ego.front_s()
        # a permissive left-turner holds at one line near the box entrance while
        # yielding, then commits through the whole crossing once past it (rather
        # than creeping to a mid-box spot that sits on a priority car's path).
        left_hold = ego.path.stop_s + 3.0
        if ego.move == LEFT and ego_front > left_hold:
            return None
        ex, ey, _ = ego.pose()
        ego_rank = _row_rank(ego.move)
        result = None
        for (s_self, other_key, s_other) in self._conflicts[(ego.origin, ego.move)]:
            if ego_front >= s_self - COMMIT_MARGIN:
                continue                      # committed / already crossing
            d_ego = s_self - ego_front
            t_ego = d_ego / max(ego.v, V_REF)
            yield_here = False
            for v in by_key.get(other_key, ()):
                if v is ego or v.crashed or not self._will_proceed(v):
                    continue
                d_other = s_other - v.front_s()
                if d_other <= -(v.length / 2 + 1.0):
                    continue                  # already cleared the point
                vx, vy, _ = v.pose()
                if math.hypot(vx - ex, vy - ey) > self._sight(ego):
                    continue                  # can't see it -> won't yield to it
                t_other = max(0.0, d_other) / max(v.v, V_REF)
                perceived = t_other * getattr(ego, "gap_bias", 1.0)
                if perceived > CRIT_GAP:
                    continue                  # big enough gap -> accept it
                v_rank = _row_rank(v.move)
                if v_rank > ego_rank:
                    yield_here = True
                elif v_rank == ego_rank and (perceived < t_ego
                                             or (abs(perceived - t_ego) < 0.3
                                                 and ego.id > v.id)):
                    yield_here = True
                if yield_here:
                    break
            if yield_here:
                s_stop = s_self - CONFLICT_SETBACK
                if ego.move == LEFT:
                    s_stop = min(s_stop, left_hold)   # hold at the entrance line
                if result is None or s_stop < result:
                    result = s_stop
        return result

    def _emergency_obstacle_s(self, ego: Vehicle, in_box):
        """Nearest point on ego's path (in the box) occupied by a visible car
        from another approach -- defensive braking regardless of right-of-way."""
        xs, ys, ss = self._box[(ego.origin, ego.move)]
        ahead = ss > ego.front_s()
        if not ahead.any():
            return None
        xs, ys, ss = xs[ahead], ys[ahead], ss[ahead]
        ex, ey, _ = ego.pose()
        cross = geo.CROSS[ego.origin]
        best = None
        for other in in_box:
            if other is ego or other.origin == ego.origin:
                continue
            # a driver on green does not expect a perpendicular red-runner, so
            # does not brake for one in time (this is what makes running dangerous)
            if other.decided_run and other.origin in cross \
                    and self._sig(ego.origin, ego.move) == "G":
                continue
            ox, oy, _ = other.pose()
            if math.hypot(ox - ex, oy - ey) > self._sight(ego):
                continue
            d = np.hypot(xs - ox, ys - oy)
            k = int(np.argmin(d))
            if d[k] < (ego.width + other.width) / 2 + 1.0:
                if best is None or ss[k] < best:
                    best = float(ss[k])
        return best

    # ---------- per-vehicle decision ----------
    def _plan(self, ego: Vehicle, by_key, in_box):
        v0_eff = max(1.0, ego.desired_speed_here())
        # speed governing: within the zone approaching (and through) the box, no
        # one may exceed the limit -- removes speeding as a cause in the conflict area
        if self.cfg.speed_govern and ego.front_s() > ego.path.stop_s - self.cfg.govern_dist:
            v0_eff = min(v0_eff, self.cfg.driver.v0)

        def obstacle_accel(gap):
            return idm_accel(ego.v, v0_eff, ego.v, gap, ego.T, ego.a_max, ego.b, ego.s0)

        a = ego.a_max * (1 - (ego.v / v0_eff) ** 4)      # free road

        gap, lead, same_lane_gap = self._leaders(ego)
        # a rear-end is a true same-lane overlap with real closing speed
        if same_lane_gap is not None and same_lane_gap < -0.2 and lead is not None \
                and (ego.v - lead.v) > 0.5 and not ego.crashed and not lead.crashed:
            if not self._replay:
                x, y, _ = ego.pose()
                self.metrics.register_rear_end(ego.v - lead.v, (x, y), self.t, ego, lead)
                ego.crashed = lead.crashed = True
            return a
        if lead is not None:
            dv = ego.v - lead.v
            a = min(a, idm_accel(ego.v, v0_eff, dv, gap, ego.T, ego.a_max, ego.b, ego.s0))

        # signal + yellow/red decision
        state = self._sig(ego.origin, ego.move)
        # dynamic all-red interlock: hold a green while a conflicting (cross) car
        # is still in the central crossing square -- the potential victim is never
        # released into a car clearing late or running the light
        if self.cfg.interlock and state == "G" and ego.front_s() < ego.path.stop_s:
            cross = CROSS[ego.origin]
            for v in in_box:
                if v is ego or v.crashed or v.origin not in cross:
                    continue
                vx, vy, _ = v.pose()
                if abs(vx) < geo.HALF_BOX + 1.5 and abs(vy) < geo.HALF_BOX + 1.5:
                    state = "R"
                    break
        prev = self._prev_state.get(ego.id, state)
        eff_stop = self._eff_stop(ego.path)
        if prev == "G" and state == "Y" and ego.decided_run is None \
                and ego.front_s() < eff_stop:
            d_stop = eff_stop - ego.front_s()
            comfortable = ego.v * ego.v / (2 * ego.b)
            if d_stop < comfortable:
                ego.decided_run = True                    # dilemma zone: can't stop (not a fault)
            else:
                ego.decided_run = self.rng.random() < ego.p_run
                if ego.decided_run:
                    ego.violated = True                   # deliberate red-run (a fault)
        if state == "G":
            ego.decided_run = None
        self._prev_state[ego.id] = state

        must_stop = (state == "R") or (state == "Y" and not ego.decided_run)
        if must_stop and ego.front_s() < eff_stop:
            a = min(a, obstacle_accel(eff_stop - ego.front_s()))

        # crossing conflicts (gap acceptance / right-of-way)
        cs = self._conflict_obstacle_s(ego, by_key)
        if cs is not None:
            a = min(a, obstacle_accel(cs - ego.front_s()))

        # emergency: brake for a visible car already in the box on my path
        es = self._emergency_obstacle_s(ego, in_box)
        if es is not None:
            a = min(a, obstacle_accel((es - 2.0) - ego.front_s()))

        return a

    def _detect_collisions(self):
        """Hook so subclasses (roundabout) can classify collisions differently."""
        self.metrics.check_collisions(self.vehicles, self.t)

    # ---------- main loop ----------
    def step(self):
        m = self.metrics
        by_key = {}
        in_box = []
        for v in self.vehicles:
            if not v.crashed:
                by_key.setdefault((v.origin, v.move), []).append(v)
                x, y, _ = v.pose()
                if abs(x) < 2.0 * geo.LANE_W and abs(y) < 2.0 * geo.LANE_W:
                    in_box.append(v)

        # dilemma-zone green extension: freeze the phase clock while a car on a
        # green movement is caught in the dilemma zone (can't comfortably stop),
        # so it clears legally instead of running the red or slamming the brakes
        if self.cfg.green_extend:
            extend = False
            if self._ext_run < self.cfg.max_green_ext:
                for v in self.vehicles:
                    es = self._eff_stop(v.path)
                    if v.crashed or v.front_s() >= es:
                        continue
                    if self._sig(v.origin, v.move) != "G":
                        continue
                    d = es - v.front_s()
                    if d > 0 and v.v > 3.0 and d < v.v * v.v / (2 * v.b):
                        extend = True
                        break
            if extend:
                self._ext += self.cfg.dt
                self._ext_run += self.cfg.dt
            else:
                self._ext_run = 0.0
        self._eff_t = self.t - self._ext

        planned = {}
        for v in self.vehicles:
            if not v.crashed:
                planned[v.id] = self._plan(v, by_key, in_box)
        for v in self.vehicles:
            if not v.crashed:
                v.apply(planned.get(v.id, 0.0))
                v.hist.append((self.t, v.s, v.v))

        self._detect_collisions()
        m.update_ttc(self.vehicles, self.t)
        m.update_pet(self.vehicles, self.t)

        keep = []
        for v in self.vehicles:
            if v.crashed:
                continue
            if v.done:
                m.cleared += 1
                free = v.path.length / max(v.v0, 1.0)
                m.total_delay += max(0.0, (self.t - v.spawn_time) - free)
                self._prev_state.pop(v.id, None)
                continue
            keep.append(v)
        self.vehicles = keep

        self._spawn()
        self.t += self.cfg.dt

    def run(self):
        n = int(self.cfg.duration / self.cfg.dt)
        for _ in range(n):
            self.step()
        return self.metrics.summary()

    # ---------- counterfactual fault attribution ----------
    #
    # For each crash we ask: if driver A alone had been ideal (obeyed the limit
    # and the signal, judged the gap correctly, reacted promptly, and perceived
    # the intersection properly), would the crash still happen? Same for B. We
    # answer by replaying just those two cars from a few seconds before impact.
    #
    #   averted by fixing A AND by fixing B  -> two-fault  (needed two mistakes)
    #   averted by fixing exactly one        -> single-fault (one mistake sufficed)
    #   averted by neither alone             -> each mistake alone sufficed
    #                                           (also a single point of failure)

    def _at(self, veh, t0):
        """(s, v) for a vehicle at time ~t0 from its recorded history."""
        best = min(veh.hist, key=lambda h: abs(h[0] - t0))
        return best[1], best[2]

    def _clone(self, src, s, v, ideal):
        c = Vehicle(src.path, self.cfg.driver, self.cfg.dt, self.rng, 0.0)
        c.s, c.v, c.a = s, v, 0.0
        c.length, c.width = src.length, src.width
        if ideal:
            c.v0 = self.cfg.driver.v0          # obey the speed limit
            c.reaction_delay = BASE_REACTION    # alert
            c.gap_bias = 1.0                    # judge gaps correctly
            c.sight_distance = 1e9              # perceive properly (compensates for occlusion)
            c.p_run = 0.0                       # obey the signal
            c.is_speeding = False
            c.decided_run = None
        else:
            for attr in ("v0", "T", "a_max", "b", "s0", "reaction_delay", "gap_bias",
                         "sight_distance", "p_run", "turn_speed", "critical_gap",
                         "is_speeding"):
                setattr(c, attr, getattr(src, attr))
            c.decided_run = src.decided_run
        n = max(1, int(round(c.reaction_delay / self.cfg.dt)))
        c.accel_buf = deque([0.0] * n, maxlen=n)
        c._pcache = None
        return c

    def _run_fix(self, sim_src, ghost_src, t0, t_end):
        """Re-simulate an IDEAL version of sim_src while ghost_src replays its
        actual recorded trajectory. Returns True if they still collide."""
        ts = [h[0] for h in ghost_src.hist]
        ss = [h[1] for h in ghost_src.hist]
        vs = [h[2] for h in ghost_src.hist]
        sts = [h[0] for h in sim_src.hist]     # the corrected car's own record
        sss = [h[1] for h in sim_src.hist]
        sim = self._clone(sim_src, *self._at(sim_src, t0), ideal=True)
        ghost = self._clone(ghost_src, *self._at(ghost_src, t0), ideal=False)
        saved = (self.vehicles, self.t, self._eff_t, self._prev_state, self._replay)
        self._replay, self._prev_state, self.vehicles = True, {}, [sim, ghost]
        collided = False
        try:
            t = t0
            while t <= t_end:
                self.t = self._eff_t = t
                ghost.s = float(np.interp(t, ts, ss))     # pin ghost to reality
                ghost.v = float(np.interp(t, ts, vs))
                ghost._pcache = None
                if ghost.s >= ghost.path.length or sim.done:
                    break
                by_key, in_box = {}, []
                for v in (sim, ghost):
                    by_key.setdefault((v.origin, v.move), []).append(v)
                    x, y, _ = v.pose()
                    if abs(x) < 2.0 * geo.LANE_W and abs(y) < 2.0 * geo.LANE_W:
                        in_box.append(v)
                sim.apply(self._plan(sim, by_key, in_box))
                # a corrected driver may only be MORE cautious, never advance
                # past where it actually was (prevents it escaping real constraints)
                rec_s = float(np.interp(t, sts, sss))
                if sim.s > rec_s:
                    sim.s, sim._pcache = rec_s, None
                if self.metrics._overlap(sim.footprint(), ghost.footprint()):
                    collided = True
                    break
                t += self.cfg.dt
        finally:
            self.vehicles, self.t, self._eff_t, self._prev_state, self._replay = saved
        return collided

    def attribute(self):
        """Classify every logged crash. For each, hold one driver on its actual
        trajectory and ask whether the other driving ideally would have avoided
        it (a but-for counterfactual). Returns a summary dict and per-crash list.

            averted by fixing either driver  -> two-fault (needed two mistakes)
            averted by fixing exactly one    -> single-fault (one mistake sufficed)
            averted by neither               -> each driver's mistake alone sufficed
        """
        res = {"total": 0, "classified": 0, "two_fault": 0,
               "single_fault": 0, "each_sufficient": 0, "skipped": 0}
        detail = []
        for ev in self.metrics.collisions:
            A, B = ev.get("va"), ev.get("vb")
            res["total"] += 1
            if A is None or B is None or len(A.hist) < 5 or len(B.hist) < 5:
                res["skipped"] += 1
                continue
            t0 = max(A.hist[0][0], B.hist[0][0], ev["t"] - REWIND)
            if t0 > ev["t"] - 1.0:                       # too little runway to replay
                res["skipped"] += 1
                continue
            res["classified"] += 1
            if ev["type"] == "rear_end":
                # The counterfactual is unreliable for rear-ends: the follower's
                # path was a *response* to the leader, so holding it fixed while
                # correcting the leader is ill-defined. Attribute by rule instead
                # -- a rear-end is the follower's fault, and needs a second mistake
                # only when someone was actually above the limit at impact (a speed
                # differential). Using speed-at-impact (not the speeder disposition)
                # keeps it fair under speed governing.
                def _spd(V):
                    return min(V.hist, key=lambda h: abs(h[0] - ev["t"]))[2] if V.hist else 0.0
                limit = self.cfg.driver.v0
                if max(_spd(A), _spd(B)) > 1.05 * limit:
                    res["two_fault"] += 1
                    detail.append((ev["type"], "two-fault"))
                else:
                    res["single_fault"] += 1
                    detail.append((ev["type"], "single-fault"))
                continue
            t_end = ev["t"] + 0.5
            avert_a = not self._run_fix(A, B, t0, t_end)   # A ideal vs B as-it-happened
            avert_b = not self._run_fix(B, A, t0, t_end)   # B ideal vs A as-it-happened
            if avert_a and avert_b:
                res["two_fault"] += 1
                verdict = "two-fault"
            elif avert_a or avert_b:
                res["single_fault"] += 1
                verdict = "single-fault"
            else:
                res["each_sufficient"] += 1
                verdict = "each-sufficient"
            detail.append((ev["type"], verdict))
        return res, detail
