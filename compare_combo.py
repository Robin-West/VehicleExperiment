"""Combined package: interlock + modest setback (15 ft, with visibility) + governing.

Shows each component alone and the combination, so we can see whether they stack.
Hypothesis: governing cancels the setback's entry-speed penalty, leaving its
visibility benefit; the interlock removes T-bones.

    python compare_combo.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

DEMAND = 0.08
SEEDS = 8
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)
WEIGHT = {"right_angle": 3.0, "left_turn": 2.5, "entry": 2.0, "rear_end": 1.0}
SETBACK = 15 * 0.3048   # 15 ft

CONFIGS = [
    ("baseline", dict()),
    ("interlock", dict(interlock=True)),
    ("governing", dict(speed_govern=True, govern_dist=150.0)),
    ("setback 15ft +sight", dict(stop_setback=SETBACK, setback_sight=True)),
    ("COMBINED (all 3)", dict(interlock=True, speed_govern=True, govern_dist=150.0,
                              stop_setback=SETBACK, setback_sight=True)),
]


def study(kw):
    types, injury, attr, cleared, spawned = Counter(), 0.0, Counter(), 0, 0
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER),
                        signal=SignalConfig(mode="permissive"), **kw)
        sim = Simulation(cfg)
        summ = sim.run()
        cleared += summ["cleared"]
        spawned += summ["spawned"]
        for c in sim.metrics.collisions:
            types[c["type"]] += 1
            injury += WEIGHT.get(c["type"], 1.0) * c["severity"] ** 2
        res, _ = sim.attribute()
        attr.update(res)
    return types, injury, attr, cleared, spawned


def pct(n, d):
    return f"{100*n/d:.0f}%" if d else "n/a"


if __name__ == "__main__":
    print(f"demand={DEMAND}/approach, {SEEDS} seeds x {DURATION:.0f}s, mix={DRIVER}")
    print(f"{'config':22s} {'total':>6} {'rear':>5} {'Tbone':>6} {'left':>5} "
          f"{'injury':>8} {'2-flt':>6} {'cleared':>9}")
    base_inj = None
    for name, kw in CONFIGS:
        t, inj, a, cl, sp = study(kw)
        if base_inj is None:
            base_inj = inj
        delta = f"({100*(inj-base_inj)/base_inj:+.0f}%)"
        print(f"{name:22s} {sum(t.values()):>6} {t['rear_end']:>5} {t['right_angle']:>6} "
              f"{t['left_turn']:>5} {inj:>8.0f} {pct(a['two_fault'], a['classified']):>6} "
              f"{cl}/{sp}  {delta}")
