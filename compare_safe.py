"""Current-tech infrastructure safety gates vs the permissive baseline.

All are deployable with today's sensors + signals (no in-car tech):
  - interlock:            hold a conflicting green while a car is in the crossing box
  - protected:            actuated protected-left (left only when oncoming is stopped)
  - interlock+protected:  the "current-tech safe intersection"
  - all:                  + dilemma-zone green extension (attacks the rear-end residue)

Reports crash count by type, an injury-weighted severity score
(type_weight x closing_speed^2), and the single- vs two-fault split.

    python compare_safe.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

DEMAND = 0.08
SEEDS = 8
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)
# injury weight by crash geometry (side-impacts far worse than rear impacts)
WEIGHT = {"right_angle": 3.0, "left_turn": 2.5, "entry": 2.0, "rear_end": 1.0}

CONFIGS = [
    ("baseline (permissive)", dict()),
    ("interlock", dict(interlock=True)),
    ("protected-left", dict(signal=SignalConfig(mode="protected"))),
    ("interlock + protected", dict(interlock=True, signal=SignalConfig(mode="protected"))),
    ("all + green-extend", dict(interlock=True, signal=SignalConfig(mode="protected"),
                                green_extend=True)),
]


def study(kw):
    types, injury, attr, cleared, spawned = Counter(), 0.0, Counter(), 0, 0
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER), **kw)
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
    print(f"{'config':24s} {'total':>6} {'rear':>5} {'Tbone':>6} {'left':>5} "
          f"{'injury':>8} {'2-fault':>8} {'cleared':>8}")
    for name, kw in CONFIGS:
        t, inj, a, cl, sp = study(kw)
        two = a["two_fault"]
        rep = a["classified"]
        print(f"{name:24s} {sum(t.values()):>6} {t['rear_end']:>5} {t['right_angle']:>6} "
              f"{t['left_turn']:>5} {inj:>8.0f} {pct(two, rep):>8} {cl}/{sp}")
