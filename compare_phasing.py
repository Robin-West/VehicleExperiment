"""Permissive vs protected-only left-turn phasing, same demand and driver mix.

Tests the prediction that protected-only phasing converts single-fault crashes
into two-fault ones without reducing the total much.

    python compare_phasing.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

DEMAND = 0.08
SEEDS = 8
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)


def study(mode):
    types, attr, by = Counter(), Counter(), Counter()
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER), signal=SignalConfig(mode=mode))
        sim = Simulation(cfg)
        sim.run()
        for c in sim.metrics.collisions:
            types[c["type"]] += 1
        res, detail = sim.attribute()
        attr.update(res)
        for t, v in detail:
            by[(t, v)] += 1
    return types, attr, by


def pct(n, d):
    return f"{100*n/d:.0f}%" if d else "n/a"


def report(name, types, attr, by):
    rep = attr["classified"]
    two = attr["two_fault"]
    spf = attr["single_fault"] + attr["each_sufficient"]
    tot = sum(types.values())
    print(f"\n===== {name} =====")
    print(f" crashes: total {tot}  (rear_end {types['rear_end']}, "
          f"T-bone {types['right_angle']}, left_turn {types['left_turn']})")
    print(f" TWO-FAULT: {two} ({pct(two, rep)})   "
          f"SINGLE POINT OF FAILURE: {spf} ({pct(spf, rep)})")
    lt = sum(c for (t, v), c in by.items() if t == "left_turn")
    lt_two = sum(c for (t, v), c in by.items() if t == "left_turn" and v == "two-fault")
    print(f" left-turn crashes: {lt}, of which two-fault {lt_two} ({pct(lt_two, lt)})")


if __name__ == "__main__":
    print(f"demand={DEMAND}/approach, {SEEDS} seeds x {DURATION:.0f}s, mix={DRIVER}")
    for name, mode in (("PERMISSIVE LEFT", "permissive"),
                       ("PROTECTED-ONLY LEFT", "protected")):
        t, a, b = study(mode)
        report(name, t, a, b)
