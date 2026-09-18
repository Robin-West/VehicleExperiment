"""Speed governing near the intersection: does removing speeding as a cause
eliminate crashes (rather than relocate them)?

    python compare_govern.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

DEMAND = 0.08
SEEDS = 8
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)


def study(govern):
    types, sev, attr, by = Counter(), Counter(), Counter(), Counter()
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER),
                        signal=SignalConfig(mode="permissive"), speed_govern=govern,
                        govern_dist=150.0)   # govern the whole approach (no boundary wave)
        sim = Simulation(cfg)
        sim.run()
        for c in sim.metrics.collisions:
            types[c["type"]] += 1
            sev[c["type"]] += c["severity"]
        res, detail = sim.attribute()
        attr.update(res)
        for t, v in detail:
            by[(t, v)] += 1
    return types, sev, attr, by


def pct(n, d):
    return f"{100*n/d:.0f}%" if d else "n/a"


def report(name, types, sev, attr, by):
    rep = attr["classified"]
    two = attr["two_fault"]
    spf = attr["single_fault"] + attr["each_sufficient"]
    tot = sum(types.values())
    print(f"\n===== {name} =====")
    print(f" crashes: total {tot}  (rear_end {types['rear_end']}, "
          f"T-bone {types['right_angle']}, left_turn {types['left_turn']})")
    print(f" total impact severity (closing speed, m/s): {sum(sev.values()):.0f}")
    print(f" TWO-FAULT: {two} ({pct(two, rep)})   "
          f"SINGLE POINT OF FAILURE: {spf} ({pct(spf, rep)})")


if __name__ == "__main__":
    print(f"demand={DEMAND}/approach, {SEEDS} seeds x {DURATION:.0f}s, mix={DRIVER}")
    for name, gov in (("NO GOVERNING (speeders speed through)", False),
                      ("SPEED-GOVERNED near intersection", True)):
        t, sv, a, b = study(gov)
        report(name, t, sv, a, b)
