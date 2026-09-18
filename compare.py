"""Compare the signalized intersection vs the roundabout on the single-point-of-
failure question, under the same demand and the same driver imperfections.

For each configuration: crash counts by type, and the counterfactual fault
attribution (how many crashes needed two mistakes vs one).

    python compare.py
"""
from collections import Counter

from sim import Simulation, Roundabout, SimConfig, DriverConfig

DEMAND = 0.08
SEEDS = 8
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)
MERGE = ("left_turn", "right_angle", "entry")   # crashes where two paths meet


def study(cls):
    types = Counter()
    sev = Counter()          # summed impact speed (severity proxy) by type
    attr = Counter()
    by = Counter()
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER))
        sim = cls(cfg)
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
    print(f"\n===== {name} =====")
    print(f" crashes: rear_end={types['rear_end']} right_angle={types['right_angle']} "
          f"left_turn={types['left_turn']} entry={types['entry']}  "
          f"(total {sum(types.values())})")
    crossing_sev = sev['right_angle'] + sev['left_turn']
    print(f" total impact severity (sum of closing speeds, m/s): "
          f"{sum(sev.values()):.0f}  "
          f"[crossing (T-bone+left) {crossing_sev:.0f}, "
          f"rear-end {sev['rear_end']:.0f}, entry {sev['entry']:.0f}]")
    print(f" fault attribution ({rep} classified):")
    print(f"   two-fault (needed two mistakes):     {two:3d}  ({pct(two, rep)})")
    print(f"   single point of failure (one enough):{spf:3d}  ({pct(spf, rep)})")
    merge = sum(c for (t, v), c in by.items() if t in MERGE)
    merge_two = sum(c for (t, v), c in by.items() if t in MERGE and v == "two-fault")
    print(f"   crossing/merge crashes: {merge}, of which two-fault {merge_two} "
          f"({pct(merge_two, merge)})")


if __name__ == "__main__":
    print(f"demand={DEMAND}/approach, {SEEDS} seeds x {DURATION:.0f}s, driver mix={DRIVER}")
    for name, cls in (("SIGNALIZED INTERSECTION", Simulation),
                      ("ROUNDABOUT", Roundabout)):
        t, sv, a, b = study(cls)
        report(name, t, sv, a, b)
