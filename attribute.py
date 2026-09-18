"""Fault attribution: what fraction of crashes required TWO mistakes?

Tests the hypothesis that the current rules are a single-point-of-failure system.
For each crash, a counterfactual replay asks whether correcting either driver
alone would have prevented it (see Simulation.attribute).

    python attribute.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig


CROSSING = ("left_turn", "right_angle")   # crashes that involve two paths meeting


def run(seeds=8, duration=500.0, **driver_kw):
    agg, by_type = Counter(), Counter()
    for sd in range(seeds):
        cfg = SimConfig(duration=duration, seed=sd, driver=DriverConfig(**driver_kw))
        sim = Simulation(cfg)
        sim.run()
        res, detail = sim.attribute()
        agg.update(res)
        for typ, verdict in detail:
            by_type[(typ, verdict)] += 1
    return agg, by_type


def pct(n, d):
    return f"{100*n/d:.0f}%" if d else "n/a"


if __name__ == "__main__":
    # a realistic mix of imperfections, including speeding
    agg, by_type = run(
        reaction_delay=0.7, p_run=0.15, gap_error=0.4, sight_distance=35.0,
        p_speed=0.30, speed_excess=0.35)

    rep = agg["classified"]
    print(f"crashes: {agg['total']} total, {rep} classified, "
          f"{agg['skipped']} skipped (too little history)\n")

    def block(title, types):
        n = {v: sum(c for (t, vv), c in by_type.items() if t in types and vv == v)
             for v in ("two-fault", "single-fault", "each-sufficient")}
        tot = sum(n.values())
        two = n["two-fault"]
        spf = n["single-fault"] + n["each-sufficient"]
        print(f"{title} ({tot} crashes)")
        print(f"   TWO-FAULT  (needed two mistakes):     {two:3d}  ({pct(two, tot)})")
        print(f"   SINGLE POINT OF FAILURE (one enough): {spf:3d}  ({pct(spf, tot)})")
        print()

    block("ALL crashes", ("rear_end",) + CROSSING)
    block("CROSSING crashes only (left-turn + right-angle)", CROSSING)
    block("REAR-END crashes only", ("rear_end",))

    print("raw by type and verdict:")
    for k, v in sorted(by_type.items()):
        print(f"   {k[0]:12s} {k[1]:16s} {v}")
