"""Move the stop line back (standard permissive intersection, same driver mix).

Two series:
  - geometry only:   the stop line moves back, sight unchanged (timing/clearance effect)
  - + visibility:    the setback also improves cross-traffic sight (the hypothesis)

Sweeps 0/10/20/30/40/50 ft.

    python compare_setback.py
"""
from collections import Counter

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

DEMAND = 0.08
SEEDS = 6
DURATION = 400.0
DRIVER = dict(reaction_delay=0.7, gap_error=0.4, sight_distance=35.0,
              p_speed=0.30, speed_excess=0.35, p_run=0.15)
WEIGHT = {"right_angle": 3.0, "left_turn": 2.5, "entry": 2.0, "rear_end": 1.0}
FEET = [0, 10, 20, 30, 40, 50]
FT_M = 0.3048


def study(setback_m, sight):
    types, injury = Counter(), 0.0
    for sd in range(SEEDS):
        cfg = SimConfig(duration=DURATION, seed=sd, arrival_rate=DEMAND,
                        driver=DriverConfig(**DRIVER),
                        signal=SignalConfig(mode="permissive"),
                        stop_setback=setback_m, setback_sight=sight)
        sim = Simulation(cfg)
        sim.run()
        for c in sim.metrics.collisions:
            types[c["type"]] += 1
            injury += WEIGHT.get(c["type"], 1.0) * c["severity"] ** 2
    return types, injury


def series(name, sight):
    print(f"\n--- {name} ---")
    print(f"{'setback':>8} {'total':>6} {'rear':>5} {'Tbone':>6} {'left':>5} {'injury':>8}")
    for ft in FEET:
        t, inj = study(ft * FT_M, sight)
        print(f"{ft:>5}ft {sum(t.values()):>6} {t['rear_end']:>5} {t['right_angle']:>6} "
              f"{t['left_turn']:>5} {inj:>8.0f}")


if __name__ == "__main__":
    print(f"demand={DEMAND}/approach, {SEEDS} seeds x {DURATION:.0f}s, mix={DRIVER}")
    series("GEOMETRY ONLY (sight unchanged)", sight=False)
    series("GEOMETRY + VISIBILITY (setback improves sight)", sight=True)
