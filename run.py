"""Run the experiment's headline scenarios and print a metrics table.

Averages over a few random seeds (violations/misjudgement are stochastic).

    python run.py

Findings this reproduces:
  * validation      - careful drivers barely crash.
  * reaction delay  - drives rear-ends (the dominant single-failure effect).
  * limited sight   - drives rear-ends and left-turn crashes.
  * red-running     - drives right-angle T-bones; worse with a short all-red
                      clearance interval, and shows up strongly as PET near-misses.
  * gap misjudge    - inflates left-turn conflicts; becomes crashes when the
                      other driver also can't react (delay / limited sight).
  * protected left  - removes the permissive left-turn conflict.
"""
from sim import Simulation, SimConfig, DriverConfig, SignalConfig

COLS = ["rear_end", "right_angle", "left_turn", "ttc_conflicts",
        "pet_events", "avg_delay_s"]


def avg(seeds=4, duration=300.0, signal=None, **driver_kw):
    tot = {c: 0.0 for c in COLS}
    for sd in range(seeds):
        cfg = SimConfig(duration=duration, seed=sd,
                        driver=DriverConfig(**driver_kw),
                        signal=signal or SignalConfig())
        r = Simulation(cfg).run()
        for c in COLS:
            tot[c] += r[c]
    return {c: tot[c] / seeds for c in COLS}


def row(label, m):
    print(f"{label:34s} re={m['rear_end']:5.1f} ra={m['right_angle']:4.1f} "
          f"lt={m['left_turn']:4.1f} | ttc={m['ttc_conflicts']:5.0f} "
          f"pet={m['pet_events']:5.0f} delay={m['avg_delay_s']:4.1f}")


if __name__ == "__main__":
    print("per-run averages over 4 seeds x 300 s (collisions by type):\n")
    row("baseline (careful drivers)", avg())
    row("reaction delay 1.2 s", avg(reaction_delay=1.2))
    row("limited sight 20 m", avg(sight_distance=20.0))
    row("red-running p=0.4", avg(p_run=0.4))
    row("red-running p=0.4, short all-red",
        avg(p_run=0.4, signal=SignalConfig(all_red=0.0)))
    row("gap misjudge 0.6 (alone)", avg(gap_error=0.6))
    row("gap misjudge 0.6 + delay 0.8", avg(gap_error=0.6, reaction_delay=0.8))
    row("ALL failures", avg(reaction_delay=1.0, p_run=0.2,
                            gap_error=0.4, sight_distance=30.0))
    print()
    base = dict(reaction_delay=0.8, gap_error=0.4, sight_distance=40.0)
    row("permissive left (busy)",
        avg(duration=300.0, signal=SignalConfig(mode="permissive"), **base))
    row("protected left (busy)",
        avg(duration=300.0, signal=SignalConfig(mode="protected"), **base))
