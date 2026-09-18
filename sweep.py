"""Parameter sweeps + plots for the signalized-intersection experiment.

Each failure knob is swept independently (Monte Carlo over seeds), then the
permissive vs protected left-turn signal designs are compared. Figures are
written to results/.

    python sweep.py             # full sweep (slower)
    python sweep.py --quick     # fewer seeds / points for a fast look
"""
from __future__ import annotations

import argparse
import statistics
from dataclasses import replace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim import Simulation, SimConfig, DriverConfig, SignalConfig

RESULTS = "results"


def run_point(seeds, duration=300.0, signal=None, **driver_kw):
    """Average metrics over `seeds` random seeds for one configuration."""
    keys = ["rear_end", "right_angle", "left_turn", "collisions_total",
            "ttc_conflicts", "pet_events", "avg_delay_s", "cleared"]
    acc = {k: [] for k in keys}
    for sd in range(seeds):
        cfg = SimConfig(duration=duration, seed=sd,
                        driver=DriverConfig(**driver_kw),
                        signal=signal or SignalConfig())
        r = Simulation(cfg).run()
        for k in keys:
            acc[k].append(r[k])
    return {k: statistics.mean(v) for k, v in acc.items()}


def sweep(param, values, seeds, duration, base=None):
    base = base or {}
    out = []
    for val in values:
        kw = dict(base)
        kw[param] = val
        out.append((val, run_point(seeds, duration=duration, **kw)))
        print(f"  {param}={val}: ", {k: round(v, 2) for k, v in out[-1][1].items()})
    return out


def plot_sweep(data, xlabel, title, fname, series):
    xs = [d[0] for d in data]
    plt.figure(figsize=(7, 4.5))
    for key, label, style in series:
        plt.plot(xs, [d[1][key] for d in data], style, label=label)
    plt.xlabel(xlabel)
    plt.ylabel("events per 300 s")
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"{RESULTS}/{fname}", dpi=110)
    plt.close()
    print(f"  wrote {RESULTS}/{fname}")


def main():
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    os.makedirs(RESULTS, exist_ok=True)
    seeds = 6 if args.quick else 20
    dur = 200.0 if args.quick else 400.0

    coll = [("rear_end", "rear-end", "o-"),
            ("right_angle", "right-angle (T-bone)", "s-"),
            ("left_turn", "left-turn", "^-")]

    print("1. reaction delay -> rear-end")
    d = sweep("reaction_delay", [0, 0.5, 1.0, 1.5, 2.0, 2.5], seeds, dur)
    plot_sweep(d, "reaction delay (s)", "Reaction delay vs collisions",
               "sweep_reaction_delay.png", coll)

    print("2. red-light running -> right-angle")
    d = sweep("p_run", [0, 0.1, 0.2, 0.3, 0.5, 0.7], seeds, dur,
              base={"reaction_delay": 0.6, "sight_distance": 40.0})
    plot_sweep(d, "red-running probability", "Red-light running vs collisions",
               "sweep_red_running.png", coll)

    print("3. gap misjudgement -> left-turn")
    d = sweep("gap_error", [0, 0.2, 0.4, 0.6, 0.8], seeds, dur,
              base={"reaction_delay": 0.4})
    plot_sweep(d, "gap misjudgement (std-dev)", "Gap misjudgement vs collisions",
               "sweep_gap_misjudge.png", coll)

    print("4. limited sight -> amplifies")
    d = sweep("sight_distance", [200, 60, 40, 30, 20, 15], seeds, dur,
              base={"reaction_delay": 0.6, "p_run": 0.1})
    plot_sweep(d, "sight distance (m)  [smaller = worse]",
               "Sight distance vs collisions", "sweep_sight.png", coll)

    print("5. permissive vs protected left (safety vs demand)")
    rates = [0.06, 0.10, 0.14, 0.18, 0.22]
    perm, prot = [], []
    for rate in rates:
        base = dict(reaction_delay=0.8, gap_error=0.4, p_run=0.1, sight_distance=40.0)
        cfgp = SignalConfig(mode="permissive")
        cfgq = SignalConfig(mode="protected")
        pp = run_point(seeds, duration=dur, signal=cfgp,
                       **base) if True else None
        # reuse run_point via monkey: it doesn't take arrival_rate, so inline:
        perm.append((rate, _run_rate(seeds, dur, rate, cfgp, base)))
        prot.append((rate, _run_rate(seeds, dur, rate, cfgq, base)))
        print(f"  rate={rate}: perm lt={perm[-1][1]['left_turn']:.1f} "
              f"prot lt={prot[-1][1]['left_turn']:.1f}")

    plt.figure(figsize=(7, 4.5))
    plt.plot(rates, [p[1]["left_turn"] for p in perm], "^-", label="permissive left")
    plt.plot(rates, [p[1]["left_turn"] for p in prot], "s-", label="protected left")
    plt.xlabel("arrival rate per approach (veh/s)")
    plt.ylabel("left-turn collisions per run")
    plt.title("Permissive vs protected left turns")
    plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout()
    plt.savefig(f"{RESULTS}/permissive_vs_protected.png", dpi=110)
    plt.close()
    print(f"  wrote {RESULTS}/permissive_vs_protected.png")

    # safety-vs-delay frontier
    plt.figure(figsize=(7, 4.5))
    plt.scatter([p[1]["avg_delay_s"] for p in perm],
                [p[1]["collisions_total"] for p in perm], marker="^", label="permissive")
    plt.scatter([p[1]["avg_delay_s"] for p in prot],
                [p[1]["collisions_total"] for p in prot], marker="s", label="protected")
    plt.xlabel("average delay (s)")
    plt.ylabel("total collisions per run")
    plt.title("Safety vs delay")
    plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout()
    plt.savefig(f"{RESULTS}/safety_vs_delay.png", dpi=110)
    plt.close()
    print(f"  wrote {RESULTS}/safety_vs_delay.png")


def _run_rate(seeds, dur, rate, signal, driver_kw):
    import statistics
    keys = ["left_turn", "right_angle", "rear_end", "collisions_total", "avg_delay_s"]
    acc = {k: [] for k in keys}
    for sd in range(seeds):
        cfg = SimConfig(duration=dur, seed=sd, arrival_rate=rate,
                        driver=DriverConfig(**driver_kw), signal=signal)
        r = Simulation(cfg).run()
        for k in keys:
            acc[k].append(r[k])
    return {k: statistics.mean(v) for k, v in acc.items()}


if __name__ == "__main__":
    main()
