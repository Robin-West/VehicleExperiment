"""Fixed-time traffic signal controller.

Two modes for the left turn, since that is where a lot of the safety story lives:

  * permissive  - left turns share the through green and must yield to oncoming
                  through traffic (exposes drivers to gap misjudgement).
  * protected   - a leading left-turn phase runs while opposing through is red,
                  so protected lefts never conflict with oncoming through.

State per movement is one of "G" (green), "Y" (yellow), "R" (red).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .geometry import ORIGINS, MOVES, THROUGH, LEFT, RIGHT

GREEN, YELLOW, RED = "G", "Y", "R"
NS = ("S", "N")
EW = ("E", "W")


def _all_red():
    return {o: {m: RED for m in MOVES} for o in ORIGINS}


def _set(states, origins, moves, val):
    for o in origins:
        for m in moves:
            states[o][m] = val
    return states


@dataclass
class SignalConfig:
    mode: str = "permissive"        # "permissive" or "protected"
    green_main: float = 25.0        # through/right green per direction (s)
    green_left: float = 6.0         # protected left green (protected mode only)
    yellow: float = 3.5             # yellow interval (s) -- the dilemma-zone knob
    all_red: float = 1.5            # all-red clearance (s)


class SignalController:
    def __init__(self, cfg: SignalConfig):
        self.cfg = cfg
        self.intervals = self._build()          # list of (duration, name, states)
        self.cycle = sum(d for d, _, _ in self.intervals)

    def _build(self):
        c = self.cfg
        iv = []
        if c.mode == "permissive":
            for grp in (NS, EW):
                g = _set(_all_red(), grp, MOVES, GREEN)
                iv.append((c.green_main, f"{grp[0]}{grp[1]}_green", g))
                y = _set(_all_red(), grp, MOVES, YELLOW)
                iv.append((c.yellow, f"{grp[0]}{grp[1]}_yellow", y))
                iv.append((c.all_red, "all_red", _all_red()))
        elif c.mode == "protected":
            for grp in (NS, EW):
                lg = _set(_all_red(), grp, (LEFT,), GREEN)
                iv.append((c.green_left, f"{grp[0]}{grp[1]}_left_green", lg))
                ly = _set(_all_red(), grp, (LEFT,), YELLOW)
                iv.append((c.yellow, f"{grp[0]}{grp[1]}_left_yellow", ly))
                tg = _set(_all_red(), grp, (THROUGH, RIGHT), GREEN)
                iv.append((c.green_main, f"{grp[0]}{grp[1]}_thru_green", tg))
                ty = _set(_all_red(), grp, (THROUGH, RIGHT), YELLOW)
                iv.append((c.yellow, f"{grp[0]}{grp[1]}_thru_yellow", ty))
                iv.append((c.all_red, "all_red", _all_red()))
        else:
            raise ValueError(f"unknown signal mode {c.mode!r}")
        return iv

    def _phase(self, t: float):
        tc = t % self.cycle
        acc = 0.0
        for dur, name, states in self.intervals:
            if tc < acc + dur:
                return name, states, tc - acc, dur
            acc += dur
        # numerical edge case
        dur, name, states = self.intervals[-1]
        return name, states, dur, dur

    def state(self, origin: str, move: str, t: float) -> str:
        _, states, _, _ = self._phase(t)
        return states[origin][move]

    def phase_name(self, t: float) -> str:
        return self._phase(t)[0]
