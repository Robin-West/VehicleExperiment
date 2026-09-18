# Vehicle Experiment — Signalized Intersection Safety

A thought experiment we can model and test: **how do the rules and imperfections
at a single traffic-light intersection change how often, and how badly, cars crash?**

Status: framing / spec. First build scope is fixed (see below).

---

## The question

Hold the intersection geometry and the traffic *demand* fixed. Introduce realistic
driver imperfection. Then ask: given a traffic light, **where does danger come from,
and which design + behavior knobs make it worse or better?**

The key modeling insight: with perfect, instantly-reacting, rule-obeying drivers a
signalized intersection basically never crashes. **Accidents emerge from the rules
interacting with driver imperfection.** So the failure model is the engine of the
experiment, not a footnote.

## Scope — first build

- **Control rule:** fixed-time traffic light only.
- **Later comparison arms (not now, but design for pluggability):** four-way stop,
  two-way stop / yield, roundabout.
- **Deliverables:** (1) a Python agent-based simulation + parameter sweeps + plots
  (the analysis backbone), then (2) an interactive top-down animation with live
  sliders and conflict counters.
- **Formalism:** light / intuitive. Rules and behaviors in plain language; equations
  available but not foregrounded.

## Geometry

- One intersection, two perpendicular two-lane roads: approaches N, S, E, W.
- Movements per approach: through, left, right. Right turns are low-risk (permitted on
  green, yield if on red). Left turns cross oncoming traffic — the main danger.
- Longitudinal motion by a standard car-following model (Intelligent Driver Model /
  IDM): each car's acceleration depends on its speed, gap to the car ahead, and
  closing rate. A red light is treated as a stopped obstacle at the stop line
  (unless the driver is violating it).

## Signal design (sub-knobs inside "traffic light")

- Phases: NS-green / EW-red, swap, with **yellow** + **all-red clearance** intervals.
- **Cycle length** and **green split** — throughput vs delay knob.
- **Yellow duration** — too short creates a "dilemma zone" (can't safely stop *or*
  clear) → rear-end and red-running risk.
- **Left turns: permissive vs protected.** Permissive (turn on green, yield to
  oncoming) exposes drivers to gap misjudgment. Protected (dedicated arrow) removes
  that conflict at a throughput cost.

## The failure model (all four knobs in the first build)

Each knob maps to a distinct real-world crash type — this is what makes the
hypotheses mechanistic:

| Knob | Parameter | Primary crash type it drives |
|------|-----------|------------------------------|
| **Reaction delay** | lag τ before responding to lead car / signal change | Rear-end |
| **Rule violations** | probability of running yellow/red (rises with speed & closeness at onset) | Right-angle (T-bone) — highest severity |
| **Gap misjudgment** | error in estimating oncoming speed/gap; unsafe-gap acceptance | Left-turn-opposing |
| **Limited sight lines** | occlusion distance before cross/oncoming traffic is visible | Amplifies all of the above (shrinks effective reaction time) |

## How we measure "accidents"

Real crashes are rare and noisy, so we also count near-misses (standard surrogate
safety measures):

- **Collisions** — two cars occupy the same space. Logged *by type* (rear-end /
  right-angle / left-turn) and by impact speed (severity proxy).
- **Time-to-Collision (TTC)** — count of moments where TTC between a pair drops below
  ~1.5 s. A strong danger signal without needing an actual hit.
- **Post-Encroachment Time (PET)** — time gap between one car leaving a conflict point
  and the next arriving. Small PET = close shave.
- **Throughput / delay** — vehicles cleared, average delay per vehicle, max queue.
  So every configuration gets a **safety score and a throughput score**.

## Hypotheses (falsifiable predictions)

1. Rear-end conflicts rise with reaction delay and with too-short yellow (dilemma zone).
2. Right-angle collisions scale with red-running probability *and* approach speed;
   they dominate the severity tally.
3. Permissive left-turn crashes rise sharply with gap-misjudgment error and with
   oncoming volume; protected-left phasing nearly eliminates them — at a throughput cost.
4. Limited sight lines amplify every failure mode nonlinearly, worst near capacity.
5. There is a safety–throughput frontier: pushing cycle/green for throughput degrades
   safety past a point. Some configurations are Pareto-dominated.

## Experiment matrix

- **Independent variables:** arrival rate (demand) × each failure knob × signal-design
  knobs (cycle, yellow, permissive/protected left).
- **Stochastic:** violations and misjudgment are random → Monte Carlo, N seeds per
  configuration, report means with confidence intervals.
- **Outputs:** safety-vs-delay scatter (Pareto view), crash-type breakdown vs each
  knob, space-time / conflict heatmaps.

## Validation / sanity checks

- Zero failure knobs → zero collisions.
- Rear-end count monotonic in reaction delay.
- Protected lefts sharply reduce left-turn collisions vs permissive.
- Delay grows with demand in the expected (Webster-like) way; queues stay bounded
  below capacity and blow up above it.

## Build order

1. Core sim: agents on four approaches, IDM longitudinal, pluggable **controller**
   (the rule) and pluggable **driver-error** model, metrics logging.
2. Signal controller (fixed-time, with the sub-knobs) + collision/TTC/PET detection.
3. Sweep harness + plots.
4. Interactive top-down visualization (sliders for demand, each error knob, signal
   timing; live crash/conflict counters; rule dropdown wired for future arms).
