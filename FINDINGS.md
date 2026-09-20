# Can rule changes at an intersection prevent single-fault crashes?

Findings from an agent-based simulation of one standard `+` intersection.
Companion to [DESIGN.md](DESIGN.md) (framing) and [README.md](README.md) (how to run).

## TL;DR

We started from a sharp hypothesis: today's intersection rules are a
**single-point-of-failure** system (one driver's mistake is enough to cause a
crash), and could perhaps be redesigned so a crash **requires two vehicles to
each make a mistake** ("defense in depth") — removing any one driver's ability to
cause a collision alone. Seven experiments later, **we found no rule or guardrail
that forces that two-vehicle requirement** — that specific cure never
materialised. But a better principle did:

> **You don't make crashes need two mistakes. You make sure one mistake stays
> low-energy.** Safety comes from removing *impact energy* and *catastrophic
> crash geometries*, not from engineering a two-fault gate.

The single most effective lever was **speed governing near the intersection
(−55% injury)**; the best package was **interlock + protected-left phasing
(−65 to −76%)**. Notably, **stacking more measures can backfire** — a
governing + setback + interlock combo did *worse* than governing alone.

---

## 1. The question

A crash needs two vehicles in the same place at the same time. A **single-fault**
crash is one where *one* driver's error was sufficient — a victim simply happened
to be in the conflict zone when the erring driver arrived. The hypothesis, stated
precisely: most intersection crashes are single-fault, and the right rules or
guardrails could **force situations where a crash requires two vehicles to each
make a mistake** — so that any one driver's error, on its own, can no longer
cause a collision. If most crashes need only one mistake, converting them to need
two should sharply reduce the crash count.

## 2. The model (in one paragraph)

An agent-based micro-simulation: cars follow the Intelligent Driver Model along
fixed paths through a signalised `+` intersection; every crossing is resolved by
right-of-way gap acceptance. Four **driver-error knobs** (reaction delay, gap
misjudgement, limited sight, red-running) plus **speeding** corrupt those
decisions and produce crashes, classified by type (rear-end, right-angle/T-bone,
left-turn, roundabout entry-merge). Fault is assigned by a **but-for
counterfactual** (would fixing one driver alone have prevented it?) for crossing
crashes, and by a transparent rule for rear-ends (follower's fault unless a
speeder created the closing-speed differential). Harm is scored as an
**injury-weighted severity**: `type_weight × (closing speed)²`, with side-impacts
weighted above rear impacts (T-bone ×3, left-turn ×2.5, entry ×2, rear-end ×1),
because injury rises with impact energy and side-impacts are more injurious.

## 3. The diagnosis: today's rules are overwhelmingly single-fault

Over 438 crashes under a realistic driver mix, **96% were single-point-of-failure**
— one mistake was enough. Even the crossing crashes (T-bone + left-turn), the ones
most likely to involve two cars misbehaving, were single-fault ~80% of the time.
Rear-ends were the stubborn core: almost always the follower's mistake alone.
See `results/fault_attribution.png`.

This confirmed the hypothesis's *premise*. The rest of the project tested its
*cure*.

## 4. The seven experiments

### Structural findings (crash-type composition)

**Roundabout** (`results/compare_roundabout.png`). Because all traffic circulates
one way, **T-bone and left-turn crashes become geometrically impossible (→ 0).**
But it is rear-end- and entry-merge-heavy, so raw crash *count* rose. It removes
the severe crash *types* rather than making crashes need two faults. (Our model
does not reproduce roundabouts' real-world injury reduction — see Limitations.)

**Protected-only left phasing** (`results/compare_phasing.png`). Cuts left-turn
crashes (31→14) but the longer cycles add rear-ends (125→172); total crashes rose
(170→201). It *relocates* crashes rather than eliminating the single-fault nature.
This matches the real-world tradeoff: protected phasing reduces angle/left crashes
and increases rear-ends.

### Injury-weighted comparison (8 seeds, demand 0.08/approach, same driver mix)

All share the same baseline (injury score 13,850), so they rank directly:

| intervention | injury score | vs baseline | note |
|---|---|---|---|
| baseline (permissive fixed-time) | 13,850 | — | 96% single-fault |
| interlock only | 14,629 | +6% | T-bones already rare here → noise |
| stop-line setback 15 ft (+visibility) | 13,949 | +1% | but cut total crashes 170→110 and *raised* throughput |
| protected-left only | 9,246 | −33% | removes permissive-left conflict |
| **speed governing** | **6,199** | **−55%** | **best single lever** |
| combined: govern + setback + interlock | 7,637 | −45% | **worse than governing alone** |
| interlock + protected-left | 4,861 | −65% | compatible mechanisms stack |
| interlock + protected + green-extension | **3,332** | **−76%** | **best overall** |

Figures: `results/compare_govern.png`, `results/compare_safe.png`,
`results/compare_setback.png`, `results/compare_combo.png`.

**Speed governing (−55%)** is the standout single lever. Injury scales with kinetic
energy (∝ v²), so capping speed in the conflict zone drains energy from *every*
crash and removes speeding-caused crashes outright.

**The safety-gate package (−76%)** is the best overall. The **interlock** (hold a
conflicting green until the box is verifiably clear) is a genuine two-fault gate
for T-bones built from current sensors + signals — but its benefit only shows when
red-running is common; alone in this mix it was noise. **Protected-left** does the
heavy lifting by removing the frequent left-turn conflict; **green-extension**
trims the rear-end residue.

**Stop-line setback** (`results/compare_setback.png`) has two effects that fight:
better sight of cross traffic (helps) versus a longer runway that lets cars enter
the box *faster* (hurts). Net: a shallow sweet spot near 10–20 ft, worse if pushed
further. Note it *did* cut total crashes and improve throughput — a flow benefit,
not a severity one.

**The combined package underperformed** (−45% vs governing's −55%): the setback
lets cars reach the governed speed *before* the box, so they cross the conflict
zone faster — partly cancelling governing. More measures were not safer here.

## 5. What we learned

1. **Harm is about energy, not fault-counting.** Every lever that won reduced
   impact energy or removed a high-energy geometry. The metric that mattered was
   injury-weighted severity — raw crash *count* repeatedly hid the benefit
   (good interventions often *add* minor rear-ends while removing severe crashes).

2. **We did not find any rule or guardrail that forces a two-vehicle-mistake
   requirement.** This was the central goal, and it is the clearest negative
   result of the project: **none** of the seven interventions changed the
   underlying logic so that a crash requires *two vehicles to each err*. Instead
   they *eliminated* a crash type (roundabout, protected-left), *drained energy*
   (governing), or *relocated* crashes (protected → more rear-ends) — and the
   fault attribution showed the two-fault share staying flat or *falling*, never
   rising because we had built a genuine gate. The surviving crashes were, if
   anything, *more* single-fault (rear-ends).

   The one conceptual near-miss was the **interlock**: to cause a T-bone you would
   need a driver to run the light *and* the interlock to fail to hold the cross
   traffic. But that is "a driver's error **plus a sensor/infrastructure
   failure**," not two *drivers* each making a mistake — and we never modelled
   interlock failures, so we did not even demonstrate it empirically as a gate; in
   this driver mix its measured benefit was marginal. The deeper reason a true
   two-vehicle gate never appeared: without in-car technology (AEB / V2X, excluded
   as not yet ubiquitous) you cannot place a second check *on the erring driver*,
   and rear-ends — the dominant residual crash — are intrinsically a single
   driver's fault with no second vehicle to hold.

3. **Combining helps only when mechanisms are compatible.** Interlock + protected
   stacked (both remove crossing crashes). Governing + setback interfered (setback's
   entry-speed works against governing's speed cap). Layering measures blindly can
   make them cancel.

4. **Rear-ends are the irreducible single-fault residue.** No intersection rule
   gates them — the follower's error alone is sufficient, and there is no other
   traffic to hold. Every measure that adds stopping adds rear-ends. Reducing them
   needs following-distance / auto-braking enforcement (in-car tech), which was out
   of scope.

5. **The benefit, when it came, was a severity downgrade — not fewer crashes.**
   Most levers did *not* reduce the number of accidents: governing and interlock
   left the count roughly flat, and the roundabout, protected phasing, and the
   interlock+protected package actually *increased* it — almost entirely by
   substituting **single-fault rear-end collisions** for the crossing crashes they
   removed. Because rear-ends are lower-severity, injury-weighted harm still fell,
   so this was a genuine safety gain — but it came as a *downgrade of crash type*
   (violent side-impact → minor rear-end), not as the intended reduction in the
   *number* of collisions. The lone exception was the **stop-line setback**, which
   reduced the total count and improved throughput while barely moving severity —
   the mirror image of every other lever.

## 6. Practical takeaways

- **Biggest, simplest win:** enforce speed in the conflict zone (−55% injury).
- **Best package with current infrastructure:** sensor-actuated protected-left +
  dynamic all-red interlock (−65 to −76% injury), no in-car tech required.
- **Use severity, not crash count,** to judge intersection safety — count rewards
  the wrong thing.
- **Don't stack measures blindly** — check that their mechanisms don't fight.

## 7. Limitations (read before trusting a number)

- **Injury proxy.** Severity = `type_weight × (closing speed)²` is a stand-in for
  real injury outcomes, not calibrated to crash data.
- **Flat sight model.** Drivers perceive conflicts within a fixed radius; occlusion
  and the setback's visibility benefit were hand-modelled assumptions, and the real
  sight-triangle physics can point the other way.
- **Roundabout is rear-end-heavy** and does not reproduce roundabouts' real
  injury reduction; trust its *structural* result (no crossing crashes), not its
  totals.
- **Fault attribution evolved mid-project** (a but-for counterfactual for crossing
  crashes; a speed-at-impact rule for rear-ends). Two-fault percentages across the
  earliest experiments are therefore not perfectly comparable; the injury-weighted
  severity is the consistent headline.
- **One intersection, one demand level (0.08/approach), one driver mix, 6–8 random
  seeds.** Results are directional; injury is high-variance (dominated by rare
  high-speed crashes). Treat single-digit-percent differences as noise.
- **No pedestrians, no cyclists, single vehicle class, right-on-green only.**

## 8. Reproduce

```bash
pip install numpy matplotlib
python run.py               # headline scenarios + validation
python attribute.py         # fault diagnosis (single vs two-fault)
python compare.py           # signalized vs roundabout
python compare_phasing.py   # permissive vs protected-left
python compare_govern.py    # speed governing
python compare_safe.py      # interlock / protected / green-extension package
python compare_setback.py   # stop-line setback sweep (10-50 ft)
python compare_combo.py     # governing + setback + interlock package
```

Model code is in `sim/`; figures land in `results/`.
