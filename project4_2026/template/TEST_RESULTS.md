# Exercises 1 and 2: verification record

Both `Project4Pendulum` and `Project4Car` build successfully with the starter's
`make` in `ompl_env` (OMPL 2.0.0, Python 3.12.14, C++17, AppleClang 21, macOS).
`-Wall -Wextra -Wpedantic` produced no warnings. Linux was not executed here.
The initial build reproduced six collision-helper declaration errors; the final
build has none. CollisionChecking.h and both RG-RRT placeholder files are unchanged.

The table records the actual seeds 1, 2, and 3, with a 30-second per-run solve
budget. Each pass is an **exact** solution within the full state-space goal
region and passed validation before export. These are functional smoke tests;
Exercise 4 benchmarking is not implemented and no statistical comparison is claimed.

| System | Planner | Torque / environment | Exact runs | Solve time range (s) | Motion duration range (s) | Largest goal distance |
|---|---|---|---|---|---|---|
| Pendulum | RRT | 3 | 3/3 | 0.2724-0.9462 | 8.10-8.45 | 0.096322 |
| Pendulum | RRT | 5 | 3/3 | 0.2356-0.6023 | 4.75-5.40 | 0.074214 |
| Pendulum | RRT | 10 | 3/3 | 0.0452-0.2356 | 2.00-3.00 | 0.064797 |
| Pendulum | KPIECE1 | 3 | 3/3 | 0.0063-0.4427 | 9.75-13.25 | 0.085828 |
| Pendulum | KPIECE1 | 5 | 3/3 | 0.0098-0.0187 | 6.35-9.00 | 0.078669 |
| Pendulum | KPIECE1 | 10 | 3/3 | 0.0038-0.2668 | 4.40-9.15 | 0.093445 |
| Car | RRT | Street | 3/3 | 0.1579-20.3690 | 18.10-35.05 | 0.498120 |
| Car | KPIECE1 | Street | 3/3 | 0.0953-2.7283 | 16.15-30.25 | 0.457869 |

Goal thresholds are 0.1 for the pendulum and 0.5 for the car in the documented
compound metrics. All 24 paths reach their goal regions. All state velocities,
workspace positions, headings, and controls respect their bounds. Both car
planners use the same nontrivial street, controls, dynamics, and start/goal.

Validation evidence:

- Three CTest checks passed: `collisions`, `pendulum_model`, `car_model`.
  Deliberate invalid pendulum/car velocities, out-of-bounds car position, and
  points inside both street obstacles are rejected. Reverse car states are
  accepted. Point inside/outside/boundary, circle contact, and rotated and
  contained square cases pass. Model/projection/propagation checks pass.
- Every successful C++ run passes `PathControl::check()`, start/goal checks,
  control bounds, and dense (at most 5 ms) propagated-state validity checks.
- All 24 exported trajectories pass independent SciPy DOP853 integration with
  collision/bounds checks at at most 2.5 ms intervals. This checks both the
  original control endpoints and every plotted/interpolated state.
  The maximum local component discrepancy is 1.2e-09 (threshold 1e-5).
- Both executables were deliberately given a 1-microsecond solve budget:
  they returned failure explicitly and exported no solution states/controls.
- RG-RRT and benchmark menu choices report unimplemented status without
  crashing; empty stdin terminates cleanly with an error.
- Successful interactive menu runs were also checked for pendulum RRT and
  car KPIECE1, including exact solution status and trajectory export.
  Their console output is in `output/logs/interactive_checks.log`.
- All 24 state/control/metadata exports exist. The seed-1 solutions produce
  11 PNG figures, visually inspected for periodic-angle seams, obstacle
  alignment, path avoidance, and goal arrival. The one-page checkpoint PDF
  was rendered and visually reviewed.

Evidence files and commands:

- `output/run_results.json`, `output/logs/`: actual per-run status and console logs.
- `output/validation.json`: independent per-trajectory validation results.
- `output/trajectories/`: actual OMPL state matrices, controls/durations, metadata.
- `output/street.txt`: rectangles exported directly from `makeStreet()`.
- `output/plots/`: pendulum and car PNGs, including comparison figures.
- `output/pdf/checkpoint_report.pdf`: one-page checkpoint report.
- Reproduction commands are in `README`; scripts reside in `src/`.

No tested configuration failed. Sampling planners can fail on other seeds or
with shorter solve budgets; the slowest observed run was car RRT seed 2
(20.369 s). Paths are feasible examples rather than optimized motions.
Collision validation is numerical and densely sampled, not a formal proof
of continuous collision avoidance. The point-car model is intentional.
Exercises 1 and 2 are implemented and tested. Exercises 3 and 4 remain outside
scope. Add individual contribution details and NetIDs before submission.
