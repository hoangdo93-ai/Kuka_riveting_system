# KUKA A380 Landing Flap Riveting Automation

A ROS2/C++ portfolio project implementing trajectory planning and motion
control for an automated riveting cell: a 6-DOF KUKA robot on a linear
rail, drilling a 6×7 grid of rivet holes across an Airbus A380 landing
flap (24 m × 3.5 m). Built as an application portfolio piece for an ETH
Zürich "Robotics – Control Engineer" position.

---

## What this project demonstrates

- Motion planning from first principles (cubic polynomial trajectories,
  velocity/acceleration-limited timing) - not just library calls
- A layered ROS2 architecture: pure algorithmic core → service-based
  planning → action-based execution → a swappable hardware backend
- Deliberate ROS2 idiom choices (Service for short request/response,
  Action for long-running cancelable work with feedback)
- A pluggable simulation/hardware boundary (`IMotionBackend`), so the
  exact same state machine drives either a `sleep()`-based simulation or
  a real TCP connection to a KUKA KRC4 controller
- Honest engineering judgment about safety: clear documentation of where
  this software's responsibility ends and certified hardware safety
  begins
- A verification discipline that goes beyond "it looks correct": Phase 6
  actually compiled and ran the full stack end-to-end, which is how
  **five real defects** were found and fixed (see [Verification](#verification-what-was-actually-tested))

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│ Phase 6: System Integration & Production Readiness            │
│   End-to-end tests, safety architecture, deployment checklist │
└───────────────────────────┬─────────────────────────────────────┘
                             │
        ┌────────────────────┴────────────────────┐
        │                                          │
┌───────▼────────────────┐          ┌──────────────▼───────────────┐
│ Phase 3: Path Planning  │          │ Phase 4-5: Motion Control     │
│ Node (ROS2 Service)     │◄─────────┤ Node (ROS2 Action Server)     │
│ "/path_planning/plan"   │  calls   │ "motion_control/execute_      │
│                         │  for     │  riveting"                    │
│                         │  pre-    │  State machine: IDLE →        │
│                         │  flight  │  PLANNING → EXECUTING →       │
│                         │  check   │  COMPLETED/ERROR              │
└───────────┬─────────────┘          └───────────┬────────────────────┘
            │                                     │
            │          both use                   │ delegates to
            └──────────────┬──────────────────────┘
                            │
        ┌───────────────────┴────────────────────┐
        │                                         │
┌───────▼────────────┐              ┌─────────────▼──────────────┐
│ Phase 1: Geometry   │              │ Phase 2: Trajectory         │
│ Calculator          │              │ Generator                   │
│ → 42 boring points  │              │ → cubic-polynomial motion   │
│   (6×7 grid)        │              │   between any two points    │
└─────────────────────┘              └─────────────────────────────┘
                                                    │
                                      ┌─────────────▼──────────────┐
                                      │ Phase 5: IMotionBackend      │
                                      │  ├─ SimulatedMotionBackend   │
                                      │  └─ KRC4MotionBackend        │
                                      │      → KRC4Interface (TCP)   │
                                      │      → real KRC4 controller  │
                                      │        or mock_krc4_server.py│
                                      └───────────────────────────────┘
```

---

## Entry Points: How Is This Actually Started?

There are **two independent ROS2 executables**, each with its own `main()`.
A natural first guess is that each node owns exactly one of the two core
algorithm headers - but that's not quite how it's built, and the
distinction matters for understanding the system:

### 1. `path_planning_node` (`path_planning_node_PHASE3.cpp` → `main()`)

Exposes the **`/path_planning/plan` service**. On each request it uses
**both** core modules together, not just geometry:

```
main() → PathPlanningNode
  ├─ geometry_calculator_.generate_boring_points(params)   [Phase 1]
  │    → 42 Point3D boring points
  └─ FOR each adjacent pair of points:
       trajectory_generator_.generate_point_to_point(...)  [Phase 2]
         → a full cubic-polynomial travel trajectory
```

It uses `geometry_calculator.hpp` to get the drilling grid, **and**
`trajectory_generator.hpp` to pre-compute and validate every travel
motion between consecutive points, packing all of it into the service
response (useful for pre-flight validation and visualization, e.g. in
RViz).

### 2. `motion_control_node` (`motion_control_node_PHASE5.cpp` → `main()`)

Exposes the **`motion_control/execute_riveting` action**. It also uses
**both** modules, for a different reason:

```
main() → MotionControlNode
  ├─ request_trajectory_plan()
  │    → calls /path_planning/plan as a pre-flight feasibility check
  │    → then geometry_calculator_.generate_boring_points(params) [Phase 1]
  │      regenerates the SAME 42 points locally and deterministically
  │      (same parameters → same points), rather than parsing the
  │      service's flattened waypoint array - see Phase 5 docs for why
  └─ execute_point_sequence(): for each point
       ├─ trajectory_generator_.generate_point_to_point(...) [Phase 2]
       │    → travel trajectory fed into the motion backend
       └─ motion_backend_->approach_point() / drill() / retract()
            (SimulatedMotionBackend or KRC4MotionBackend - Phase 5)
```

**So the accurate split is not "node A owns geometry, node B owns
trajectory" - both nodes depend on both Phase 1 and Phase 2, each using
them for a different purpose** (planning/validation vs. local,
deterministic execution control).

### Where does "simulation" actually live?

There isn't a single standalone "simulation file" that the system
launches - simulation is a **runtime-selectable backend**, not a
separate program. Three related-but-different things share that name,
worth distinguishing:

| File | What it actually is |
|------|----------------------|
| `simulated_motion_backend.hpp` | The **production** simulation path: an `IMotionBackend` implementation used *inside* `motion_control_node` whenever `hardware.use_real_hardware: false` (the default). This is what actually runs when you launch the real system without a robot attached. |
| `INTEGRATION_DEMO.cpp` | A **historical**, fully standalone Phase 1+2 demo written early in the project, before the `IMotionBackend` abstraction existed. Useful for understanding the core math in isolation, but superseded in practice. |
| `test_end_to_end_pipeline.cpp` (Phase 6) | The **current, verified** standalone demonstration: drives the actual production `GeometryCalculator`, `TrajectoryGenerator`, and `SimulatedMotionBackend` classes end-to-end outside of ROS2, and was actually compiled and run (see below) - the most trustworthy "simulation entry point" in the repo today. |

If the goal is "run the whole system in simulation", the real answer is:
launch `path_planning_node` and `motion_control_node` together with
`hardware.use_real_hardware: false` (the default) - not a separate
simulation executable.

---

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select kuka_riveting_system
source install/setup.bash

# Run both nodes (simulation mode by default)
ros2 launch kuka_riveting_system full_system.launch.py

# In another terminal: trigger a full riveting run
ros2 action send_goal /motion_control/execute_riveting \
    kuka_riveting/ExecuteRiveting "{}" --feedback

# Watch live status
ros2 topic echo /motion_control/status
```

To exercise the real hardware protocol path without physical hardware:

```bash
python3 mock_krc4_server.py --port 54600 --speed-factor 5.0
# then set hardware.use_real_hardware: true, hardware.robot_ip: "127.0.0.1"
# in motion_control_params.yaml and relaunch
```

See `DEPLOYMENT_CHECKLIST.md` for the full staged rollout procedure
toward real hardware.

---

## Verification: What Was Actually Tested

This project distinguishes between code that was *written and reasoned
about* and code that was *actually compiled and run* - the two are not
the same, and conflating them is how defects hide.

**Actually compiled and run:**
- Phase 1-2 core logic: 12 unit tests, 100% pass
- `KRC4Interface` / `KRC4MotionBackend`: integration tests against a live
  `mock_krc4_server.py` process
- `test_end_to_end_pipeline.cpp`: full 42-point simulated run, mid-run
  cancellation, degenerate-input handling - all actually executed
- `test_end_to_end_hardware.cpp`: the complete Geometry → Trajectory →
  real TCP protocol → mock controller round trip, actually executed

**Written to real conventions, not executable in this sandbox** (no ROS2
installation available here): the ROS2 node files themselves as running
processes, and `test_full_system_launch.py` (`launch_testing`).

**Found as a direct result of this discipline - five real defects**,
none of which were visible from reading the code alone:
1. `Point3D` was missing `distance_to()`, called throughout Phase 2
2. A stray `#endif` (leftover from `#pragma once` conversion) in two headers
3. `GeometryCalculator::generate_boring_points()` was called with an
   argument in three node files against a zero-argument API
4. An uncaught exception path in `motion_control_node` that could crash
   the entire node process on degenerate input
5. `path_planning_node`'s constructor never actually registered the
   `/path_planning/plan` service, and its handler used a placeholder
   string-based signature instead of the real ROS2 service types

All five are root-caused and fixed in the current files. See
`PHASE6_PRODUCTION_READINESS_SUMMARY.md` for the full writeup.

---

## Documentation Map

| Phase | Design / Implementation | Explanation (German) |
|-------|---------------------------|------------------------|
| 1-2 | `01_SYSTEM_DESIGN.md`, `FEHLERANALYSE_KORREKTUREN.md` | - |
| 3 | `PHASE3_ROS2_PATH_PLANNING_DESIGN.md`, `PHASE3_IMPLEMENTATION_SUMMARY.md` | `PHASE3_DEUTSCHE_ERKLAERUNG.md` |
| 4 | `PHASE4_COMPLETE_OVERVIEW.md` | `PHASE4_DEUTSCHE_ERKLAERUNG.md` |
| 5 | `PHASE5_COMPLETE_OVERVIEW.md` | `PHASE5_DEUTSCHE_ERKLAERUNG.md` |
| 6 | `PHASE6_PRODUCTION_READINESS_SUMMARY.md`, `PHASE6_SAFETY_ARCHITECTURE.md`, `DEPLOYMENT_CHECKLIST.md` | `PHASE6_DEUTSCHE_ERKLAERUNG.md` |

---

## Safety Notice

This software implements **supervisory-level** safety measures only
(motion limit validation, cancellation handling, connection monitoring).
It is **not** a substitute for certified hardware safety systems
(E-stop circuits, light curtains, safety-rated PLCs per ISO 13849-1 /
ISO 10218). Full hazard analysis and the layered safety model are in
`PHASE6_SAFETY_ARCHITECTURE.md`.

---

## Known Limitations

- The KRC4 wire protocol is a simplified, demonstrative approximation of
  KUKA.EKI, not the real (proprietary, project-specific) schema
- No collision detection against real environment geometry - only
  kinematic limit validation
- No real KUKA hardware was available to test against; all hardware-path
  testing used `mock_krc4_server.py`
- No mid-execution resume after a failure (each goal restarts at point 1)

Full list in `PHASE6_PRODUCTION_READINESS_SUMMARY.md`.

---

## Project Structure

```
include/kuka_riveting/
  geometry_calculator.hpp          trajectory_generator.hpp
  motion_backend_interface.hpp     simulated_motion_backend.hpp
  krc4_interface.hpp               krc4_motion_backend.hpp
  path_planning_node.hpp           motion_control_node.hpp
  e2e_pipeline_runner.hpp

src/
  path_planning_node_PHASE3.cpp    motion_control_node_PHASE5.cpp
  krc4_interface.cpp               krc4_motion_backend.cpp

srv/PlanTrajectory.srv             action/ExecuteRiveting.action

scripts/mock_krc4_server.py

config/path_planning_params.yaml   config/motion_control_params.yaml
launch/path_planning.launch.py     launch/motion_control.launch.py
launch/full_system.launch.py

test/
  test_end_to_end_pipeline.cpp     test_end_to_end_hardware.cpp
  test_full_system_launch.py       test_riveting_system.cpp

docs/  (all PHASE*.md, DEPLOYMENT_CHECKLIST.md, design documents)
```

---

## License

MIT (portfolio project)
