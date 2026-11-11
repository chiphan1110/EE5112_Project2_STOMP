# Task 4: Collision-Avoidance Scenarios for the UR5

Three new planning scenarios for the UR5 with several obstacles each. In every scenario the
straight-line initial path collides, and STOMP has to find a way around the objects.

## Scenarios

| ID | Name | Obstacles (besides the table) | Challenge |
|---|---|---|---|
| 1 | A: cluttered table | tall box, bottle (cylinder), small box | lift the wrist over three objects of different heights |
| 2 | B: wall and ceiling | low wall, ceiling plate above it | reach over the wall and under the ceiling at the same time |
| 3 | C: kitchen shelf | tall pillar, shelf (two boards and a back wall) | pass the pillar, then enter a 38 cm shelf opening, holding a "cup" sideways |

All scenario data (poses, IK seed, obstacles, plot settings and, for C, STOMP settings) are in
`scenarioUR5.m`. Adding a scenario means adding one `case` block there.

## How to run

From this folder in MATLAB:

```matlab
scenarioID = 1; check_scenario                   % optional: sanity checks before planning
scenarioID = 1; rng(0); UR5_STOMP_Path_Planning  % plan scenario A (1 = A, 2 = B, 3 = C)
plot_task4_results                               % figures of that scenario
difficulty_table                                 % all three scenarios side by side (no videos, about 1 min)
```

**Expected results (`rng(0)`):** all three scenarios are collision-free on the first attempt,
after 25 (A), 12 (B) and 46 (C) iterations.

### Options

Set these variables **before** running `UR5_STOMP_Path_Planning`:

| Variable | Default | Effect |
|---|---|---|
| `scenarioID` | `1` | Scenario to plan: 1 = A, 2 = B, 3 = C |
| `enableVideoTraining` | `1` | Record the training video (`0` to skip) |
| `enableVideo` | `1` | Record the final-path video (`0` to skip) |
| `displayAnimation` | `1` | Animate the final path in the scene figure (`0` to skip) |

## Files

### Main scripts

| File | Purpose |
|---|---|
| `UR5_STOMP_Path_Planning.m` | Entry point: loads the scenario, solves IK, prepares the initial path, runs STOMP (with restarts) |
| `scenarioUR5.m` | Definitions of scenarios A, B and C |
| `helperSTOMP.m` | STOMP set-up and main loop, final collision check, videos, saving the path |
| `check_scenario.m` | Checks that IK succeeds, start and goal are collision-free, and the initial path does collide |
| `plot_task4_results.m` | Path overlay and cost figure of the current scenario |
| `difficulty_table.m` | Runs all scenarios and compares how hard they are |

### Helpers and STOMP functions

| File | Purpose |
|---|---|
| `helperCreateObstaclesUR5.m` | Builds any number of boxes and vertical cylinders, both as collision objects (display and final check) and in the voxel grid (STOMP cost) |
| `helperInitialVisualizerUR5.m` | Scene figure with the UR5 at the start configuration |
| `computeScrewAxes.m`, `updateJointsWorldPosition.m` | PoE forward kinematics (from Task 3) |
| `stomp*.m`, `sEDT_3d.m` | STOMP functions and distance transform (as in Tasks 1–3) |

## Outputs

| File | Content |
|---|---|
| `Task4_<scenario>_overlay.png` | Robot at 7 waypoints of the initial guess (left) and the STOMP path (right), obstacles in scenario colours |
| `Task4_<scenario>_cost.png` | Total cost per iteration (iteration 0 = initial guess) |
| `Task4_<scenario>_training.mp4` | Path after selected iterations |
| `Task4_<scenario>_planned_path.mp4` | Final path, waypoint by waypoint |
| `Task4_difficulty_table.txt` | Output of `difficulty_table.m` |
| `Theta_UR5_<scenario>_nDisc20_nPaths_<N>.mat` | Final path `theta` of each scenario |

`difficulty_table.m` reports, per scenario:

- colliding waypoints and obstacle cost of the initial path;
- attempts, iterations and STOMP time;
- whether the final path is collision-free;
- the smallest real clearance between robot and obstacles;
- how far the gripper moved compared with the initial path.

## Implementation notes

- **Stopping rule:** STOMP continues while the total cost still changes by more than 0.1 **or**
  any collision sphere is still inside the 5 cm safety margin (up to the iteration limit).
- **Settings per scenario:** A and B use 20 samples, exploration scale 1.5 and 50 iterations.
  The harder scenario C uses 40 samples, scale 3 and 150 iterations.
- **Random restarts:** if the final path still collides, STOMP is run again from the same
  colliding initial path with new random samples (at most 5 attempts).
- **Joint wrap-around:** if a joint's start and goal angles differ by more than π, the goal angle
  is shifted by 2π (same pose) when the joint limits allow it, so the initial path turns the short way.
