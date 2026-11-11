# Task 5: End-Effector Orientation Constraint

Adds a constraint that keeps one axis of the end-effector aligned with a fixed world direction
during the whole motion (think of carrying a cup of coffee), on top of the obstacle avoidance of
Task 4. Each scenario is planned **without** and **with** the constraint to show the difference.

## The constraint

At every waypoint the forward kinematics gives the end-effector rotation `R`. Column `a` of `R`
is the chosen tool axis in world coordinates, and `d` is the direction it should keep. The
constraint cost is the l1 norm of the difference:

```
q_c = w * || R(:,a) - d ||_1        (w = 100)
S   = 1000 * q_o + q_c              (local cost per waypoint; q_o = obstacle cost)
```

| Scenario | Constrained axis | Target direction `d` | Meaning |
|---|---|---|---|
| A: cluttered table | tool z-axis | (0, 0, −1) | gripper keeps pointing down |
| B: wall and ceiling | tool z-axis | (0, 0, −1) | gripper keeps pointing down |
| C: kitchen shelf | tool y-axis | (0, 0, 1) | cup stays upright |

The start and goal poses of every scenario already satisfy their constraint.

**Alignment error.** Results are reported as the angle between the constrained axis and its
target, `arccos(R(:,a) · d)`, in degrees.

## How to run

From this folder in MATLAB:

```matlab
scenarioID = 3; rng(0); UR5_STOMP_Path_Planning   % plan scenario C with the constraint
plot_task5_results                                % all scenarios without/with the constraint (about 2 min, no videos)
task5_extra_study                                 % optional: seeds, weight sweep, iteration limit (about 5 min)
```

To plan **without** the constraint, set `useOrientationConstraint = false;` before
`UR5_STOMP_Path_Planning`.

**Expected results (`rng(0)`):** all paths are collision-free, and the maximum alignment error drops
from 47.5° to 1.5° (A), from 16.8° to 1.7° (B) and from 24.7° to 0.4° (C).

### Options

| Variable | Default | Effect |
|---|---|---|
| `scenarioID` | `1` | Scenario: 1 = A, 2 = B, 3 = C |
| `useOrientationConstraint` | `true` | `false` plans without the constraint |
| `enableVideoTraining`, `enableVideo` | `1` | Record the videos (`0` to skip) |
| `displayAnimation` | `1` | Animate the final path (`0` to skip) |
| `p5scenarios`, `p5seed` | `[1 2 3]`, `0` | Scenarios and seed used by `plot_task5_results` |

## Files

### Main scripts

| File | Purpose |
|---|---|
| `UR5_STOMP_Path_Planning.m` | Entry point: scenario, constraint settings, IK, initial path, STOMP with restarts |
| `stompTrajCost.m` | Cost function with the orientation constraint; also returns the obstacle part of the cost separately |
| `scenarioUR5.m` | Scenarios A, B and C, each with its constrained axis and target direction |
| `helperSTOMP.m` | STOMP set-up and main loop, final collision check, videos, saving the path |
| `plot_task5_results.m` | Plans every scenario without and with the constraint (same seed) and makes the figures |
| `task5_extra_study.m` | Scenario C with seeds 0–5, weights w = 1 to 300, and why some runs reach the iteration limit |

### Helpers and STOMP functions

`check_scenario.m`, `helperCreateObstaclesUR5.m`, `helperInitialVisualizerUR5.m`, the PoE
forward kinematics (`computeScrewAxes.m`, `updateJointsWorldPosition.m`), the remaining STOMP
functions (`stomp*.m`) and `sEDT_3d.m` are as in Task 4.

## Outputs

| File | Content |
|---|---|
| `Task5_<scenario>_before_after.png` | Path without (left) and with (right) the constraint; orange arrows show the constrained axis at every waypoint |
| `Task5_<scenario>_alignment_error.png` | Alignment error at each waypoint, without and with the constraint |
| `Task5_results.txt` | Iterations, STOMP time, collision status, max/mean alignment error and gripper path length for every case |
| `Task5_extra_study.txt` | Output of `task5_extra_study.m` |
| `Task5_<scenario>_with_constraint_training.mp4` | Constrained path after selected iterations |
| `Task5_<scenario>_with_constraint_planned_path.mp4` | Final constrained path, waypoint by waypoint |
| `Theta_UR5_<scenario>_<with/without>_constraint_*.mat` | Final paths |

## Implementation notes

- **Stopping rule:** the obstacle part of the stopping rule (Task 4) ignores the constraint cost,
  which is small but almost never exactly zero.
- **Soft constraint:** tilting is strongly penalised but not forbidden, so a small alignment error
  remains. Only one axis is fixed; the gripper may still rotate about it.
- **`stompOverrides`:** `task5_extra_study.m` changes the constraint weight through a struct
  `stompOverrides` that `UR5_STOMP_Path_Planning.m` reads if it exists. Normal runs do not define
  it.
