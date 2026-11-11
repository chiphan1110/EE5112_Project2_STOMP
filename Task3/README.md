# Task 3: Product-of-Exponentials Forward Kinematics for the UR5

Replaces MATLAB's `getTransform` in the planner with forward kinematics based on twists, the
**Product of Exponentials (PoE)** formula

```
T_k(theta) = e^[S_1]theta_1 · e^[S_2]theta_2 · ... · e^[S_k]theta_k · M_k
```

The screw axes `S_k` are derived from the robot model, and the implementation is verified
against `getTransform` and the textbook library. The planning scenario is the same as in Task 2.

## How to run

From this folder in MATLAB:

```matlab
verify_task3                     % screw axes + correctness and speed of the PoE code (under a minute)
plot_screw_axes                  % figure of the six screw axes on the UR5
compare_task2_task3              % full planner: Task 2 (getTransform) vs Task 3 (PoE), needs ../Task2
rng(0); UR5_STOMP_Path_Planning  % the planner itself, now with PoE forward kinematics
```

**Expected results:**

- screw axes from three independent methods agree to about 1e-11;
- PoE matches `getTransform` to about 1e-15 for all frames;
- PoE is roughly 20x faster per call, and the STOMP loop roughly 10x faster;
- with `rng(0)` the PoE planner gives exactly the same path as Task 2 (11 iterations, Q = 10.84).

### Options

The planner records no videos by default, because the path is identical to Task 2. Set
`enableVideoTraining = 1` and/or `enableVideo = 1` before `UR5_STOMP_Path_Planning` to record them.

## Files

### Forward kinematics

| File | Purpose |
|---|---|
| `computeScrewAxes.m` | Derives `S_k = [w_k; -w_k x q_k]`, the zero-configuration frames `M_k` and the end-effector pose `M` from the robot model at the zero configuration; also precomputes the matrices used by the exponential |
| `updateJointsWorldPosition.m` | PoE forward kinematics used by the planner. Same inputs and outputs as the `getTransform` version (6 joint frames and `tool0`) |
| `updateJointsWorldPosition_getTransform.m` | The `getTransform` version, kept as a reference for the verification |

### Verification and figures

| File | Purpose |
|---|---|
| `verify_task3.m` | Checks the screw axes three ways (lecture table, `computeScrewAxes`, matrix logarithm), compares PoE with `getTransform` on 1000 random configurations and with `FKinSpace`, tests a prismatic joint, and times both methods |
| `compare_task2_task3.m` | Runs the Task 2 and Task 3 planners with the same seed and compares path, cost and planning time |
| `plot_screw_axes.m` | UR5 at the zero configuration with the screw axes as arrows, and a table of `S_k` and `M` |
| `mr_library/` | *Modern Robotics* functions (`FKinSpace`, `MatrixLog6`, ...), MIT License. Used only by `verify_task3.m` |

### Planner

`UR5_STOMP_Path_Planning.m`, `helperSTOMP.m`, the STOMP functions (`stomp*.m`) and the helpers
(`helperCreateObstaclesUR5.m`, `helperInitialVisualizerUR5.m`, `sEDT_3d.m`) are as in Task 2.

## Outputs

| File | Content |
|---|---|
| `Task3_verification.txt` | Output of `verify_task3.m` |
| `Task3_compare_STOMP.txt` | Output of `compare_task2_task3.m` |
| `Task3_screw_axes.png` | Screw-axis figure |
| `Theta_UR5_nDisc20_nPaths_20.mat` | Final path `theta` of the PoE planner |

## Implementation notes

- **Screw axes:** for each moving joint, the pose `T_k` of the body it drives is read once at
  the zero configuration. The joint axis from the robot model, rotated by `T_k`, gives `w_k`; the
  origin of `T_k` is a point `q_k` on the axis; `v_k = -w_k x q_k`. Prismatic joints use `w = 0`
  and `v` = the slide direction.
- **Efficiency:**
  - the screw axes are computed once per robot and cached;
  - the matrix exponential uses the closed-form Rodrigues formula with precomputed `[w]`, `[w]^2`, `[w]v` and `[w]^2 v`;
  - one running product `P_k = P_(k-1) e^[S_k]theta_k` gives every joint frame (`P_k M_k`) in a single pass.
- `getTransform` is used only inside `computeScrewAxes.m`, to read the zero-configuration geometry;
  it is never called during planning.
- Timings depend on the computer and vary from run to run.
