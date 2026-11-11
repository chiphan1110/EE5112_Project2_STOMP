# Task 2: STOMP Path Planning for the UR5

Runs the STOMP planner on a different manipulator, the 6-joint **Universal Robots UR5**, and
checks how well the collision-sphere model covers the real robot.

## Scenario

| | Value |
|---|---|
| Robot | Universal Robots UR5 (`loadrobot('universalUR5')`), end-effector `tool0` (flange) |
| Start pose | position (0.45, −0.35, 0.25) m, tool pointing down |
| Goal pose | position (0.45, 0.35, 0.25) m, tool pointing down |
| Obstacle | 16 cm cube centred at (0.57, 0, 0.25) m |
| Initial path | straight line in joint space (10 of 20 waypoints collide with the box) |

## How to run

From this folder in MATLAB:

```matlab
rng(0); UR5_STOMP_Path_Planning   % plan, check, animate, record videos (about 1 min)
plot_task2_results                % report figures (uses the workspace of the run above)
sphere_coverage_test              % optional: sphere model vs real meshes (about 1 min)
```

**Expected result (`rng(0)`):** converges in 11 iterations, final cost Q = 10.84, `isTrajectoryInCollision = 0`.

### Options

Set these variables **before** running `UR5_STOMP_Path_Planning`:

| Variable | Default | Effect |
|---|---|---|
| `enableVideoTraining` | `1` | Record the training video (`0` to skip) |
| `enableVideo` | `1` | Record the final-path video (`0` to skip) |

## Files

### Main scripts

| File | Purpose |
|---|---|
| `UR5_STOMP_Path_Planning.m` | Entry point: loads the UR5, solves IK for start and goal, builds the obstacle, calls `helperSTOMP` |
| `helperSTOMP.m` | STOMP set-up and main loop, final collision check, videos, animation, saving the path |
| `plot_task2_results.m` | Report figures: path overlay, cost per iteration, sphere model |
| `sphere_coverage_test.m` | Compares the sphere model with `checkCollision` on 1500 random configurations |

### STOMP functions

Same algorithm as Task 1: `stompSamples.m`, `stompTrajCost.m`, `stompObstacleCost.m`,
`stompUpdateProb.m`, `stompDTheta.m`, `stompUpdateTheta.m`, `stompRobotSphere.m`.

`updateJointsWorldPosition.m` (forward kinematics with `getTransform`) is written for any serial
robot from `loadrobot`:

- it finds the body driven by each moving joint, so robots whose first bodies are fixed (the UR5
  starts with `base_link` and `base`) are handled correctly;
- it returns the end-effector frame as an extra row, so the spheres also cover the last link.

### Helpers

| File | Purpose |
|---|---|
| `helperCreateObstaclesUR5.m` | Creates the box as a collision object and as a voxel grid with a signed distance field |
| `helperInitialVisualizerUR5.m` | Scene figure with the UR5 at the start configuration |
| `sEDT_3d.m` | Signed Euclidean distance transform of the voxel grid (`bwdist`) |

## Outputs

| File | Content |
|---|---|
| `Task2_path_overlay.png` | Robot at 7 waypoints of the initial guess (left) and the STOMP path (right) |
| `Task2_cost.png` | Total cost per iteration (iteration 0 = initial guess) |
| `Task2_spheres.png` | The 35 collision spheres on the UR5: start pose and the waypoint with the smallest sphere clearance |
| `Task2_sphere_coverage.txt` | Result of the sphere-coverage test |
| `UR5_Training.mp4` | Path after every iteration (1 to 11) |
| `UR5_planned_path.mp4` | Final path, waypoint by waypoint |
| `Theta_UR5_nDisc20_nPaths_20.mat` | Final path `theta` (6 joints x 20 waypoints) |

## Implementation notes

- **Inverse kinematics seed:** IK starts from a "bent forward, tool down" configuration so that
  the start and goal solutions lie in the same arm branch, which keeps the straight-line initial
  path in front of the robot.
- **Sphere coverage test:** random configurations near the initial path
  (`rng(1)`, Gaussian noise of 0.35 rad per joint). For each one, the sphere clearance
  (signed distance at each sphere centre minus the radius) is compared with `checkCollision`.
- STOMP parameters, joint-limit clipping and the final mesh-based collision check are as in Task 1.
