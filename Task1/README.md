# Task 1: STOMP Path Planning for the Kinova Gen3

Plans a collision-free path for the 7-joint **Kinova Gen3** from a start pose to a goal pose with a
box obstacle in between, using STOMP.

## Scenario

| | Value |
|---|---|
| Robot | Kinova Gen3 (`loadrobot('kinovaGen3')`), end-effector `EndEffector_Link` |
| Start pose | position (0.40, 0, 0.20) m, gripper pointing down |
| Goal pose | position (0.35, 0.55, 0.35) m, gripper pointing down |
| Obstacle | 16 cm cube centred at (0.40, 0.46, 0.26) m |
| Initial path | straight line in joint space between the IK solutions (collides with the box) |

## How to run

From this folder in MATLAB:

```matlab
rng(0); KINOVA_STOMP_Path_Planning   % plan, check, animate, record videos (about 1 min)
plot_task1_results                   % report figures (uses the workspace of the run above)
```

**Expected result (`rng(0)`):** converges in 10 iterations, `isTrajectoryInCollision = 0`.

### Options

Set these variables **before** running `KINOVA_STOMP_Path_Planning`:

| Variable | Default | Effect |
|---|---|---|
| `enableVideoTraining` | `1` | Record the training video (`0` to skip) |
| `enableVideo` | `1` | Record the final-path video (`0` to skip) |

## Files

### Main scripts

| File | Purpose |
|---|---|
| `KINOVA_STOMP_Path_Planning.m` | Entry point: loads the robot, solves IK for start and goal, builds the obstacle, calls `helperSTOMP` |
| `helperSTOMP.m` | STOMP set-up and main loop, final collision check, videos, animation, saving the path |
| `plot_task1_results.m` | Report figures: initial vs optimised path, and cost per iteration |

### STOMP functions

| File | Purpose |
|---|---|
| `stompSamples.m` | Draws noisy trajectories `theta + eps`, `eps ~ N(0, R^-1)`, start and goal fixed |
| `stompTrajCost.m` | Local cost of every waypoint and the total cost `Q = sum(S) + 1/2 theta' R theta` |
| `stompObstacleCost.m` | Obstacle cost of the robot spheres (STOMP paper, Eq. 13): margin 5 cm, weighted by sphere speed |
| `stompUpdateProb.m` | Turns the sample costs into probabilities (exponential weighting) |
| `stompDTheta.m` | Probability-weighted sum of the sample noise (the update direction) |
| `stompUpdateTheta.m` | Smooths the update with the matrix `M` and applies it |
| `stompRobotSphere.m` | Places collision spheres (r = 5 cm) along the links |
| `updateJointsWorldPosition.m` | Forward kinematics (`getTransform`): joint positions used by the sphere model |

### Helpers

| File | Purpose |
|---|---|
| `helperCreateObstaclesKINOVA.m` | Creates the box as a collision object and as a voxel grid with a signed distance field |
| `helperInitialVisualizerKINOVA.m` | Scene figure with the robot at the start configuration |
| `sEDT_3d.m` | Signed Euclidean distance transform of the voxel grid (`bwdist`) |

## Outputs

| File | Content |
|---|---|
| `Task1_path_overlay.png` | Robot at 7 waypoints of the initial guess (left) and the STOMP path (right); poses in collision drawn in red |
| `Task1_cost.png` | Total cost per iteration, from the initial guess (iteration 0) to convergence |
| `KinvaGen3_Training.mp4` | Path after every iteration (1 to 10) |
| `KinvaGen3_planned_path.mp4` | Final path, waypoint by waypoint |
| `Theta_nDisc20_nPaths_20.mat` | Final path `theta` (7 joints x 20 waypoints) |

## Implementation notes

- **Parameters:** 20 waypoints, 20 sample paths, stop when the cost changes by less than 0.1 or after 50 iterations.
- **Joint limits:** after each update the path is clipped to the joint position limits of the robot model.
- **Final check:** `checkCollision` on the real robot and box meshes (self-collision ignored), independent of the sphere model used in the cost.
