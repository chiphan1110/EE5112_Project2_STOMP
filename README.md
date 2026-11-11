# EE5112 Project 2: STOMP Path Planning for Robot Manipulators (Group 12)

MATLAB code for collision-free path planning of robot manipulators with **STOMP**
(Stochastic Trajectory Optimization for Motion Planning, Kalakrishnan et al., ICRA 2011).
The planner is completed and tested on a Kinova Gen3, moved to a Universal Robots UR5, given
Product-of-Exponentials forward kinematics, tested in new multi-object scenarios, and finally
extended with an end-effector orientation constraint.

## Contents

| Folder | Task | What it shows |
|---|---|---|
| [`Task1/`](Task1/README.md) | Complete the STOMP sample code | Kinova Gen3 avoiding a box |
| [`Task2/`](Task2/README.md) | A different manipulator | Same planner on the UR5, plus a check of the collision-sphere model |
| [`Task3/`](Task3/README.md) | Twists and PoE forward kinematics | Screw axes of the UR5 and a fast PoE implementation, verified against `getTransform` |
| [`Task4/`](Task4/README.md) | Own collision scenarios | Three multi-object scenarios (cluttered table, wall and ceiling, kitchen shelf) |
| [`Task5/`](Task5/README.md) | End-effector orientation constraint | Keeping a tool axis aligned (e.g. a cup upright) while avoiding obstacles |

Each folder is **self-contained**: it holds every file it needs, so a task can be run on its own
without changing the MATLAB path. The only exception is `Task3/compare_task2_task3.m`, which
also runs the planner in `../Task2`.

## Requirements

- **MATLAB R2025a or newer** (developed and tested with R2026a; figure themes need R2025a+)
- **Robotics System Toolbox**: robot models (`loadrobot`), inverse kinematics, collision checking
- **Statistics and Machine Learning Toolbox**: `mvnrnd` for sampling noisy trajectories
- **Image Processing Toolbox**: `bwdist` for the signed distance field (`sEDT_3d.m`)

## Quick start

1. Open MATLAB and `cd` into one task folder, e.g. `Task2`.
2. Run the main planning script with a fixed random seed:
   ```matlab
   rng(0); UR5_STOMP_Path_Planning
   ```
3. Run the figure script of that task, e.g. `plot_task2_results`.

Each task README lists the exact commands, the outputs and the expected results.

## The algorithm in one paragraph

The path is a sequence of 20 joint-space waypoints, initialised as a straight line between the
start and goal configurations (which always collides with an obstacle). In each iteration STOMP
draws 20 noisy copies of the path, scores every waypoint of every copy with a cost
(an obstacle cost from a sphere model of the robot and a signed distance field of the
obstacles, plus an orientation cost in Task 5), turns the costs into probabilities and moves the
path towards the low-cost samples with a smoothed update. It stops when the total cost no longer
changes (and, from Task 4, when no sphere is inside the safety margin). The final path is checked
with MATLAB's `checkCollision` on the real robot and obstacle meshes.

## Reproducibility

- All results in the report use `rng(0)` before the planning script. STOMP's only randomness is
  the trajectory sampling, so the same seed gives exactly the same path.
- Timings depend on the computer and vary from run to run.

## Common outputs

| File | Content |
|---|---|
| `*.png` | Report figures (made by the `plot_*` scripts) |
| `*_training.mp4` / `*Training.mp4` | Path after selected STOMP iterations |
| `*_planned_path.mp4` | Final planned path, waypoint by waypoint |
| `Theta_*.mat` | Final planned path `theta` (joints x waypoints) |
| `*.txt` | Numerical results of the analysis scripts |

## Third-party code

- The STOMP sample code and helper functions were provided with the EE5112 project.
- `Task3/mr_library/` contains functions from the *Modern Robotics* library
  (Lynch and Park, NxRLab, MIT License, see `LICENSE_ModernRobotics.txt`). They are used only as an
  independent reference in the verification script, not by the planner.
