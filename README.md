# Two-Link Arm — Forward & Inverse Kinematics

Forward kinematics model of a 2-link planar arm, plus a Jacobian transpose
inverse kinematics solver that converges the end effector onto an arbitrary
target position.

## Files

- `two_link_arm.py` — `TwoLinkArm` class: given joint angles, computes elbow
  and wrist (end effector) position and plots the arm
- `jacobian_ik.py` — solves the inverse problem: given a target (x, y),
  iteratively updates joint angles using the Jacobian transpose method until
  the end effector reaches the target. Includes an interactive mode — click
  anywhere on the plot and the arm animates to reach that point.

## Method

Forward kinematics is a direct trig evaluation from joint angles to end
effector position. The inverse problem — going from a desired end-effector
position back to joint angles — doesn't have as clean a closed form once you
want a general, extensible solution, so this uses an iterative approach
instead:

```
dq = alpha * J^T * (target - current_position)
```

where `J` is the 2x2 Jacobian of end-effector position with respect to joint
angles. This nudges the joint angles in the direction that reduces
end-effector error, repeated until convergence. It's the same family of idea
as Newton-type iterative solvers used in nonlinear FEA — take a local
linearization (the Jacobian), step toward the solution, repeat.

## Running it

```bash
python two_link_arm.py      # forward kinematics demo
python jacobian_ik.py        # inverse kinematics demo (single target)
```

For the interactive click-to-target version, open `jacobian_ik.py`, uncomment
the last two lines (`InteractiveIK()` / `plt.show()`), and run it — click
anywhere in the plot window and watch the arm converge.

## Example result

Target `(1.2, 0.8)` converged in 115 iterations to `(1.2007, 0.8007)`.

## Limitations

- Jacobian transpose method is simple and robust but converges more slowly
  than a full inverse-Jacobian (pseudoinverse) approach
- No joint limits — the arm can reach unphysical configurations
- 2-link planar only; doesn't extend to redundant manipulators without
  changes to the Jacobian handling

## Origin

Ported from an earlier MATLAB implementation (mouse-click target selection,
iterative Jacobian update loop) into a cleaner, class-based Python version.
