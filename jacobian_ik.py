"""
Jacobian-based inverse kinematics for a 2-link planar arm.

Given a target (x, y), iteratively updates joint angles using the
transpose-Jacobian method:

    dq = alpha * J^T * (target - end_effector)

until the end effector converges to the target. Ported from an original
MATLAB implementation (mouse-click target selection with a while loop)
into Python, using TwoLinkArm for forward kinematics.
"""
import numpy as np
import matplotlib.pyplot as plt
from two_link_arm import TwoLinkArm


def jacobian(theta, link_lengths):
    l0, l1 = link_lengths
    t0, t1 = theta
    dx_dt0 = -l0 * np.sin(t0) - l1 * np.sin(t0 + t1)
    dx_dt1 = -l1 * np.sin(t0 + t1)
    dy_dt0 = l0 * np.cos(t0) + l1 * np.cos(t0 + t1)
    dy_dt1 = l1 * np.cos(t0 + t1)
    return np.array([[dx_dt0, dx_dt1],
                      [dy_dt0, dy_dt1]])


def solve_ik(arm, target, alpha=0.2, tol=1e-3, max_iters=500):
    """Iteratively solve for joint angles that place the end effector at target."""
    theta = np.array(arm.joint_angles, dtype=float)
    history = [theta.copy()]

    for _ in range(max_iters):
        arm.update_joints(theta)
        error = np.array(target) - arm.wrist
        if np.linalg.norm(error) < tol:
            break
        J = jacobian(theta, arm.link_lengths)
        dtheta = alpha * (J.T @ error)
        theta = theta + dtheta
        history.append(theta.copy())

    arm.update_joints(theta)
    return theta, history


class InteractiveIK:
    """Click anywhere on the plot to set a target; the arm animates to reach it."""

    def __init__(self, link_lengths=(1.0, 1.0)):
        self.arm = TwoLinkArm(joint_angles=(0.3, 0.3), link_lengths=link_lengths)
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self._draw()

    def _draw(self, target=None):
        self.ax.clear()
        self.arm.plot(self.ax)
        if target is not None:
            self.ax.plot(*target, 'g*', markersize=15)
        limit = sum(self.arm.link_lengths) * 1.2
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title("Click to set target — arm converges via Jacobian transpose IK")
        self.fig.canvas.draw()

    def on_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        target = (event.xdata, event.ydata)
        theta, history = solve_ik(self.arm, target)
        for q in history:
            self.arm.update_joints(q)
            self._draw(target=target)
            plt.pause(0.02)


if __name__ == "__main__":
    # Non-interactive demo: solve for one target and plot the result
    arm = TwoLinkArm(joint_angles=(0.3, 0.3))
    target = (1.2, 0.8)
    theta, history = solve_ik(arm, target)

    fig, ax = plt.subplots()
    arm.plot(ax)
    ax.plot(*target, 'g*', markersize=15, label='target')
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-2.2, 2.2)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_title(f"Jacobian IK converged in {len(history)} iterations")
    plt.savefig("jacobian_ik_demo.png", dpi=120)
    print(f"Converged in {len(history)} iterations")
    print(f"Final joint angles (rad): {theta}")
    print(f"Final end-effector position: {arm.wrist}, target was {target}")

    # To run the interactive click-to-target version instead, use:
    # InteractiveIK()
    # plt.show()
