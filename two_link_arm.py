"""
Forward kinematics model for a 2-link planar arm.
"""
from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt


class TwoLinkArm:
    def __init__(self, joint_angles=(0.0, 0.0), link_lengths=(1.0, 1.0)):
        self.shoulder = np.array([0, 0])
        self.link_lengths = link_lengths
        self.update_joints(joint_angles)

    def update_joints(self, joint_angles):
        self.joint_angles = joint_angles
        self.forward_kinematics()

    def forward_kinematics(self):
        theta0, theta1 = self.joint_angles
        l0, l1 = self.link_lengths

        self.elbow = self.shoulder + np.array([l0 * cos(theta0), l0 * sin(theta0)])
        self.wrist = self.elbow + np.array(
            [l1 * cos(theta0 + theta1), l1 * sin(theta0 + theta1)]
        )

    def plot(self, ax=None):
        ax = ax or plt.gca()
        ax.plot([self.shoulder[0], self.elbow[0]], [self.shoulder[1], self.elbow[1]], 'r-', linewidth=3)
        ax.plot([self.elbow[0], self.wrist[0]], [self.elbow[1], self.wrist[1]], 'b-', linewidth=3)
        ax.plot(self.shoulder[0], self.shoulder[1], 'ko')
        ax.plot(self.elbow[0], self.elbow[1], 'ko')
        ax.plot(self.wrist[0], self.wrist[1], 'ko')


if __name__ == "__main__":
    arm = TwoLinkArm(joint_angles=(0.5, 0.8))
    fig, ax = plt.subplots()
    arm.plot(ax)
    ax.set_xlim(-2.2, 2.2)
    ax.set_ylim(-2.2, 2.2)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title("Two-Link Arm — Forward Kinematics")
    plt.savefig("forward_kinematics_demo.png", dpi=120)
    print("Saved forward_kinematics_demo.png")
