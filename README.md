# Two-Link Arm Kinematics

<p align="center">
  <img src="images/jacobian_ik_demo.png" width="650">
</p>

<p align="center">
<em>Two-link planar robotic manipulator solving forward and inverse kinematics using the Jacobian Transpose Method.</em>
</p>

## Overview

This project implements both forward and inverse kinematics for a two-link planar robotic manipulator in Python.

Forward kinematics computes the position of the end effector from a given set of joint angles, while inverse kinematics determines the joint angles required to reach a desired target position. The inverse problem is solved iteratively using the Jacobian Transpose Method, which updates the joint angles until the end effector converges to the specified target.

The project demonstrates the mathematical principles behind robotic manipulator kinematics and iterative numerical methods that are widely used in robotics, optimization, and computational engineering.

## Repository Structure

```text
Two-Link-Robot-Kinematics/
│
├── two_link_arm.py
├── jacobian_ik.py
├── images/
│   └── ik_demo.png
├── README.md
├── requirements.txt
└── LICENSE
```

## Methodology

### Forward Kinematics

The forward kinematics model computes the elbow and end-effector positions directly from the joint angles using trigonometric relationships.

### Inverse Kinematics

The inverse problem is solved iteratively using the Jacobian Transpose Method:

```text
Δq = αJᵀ(xtarget − xcurrent)
```

where:

- **J** is the Jacobian matrix relating joint motion to end-effector motion.
- **α** is the step size controlling the update magnitude.
- **(xtarget − xcurrent)** is the position error of the end effector.

The joint angles are updated repeatedly until the position error falls below the specified convergence tolerance.


## Running the Project

Run the forward kinematics demonstration:

```bash
python two_link_arm.py
```

Run the inverse kinematics solver:

```bash
python jacobian_ik.py
```

To enable the interactive click-to-target mode, uncomment the final lines in `jacobian_ik.py`:

```python
InteractiveIK()
plt.show()
```

Click anywhere inside the workspace and the robotic arm will iteratively move toward the selected target.

## Example Result

| Parameter | Value |
|-----------|------:|
| Target Position | (1.2, 0.8) |
| Final Position | (1.2007, 0.8007) |
| Iterations | 115 |

The solver converges smoothly to the desired target while maintaining stable iterative updates.

## Limitations

- Uses the Jacobian Transpose Method, which is simple and robust but generally converges more slowly than Jacobian pseudoinverse or damped least-squares approaches.
- Does not include joint limits or collision avoidance.
- Limited to a planar two-link manipulator.
- Dynamic effects such as inertia, velocity, and torque are not considered.

Future improvements could include implementing Jacobian pseudoinverse and damped least-squares inverse kinematics, adding joint limits, and extending the solver to manipulators with more degrees of freedom.

## Skills Demonstrated

- Python
- NumPy
- Matplotlib
- Robot Kinematics
- Forward Kinematics
- Inverse Kinematics
- Jacobian Methods
