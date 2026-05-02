# 4-DOF Robot Arm — Kinematics, Dynamics & Trajectory Planning

**Course project — Applied Robotics Technologies, MSc Industrial Automation Engineering**  
**Università degli Studi di Pavia, A.Y. 2025/2026**  
**Group D — A.R.T. (Applied Robotics Technologies)**

---

## Overview

This project implements a complete kinematic and dynamic analysis of a 4-DOF anthropomorphic robot arm, including trajectory planning and Simscape-based validation. The robot is a cylindrical-coordinate serial manipulator with one rotational base joint and three elbow joints, carrying a 0.5 kg payload.

The work is split into two main tasks:
- **Task 1:** Point-to-point trajectory planning using lines and parabolic blends in joint space
- **Task 2:** Cartesian S-curve trajectory planning with full dynamic analysis

---

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| DOF | 4 (1R + 3R serial chain) |
| Link lengths | l0=200mm, l1=329mm, l2=311.5mm, l3=106mm |
| Total mass | ~39 kg (base + links + payload) |
| Payload | 0.5 kg |
| Joint limits | ±180° (q1, q3, q4), ±120° (q2) |

---

## What's Implemented

### Kinematics
- **Forward kinematics** (`ROBOTdir_4R.m`) — computes end-effector pose (x, y, z, θ) from joint angles using geometric decomposition
- **Inverse kinematics** (`ROBOTinv_4R.m`) — closed-form solution supporting elbow-up and elbow-down configurations via law of cosines
- **Geometric Jacobian** (`ROBOTjac_4R.m`) — 4×4 Jacobian mapping joint velocities to end-effector velocities
- **Jacobian-based dynamics** (`ROBOTjacdin_4R.m`, `ROBOTjacPdin_4R.m`) — dynamic model using the Jacobian formulation for joint torque computation
- **Direct dynamics** (`ROBOTdirdin_4R.m`) — forward dynamics for simulation

### Trajectory Planning
- **Lines and parabolic blends** (`LinesParabolas_task1.m`) — joint-space point-to-point trajectories with smooth parabolic blending at via-points, minimum rise time optimisation
- **S-curve Cartesian trajectory** (`Sshape_task2.m`) — smooth Cartesian path following with velocity and acceleration profiles
- **Workspace analysis** (`PlotArea4R_Catalog.m`) — reachable workspace computation and visualisation

### Simscape Validation
- `Simscape_task1_motors.slx` — Simscape Multibody model for Task 1 trajectory validation
- `Simscape_task2_motors.slx` — Simscape Multibody model for Task 2 S-curve trajectory validation

Results from analytical models are compared directly against Simscape simulation to validate correctness of the derived equations.

---

## Key Results

- Closed-form inverse kinematics validated against forward kinematics with sub-millimetre accuracy
- Parabolic blend trajectories satisfy joint limits and continuity constraints at all via-points
- Dynamic torque profiles computed for both tasks, showing peak torques at path initiation
- Simscape models confirm analytical joint trajectories within numerical tolerance

---

## Repository Structure

```
├── ROBOTdir_4R.m              # Forward kinematics
├── ROBOTinv_4R.m              # Inverse kinematics (elbow-up/down)
├── ROBOTjac_4R.m              # Geometric Jacobian
├── ROBOTjacdin_4R.m           # Jacobian-based dynamics
├── ROBOTjacPdin_4R.m          # Jacobian dynamics (alternative formulation)
├── ROBOTdirdin_4R.m           # Direct dynamics
├── LinesParabolas_task1.m     # Task 1: joint-space trajectory planning
├── Sshape_task2.m             # Task 2: Cartesian S-curve trajectory
├── PlotArea4R_Catalog.m       # Workspace visualisation
├── test_robot_4R_Conditioning.m # Jacobian conditioning analysis
├── min_rise_time.m            # Minimum rise time optimisation
├── Simscape_task1_motors.slx  # Simscape model — Task 1
├── Simscape_task2_motors.slx  # Simscape model — Task 2
├── PlotRobot2.m / PlotRobot3.m # Robot visualisation utilities
└── docs/
    └── GroupD_ART_Report.pdf  # Full technical report
    └── GroupD_ART_Presentation.pdf  # Project presentation
```

---

## How to Run

**Requirements:** MATLAB R2023a or later, Simulink, Simscape Multibody toolbox

1. Clone the repository and open MATLAB
2. Add the project folder to your MATLAB path
3. Run `LinesParabolas_task1.m` for Task 1 trajectory analysis
4. Run `Sshape_task2.m` for Task 2 Cartesian trajectory analysis
5. Open `Simscape_task1_motors.slx` or `Simscape_task2_motors.slx` in Simulink for validation

---

## Context

This project was completed as part of the **Applied Robotics Technologies** course at the University of Pavia (MSc Industrial Automation Engineering). It demonstrates analytical derivation and implementation of robot kinematics and dynamics from first principles, validated through physical simulation.

**Author:** Sergi Goldsmits Ybarra  
**Contact:** sergigoldsmits2000@gmail.com  
**LinkedIn:** [linkedin.com/in/sergigoldsmits00](https://linkedin.com/in/sergigoldsmits00)
