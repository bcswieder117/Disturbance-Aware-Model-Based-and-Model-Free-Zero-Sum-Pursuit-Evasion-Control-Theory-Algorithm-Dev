# Model-Based and Q-Learning-Based Pursuit–Evasion Control of Autonomous Ground Vehicles Using QCar-Compatible State-Space Models

> **Project Status:** Active Master of Science thesis project at Tennessee Technological University. This repository contains the reproducible MATLAB implementation, simulation studies, literature documentation, and supporting materials developed for the thesis. The final defended thesis, archival citation, and approved publication links will be added after institutional submission and approval.

## Overview

This repository documents a research project at the intersection of optimal control, differential games, reinforcement learning, and autonomous ground vehicles.

The project develops a controlled two-vehicle pursuit–evasion benchmark for autonomous ground vehicles. A pursuer attempts to drive the relative vehicle configuration into a capture region, while an evader attempts to delay or avoid capture. The benchmark is designed to support a direct comparison between a model-based finite-horizon game-theoretic controller and a data-driven fitted-Q policy-recovery method.

The work emphasizes a shared modeling foundation: both approaches use the same vehicle state definition, capture condition, nominal engagement geometry, nonlinear rollout, actuator limits, and performance metrics.

## Research Focus

- **Ground-vehicle modeling:** QCar-compatible kinematic bicycle models with planar position, heading angle, longitudinal speed, acceleration, and steering inputs.
- **Model-based control:** Finite-horizon, two-player, zero-sum linear-quadratic pursuit–evasion control using a Riccati saddle-point recursion.
- **Data-driven recovery:** Stage-wise fitted quadratic Q-learning to recover the nominal finite-horizon saddle policy from sampled transitions.
- **Nonlinear validation:** Evaluation of the resulting policies on nonlinear, actuator-limited kinematic-bicycle dynamics.
- **Robustness studies:** Sensitivity to wheelbase mismatch, steering and longitudinal-command mismatch, command delay, actuator-limit severity, process noise, and measurement noise.
- **QCar transfer pathway:** A simulation-oriented bridge toward future QLabs and QCar experiments.

## Presentation

This work was presented at Tennessee Tech's **2026 Research and Creative Inquiry Symposium**.

[View the Tennessee Tech Research and Creative Inquiry Symposium page](https://www.tntech.edu/research/research-day.php)

## Abstract

Autonomous ground vehicles operate in environments where safe behavior depends on interaction with other moving agents. This thesis develops a controlled pursuit–evasion benchmark for comparing model-based game control with data-driven policy recovery for two car-like autonomous ground vehicles.

Each vehicle is represented by a QCar-compatible kinematic bicycle model with physically meaningful states: planar position, heading angle, and longitudinal speed. Acceleration and steering angle serve as the control inputs. This representation preserves car-like motion and steering geometry while remaining suitable for linearization, finite-horizon game design, sampled-data learning, MATLAB simulation, and future QCar-oriented implementation.

The model-based component formulates a finite-horizon, two-player, zero-sum linear-quadratic pursuit–evasion game. A local discrete-time game model is obtained by linearizing and discretizing the nonlinear kinematic bicycle dynamics. A Riccati-type saddle-point recursion produces stage-dependent feedback policies for the pursuer and evader, which are evaluated on nonlinear rollouts with acceleration, steering, and speed constraints.

The data-driven component uses stage-wise fitted quadratic Q-learning to recover the same finite-horizon saddle policy from sampled states, actions, costs, and successor states without providing the learner with the analytical transition matrices used by the Riccati recursion. This creates a controlled comparison between an analytical controller derived from known dynamics and a model-free recovery procedure operating on the same pursuit–evasion game.

Finally, the nominal model-based saddle policy is evaluated under QCar-relevant perturbations, including wheelbase mismatch, steering and longitudinal-command mismatch, delay, noise, and actuator-limit severity. These experiments provide simulation-based transfer-readiness evidence while clearly distinguishing robustness evaluation from completed physical QCar hardware validation.

## Scope

This repository presents a reproducible simulation and analysis pipeline. It does not claim completed physical QCar validation, universal reinforcement-learning performance, or a full autonomous-driving traffic simulator.

The current scope is limited to:

- One pursuer and one evader.
- Reduced-order kinematic bicycle vehicle dynamics.
- A finite-horizon local linearization for model-based control synthesis.
- Matched nominal fitted-Q policy recovery.
- Nonlinear actuator-limited rollout evaluation.
- QCar-relevant perturbation and sensitivity studies.

## Citation

Citation information for the final thesis will be added after the thesis has been submitted and archived.
