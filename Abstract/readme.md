# Model-Based and Q-Learning-Based Pursuit–Evasion Control of Autonomous Ground Vehicles Using QCar-Compatible State-Space Models

> **Project Status:** Active MSc thesis project at Tennessee Technological University. This repository contains the reproducible MATLAB simulations, code, literature documentation, and supporting materials developed for the thesis.

## Overview

This research develops a controlled pursuit–evasion benchmark for two autonomous ground vehicles. The project connects optimal control, differential games, data-driven policy recovery, and autonomous-vehicle modeling through a shared simulation environment.

The benchmark compares a finite-horizon model-based Riccati saddle policy with a stage-wise fitted-Q recovery method. Both approaches use the same vehicle-state definition, capture condition, initial engagement geometry, nonlinear rollout, actuator limits, and performance metrics.

## Abstract

Autonomous ground vehicles usually operate in environments where safety-critical behavior relies on interaction with other moving agents. This thesis aims to investigate pursuit-evasion as a controlled benchmark to compare model-based game control and data-driven policy recovery for two car-like autonomous ground vehicles. A QCar-compatible kinematic bicycle model is used such that each vehicle retains physically meaningful states, which includes planar position, heading angle, and longitudinal speed, with acceleration and steering as control inputs. 

The model-based portion formulates a finite-horizon, two-player, zero-sum linear-quadratic pursuit-evasion game. The pursuer seeks to drive the relative vehicle configuration into a prescribed capture set, while the evader seeks to delay capture. A local discrete-time game is obtained by linearizing and discretizing the nonlinear kinematic bicycle dynamics. A Riccati-type saddle-point recursion that produces stage-dependent feedback policies for both players, which are then evaluated on the nonlinear actuator-limited vehicle roll-out. 

The data-driven policy recovery uses fitted quadratic Q-learning to recover the same finite-horizon saddle policy from the sampled states, actions, costs, and successor states without providing the learner with the analytical transition matrices used by the Riccati recursion. As a result, this creates a direct comparison between an analytical controller that is derived from known dynamics and a model-free recovery procedure that is operating on the same game. 

Finally, QCar-relevant perturbations are introduced, such as wheelbase mismatch, steering, and longitudinal-command mismatch, noise, delay, and actuator-limit severity. These studies provide the transfer-readiness evidence by showing how the nominal saddle policy responds to platform relevant derivations while distinguishing simulation validation from completed hardware validation. This result is a reproducible pipeline for QCar-oriented pursuit-evasion research and hardware experiments. 

## Research Focus

- QCar-compatible kinematic bicycle modeling for autonomous ground vehicles
- Finite-horizon zero-sum linear-quadratic pursuit–evasion games
- Riccati saddle-point control
- Stage-wise fitted quadratic Q-learning
- Nonlinear actuator-limited vehicle rollouts
- Robustness to mismatch, delay, noise, and actuator limitations
- Future transfer toward QLabs and QCar implementation

## Scope

This repository presents a reproducible simulation and analysis pipeline. It does not claim completed physical QCar validation, universal reinforcement-learning performance, or a full autonomous-driving traffic simulator.

The current work is limited to:

- One pursuer and one evader
- Reduced-order kinematic bicycle dynamics
- A fixed nominal operating-point linearization
- A finite-horizon model-based game solution
- Matched nominal fitted-Q policy recovery
- Nonlinear actuator-limited rollout evaluation
- QCar-relevant perturbation and sensitivity studies

## Repository Contents

- MATLAB implementations for Chapters 4–6
- Model-based finite-horizon pursuit–evasion simulations
- Fitted-Q recovery diagnostics
- Robustness and perturbation experiments
- Literature documentation and thesis-supporting notes
- Reproducible figures, CSV summaries, and run manifests

## Citation

Citation information for the final archived thesis will be added after thesis submission and approval.
