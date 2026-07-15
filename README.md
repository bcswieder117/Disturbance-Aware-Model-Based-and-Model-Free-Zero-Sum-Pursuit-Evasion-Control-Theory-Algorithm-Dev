# Disturbance-Aware Model-Based and Model-Free Zero-Sum Pursuit-Evasion Control: Theory, Algorithm Development, and MATLAB Validation

> **Project Status:** Active Master of Science thesis project at Tennessee Technological University. This repository contains the reproducible MATLAB implementations, numerical studies, literature documentation, and supporting materials developed for the thesis. The final defended thesis, archival citation, and approved publication links will be added following institutional submission and approval.

## Overview

This repository documents a theoretical and computational research project at the intersection of optimal control, zero-sum dynamic games, model-free control, disturbance compensation, and pursuit–evasion systems.

The thesis investigates how established model-based and model-free zero-sum control algorithms can be extended for a disturbance-aware pursuit–evasion setting. The work does not introduce an entirely new family of algorithms. Instead, it begins with established policy-iteration, value-iteration, and Q-learning methods and develops the additional mathematical and computational structures needed to account for persistent command-channel offsets and their compensation.

The model-based portion extends generalized algebraic Riccati equation methods using known system dynamics. The model-free portion extends quadratic Q-learning methods that estimate the game Q-function and recover the corresponding feedback policies from sampled state, input, cost, and successor-state data without requiring direct access to the analytical state-transition matrices.

Both approaches are developed under a common zero-sum game formulation, disturbance model, compensation structure, and numerical evaluation framework. MATLAB is used to examine convergence, closed-loop stability, saddle-point validity, command-bias estimation, finite-window game cost, and deviation from nominal behavior.

The completed thesis is intended to provide a theoretical and computational foundation for future implementation on autonomous ground-vehicle platforms. Physical QCar or QLabs deployment is treated as future work and is not claimed as part of the present thesis.

## Research Direction

The thesis is organized around four established zero-sum control methods:

1. Model-based policy iteration.
2. Model-based value iteration.
3. Model-free Q-learning policy iteration.
4. Model-free Q-learning value iteration.

These methods are not replaced by newly invented algorithms. Their original mathematical structures are retained and extended to fit the disturbance-aware pursuit–evasion problem considered in this thesis.

The main extension introduces persistent command-channel offsets of the form

\[
u_{P,k}^{\mathrm{actual}}
=
u_{P,k}^{\mathrm{command}}
+
b_P,
\]

\[
u_{E,k}^{\mathrm{actual}}
=
u_{E,k}^{\mathrm{command}}
+
b_E,
\]

where \(b_P\) and \(b_E\) represent constant or slowly varying command offsets affecting the pursuer and evader channels.

The corresponding compensated commands are

\[
u_{P,k}^{\mathrm{command}}
=
Lx_k-\hat b_P,
\]

\[
u_{E,k}^{\mathrm{command}}
=
Kx_k-\hat b_E,
\]

where \(L\) and \(K\) are the minimizing and maximizing feedback gains and \(\hat b_P\) and \(\hat b_E\) are estimated command-channel offsets.

Under this compensation structure, the closed-loop system becomes

\[
x_{k+1}
=
(A+BL+EK)x_k
+
B(b_P-\hat b_P)
+
E(b_E-\hat b_E).
\]

Exact bias estimates recover the nominal closed-loop dynamics, while imperfect estimates leave only the residual estimation error.

## Research Focus

- **Zero-sum pursuit–evasion control:** Formulation of a discrete-time, two-player dynamic game involving a minimizing pursuer and a maximizing evader.

- **Model-based policy iteration:** Extension of the established generalized algebraic Riccati policy-iteration method with command-bias estimation and affine compensation.

- **Model-based value iteration:** Extension of the established generalized algebraic Riccati value-iteration method using the same disturbance and compensation formulation.

- **Model-free Q-learning policy iteration:** Extension of an off-policy quadratic Q-learning method that estimates the game Q-function from measured transition data.

- **Model-free Q-learning value iteration:** Extension of the corresponding Q-learning value-iteration method under the same zero-sum cost and disturbance structure.

- **Persistent command disturbances:** Representation of calibration-like offsets between commanded and realized pursuer and evader inputs.

- **Disturbance estimation:** Model-based estimation from state-transition residuals and model-free estimation from measured command-channel data.

- **Affine compensation:** Modification of the implemented control commands to cancel estimated persistent input offsets.

- **MATLAB verification:** Numerical analysis of convergence, Riccati residuals, saddle-point curvature, closed-loop stability, bias estimation, and compensation performance.

- **Foundational vehicle application:** Development of a framework that can later be transferred to autonomous ground-vehicle models and physical platforms.

## Model-Based Methods

The model-based portion uses known system matrices in the discrete-time zero-sum game

\[
x_{k+1}
=
Ax_k
+
Bu_{P,k}
+
Eu_{E,k}.
\]

The quadratic game cost is written as

\[
J
=
\sum_{k=0}^{\infty}
\left(
x_k^{\top}Qx_k
+
u_{P,k}^{\top}R_Pu_{P,k}
-
u_{E,k}^{\top}R_Eu_{E,k}
\right).
\]

The pursuer minimizes the cost, while the evader maximizes it.

The model-based study retains the established generalized algebraic Riccati equation solution structure and evaluates two numerical approaches:

- Policy iteration, which alternates between policy evaluation and policy improvement.
- Value iteration, which repeatedly applies the generalized Riccati mapping and updates the feedback gains.

The disturbance-aware extension is added through:

- Persistent command-channel offsets.
- Model-based bias calibration.
- Affine command compensation.
- Nominal, uncompensated, estimated-compensation, and exact-compensation simulations.
- Stability, curvature, convergence, and Riccati-residual checks.

## Model-Free Methods

The model-free portion uses sampled transition data rather than direct access to the analytical system matrices.

The joint state-action vector is written as

\[
z_k
=
\begin{bmatrix}
x_k \\
u_{P,k} \\
u_{E,k}
\end{bmatrix},
\]

and the quadratic Q-function is represented by

\[
Q(z_k)
=
z_k^{\top}Hz_k.
\]

The matrix \(H\) is estimated from sampled states, actions, stage costs, and successor states. The pursuer and evader policies are then recovered from the appropriate matrix partitions.

The model-free methods retain the established quadratic Q-learning policy-iteration and value-iteration structures. The disturbance-aware extension adds:

- Commanded-versus-realized input measurements.
- Persistent command-offset estimation.
- Affine compensation during control execution.
- Nominal, uncompensated, and compensated comparisons.
- Bellman-regression, feature-rank, conditioning, gain, and convergence diagnostics.

The model-free designation refers to the fact that the Q-learning updates do not require the analytical state-transition matrices. Measured command-channel information may still be used to estimate persistent actuator offsets.

## MATLAB Validation

The MATLAB studies are designed to verify the mathematical and computational behavior of the extended algorithms.

The primary evaluation cases are:

1. **Nominal operation:** No command-channel offsets are present.

2. **Persistent offsets without compensation:** Constant offsets are added to the realized pursuer and evader inputs.

3. **Persistent offsets with estimated compensation:** Estimated biases are subtracted from the commanded inputs.

4. **Persistent offsets with exact compensation:** The true offsets are used as a verification case to confirm recovery of the nominal trajectory.

The numerical outputs include:

- Policy-iteration or value-iteration convergence histories.
- Generalized algebraic Riccati equation residuals.
- Closed-loop spectral radius.
- Minimizing-player curvature.
- Maximizing-player Schur-complement curvature.
- True and estimated command biases.
- Bias-estimation error.
- Calibration fit error.
- Final state norms.
- Predicted steady-state offsets.
- Reduction in steady-state error.
- Finite-window zero-sum game cost.
- Deviation from the nominal trajectory.
- Exact-compensation verification error.

Each MATLAB script automatically creates a results folder containing:

- MATLAB `.fig` files.
- High-resolution `.png` figures.
- Numerical summary `.csv` files.
- Complete workspace `.mat` files.

## Current Model-Based Policy-Iteration Results

The completed model-based policy-iteration benchmark demonstrates that:

- The generalized Riccati policy iteration converges to a stabilizing saddle solution.
- The final generalized algebraic Riccati residual is near machine precision.
- The closed-loop matrix is Schur stable.
- The pursuer and evader curvature conditions are satisfied.
- Persistent command offsets create a nonzero steady-state deviation.
- Estimated affine compensation substantially reduces the resulting error.
- Exact compensation recovers the nominal trajectory to numerical precision.

These results verify the compensation extension on the supplied zero-sum benchmark before the same structure is transferred to the full pursuit–evasion model.

## Relationship to Autonomous Ground Vehicles

The current work is theoretical and computational. The algorithms are first verified on established discrete-time zero-sum state-space models so that the effect of each extension can be isolated and evaluated.

The later pursuit–evasion formulation will use the previously established autonomous ground-vehicle modeling foundation, including physically meaningful vehicle states and control inputs. Depending on the final model, each vehicle may be represented by states such as

\[
z_{i,k}
=
\begin{bmatrix}
x_{i,k} &
y_{i,k} &
\psi_{i,k} &
v_{i,k}
\end{bmatrix}^{\top},
\qquad
i\in\{P,E\},
\]

with inputs such as

\[
u_{i,k}
=
\begin{bmatrix}
a_{i,k} &
\delta_{i,k}
\end{bmatrix}^{\top}.
\]

In that setting, the scalar command offsets used in the benchmark are replaced by acceleration and steering offset vectors:

\[
b_P
=
\begin{bmatrix}
b_{a,P} \\
b_{\delta,P}
\end{bmatrix},
\qquad
b_E
=
\begin{bmatrix}
b_{a,E} \\
b_{\delta,E}
\end{bmatrix}.
\]

The present MATLAB work therefore establishes the algorithmic and numerical foundation needed before attempting nonlinear vehicle simulation or hardware implementation.

## Presentation

This work was presented at Tennessee Tech's **2026 Research and Creative Inquiry Symposium**.

[View the Tennessee Tech Research and Creative Inquiry Symposium page](https://www.tntech.edu/research/research-day.php)

The symposium presentation reflected an earlier stage of the thesis. The current research direction places greater emphasis on disturbance-aware extensions of established model-based and model-free zero-sum algorithms and on MATLAB-based theoretical validation.

## Abstract

This thesis develops a disturbance-aware framework for discrete-time zero-sum pursuit–evasion control using established model-based and model-free solution methods. The work is theoretical and computational in scope. Rather than introducing an entirely new control algorithm, it extends existing policy-iteration, value-iteration, and Q-learning formulations so that persistent command-channel offsets can be estimated and compensated within a pursuit–evasion setting. The resulting framework is intended to provide the mathematical and computational foundation for later implementation on autonomous ground-vehicle platforms.

The pursuit–evasion problem is formulated as a two-player zero-sum dynamic game in which the pursuer minimizes a quadratic performance index while the evader maximizes the same objective. The model-based portion extends generalized algebraic Riccati equation policy iteration and value iteration by incorporating persistent pursuer and evader command offsets, model-based bias estimation, and affine command compensation. The original Riccati and policy-update structures are retained, while the additional compensation terms account for differences between commanded and realized inputs.

The model-free portion applies corresponding Q-learning policy-iteration and value-iteration methods to the same disturbance-aware game. These methods estimate the quadratic action-value function and recover the pursuer and evader feedback policies from sampled state, input, cost, and successor-state data without requiring direct knowledge of the analytical state-transition matrices. This establishes a common comparison between model-based methods that use known dynamics and model-free methods that recover the same game policies from measured interaction data.

MATLAB experiments are used to examine convergence, saddle-point validity, command-bias estimation, closed-loop stability, finite-window game cost, and deviation from nominal behavior. The numerical studies compare nominal operation, persistent offsets without compensation, and persistent offsets with estimated compensation. The resulting formulation demonstrates how established zero-sum game algorithms can be adapted to a disturbance-compensated pursuit–evasion problem while preserving their original mathematical structure. The thesis therefore serves as foundational work for future nonlinear vehicle studies, QCar implementation, and experimental validation rather than claiming completed hardware deployment.

## Scope

This repository presents a reproducible theoretical, algorithmic, and MATLAB-based validation framework.

The current scope includes:

- One minimizing pursuer and one maximizing evader.
- Discrete-time linear or locally linearized system models.
- Two-player zero-sum quadratic game formulations.
- Generalized algebraic Riccati policy iteration.
- Generalized algebraic Riccati value iteration.
- Quadratic Q-learning policy iteration.
- Quadratic Q-learning value iteration.
- Persistent command-channel offsets.
- Model-based and measurement-based bias estimation.
- Affine command compensation.
- MATLAB convergence and closed-loop studies.
- Foundational development for later vehicle implementation.

The current scope does not include:

- Completed physical QCar experiments.
- Completed QLabs deployment.
- A new policy-iteration or value-iteration theory.
- A claim that the underlying algorithms were invented in this thesis.
- Deep reinforcement-learning actor-critic methods.
- A full autonomous-driving traffic simulator.
- Multi-pursuer or multi-evader coordination.
- Global optimality guarantees for nonlinear vehicle dynamics.
- High-fidelity tire, suspension, or three-dimensional vehicle dynamics.

## Research Contribution

The contribution of this thesis is not the invention of policy iteration, value iteration, or Q-learning.

The contribution is the systematic extension and evaluation of established model-based and model-free zero-sum algorithms for a disturbance-aware pursuit–evasion problem.

The primary contributions are:

1. A common disturbance-aware zero-sum pursuit–evasion formulation.

2. Extension of model-based policy iteration with persistent command-offset estimation and affine compensation.

3. Extension of model-based value iteration using the same compensation structure.

4. Extension of model-free Q-learning policy iteration for the same disturbance-aware game.

5. Extension of model-free Q-learning value iteration under the same information and compensation assumptions.

6. A reproducible MATLAB framework for comparing nominal, uncompensated, estimated-compensation, and exact-compensation behavior.

7. A mathematical and computational foundation for future nonlinear vehicle and QCar implementation.

## Future Work

Future research may extend this framework through:

- Nonlinear kinematic bicycle dynamics.
- Dynamic bicycle models.
- Vector-valued acceleration and steering biases.
- Time-varying or stochastic disturbances.
- Online bias estimation.
- Output-feedback implementations.
- Partial-state measurements.
- Input saturation and command delay.
- Multi-pursuer and multi-evader games.
- QLabs simulation.
- Physical QCar testing.
- Hardware-based disturbance calibration.
- Experimental comparison between model-based and model-free implementations.

## Citation

Citation information for the final thesis will be added after the thesis has been submitted, defended, approved, and archived.

A complete archival citation and permanent institutional link will be added when available.
