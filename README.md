# Disturbance-Aware Model-Based and Model-Free Zero-Sum Pursuit-Evasion Control: Theory, Algorithm Development, and MATLAB Validation

> **Project Status:** Active Master of Science thesis project at Tennessee Technological University. This repository contains the reproducible MATLAB implementations, numerical studies, literature documentation, and supporting materials developed for the thesis. The final defended thesis, archival citation, and approved publication links will be added following institutional submission and approval.

## Overview

This repository documents a theoretical and computational research project at the intersection of optimal control, zero-sum dynamic games, model-free control, disturbance compensation, and pursuit–evasion systems.

The thesis investigates how established model-based and model-free zero-sum control algorithms can be extended to a disturbance-aware pursuit–evasion setting. The work does not introduce an entirely new family of control algorithms. Instead, it begins with established policy-iteration, value-iteration, and Q-learning methods and adds the mathematical and computational structures needed to represent, estimate, and compensate persistent command-channel offsets.

The model-based methods use known system dynamics and generalized algebraic Riccati equation formulations. The model-free methods estimate a quadratic action-value function from sampled state, input, cost, and successor-state data without requiring direct access to the analytical state-transition matrices.

All four methods are studied under a common zero-sum game formulation, command-disturbance model, compensation structure, and MATLAB evaluation framework.

The completed thesis is intended to provide the theoretical and computational foundation for later nonlinear vehicle simulation and possible QCar or QLabs implementation. Completed physical QCar validation is not claimed as part of the present work.

## Research Direction

The thesis is organized around four established zero-sum control methods:

1. Model-based policy iteration.
2. Model-based value iteration.
3. Model-free Q-learning policy iteration.
4. Model-free Q-learning value iteration.

The mathematical structures of the original algorithms are retained. The thesis extends their implementation to account for persistent differences between commanded and realized pursuer and evader inputs.

The objective is to develop matched disturbance-aware versions of the four methods rather than replace the original algorithms with unrelated formulations.

## Zero-Sum Game Formulation

The nominal discrete-time system is represented by

$$
x_{k+1}
=
Ax_k
+
B_Pu_{P,k}
+
B_Eu_{E,k},
$$

where:

- \(x_k\) is the game state;
- \(u_{P,k}\) is the pursuer input;
- \(u_{E,k}\) is the evader input;
- \(B_P\) is the pursuer input matrix;
- \(B_E\) is the evader input matrix.

The pursuer acts as the minimizing player, while the evader acts as the maximizing player.

A representative infinite-horizon quadratic game cost is

$$
J
=
\sum_{k=0}^{\infty}
\left(
x_k^{\mathsf T}Qx_k
+
u_{P,k}^{\mathsf T}R_Pu_{P,k}
-
u_{E,k}^{\mathsf T}R_Eu_{E,k}
\right).
$$

The corresponding feedback policies are written as

$$
u_{P,k}=Lx_k,
$$

and

$$
u_{E,k}=Kx_k,
$$

where \(L\) is the pursuer feedback gain and \(K\) is the evader feedback gain.

Under these policies, the nominal closed-loop system is

$$
x_{k+1}
=
\left(
A+B_PL+B_EK
\right)x_k.
$$

## Persistent Command-Channel Offsets

The disturbance-aware extension assumes that the realized inputs may differ from the commanded inputs because of persistent command-channel offsets.

For the pursuer,

$$
u_{P,k}^{\mathrm{actual}}
=
u_{P,k}^{\mathrm{command}}
+
b_P,
$$

and for the evader,

$$
u_{E,k}^{\mathrm{actual}}
=
u_{E,k}^{\mathrm{command}}
+
b_E.
$$

The terms \(b_P\) and \(b_E\) represent constant or slowly varying offsets affecting the pursuer and evader command channels.

Possible physical interpretations include:

- acceleration-command calibration error;
- steering-center offset;
- persistent actuator mismatch;
- repeatable differences between commanded and realized inputs;
- approximately constant implementation error over an experiment.

Without compensation, the disturbed closed-loop system becomes

$$
x_{k+1}
=
\left(
A+B_PL+B_EK
\right)x_k
+
B_Pb_P
+
B_Eb_E.
$$

The feedback gains may remain stabilizing, but the persistent affine term can produce a nonzero steady-state deviation from nominal behavior.

## Command-Offset Estimation

For the model-based methods, the command offsets can be estimated using the known system matrices and measured state transitions.

The transition residual is

$$
r_k
=
x_{k+1}
-
Ax_k
-
B_Pu_{P,k}^{\mathrm{command}}
-
B_Eu_{E,k}^{\mathrm{command}}.
$$

Under the persistent-offset model,

$$
r_k
=
B_Pb_P
+
B_Eb_E
+
\eta_k,
$$

where \(\eta_k\) represents measurement noise, process noise, or numerical error.

The combined bias vector is

$$
b
=
\begin{bmatrix}
b_P \\
b_E
\end{bmatrix},
$$

and the combined input matrix is

$$
B_b
=
\begin{bmatrix}
B_P & B_E
\end{bmatrix}.
$$

The calibration model can therefore be written as

$$
r_k=B_bb+\eta_k.
$$

An estimate of the persistent command offsets may then be obtained from a batch of transition residuals using least squares.

## Affine Command Compensation

The estimated pursuer and evader offsets are denoted by \(\hat b_P\) and \(\hat b_E\).

The compensated pursuer command is

$$
u_{P,k}^{\mathrm{command}}
=
Lx_k-\hat b_P,
$$

and the compensated evader command is

$$
u_{E,k}^{\mathrm{command}}
=
Kx_k-\hat b_E.
$$

After the true offsets enter the realized inputs,

$$
u_{P,k}^{\mathrm{actual}}
=
Lx_k-\hat b_P+b_P,
$$

and

$$
u_{E,k}^{\mathrm{actual}}
=
Kx_k-\hat b_E+b_E.
$$

The compensated closed-loop dynamics are therefore

$$
x_{k+1}
=
\left(
A+B_PL+B_EK
\right)x_k
+
B_P\left(b_P-\hat b_P\right)
+
B_E\left(b_E-\hat b_E\right).
$$

When the estimates are exact,

$$
\hat b_P=b_P,
\qquad
\hat b_E=b_E,
$$

the disturbance terms cancel and the nominal closed-loop dynamics are recovered:

$$
x_{k+1}
=
\left(
A+B_PL+B_EK
\right)x_k.
$$

When the estimates are imperfect, the remaining affine term depends only on the bias-estimation errors.

## Model-Based Methods

### Policy Iteration

The model-based policy-iteration method alternates between policy evaluation and policy improvement.

For fixed feedback gains \(L_i\) and \(K_i\), the closed-loop matrix is

$$
A_i
=
A+B_PL_i+B_EK_i.
$$

The policy-evaluation step determines the value matrix associated with the current pursuer and evader policies.

The policy-improvement step then updates the gains from the generalized zero-sum quadratic game conditions.

The disturbance-aware extension does not replace these Riccati or gain-update equations. Instead, it adds:

- command-offset calibration;
- estimated affine compensation;
- nominal and disturbed simulations;
- closed-loop checks;
- convergence histories;
- numerical output and figure generation.

### Value Iteration

The model-based value-iteration method repeatedly applies the generalized Riccati mapping and updates the corresponding pursuer and evader feedback gains.

The disturbance-aware extension uses the same command-offset model and compensation structure as the policy-iteration method.

This creates a direct comparison between two established model-based solution procedures operating on the same zero-sum game.

## Model-Free Methods

The model-free methods recover the zero-sum feedback policies from sampled transition data without directly using the analytical state-transition matrices in the Q-learning update.

The joint state-action vector is

$$
z_k
=
\begin{bmatrix}
x_k \\
u_{P,k} \\
u_{E,k}
\end{bmatrix}.
$$

The quadratic Q-function is represented as

$$
Q_H(z_k)
=
z_k^{\mathsf T}Hz_k,
$$

where \(H\) is a symmetric matrix estimated from sampled data.

The matrix can be partitioned as

$$
H
=
\begin{bmatrix}
H_{xx} & H_{xP} & H_{xE} \\
H_{Px} & H_{PP} & H_{PE} \\
H_{Ex} & H_{EP} & H_{EE}
\end{bmatrix}.
$$

The pursuer and evader gains are recovered from the appropriate blocks of \(H\) using the zero-sum saddle-point conditions.

### Q-Learning Policy Iteration

The model-free policy-iteration method evaluates the Q-function associated with fixed pursuer and evader policies using sampled transitions. It then improves both policies using the estimated quadratic Q-function matrix.

The method retains the original off-policy Q-learning policy-iteration structure. The disturbance-aware extension adds:

- commanded and realized input histories;
- persistent command-offset estimation;
- affine compensation;
- matched disturbed and nominal simulations;
- regression and conditioning diagnostics.

### Q-Learning Value Iteration

The model-free value-iteration method estimates successive quadratic Q-functions from sampled state transitions and updates the pursuer and evader policies from the resulting matrix partitions.

The same persistent-offset and compensation formulation is applied so that the model-free policy-iteration and value-iteration methods can be compared under matched conditions.

## Common Evaluation Cases

Each algorithm is evaluated using the same primary cases.

### 1. Nominal Operation

No persistent command offsets are present:

$$
b_P=0,
\qquad
b_E=0.
$$

This case establishes the nominal algorithm behavior.

### 2. Persistent Offsets Without Compensation

The true offsets enter the realized inputs, but no bias estimate is subtracted from the commands.

This case measures the effect of persistent actuator or command mismatch.

### 3. Persistent Offsets With Estimated Compensation

The estimated biases are subtracted from the commanded inputs.

This case measures how effectively the estimation and compensation procedure restores nominal behavior.

### 4. Exact Compensation Verification

The true offsets are used in the compensation law.

This is a numerical verification case. It is used to confirm that exact cancellation of the command offsets recovers the nominal trajectory to numerical precision.

## MATLAB Validation

MATLAB is used to examine the theoretical and computational behavior of the four extended algorithms.

The numerical studies may include:

- policy-iteration convergence;
- value-iteration convergence;
- Q-learning regression convergence;
- generalized algebraic Riccati equation residuals;
- closed-loop spectral radius;
- minimizing-player curvature;
- maximizing-player Schur-complement curvature;
- game-matrix conditioning;
- feature-matrix rank;
- regression conditioning;
- Bellman residuals;
- true and estimated command offsets;
- bias-estimation errors;
- calibration residuals;
- nominal state trajectories;
- uncompensated state trajectories;
- compensated state trajectories;
- predicted steady-state deviations;
- finite-window zero-sum game cost;
- deviation from nominal behavior;
- exact-compensation verification error.

The MATLAB scripts are structured to save reproducible outputs such as:

- MATLAB figure files;
- high-resolution PNG figures;
- CSV summary tables;
- MAT files containing the complete numerical workspace.

## Relationship to Pursuit–Evasion Control

The current algorithms are first evaluated on established discrete-time zero-sum state-space models. This allows the Riccati, Q-learning, disturbance-estimation, and compensation components to be studied without immediately introducing the additional complexity of nonlinear vehicle dynamics.

The resulting framework is then intended to support a pursuit–evasion interpretation in which:

- the pursuer minimizes the zero-sum objective;
- the evader maximizes the same objective;
- the pursuer attempts to reduce relative separation;
- the evader attempts to delay or prevent capture;
- persistent command mismatch affects both players;
- compensation is applied without replacing the original game solution method.

## Relationship to Autonomous Ground Vehicles

The broader thesis uses autonomous ground-vehicle pursuit–evasion as the motivating application.

A representative vehicle state may be written as

$$
z_{i,k}
=
\begin{bmatrix}
p_{x,i,k} \\
p_{y,i,k} \\
\psi_{i,k} \\
v_{i,k}
\end{bmatrix},
\qquad
i\in\{P,E\},
$$

where:

- \(p_{x,i,k}\) and \(p_{y,i,k}\) are planar coordinates;
- \(\psi_{i,k}\) is the vehicle heading;
- \(v_{i,k}\) is the longitudinal speed.

A representative control vector is

$$
u_{i,k}
=
\begin{bmatrix}
a_{i,k} \\
\delta_{i,k}
\end{bmatrix},
$$

where \(a_{i,k}\) is the acceleration command and \(\delta_{i,k}\) is the steering command.

For a vehicle implementation, the persistent command offsets may become vector-valued:

$$
b_P
=
\begin{bmatrix}
b_{a,P} \\
b_{\delta,P}
\end{bmatrix},
$$

and

$$
b_E
=
\begin{bmatrix}
b_{a,E} \\
b_{\delta,E}
\end{bmatrix}.
$$

The current linear zero-sum studies therefore serve as the theoretical and computational foundation for later nonlinear vehicle extensions.

## Research Focus

- **Zero-sum pursuit–evasion formulation:** Development of a two-player game with a minimizing pursuer and a maximizing evader.

- **Model-based policy iteration:** Extension of generalized Riccati policy iteration with persistent command-offset estimation and affine compensation.

- **Model-based value iteration:** Extension of generalized Riccati value iteration using the same disturbance model and compensation structure.

- **Model-free Q-learning policy iteration:** Extension of quadratic Q-learning policy iteration using sampled transition data.

- **Model-free Q-learning value iteration:** Extension of quadratic Q-learning value iteration under the same game and disturbance assumptions.

- **Persistent command disturbances:** Representation of repeatable differences between commanded and realized inputs.

- **Disturbance estimation:** Estimation of command offsets from model-based transition residuals or measured command-channel data.

- **Affine compensation:** Cancellation of estimated command offsets during policy execution.

- **Matched MATLAB validation:** Evaluation of all four methods using common disturbance cases and numerical criteria.

- **Vehicle-oriented foundation:** Preparation for later nonlinear kinematic-bicycle, QLabs, or QCar studies.

## Presentation

This work was presented at Tennessee Tech's **2026 Research and Creative Inquiry Symposium**.

[View the Tennessee Tech Research and Creative Inquiry Symposium page](https://www.tntech.edu/research/research-day.php)

The symposium presentation reflects an earlier stage of the thesis. The current research direction places greater emphasis on disturbance-aware extensions of established model-based and model-free zero-sum algorithms and on MATLAB-based theoretical validation.

## Abstract

This thesis develops a disturbance-aware framework for discrete-time zero-sum pursuit–evasion control using established model-based and model-free solution methods. The work is theoretical and computational in scope. Rather than introducing an entirely new control algorithm, it extends existing policy-iteration, value-iteration, and Q-learning formulations so that persistent command-channel offsets can be estimated and compensated within a pursuit–evasion setting. The resulting framework is intended to provide the mathematical and computational foundation for later implementation on autonomous ground-vehicle platforms.

The pursuit–evasion problem is formulated as a two-player zero-sum dynamic game in which the pursuer minimizes a quadratic performance index while the evader maximizes the same objective. The model-based portion extends generalized algebraic Riccati equation policy iteration and value iteration by incorporating persistent pursuer and evader command offsets, model-based bias estimation, and affine command compensation. The original Riccati and policy-update structures are retained, while the additional compensation terms account for differences between commanded and realized inputs.

The model-free portion applies corresponding Q-learning policy-iteration and value-iteration methods to the same disturbance-aware game. These methods estimate the quadratic action-value function and recover the pursuer and evader feedback policies from sampled state, input, cost, and successor-state data without requiring direct knowledge of the analytical state-transition matrices. This establishes a common comparison between model-based methods that use known dynamics and model-free methods that recover the same game policies from measured interaction data.

MATLAB experiments are used to examine convergence, saddle-point validity, command-bias estimation, closed-loop stability, finite-window game cost, and deviation from nominal behavior. The numerical studies compare nominal operation, persistent offsets without compensation, and persistent offsets with estimated compensation. The resulting formulation demonstrates how established zero-sum game algorithms can be adapted to a disturbance-compensated pursuit–evasion problem while preserving their original mathematical structure. The thesis therefore serves as foundational work for future nonlinear vehicle studies, QCar implementation, and experimental validation rather than claiming completed hardware deployment.

## Scope

This repository presents a reproducible theoretical, algorithmic, and MATLAB-based validation framework.

The current scope includes:

- one minimizing pursuer and one maximizing evader;
- discrete-time linear or locally linearized models;
- two-player zero-sum quadratic games;
- generalized Riccati policy iteration;
- generalized Riccati value iteration;
- quadratic Q-learning policy iteration;
- quadratic Q-learning value iteration;
- persistent command-channel offsets;
- command-offset estimation;
- affine command compensation;
- MATLAB convergence and closed-loop studies;
- preparation for later autonomous ground-vehicle implementation.

The current scope does not include:

- completed physical QCar experiments;
- completed QLabs deployment;
- a claim that policy iteration, value iteration, or Q-learning was invented in this thesis;
- deep actor-critic reinforcement-learning methods;
- a full autonomous-driving traffic simulator;
- multi-pursuer or multi-evader coordination;
- global optimality guarantees for nonlinear vehicle dynamics;
- high-fidelity tire, suspension, or three-dimensional vehicle dynamics.

## Research Contribution

The contribution of this thesis is not the invention of policy iteration, value iteration, or Q-learning.

The contribution is the systematic extension and evaluation of established model-based and model-free zero-sum algorithms for a disturbance-aware pursuit–evasion problem.

The primary contributions are:

1. A common disturbance-aware zero-sum pursuit–evasion formulation.

2. An extension of model-based policy iteration with persistent command-offset estimation and affine compensation.

3. An extension of model-based value iteration using the same disturbance and compensation structure.

4. An extension of model-free Q-learning policy iteration under the same zero-sum game.

5. An extension of model-free Q-learning value iteration under matched disturbance and information assumptions.

6. A reproducible MATLAB framework for nominal, uncompensated, estimated-compensation, and exact-compensation studies.

7. A theoretical and computational foundation for later nonlinear vehicle and QCar implementation.

## Future Work

Future work may include:

- nonlinear kinematic-bicycle dynamics;
- dynamic bicycle models;
- vector-valued acceleration and steering offsets;
- online bias estimation;
- slowly time-varying disturbances;
- stochastic command disturbances;
- process and measurement noise;
- partial-state measurements;
- output-feedback formulations;
- actuator saturation;
- communication and command delays;
- multiple pursuers and evaders;
- QLabs simulation;
- physical QCar testing;
- hardware-based command calibration;
- experimental comparison of model-based and model-free implementations.

## Citation

Citation information for the final thesis will be added after the thesis has been submitted, defended, approved, and archived.

A complete archival citation and permanent institutional link will be added when available.

## Citation

Citation information for the final thesis will be added after the thesis has been submitted, defended, approved, and archived.

A complete archival citation and permanent institutional link will be added when available.
