# Literature Review and Research Foundations

This directory documents the literature foundation for the thesis:

**Disturbance-Aware Model-Based and Model-Free Zero-Sum Pursuit-Evasion Control: Theory, Algorithm Development, and MATLAB Validation**

The thesis brings together research from zero-sum dynamic games, generalized algebraic Riccati equations, model-based policy iteration, model-based value iteration, model-free quadratic Q-learning, persistent input disturbances, command-bias estimation, affine compensation, numerical verification, and pursuit-evasion control.

The literature is used to establish the original algorithms, define the disturbance-aware extension, justify the numerical validation criteria, and connect the theoretical framework to future autonomous-ground-vehicle applications.

---

## Purpose of This Literature Collection

The literature review supports one central objective:

> Extend established model-based and model-free zero-sum control algorithms so that persistent command-channel offsets can be represented, estimated, and compensated without replacing the original policy-iteration, value-iteration, or Q-learning structures.

The literature is not included only as background material. Each source category supports a specific part of the thesis:

- the nominal zero-sum game formulation;
- the generalized Riccati solution;
- model-based policy iteration;
- model-based value iteration;
- model-free Q-learning policy iteration;
- model-free Q-learning value iteration;
- persistent command-offset modeling;
- bias estimation and affine compensation;
- convergence and numerical well-posedness checks;
- pursuit-evasion interpretation;
- future autonomous-vehicle implementation.

---

## Thesis Algorithm Structure

The thesis is organized around four established zero-sum control methods:

1. Model-based policy iteration.
2. Model-based value iteration.
3. Model-free Q-learning policy iteration.
4. Model-free Q-learning value iteration.

The thesis does not claim that these algorithms were invented in this work.

Their original mathematical roles are retained. The research contribution is the addition of a common disturbance-aware structure involving:

- persistent command-channel offsets;
- command-offset estimation;
- affine command compensation;
- matched nominal and disturbed simulations;
- common convergence and stability checks;
- reproducible MATLAB validation.

---

## Literature Map

| Literature Area | Role in the Thesis |
|---|---|
| Zero-sum linear-quadratic dynamic games | Defines the shared two-player minimax control problem. |
| Generalized algebraic Riccati equations | Provides the model-based saddle-point solution. |
| Model-based policy iteration | Supports the first model-based algorithm. |
| Model-based value iteration | Supports the second model-based algorithm. |
| Quadratic Q-learning policy iteration | Supports the first model-free algorithm. |
| Quadratic Q-learning value iteration | Supports the second model-free algorithm. |
| Persistent actuator and command biases | Defines the disturbance model added to both players. |
| Bias estimation and offset compensation | Supports calibration, estimation, and affine compensation. |
| Numerical linear algebra and game solvability | Supports conditioning, curvature, convergence, and stability checks. |
| Pursuit-evasion game theory | Provides the pursuer-versus-evader interpretation. |
| Autonomous ground vehicles and QCar | Provides a future application and implementation pathway. |

---

## 1. Zero-Sum Linear-Quadratic Dynamic Games

This literature provides the main theoretical foundation for the thesis.

The nominal discrete-time zero-sum system is represented as:

```text
x(k+1) = A*x(k) + Bp*uP(k) + Be*uE(k)
```

where:

- `x(k)` is the game state;
- `uP(k)` is the pursuer or minimizing-player input;
- `uE(k)` is the evader or maximizing-player input;
- `Bp` is the pursuer input matrix;
- `Be` is the evader input matrix.

A representative quadratic game cost is:

```text
J = sum over k of:

    x(k)'*Q*x(k)
  + uP(k)'*RP*uP(k)
  - uE(k)'*RE*uE(k)
```

The pursuer attempts to minimize the cost, while the evader attempts to maximize the same cost.

This literature supports:

- two-player zero-sum dynamic games;
- minimax control;
- saddle-point policies;
- quadratic value functions;
- discrete-time game formulations;
- infinite-horizon and finite-horizon game theory;
- existence and uniqueness conditions;
- closed-loop stabilizing solutions.

The zero-sum game formulation is the common mathematical foundation used by all four algorithms.

---

## 2. Generalized Algebraic Riccati Equations

The generalized algebraic Riccati equation provides the model-based saddle-point solution for the nominal game.

A quadratic value function is represented as:

```text
V(x) = x'*P*x
```

where `P` is the game value matrix.

The pursuer and evader feedback policies are written as:

```text
uP(k) = L*x(k)
uE(k) = K*x(k)
```

The nominal closed-loop system is therefore:

```text
x(k+1) = (A + Bp*L + Be*K)*x(k)
```

This literature supports:

- generalized algebraic Riccati equations;
- zero-sum feedback saddle policies;
- minimizing-player and maximizing-player gain recovery;
- stabilizing value matrices;
- game-Hessian structure;
- block-matrix saddle conditions;
- Riccati residual calculations;
- closed-loop spectral-radius checks.

The Riccati equation is not modified by the disturbance-aware extension. It remains the nominal game solution from which the feedback policies are obtained.

---

## 3. Model-Based Policy Iteration

This literature supports the first model-based algorithm.

Model-based policy iteration alternates between:

1. Policy evaluation.
2. Policy improvement.

For fixed policies, the closed-loop matrix is:

```text
Ai = A + Bp*Li + Be*Ki
```

The policy-evaluation step determines the value matrix associated with the current pursuer and evader gains.

The policy-improvement step updates both feedback gains using the zero-sum saddle-point conditions.

This literature supports:

- iterative policy evaluation;
- discrete Lyapunov equations;
- zero-sum policy improvement;
- stabilizing initial policies;
- convergence of value matrices and feedback gains;
- comparison with direct Riccati solutions;
- implementation of policy iteration in MATLAB.

The disturbance-aware extension is added outside the original nominal policy-iteration structure.

It introduces:

- command-offset estimation;
- affine compensation;
- nominal and disturbed rollouts;
- convergence histories;
- Riccati residual checks;
- stability and curvature checks.

---

## 4. Model-Based Value Iteration

This literature supports the second model-based algorithm.

Model-based value iteration repeatedly applies the generalized Riccati mapping to update the value matrix.

After each value update, the pursuer and evader gains are recovered from the current matrix.

This literature supports:

- repeated Bellman or Riccati value updates;
- convergence of the value matrix;
- gain extraction from the current value estimate;
- differences between value iteration and policy iteration;
- numerical behavior under different initial conditions;
- model-based zero-sum dynamic programming.

The disturbance-aware value-iteration implementation uses the same:

- command-offset model;
- bias-estimation procedure;
- affine compensation law;
- simulation cases;
- numerical validation criteria.

The purpose is to compare two established model-based solution procedures under matched disturbance conditions.

---

## 5. Model-Free Quadratic Q-Learning

This literature supports the two model-free algorithms.

The model-free methods estimate the game action-value function from sampled transitions rather than directly using the analytical state-transition matrices in the Q-learning update.

The joint state-action vector is:

```text
z(k) = [x(k); uP(k); uE(k)]
```

The quadratic Q-function is represented as:

```text
QH(z(k)) = z(k)'*H*z(k)
```

The matrix `H` contains the state, pursuer-input, evader-input, and cross-term information needed to recover the game policies.

A representative partition is:

```text
H = [Hxx  HxP  HxE
     HPx  HPP  HPE
     HEx  HEP  HEE]
```

This literature supports:

- quadratic Q-functions;
- Bellman equations;
- model-free control of linear systems;
- off-policy learning;
- least-squares estimation;
- symmetric-matrix parameterization;
- state-action feature construction;
- policy recovery from matrix partitions;
- persistence of excitation;
- regression-rank requirements;
- conditioning of the data matrix.

The model-free designation means that the Q-learning update does not require direct access to the analytical state-transition matrices.

---

## 6. Model-Free Q-Learning Policy Iteration

This literature supports the first model-free algorithm.

Q-learning policy iteration alternates between:

1. Estimating the Q-function associated with fixed pursuer and evader policies.
2. Recovering improved policies from the estimated Q-function matrix.

The method uses sampled data consisting of:

- states;
- pursuer actions;
- evader actions;
- stage costs;
- successor states.

This literature supports:

- off-policy policy evaluation;
- Bellman regression;
- quadratic function approximation;
- least-squares Q-function fitting;
- pursuer and evader gain recovery;
- iterative policy improvement;
- data-rank and conditioning requirements;
- model-free convergence diagnostics.

The disturbance-aware extension adds:

- commanded input histories;
- realized input histories;
- persistent command-offset estimation;
- affine compensation;
- nominal and disturbed comparisons.

---

## 7. Model-Free Q-Learning Value Iteration

This literature supports the second model-free algorithm.

Q-learning value iteration estimates successive action-value functions from sampled transition data and updates the pursuer and evader policies from the estimated quadratic matrix.

This literature supports:

- model-free Bellman value updates;
- quadratic Q-function estimation;
- sample-based value iteration;
- recovery of minimizing and maximizing policies;
- differences between Q-learning policy iteration and Q-learning value iteration;
- convergence and regression diagnostics.

The same disturbance and compensation assumptions are used so that all four algorithms can be evaluated using a common framework.

---

## 8. Persistent Command-Channel Offsets

This literature defines the main disturbance model used in the thesis.

The realized pursuer input is represented as:

```text
uP_actual(k) = uP_command(k) + bP
```

The realized evader input is represented as:

```text
uE_actual(k) = uE_command(k) + bE
```

The terms `bP` and `bE` represent persistent or slowly varying command-channel offsets.

Possible interpretations include:

- actuator calibration error;
- steering-center offset;
- acceleration-command mismatch;
- repeatable actuator bias;
- difference between commanded and realized input;
- constant implementation error during an experiment.

Without compensation, the closed-loop system becomes:

```text
x(k+1) = (A + Bp*L + Be*K)*x(k)
         + Bp*bP
         + Be*bE
```

This literature supports:

- actuator bias models;
- constant input disturbances;
- matched disturbances;
- unknown affine system terms;
- command-channel mismatch;
- persistent steady-state error.

This disturbance model is applied consistently to the model-based and model-free algorithms.

---

## 9. Command-Offset Estimation

This literature supports estimation of the persistent pursuer and evader command offsets.

For the model-based methods, a transition residual can be constructed as:

```text
r(k) = x(k+1)
       - A*x(k)
       - Bp*uP_command(k)
       - Be*uE_command(k)
```

Under the persistent-offset model:

```text
r(k) = Bp*bP + Be*bE + noise(k)
```

The combined offset vector is:

```text
bias = [bP; bE]
```

The combined bias-input matrix is:

```text
Bbias = [Bp Be]
```

The calibration relationship becomes:

```text
r(k) = Bbias*bias + noise(k)
```

A batch of transition measurements can then be used to estimate the offsets.

This literature supports:

- least-squares parameter estimation;
- transition-residual methods;
- constant-disturbance estimation;
- input-bias identification;
- batch calibration;
- measurement-noise sensitivity;
- identifiability and rank conditions.

For the model-free algorithms, measured commanded and realized inputs may be used without requiring the analytical state-transition matrices in the Q-learning update.

---

## 10. Affine Command Compensation

This literature supports the compensation layer added after bias estimation.

The estimated offsets are denoted by:

```text
bP_hat
bE_hat
```

The compensated commands are:

```text
uP_command(k) = L*x(k) - bP_hat
uE_command(k) = K*x(k) - bE_hat
```

After the true offsets enter the realized input channels:

```text
uP_actual(k) = L*x(k) - bP_hat + bP
uE_actual(k) = K*x(k) - bE_hat + bE
```

The compensated closed-loop system becomes:

```text
x(k+1) = (A + Bp*L + Be*K)*x(k)
         + Bp*(bP - bP_hat)
         + Be*(bE - bE_hat)
```

When the estimates are exact:

```text
bP_hat = bP
bE_hat = bE
```

the nominal closed-loop dynamics are recovered.

This literature supports:

- offset-free control;
- actuator pre-compensation;
- affine feedback laws;
- disturbance cancellation;
- residual-error analysis;
- steady-state error reduction.

---

## 11. Numerical Well-Posedness and Verification

This literature supports the numerical criteria used to verify the four algorithms.

The relevant checks include:

- generalized Riccati residual;
- policy or value iteration convergence;
- closed-loop spectral radius;
- minimizing-player curvature;
- maximizing-player Schur-complement curvature;
- reciprocal matrix condition number;
- invertibility of game matrices;
- feature-matrix rank;
- regression conditioning;
- Bellman residuals;
- gain-update convergence;
- exact-compensation verification.

For the model-based algorithms, the nominal closed-loop matrix is:

```text
Acl = A + Bp*L + Be*K
```

A stabilizing discrete-time solution requires:

```text
spectral_radius(Acl) < 1
```

The minimizing-player control block must have the appropriate positive curvature, while the maximizing-player Schur complement must have the appropriate negative curvature.

These checks are necessary because numerical convergence alone does not guarantee a valid zero-sum saddle solution.

---

## 12. Pursuit-Evasion Interpretation

This literature provides the application interpretation of the zero-sum game.

The pursuer is identified with the minimizing player, while the evader is identified with the maximizing player.

The general interpretation is:

- the pursuer attempts to reduce a relative-state error or reach a capture condition;
- the evader attempts to increase the same cost or delay capture;
- the two players have opposing objectives;
- both players act through their own control channels;
- command disturbances may affect either player.

Important topics include:

- pursuit-evasion differential games;
- discrete-time pursuit-evasion;
- feedback saddle policies;
- relative-state formulations;
- capture conditions;
- evasion objectives;
- adversarial control.

The current thesis uses pursuit-evasion as the interpretation of the zero-sum dynamic game rather than attempting to reproduce every feature of a full nonlinear vehicle engagement.

---

## 13. Autonomous Ground Vehicles and QCar

This literature provides a future application pathway rather than the central present algorithmic contribution.

A representative vehicle state may be written as:

```text
vehicle_state_i(k) = [
    x_position_i(k)
    y_position_i(k)
    heading_i(k)
    speed_i(k)
]
```

A representative vehicle input may be written as:

```text
vehicle_input_i(k) = [
    acceleration_i(k)
    steering_i(k)
]
```

For a vehicle-oriented implementation, the scalar offsets can be replaced by vectors:

```text
bP = [
    pursuer_acceleration_offset
    pursuer_steering_offset
]
```

```text
bE = [
    evader_acceleration_offset
    evader_steering_offset
]
```

This literature supports:

- kinematic bicycle models;
- dynamic bicycle models;
- steering and acceleration command channels;
- actuator constraints;
- QCar and QLabs interfaces;
- future hardware calibration;
- future nonlinear validation.

The current thesis does not claim completed QCar or QLabs implementation.

---

## Literature-to-Thesis Traceability

| Thesis Component | Supporting Literature | Purpose |
|---|---|---|
| Problem formulation | Zero-sum games and pursuit-evasion | Defines the minimizing pursuer and maximizing evader. |
| Model-based policy iteration | Riccati theory, Lyapunov equations, policy iteration | Supports the first model-based algorithm. |
| Model-based value iteration | Dynamic programming and generalized Riccati updates | Supports the second model-based algorithm. |
| Model-free policy iteration | Quadratic Q-learning and Bellman regression | Supports the first model-free algorithm. |
| Model-free value iteration | Sample-based value iteration and Q-function approximation | Supports the second model-free algorithm. |
| Disturbance model | Actuator bias and persistent input disturbances | Defines command-channel offsets. |
| Bias estimation | Least squares, residual estimation, parameter identification | Supports command-offset calibration. |
| Compensation | Offset-free and affine control | Supports bias cancellation. |
| Numerical validation | Numerical linear algebra and game solvability | Supports stability, curvature, rank, and conditioning checks. |
| Future vehicle application | Ground-vehicle models, QCar, QLabs | Supports later nonlinear and hardware-oriented work. |

---

## Central Research Gap

Established model-based and model-free zero-sum control algorithms provide policy-iteration and value-iteration solutions for nominal linear dynamic games.

However, these methods are commonly studied under idealized input assumptions in which commanded and realized player inputs are identical.

Persistent command-channel offsets can introduce affine closed-loop errors even when the nominal pursuer and evader policies remain stabilizing.

The literature does not commonly provide a unified framework that applies the same disturbance model, bias-estimation procedure, compensation law, and numerical evaluation criteria to:

1. Model-based policy iteration.
2. Model-based value iteration.
3. Model-free Q-learning policy iteration.
4. Model-free Q-learning value iteration.

This thesis addresses that gap by extending the four established methods with:

- persistent pursuer and evader command offsets;
- command-offset estimation;
- affine compensation;
- matched nominal and disturbed simulations;
- common numerical verification;
- reproducible MATLAB implementation.

The underlying policy-iteration, value-iteration, and Q-learning methods remain established algorithms. The contribution is their matched disturbance-aware extension and evaluation within a zero-sum pursuit-evasion framework.

---

## Recommended Literature Folder Structure

```text
Literature/
├── 01_Zero_Sum_Linear_Quadratic_Games/
├── 02_Generalized_Algebraic_Riccati_Equations/
├── 03_Model_Based_Policy_Iteration/
├── 04_Model_Based_Value_Iteration/
├── 05_Quadratic_Q_Learning_Foundations/
├── 06_Q_Learning_Policy_Iteration/
├── 07_Q_Learning_Value_Iteration/
├── 08_Persistent_Input_Bias_and_Disturbances/
├── 09_Bias_Estimation_and_Identification/
├── 10_Affine_and_Offset_Free_Compensation/
├── 11_Numerical_Well_Posedness_and_Stability/
├── 12_Pursuit_Evasion_Games/
├── 13_Autonomous_Vehicles_QCar_and_QLabs/
├── references.bib
└── literature_notes.md
```

---

## Citation and Source Management

All technical claims in the thesis should be traceable to a peer-reviewed paper, textbook, technical report, dissertation, or official platform source.

Recommended practice:

- use textbooks for foundational zero-sum game and Riccati theory;
- use peer-reviewed papers for policy-iteration, value-iteration, and Q-learning claims;
- use primary sources for the four algorithms;
- use estimation and control literature for the disturbance-aware extension;
- use numerical-analysis references for conditioning and stability claims;
- use official Quanser and MathWorks documentation only for platform and software details;
- distinguish original algorithm sources from later implementation or application papers;
- verify author names, publication year, venue, page numbers, and DOI information;
- keep the BibTeX database synchronized with the PDF library;
- record how each source supports a specific thesis equation, algorithm, or design decision.

---

## Scope Notes

This literature collection supports a theoretical and computational zero-sum control study.

The current thesis focuses on:

- one minimizing pursuer and one maximizing evader;
- discrete-time linear zero-sum systems;
- generalized Riccati policy iteration;
- generalized Riccati value iteration;
- quadratic Q-learning policy iteration;
- quadratic Q-learning value iteration;
- persistent command-channel offsets;
- offset estimation;
- affine compensation;
- MATLAB-based validation.

The current thesis does not claim:

- invention of the original four algorithms;
- completed physical QCar experiments;
- completed QLabs deployment;
- deep actor-critic reinforcement learning;
- global nonlinear pursuit-evasion optimality;
- a full autonomous-driving traffic simulator;
- multi-pursuer or multi-evader coordination;
- high-fidelity tire or suspension dynamics.

---

## Associated Thesis

```text
Blaine Christopher Swieder

Disturbance-Aware Model-Based and Model-Free Zero-Sum Pursuit-Evasion Control:
Theory, Algorithm Development, and MATLAB Validation

Master of Science Thesis
Tennessee Technological University
2026
```
