# Model-Based and Q-Learning-Based Pursuit-Evasion Control of Autonomous Ground Vehicles Using QCar-Compatible State-Space Models

A portfolio of code, experiments, and documentation for my MSc Thesis at Tennessee Tech University. My research sits at the intersection of optimal control, game-theoretic multi-agent systems, and reinforcement learning, with applications to pursuit–evasion and autonomous vehicles.

This will be presented at the Tennessee Tech Research Day 2026, the link about the event is given here: https://www.tntech.edu/research/research-day.php

**Note**: By the research day, the thesis will still be in progress; however, the first simulations will be completed by the poster submission date April 14th, 2026.

## Abstract

Intelligent Transportation Systems (ITS) and Connected Automated Vehicles (CAVs) increasingly operate in environments where safety-critical decisions emerge from strategic interactions between multiple agents. This thesis investigates pursuit–evasion as a controlled benchmark for comparing two fundamentally different control design philosophies: model-based linear-quadratic (LQ) dynamic game theory and deep reinforcement learning (DRL).

A discrete-time, two-player pursuit–evasion problem is formulated in relative coordinates, capturing the geometry of aggressive following, cut-in, and collision-avoidance scenarios relevant to autonomous vehicle operation. The model-based baseline is derived as a closed-form Nash equilibrium solution to a finite-horizon, zero-sum LQ dynamic game, computed via coupled Riccati equations. The learning-based pursuer is trained using a continuous-control DRL algorithm against a fixed Nash-optimal evader, receiving no explicit knowledge of the system model. Both controllers are evaluated on identical dynamics, state spaces, and performance metrics, enabling a direct and principled comparison.

Beyond nominal conditions, three structured perturbation classes are examined: model mismatch, additive process noise, and hard input saturation. These perturbations are chosen to probe distinct structural vulnerabilities in each design philosophy — the LQ controller's dependence on an accurate plant model and unconstrained inputs, and the DRL policy's dependence on sufficient training data and reward alignment. Performance is assessed across capture rate, mean capture time, cumulative state-and-control cost, control effort, and sample efficiency. The findings of this study aim to clarify when analytical optimality translates into practical reliability and when a learned policy offers a meaningful advantage under uncertainty.
