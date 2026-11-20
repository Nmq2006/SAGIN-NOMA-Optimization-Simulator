# SAGIN-NOMA Optimization & Simulation Platform

[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/Nmq2006/SAGIN-NOMA-Optimization-Simulator.svg?style=social)](https://github.com/Nmq2006/SAGIN-NOMA-Optimization-Simulator)

A comprehensive MATLAB simulation framework and interactive GUI application for joint **UAV Trajectory Optimization** and **Power Allocation** in **Space-Air-Ground Integrated Networks (SAGIN)** empowered by **Non-Orthogonal Multiple Access (NOMA)** over generalized Rician fading channels.

---

## 🌟 Key Highlights & Modules

The repository consists of two integrated components:

1. **Simulation & Optimization Engine (`Optimal_NOMA_SAGIN_model`)**:
   - **Alternating Optimization (AO)**: Solves the non-convex joint optimization problem via Block Coordinate Descent (BCD), Successive Convex Approximation (SCA), and Dinkelbach-like fractional programming.
   - **Deep Reinforcement Learning (DRL)**: Real-time autonomous trajectory design and power control powered by **Deep Deterministic Policy Gradient (DDPG)**.
   - **Theoretical Outage Probability Analysis**: Closed-form mathematical derivations utilizing first-order **Marcum-Q functions** for two-hop dual-user downlink systems with Partial Relay Selection (PRS) and Successive Interference Cancellation (SIC) boundaries.
   - **Complete Academic Documentation**: Full technical report and LaTeX lecture notes (in both English and Vietnamese).

2. **Interactive Simulator GUI (`SAGIN_NOMA_Simulator_APP`)**:
   - Built with **MATLAB App Designer**.
   - Interactive 2D spatial trajectory visualizer ($Oxz$ plane), convergence monitors, SNR sweeps, and real-time SIC verification.
   - Pre-trained DRL models and AO benchmark datasets included for instant out-of-the-box simulation.
   - One-click export for high-resolution vector figures and `.mat` data logs.

---

## 📂 Project Structure

```text
SAGIN-NOMA-Optimization-Simulator/
├── Optimal_NOMA_SAGIN_model/         # Mathematical models & algorithmic core
│   ├── core/                         # Core algorithmic implementations
│   │   ├── ao/                       # Alternating Optimization (AO) engine
│   │   ├── drl/                      # DRL (DDPG) agents & environment
│   │   └── utils/                    # Marcum-Q, channel models, & SIC checkers
│   ├── assets/                       # High-resolution benchmark figures
│   ├── docs/                         # Technical reports & IEEE-style paper (PDF)
│   ├── LECTURE NOTES/                # Academic documentation (LaTeX - English)
│   ├── LECTURE NOTES (Vie)/          # Academic documentation (LaTeX - Vietnamese)
│   ├── run_all.m                     # Master batch execution script
│   └── test_engine.m                 # Automated unit tests & engine verification
│
├── SAGIN_NOMA_Simulator_APP/         # Graphical User Interface (GUI) Application
│   ├── SAGIN_NOMA_App.mlapp          # Main MATLAB App Designer file
│   ├── start_app.m                   # Quick-start script for the GUI
│   ├── core/                         # Computational engine adapter for GUI
│   ├── data/                         # Pre-trained DRL weights & benchmark data
│   └── exports/                      # Target directory for exported figures & data
│
├── .gitignore                        # Git exclusion rules (MATLAB & LaTeX temp files)
└── README.md                         # Repository documentation
```

---

## 🚀 Getting Started

### Prerequisites
- **MATLAB R2022b** or later.
- Required Toolboxes:
  - *Optimization Toolbox*
  - *Statistics and Machine Learning Toolbox*
  - *Deep Learning Toolbox* & *Reinforcement Learning Toolbox* (for DRL training/evaluation)

---

### Execution Instructions

#### 1. Running the Algorithmic Simulation Core
To run the full suite of optimization benchmarks (AO vs. DRL vs. Baseline) and generate all numerical results:
```matlab
% In the MATLAB Command Window:
cd('Optimal_NOMA_SAGIN_model');
run_all;
```
To run quick system verification tests:
```matlab
test_engine;
```

#### 2. Launching the Interactive GUI App
To open the graphical simulator dashboard:
```matlab
% In the MATLAB Command Window:
cd('SAGIN_NOMA_Simulator_APP');
start_app;
```
*Alternatively, you can double-click `SAGIN_NOMA_App.mlapp` in the MATLAB Current Folder panel.*

---

## 📊 Benchmark Performance Summary

Benchmarked at nominal transmit signal-to-noise ratio $\rho = \bar{\gamma}_S = \bar{\gamma}_U = 20\text{ dB}$:

| Evaluation Metric | Static Snapshot (AO) | Dynamic Initial (Unoptimized) | Dynamic AO ($N=10$) | Dynamic DRL (DDPG) |
| :--- | :---: | :---: | :---: | :---: |
| **System Outage ($\bar{P}_{\text{out}}$)** | $\sim 2.15 \times 10^{-2}$ | $\sim 3.82 \times 10^{-1}$ | **$\sim 1.48 \times 10^{-3}$** | **$\sim 2.05 \times 10^{-3}$** |
| **Outage Reduction vs. Initial** | Baseline | $0\%$ | **$\mathbf{99.61\%}$** | **$\mathbf{99.46\%}$** |
| **Average Flight Altitude ($z$)** | Static | $50.0\text{ m}$ | $35.2\text{ m}$ | $36.8\text{ m}$ |
| **SIC Condition ($x_U \le 1.21$)** | Strictly met | Strictly met | Strictly met | Strictly met |
| **Execution Latency** | Offline | N/A | High (Iterative) | **Real-time ($< 5\text{ ms}$)** |

---

## 📄 License & Attribution

This project is open-source and available under the [MIT License](LICENSE). 

For academic references or citations, please refer to the technical report located in [`Optimal_NOMA_SAGIN_model/docs/`](Optimal_NOMA_SAGIN_model/docs/).
