# Predictive Adaptive PI Control for Solar-Powered Irrigation

**Course:** MAE 506: Advanced System Modeling, Dynamics and Control (Arizona State University)  
**Team 15:** Ankur Guruprasad, Anudeep Sai Gottapu, Kapish Dubey, Yashwanth Gowda  
**Date:** December 2025

## 📌 Project Overview
This project designs, models, and simulates a robust control architecture for a solar-powered irrigation pump. Solar pumps face two distinct categories of volatility:
1.  **Supply-Side:** Fluctuations in voltage due to cloud cover.
2.  **Load-Side:** Sudden pressure changes due to pipe clogs or water table variations.

The objective was to maintain a constant flow rate of **15 L/min** despite these stochastic disturbances using a SISO control architecture.

## 🚀 Key Features
* **Physics-Based Modeling:** Derived State-Space equations for the electromechanical DC pump system.
* **Three Controller Architectures:**
    1.  **LQR (Linear Quadratic Regulator):** Optimized for stability margins.
    2.  **Standard PI:** Baseline industrial controller.
    3.  **Predictive Adaptive PI (Proposed):** A hybrid 2-DOF controller using inverse dynamics feedforward.
* **Simulation:** Validated across three scenarios: Sunny (Steady), Cloudy (Step Disturbance), and Overcast (Stochastic Noise).

## 📊 Results
The proposed **Predictive Adaptive PI** controller significantly outperformed traditional methods.

| Controller | Rise Time | Overshoot | Disturbance Response |
| :--- | :--- | :--- | :--- |
| **LQR** | 2.22 s | 7.9% | Slow Recovery |
| **Standard PI** | 3.03 s | 0.03% | Significant Flow Sag |
| **Predictive PI** | **1.20 s** | **4.9%** | **Instant (Flat)** |

> *The Predictive PI controller utilized a physics-based feedforward term to provide ~70% of the required voltage instantly, reducing rise time by approx. 60%.*

## 📂 Repository Structure
* `src/`: Contains the MATLAB simulation script (`MAE_506_Final_code.m`).
* `docs/`: Contains the Final Report (PDF) and Project Presentation (PPTX).

## ⚙️ How to Run
1.  Open MATLAB.
2.  Navigate to the `src/` folder.
3.  Run `MAE_506_Final_code.m`.
4.  The script will generate:
    * **Figures 1-3:** Flow response plots for Sunny, Cloudy, and Overcast scenarios.
    * **Figure 4:** A GUI table comparing quantitative performance metrics (Rise Time, Overshoot, Settling Time).

## 📝 License
This project is for educational purposes under the MAE 506 curriculum.
