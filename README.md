# Furuta Pendulum

A low-cost, open-source Furuta Pendulum platform developed for control research and education at UFMG.

This project is released under the MIT License.  
Copyright (c) 2024 GTI - UFMG

---

## 🔩 Hardware

- 1× ESP-WROOM-32 WEMOS D1 R32 development board
- 1× NIDEC 24H DC motor with an integrated encoder
- 1× rotary incremental encoder for the pendulum arm

---

## 🛠️ Features

- State-space modeling and linearization about the upright and downward equilibria.
- LQG control combining LQR state feedback with Kalman state estimation.
- Stabilization and disturbance-rejection experiments at both operating points.
- Nonlinear simulations and controller design in Python.
- Real-time implementation on an ESP32 using PWM and rotary encoders.
- Complete 3D-printable structure for assembling the physical prototype.

---

## ⚙️ How It Works

1. The ESP32 runs the control loop at 100 Hz (`Ts = 10 ms`) and reads the two angular positions from the encoders.
2. A Kalman filter processes the angular-position measurements and estimates the complete system state, including the two angular velocities.
3. The control voltage is computed using LQR state feedback and applied to the motor driver through PWM.
4. Separate LQG controllers stabilize the system about the upright and downward equilibria.
5. Pulse and step disturbances are applied to evaluate disturbance rejection at both operating points.
6. The sampled data are transmitted through the serial interface and compared with nonlinear Python simulations.

---

## 📂 Repository Structure

- **Arduino IDE Configuration.txt** → Arduino IDE setup notes for the ESP32.
- **Furuta 3D printing files/** → 3D-printable CAD models for the physical components.
- **Arduino codes/** → ESP32 firmware for real-time state estimation and control.
  - **LQG_up/**
    - `LQG_up.ino` → LQG firmware for stabilization and disturbance-rejection experiments about the upright equilibrium.
  - **LQG_down/**
    - `LQG_down.ino` → LQG firmware for stabilization and disturbance-rejection experiments about the downward equilibrium.
- **Python codes/** → Python notebooks for modeling, controller design, and nonlinear simulation.
  - `LQR_Controller_Upright.ipynb` → LQR design and nonlinear simulation about the upright equilibrium.
  - `LQR_Controller_Downward.ipynb` → LQR design and nonlinear simulation about the downward equilibrium.
  - `b2_experiment_model.ipynb` → Identification and validation of the pendulum viscous-friction parameter.

---

## 📸 Images

<table>
  <tr>
    <td align="center" width="50%" border="1" cellpadding="12">
      <img src="Furuta images/Furuta.png" alt="Assembled Furuta Pendulum prototype" width="57.24%"/>
    </td>
    <td align="center" width="50%" border="1" cellpadding="12">
      <img src="Furuta images/Diagram.png" alt="ESP32 wiring and electronics diagram" width="100%"/>
    </td>
  </tr>
  <tr>
    <td align="center">
      <em>Assembled Furuta Pendulum prototype.</em>
    </td>
    <td align="center">
      <em>ESP32 wiring and electronics diagram.</em>
    </td>
  </tr>
</table>
