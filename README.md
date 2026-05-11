# Furuta Pendulum
A low-cost Furuta Pendulum platform developed for control research and education at UFMG.

This project is released under the MIT License.  
Copyright (c) 2024 GTI - UFMG

---

## Hardware

- 1× ESP-WROOM-32 WEMOS D1 R32 development board
- 2× Nidec 24H DC motors
- 1× Rotary Incremental Encoder
  
---

## 🛠️ Features

- State-space modeling and linearization of the Furuta Pendulum.  
- LQR (Linear Quadratic Regulator) control design.  
- Real-time implementation on ESP32 using PWM and rotary encoders.  
- Complete 3D-printable structure for assembling the physical prototype.

---

## ⚙️ How It Works

1. The ESP32 runs the control loop at 100 Hz (Ts = 10 ms), reading angular positions and velocities from encoders.  
2. The control signal is computed via LQR feedback and applied to the motor driver using PWM modulation.  
3. Disturbances (step or pulse) can be injected to evaluate controller performance.  
4. The ESP32 sends sampled data through the serial interface, which is logged by the Python script.  
5. The collected data can then be analyzed and compared with Simulink simulations.

---

## 📂 Repository Structure

- **Arduino IDE Configuration.txt** → Configuration guide for Arduino IDE setup with ESP32 boards.

- **Arduino codes/** → ESP32 firmware implementation containing the real-time LQR control algorithm.
  - **LQR/** 
    - `LQR.ino` → Main firmware implementing the LQR controller, sensor reading (rotary encoders), PWM output generation for motor drivers, and serial telemetry transmission.

- **Python codes/** → Python scripts and Jupyter notebooks for LQR controller design, real-time data acquisition, and experimental analysis.
  - `LQR_Controller_Design.ipynb` → Comprehensive notebook for system identification, state-space modeling, linearization, and LQR gains computation.
  - `Save_Data_During.ipynb` → Real-time data acquisition script that collects angular position and velocity measurements during ESP32 operation via serial communication.
  - `Save_Data_After.ipynb` → Post-experiment data processing and logging utility for organizing collected measurements.
  - `b2_experiment_model.ipynb` → Detailed analysis notebook validating experimental data against theoretical model predictions.
  - `b2Experiment.txt` → Sample experimental data log from friction and system characterization tests.
  - `stepData.txt` → Experimental data from step input response tests for model validation.
  - `pulseData.txt` → Experimental data from pulse disturbance injection tests.

- **Furuta images/** → Visual documentation of the platform.
  - `Furuta.png` → Photograph of the assembled Furuta Pendulum physical prototype showing the mechanical structure, motors, and encoder setup.
  - `Diagram.png` → Electronic schematic diagram detailing the ESP32 connections, motor drivers, encoder wiring, and power distribution.

---

## 📸 Images

<img src="Furuta images/Furuta.png" alt="Furuta Pendulum physical prototype with assembled structure, motors, and rotary encoder" width="200"/> 

<img src="Furuta images/Diagram.png" alt="Electronic schematic showing ESP32 microcontroller, motor drivers, encoder connections, and power distribution" width="400"/>

