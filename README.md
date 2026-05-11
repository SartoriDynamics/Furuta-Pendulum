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

- **/3d_print** → STL files for 3D printing the physical structure of the Furuta Pendulum prototype.
- **/arduino_code/furuta** → Contains the control algorithm implemented on the ESP32 (Arduino IDE).  
  - **furuta.ino** → Main code that runs the real-time LQR control loop, reads encoder data, applies PWM signals to the motor, logs measurements, and sends data via serial.
- **/matlab_simulink** → MATLAB scripts and Simulink models for system modeling, simulation, and validation.  
  - **LQR.mlx** → Computes the optimal LQR gain matrix.  
  - **simulation_linear_model.slx** → Simulink model for the linearized system.  
  - **simulation_non_linear_model.slx** → Simulink model for the nonlinear system.  
  - **data_log_pulse.txt** / **data_log_step.txt** → Experimental data collected from pulse and step disturbance tests.  
  - **plots_pulse.mlx** / **plots_step.mlx** → Scripts for generating comparative plots (simulation vs. experimental data).  
- **/python_code** → Scripts for data acquisition from the ESP32 via serial communication.  
  - **get_data.ipynb** / **get_data.py** → Identical scripts for collecting and saving real-time data from the ESP32 to `data_log.txt`. 
- **3D CAD Online** → [View on Onshape](https://cad.onshape.com/documents/d0d2d40c9dd0c88f858cd038/w/86dce8532c9b6c8310216a8c/e/c2fa23f7c1c4edc232f4e580?renderMode=0&uiState=68e07acc73f9dd32bc20c706)  

---

## Images

<img src="/Furuta images/Furuta.png"
alt="Furuta Pendulum" width="200"/> 
</p>

<img src="/Furuta images/Diagram.png"
alt="Electronic schematic" width="400"/>
</p>
