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
  - Reads sensor data from rotary encoders.
  - Computes LQR control signals.
  - Generates PWM outputs to motor drivers.
  - Sends telemetry data via serial communication.
- **Python codes/** → Python scripts for real-time data acquisition and logging from the ESP32 via serial port.
  - Collects angular position and velocity measurements.
  - Logs experimental data for analysis and validation.
- **Furuta images/** → Images and diagrams of the physical prototype and electronic schematic.

---

## 📸 Images

<img src="Furuta images/Furuta.png" alt="Furuta Pendulum" width="200"/> 

<img src="Furuta images/Diagram.png" alt="Electronic schematic" width="400"/>
