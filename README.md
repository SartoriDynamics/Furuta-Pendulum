# Furuta Pendulum
A low-cost Furuta Pendulum platform developed for control research and education at UFMG.

This project is released under the MIT License.  
Copyright (c) 2024 GTI - UFMG

---

## Hardware

- 1× ESP-WROOM-32 WEMOS D1 R32 development board
- 1× Nidec 24H DC motors
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

- **Arduino IDE Configuration.txt** → Arduino IDE setup notes for ESP32.
- **Furuta 3D printing files/** → 3D printable CAD models for physical parts.
- **Arduino codes/** → ESP32 firmware.
  - **LQR/** 
    - `LQR.ino` → Main LQR control firmware.

- **Python codes/** → Python notebooks and scripts for design and analysis.
  - `LQR_Controller_Design.ipynb` → State-space modeling and LQR design.
  - `Save_Data_During.ipynb` → Real-time data acquisition.
  - `Save_Data_After.ipynb` → Post-processing and logging.
  - `b2_experiment_model.ipynb` → Experimental model validation.

---

## 📸 Images

<table>
  <tr>
    <td align="center" width="50%" border="1" cellpadding="12">
      <img src="Furuta images/Furuta.png" alt="Assembled Furuta Pendulum prototype" width="100%"/>
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
