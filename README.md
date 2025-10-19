# DigitalTwin-LineFollowingRobot with PID Control
Capstone project for Siemens Digital Twin Course — Line-Following Robot with PID Control using VSI Co-Simulation.

[![Made with Python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)  
[![Siemens Digital Twin](https://img.shields.io/badge/Platform-Siemens%20Digital%20Twin-blue)]()  
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)  

## 🧠 Project Overview  

This project was developed as part of the **Siemens Digital Twin Course (July–August 2025)**.  
It demonstrates a **Line-Following Robot** implemented using the **Virtual System Integration (VSI)** platform to simulate, control, and visualize a differential-drive robot in a co-simulation environment.  

The system integrates **three main components** — a **Simulator**, **Controller**, and **Visualizer** — all communicating via virtual CAN gateways.  

---

## ⚙️ System Architecture  

### 🧩 Components  
- **Simulator**: Models robot kinematics, sends real-time position and orientation data.  
- **Controller**: Implements PID and PD control algorithms to follow a reference trajectory.  
- **Visualizer**: Plots real and reference paths, computes KPIs (overshoot, settling time, steady-state error).  

### 🧵 Communication  
Each component exchanges data via the **VSI Gateway**, simulating real-time CAN communication channels.

---

## 🧮 Modeling and Control  

### Differential Drive Model  
\[
\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega
\]  

### PID Control Law  
\[
\omega = K_p e_y + K_d \dot{e_y} + K_\theta e_\theta + f_{ff}
\]  

where  
- \( e_y \): lateral position error  
- \( e_\theta \): heading error  
- \( f_{ff} \): feedforward curvature term  

Linear velocity is adapted based on heading error to improve stability.

---

## 🧪 Experiments  

### **E1 – PID Gain Sweep**  
Tested 3 sets of PID gains on a straight path to analyze system response.  
📈 **KPIs:** Overshoot, Settling Time, Steady-State Error  
🖼️ [E1_PID_GainSweep png](https://github.com/user-attachments/assets/12ed8e4a-dd5f-45c2-a59c-b494a9c1beff)

---

### **E2 – Curved Path Robustness**  
Evaluated controller performance on curved/sine paths.  
📈 **Observation:** Increased curvature led to higher steady-state error but stable control.  
🖼️ *Insert figure: 

![E2 – Curved Path Robustness](https://github.com/user-attachments/assets/5c6862e2-ef28-4240-9a46-a8c123d45b4a)


---

### **E3 – Noise and Disturbance Rejection**  
Injected Gaussian noise and random angular disturbances.  
📈 **Observation:** PID maintained stability; transient deviations quickly corrected.  
🖼️ ![E3 – Noise and Disturbance Rejection](https://github.com/user-attachments/assets/b97f9030-feb9-4d92-9ec8-0ed5a8a0fc5f)


---

### **E4 – PD vs PID Ablation Study**  
Compared PD (Ki = 0) and PID (Ki = 0.05) under noise conditions.  
📈 **Observation:**  
- PD: Faster but less accurate  
- PID: Better steady-state accuracy with minor lag  
🖼️ r![E4 – PD vs PID Ablation](https://github.com/user-attachments/assets/2fbc37c9-59d6-4069-80da-61b218f1790e)

---

## 📊 KPI Summary  

| Experiment | Overshoot | Settling Time | Steady-State Error | Observation |
|-------------|------------|----------------|--------------------|--------------|
| E1 – PID Gain Sweep | Low–Medium | Moderate | Small | Best balance in Set 2 |
| E2 – Curved Path | Medium | Slightly longer | Higher | Stable but delayed |
| E3 – Noise Robustness | Low | Stable | Slight increase | Robust to noise |
| E4 – PD vs PID | Medium (PD), Low (PID) | Shorter (PD) | Higher (PD) | PID preferred overall |

---

## 🧰 Technologies Used  

- **Python 3.11**  
- **NumPy**, **Matplotlib**  
- **VSI Gateway APIs** (VsiCommonPythonApi, VsiCanPythonGateway)  
- **Siemens Digital Twin (Xcelerator Ecosystem)**  
- **Linux Environment (Ubuntu)**  

---

## 🧠 Key Learning Outcomes  

✅ Understanding of Digital Twin principles and co-simulation workflows  
✅ Real-time communication between virtual systems using CAN interfaces  
✅ PID tuning and performance evaluation across dynamic scenarios  
✅ KPI analysis and visualization for control systems  

---

## 📂 Project Structure  
DigitalTwin-LineFollowingRobot/
│
├── src/
│ ├── simulator.py
│ ├── controller.py
│ ├── visualizer.py
│
├── vsi_files/
│ └── vsiBuildcommands
│
├── results/
│ ├── E1_PID_GainSweep.png
│ ├── E2_CurvedPath.png
│ ├── E3_NoiseRobustness.png
│ ├── E4_PD_vs_PID.png
│ └── KPIs.csv
│
├── report/
│ └── DT_Capstone_Report.docx
│
└── README.md

## 🧑‍💻 Author  

**Patrick Ramez**  
🎓 Siemens Digital Twin Trainee | Computer Engineering Student  
🌐 [LinkedIn](https://www.linkedin.com/in/patrick-ramez/)  
📧 [Email](mailto:patrickramez70@gmail.com)  

---

## 📜 License  
This project is licensed under the [MIT License](LICENSE).  

---

# 🚀 “Simulate, Analyze, Optimize – The Power of Digital Twins.” 

