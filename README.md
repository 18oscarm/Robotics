# Robotics Coursework Projects

This repository contains two major projects completed as part of the **University of Bristol's Robotics** unit.

## 1. Individual Project – Magnet Detection & Return Navigation
Designed and programmed an **autonomous robot** capable of detecting a **magnet** and accurately returning to its **starting point**.  
This project demonstrates a **well-structured embedded C++ codebase** with strong focus on **robust control** and **sensor integration**.

- **Precise Sensor Calibration & Navigation Algorithms** – ensured repeatable, high-accuracy performance in real-world tests.  
- **Finite State Machine (FSM)** – managed the robot's operational workflow, including magnet search, detection, and return navigation.  
- **PID Motor Control** – provided precise, dynamic motor speed regulation for smooth and accurate movement.  
- **Complementary Filter Integration** – improved magnetometer heading estimation, reducing drift and sensor noise.  
- **Object-Oriented Codebase** – modular classes for sensors, motors, and kinematics improved maintainability and readability.  
- **Result:** Awarded **100%** for the project, assessed by the robot's performance on a video.

🎥 [Watch Demo Video](https://youtu.be/n0UjlL4kPss?si=vmlw3QRdXmjSS-Dd)

## 2. Group Research Project – Autonomous Box Pushing with Arduino Robots

Researched and tested the box-pushing performance of one and two **Pololu 3Pi+** robots under varying conditions.  
Led the **programming** and control system design for the project.

- **PID Motor Control** – Built independent PID controllers for each motor, plus an optional **bumper-sensor feedback** mode for real-time trajectory correction.
- **Noise Filtering** – Added an **exponential moving average filter** to smooth sensor data and improve stability.
- **Sensor Calibration** – Wrote calibration routines to ensure consistent bumper readings across tests.
- **Performance Metrics** – Developed measures such as **Path Deviation Score (PDS)** and **Frequency of Correction** to quantify control accuracy.

- **Independently taught myself Python OpenCV** for computer vision tracking, successfully integrating it into the data analysis pipeline to measure the *Path Deviation Score (PDS)* — a custom metric quantifying straight-line pushing performance.  
- Findings included:  
  - Pushing performance generally increases with mass for both systems.  
  - A single robot outperformed two robots at certain mass ranges (185–230g).  
  - Bumper-sensor feedback improved performance for two-robot systems but had limited impact on single robots.  
- Awarded a **group grade of 72%**, with feedback highlighting:  
  > *"Well done, this was a very interesting report and well presented… your baseline clearly had utility and allowed you to draw out interesting and valuable insights… you made very effective use of cross comparison between your results and systems to make credible inferences… well done."*
