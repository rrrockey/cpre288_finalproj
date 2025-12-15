# CPRE 288 Final Project

**Project Title:** Autonomous CyBot Lawn Mower  
**Authors:** John Murphy, Josh Sadler, Ryan Rockey  
**Date:** December 13, 2025

---

## 📌 Project Overview
This project demonstrates the design and implementation of an autonomous embedded robotic system using the Iowa State CyBot platform, which uses a Texas Instruments Tiva TM4C123GH6PM microcontroller. The system integrates multiple hardware systems with low-level C software to autonomously navigate a bounded environment, detect obstacles, and return to its starting position.

The CyBot uses sensor feedback to follow the perimeter of a test field marked with white tape, avoid collisions, and communicate status information to a host computer via a GUI. The project emphasizes concepts such as interrupts, PWM control, ADC sampling, and real-time decision making.

**Objectives:**
- Interface with external hardware components:
  - Drive wheels and read from cliff sensors using the iRobot `open_interface` library
  - Servo motor controlled via PWM for sensor scanning
  - IR distance sensor read using the ADC
  - Ping (sonar) distance sensor using hardware interrupts
- Autonomously navigate the perimeter of a test field bounded by white tape
- Detect and avoid obstacles using onboard sensors
- Return to the original starting position upon completion
- Communicate sensor data and system status to a Python Tkinter GUI
