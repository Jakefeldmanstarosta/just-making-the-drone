### **Project Log Entry**

**Date:** April 28, 2026  
**Status:** System Architecture Phase / Prompt Optimization  
**Component:** ESP32 Flight Controller (920kv Brushless System)

---

#### **Summary**
Drafted a specialized system prompt for generating a functional quadcopter flight controller. The original concept was refined to account for **PWM duty cycles (1000–2000µs)** required by Electronic Speed Controllers (ESCs) rather than simple digital state switching.

#### **Optimized Prompt**
> "Act as an embedded systems engineer specializing in drone flight dynamics. I am building a quadcopter using an ESP32, 920kv brushless motors (driven by standard ESCs), and an MPU6050 IMU. 
> 
> **Hardware setup:**
> * MPU6050: SDA -> GPIO 22, SCL -> GPIO 21.
> * ESC PWM signals (50Hz, 1000µs-2000µs): Front Left -> 12, Front Right -> 13, Back Left -> 14, Back Right -> 26.
> 
> Please write Arduino C++ framework code using the `ESP32Servo` library and a standard MPU6050 library that implements a basic 'stabilize' mode. The code must include:
> 1. An initialization routine that calibrates the IMU.
> 2. A distinct arming sequence (e.g., forcing 1000µs to all motors for 5 seconds to initialize ESCs).
> 3. A basic PID loop structure for pitch and roll stabilization.
> 4. A standard Quad-X motor mixing algorithm mapping the PID outputs and a base throttle to the 1000-2000µs ESC limits.
> Provide placeholder variables for PID tuning constants ($K_p, K_i, K_d$)."

#### **Technical Considerations**
* **ESC Communication:** Shifted from "high/low" logic to **Servo Microseconds** for motor control.
* **Physics Loop:** Integrated a **PID (Proportional-Integral-Derivative)** control loop to handle micro-adjustments for stability.
* **Safety:** Identified the critical need for an "arming sequence" to prevent motor spin-up during gyro calibration.

#### **Next Steps**
* Bench test PID response with **propellers removed**.
* Implement Mahony/Madgwick filter for better IMU noise rejection.
* Define RC input protocol (SBUS/ELRS) for manual override.
