
***

# **Project Log: Flight Controller Prompt Optimization**

**Date:** April 28, 2026  
**Author:** Lead Firmware Developer  
**Component:** ESP32 Custom Flight Controller  
**Status:** Architecture & Design Phase

## **Overview**
Refined the LLM generation prompt for the baseline flight controller code. The initial concept lacked necessary hardware specifics, which would result in unsafe or non-functional control loops. We have updated the prompt to explicitly define our propulsion system, battery voltage, and ESC protocols to ensure the generated code accounts for the high-thrust 4S architecture and standard PWM limitations.

## **Hardware Profile Established**
* **MCU:** ESP32
* **Power:** 4S LiPo (14.8V nominal, 16.8V max)
* **Propulsion:** 920kv brushless motors + 1045 (10-inch) propellers
* **ESCs:** 30A SimonK Firmware (Standard PWM, 1000µs - 2000µs, Max 400Hz)
* **Sensors:** MPU6050 IMU (I2C)

## **Optimized LLM Generation Prompt**
*This prompt is version-locked for generating the baseline `main.cpp` logic.*

```text
Act as an expert embedded systems engineer specializing in drone flight dynamics. Please write a complete Arduino C++ flight controller sketch for a custom ESP32 quadcopter.

Hardware Profile:
* MCU: ESP32
* Power: 4S LiPo (14.8V nominal, 16.8V max).
* Propulsion: 920kv brushless motors with 1045 (10-inch) propellers.
* ESCs: 30A SimonK Firmware. 
    * Protocol: Standard PWM (1000µs - 2000µs). 
    * Pins: Front Left -> 12, Front Right -> 13, Back Left -> 14, Back Right -> 26.
* IMU: MPU6050 via I2C (SDA -> GPIO 22, SCL -> GPIO 21).

Software & Control Requirements:
1. Timing & Loop: Run the main physics loop at a strict 250Hz (4000µs loop time) using micros() for precision, avoiding blocking delay() functions in the main loop.
2. Motor Control: Use the ESP32Servo library. Set the ESC update rate to 400Hz if supported by the library, otherwise default to 50Hz.
3. IMU Filtering: Read raw MPU6050 data and apply a Complementary Filter (Angle = 0.98 * (Angle + Gyro * dt) + 0.02 * Accel) to calculate pitch and roll. This is critical to filter out the heavy motor vibrations from the 10-inch props.
4. PID Controller: Implement a PID loop for Pitch and Roll. Because this is a high-thrust 4S setup, keep initial Kp values conservative (e.g., 1.0). Crucially, include Anti-Integral Windup by capping the integral term to prevent the drone from flipping on spool-up.
5. Motor Mixing: Implement a standard Quad-X mixing algorithm. Set the base_throttle to 1100µs to account for the aggressive 4S power curve.

Safety Requirements (CRITICAL):
* Arming Sequence: On boot, force all ESCs to 1000µs for 5 seconds while the MPU6050 calibrates gyro offsets. Print arming status to the Serial monitor.
* Tilt Kill-Switch: If the calculated pitch or roll angle ever exceeds 45 degrees, immediately force all motor outputs to 1000µs to prevent runaway crashes.
* Voltage Monitoring: Include a placeholder function to read battery voltage from an analog pin (assume a voltage divider is used) and trigger a serial warning if voltage drops below 14.0V.
```

## **Next Steps / Action Items**
- [ ] Feed the optimized prompt into the LLM and generate the baseline `main.cpp`.
- [ ] Flash the generated code to the ESP32.
- [ ] **SAFETY CHECK:** Physically remove all 4 propellers from the motors.
- [ ] Connect the ESP32 to the Serial Monitor (115200 baud) and monitor the boot sequence.
- [ ] Perform a physical bench test: tilt the drone frame by hand and listen for correct ESC/motor compensation (motors should spool up on the side being dipped).
