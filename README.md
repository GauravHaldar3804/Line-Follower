# Line Follower Robot with PID Control

This project is a line follower robot implemented using **QTR sensors**, **SparkFun TB6612 motor driver**, and **PID control** for accurate line tracking. The project was developed and executed using **PlatformIO** for streamlined development and deployment.

## Features
- **PID Control**: Ensures smooth and precise line tracking.
- **Line Calibration**: Calibrates sensor values before operation to adapt to different environments.
- **Error Recovery**: Includes logic to recover when the robot moves off the line.
- **Real-Time Feedback**: Outputs sensor values and control parameters via the serial monitor for debugging and monitoring.

## Components
- Arduino-compatible microcontroller
- QTR-8A Reflectance Sensor Array
- SparkFun TB6612 Motor Driver
- Two DC Motors
- Power Supply (Battery)
- Miscellaneous: Chassis, Wheels, Wires, etc.

## How It Works
1. **Sensor Calibration**: 
   - The robot calibrates its sensors at startup by rotating in place and recording the minimum and maximum reflectance values.
   - These values are used to normalize sensor readings during operation.

2. **Line Detection**:
   - The QTR sensor array reads the reflectance values to determine the position of the line.

3. **PID Algorithm**:
   - Calculates the error as the difference between the desired position (setpoint) and the actual position of the line.
   - Adjusts motor speeds dynamically based on the PID formula:
     \[
     \text{adjustedSpeed} = K_p \cdot \text{error} + K_d \cdot (\text{error} - \text{previousError}) + K_i \cdot \text{sumOfErrors}
     \]

4. **Off-Line Recovery**:
   - If the robot loses the line, it makes corrective turns based on the last known position of the line.

## Software Development
The project is written in **C++** and developed in **PlatformIO**, a powerful environment for embedded system development.

### Libraries Used
- **QTRSensors**: For handling the QTR sensor array.
- **SparkFun_TB6612**: For controlling the motor driver.

### PID Configuration
The PID constants can be tuned in the `main.cpp` file for optimal performance:
- `Kp`: Proportional gain (default: `0.087`)
- `Ki`: Integral gain (default: `0`)
- `Kd`: Derivative gain (default: `0.25`)

### Sensor and Motor Pins
- QTR Sensor Pins: `{2, 3, 4, 5, 6, 7, 8, 11}`
- Emitter Pin: `12`
- Motor Driver:
  - Motor A: `AIN1`, `AIN2`, `PWMA` → `{A1, A0, 9}`
  - Motor B: `BIN1`, `BIN2`, `PWMB` → `{A4, A5, 10}`
  - Standby Pin: `A3`

## Installation and Setup
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/GauravHaldar3804/Line-Follower.git
   Install PlatformIO:


Demo
[Download and Watch the Demo](./path-to-your-video.mp4)
