(This repository is the newer verson of our older repository https://github.com/RoboTeamBZU/WRO-RoboTeamBZU.git.)

# WRO 2025 Future Engineers: Autonomous Self-Driving Vehicle
**Team Name:** RoboTeam BZU  
**Institution:** Birzeit University  
**Category:** Future Engineers – WRO 2025 Self-Driving Cars

## 🔹 Project Overview

This repository contains the complete engineering documentation and source code for Team BZU's entry into the WRO 2025 Future Engineers "A Self-Driving Car Challenge." The project centers on developing a fully autonomous vehicle capable of meeting the rigorous demands of both competition challenges. For the **Open Challenge**, the vehicle must complete three laps on a track with randomly varying widths between 600mm and 1000mm. For the more complex **Obstacle Challenge**, the vehicle must navigate three laps while correctly interpreting and responding to red and green traffic signs (passing red on the right and green on the left) before successfully identifying and executing a parallel parking maneuver into a space of variable length. The entire system is designed to be robust, relying on a multi-sensor data fusion approach and a state-driven control logic to handle the randomization elements introduced just before each competition round.

## 🔹 Vehicle Architecture & Electromechanical Components

Our vehicle implements a streamlined and effective architecture that maximizes performance using a curated set of components, all controlled by a central processor to ensure seamless integration and rapid decision-making.

### Core Computational Module:
*   **Main Controller (Raspberry Pi 4):** Serves as the singular brain of the vehicle. This powerful Single-Board Computer (SBC) is responsible for all processing tasks. It executes the main control script, which is built around a finite state machine that manages the vehicle's behavior (e.g., lane following, traffic sign response, parking search, and parking execution). The Pi directly processes data from all sensors and computes the necessary commands for the drive and steering actuators.

### Perception & Sensing Modules:
*   **Raspberry Pi Camera Module:** This is the primary sensor for high-level navigation and object detection. Using the OpenCV library in Python, we have developed robust computer vision algorithms. These algorithms perform several critical functions:
    *   **Lane Detection:** Identifies the blue and orange track lines through color filtering and contour detection. This allows the vehicle to calculate its lateral position within the lane and maintain its course.
    *   **Traffic Sign Recognition:** Detects the colored pillars (red and green) by isolating their specific RGB/HSV color profiles within the camera's field of view. The algorithm determines the color of the pillar and its position relative to the vehicle's path, which dictates the correct passing side as per the competition rules.
    *   **Parking Lot Identification:** Scans the track to locate the magenta-colored parking lot boundaries, triggering the transition from the lap-running state to the parking maneuver state in the Obstacle Challenge.
*   **Ultrasonic Sensor (HC-SR04):** Provides critical distance-to-object data. It is primarily used as a safety sensor to prevent collisions with the track walls and to detect proximity to the parking lot boundaries during the final parking sequence. Its data is fused with camera data to create a more reliable perception of the environment.
*   **TCS3200 Color Sensor:** This dedicated color sensor provides a highly reliable and fast method for detecting the track lines directly beneath the vehicle. It serves as a secondary validation system for the camera, especially in situations where lighting conditions might affect the camera's performance. It delivers precise RGB data that is used for stable line-following behavior.

### Actuation & Mobility Modules:
*   **Drive System (DC Geared Motor with L298N H-Bridge):** A single DC motor provides propulsion. The motor is connected to the drive axle, ensuring compliance with the rule requiring physically connected drive wheels. The L298N H-Bridge motor driver, controlled directly by the Raspberry Pi's GPIO pins, allows the Pi to dictate the direction and speed of the vehicle via Pulse-Width Modulation (PWM).
*   **Steering System (Steering Servo Motor):** A standard servo motor controls the angle of the front wheels. The Raspberry Pi calculates the required steering angle based on the error signal from the lane detection algorithms (from either the camera or the TCS3200) and sends a corresponding PWM signal to the servo for precise directional control.
*   **Power System:** A single high-capacity battery pack powers the entire system. A dual-channel voltage regulator provides stable 5V power to the Raspberry Pi and sensors, and the necessary voltage to the motor driver for the drive and steering motors, preventing system resets during high-current draws.

## 🔹 Software Architecture & Module Integration

The software is structured into modular components that run concurrently on the Raspberry Pi, creating a cohesive and responsive autonomous system.

### 1. `main_controller.py` (Primary Control Script)
This script is the core of the autonomous system, integrating all modules.
*   **Finite State Machine (FSM):** Manages the vehicle's operational mode. Key states include `LANE_FOLLOWING`, `TRAFFIC_SIGN_DETECTION`, `PARKING_SEARCH`, and `PARALLEL_PARKING`. Transitions between states are triggered by sensor inputs (e.g., camera detects a parking lot, lap counter reaches 3).
*   **Sensor Fusion Logic:** Combines inputs from the camera, ultrasonic sensor, and TCS3200 to form a more confident understanding of the vehicle's environment. For example, the camera identifies a distant traffic sign, while the TCS3200 ensures the vehicle stays centered on the lane as it approaches.
*   **Lap Counter:** Tracks the number of completed laps by monitoring the vehicle's passage through the starting section using a combination of visual cues from the camera and the unique track layout.

### 2. `vision_processing.py` (Computer Vision Module)
This module handles all image processing tasks from the Pi Camera.
*   **Lane Detection Algorithm:** Applies Gaussian blur, converts the image to HSV color space, and uses color masks to isolate the blue and orange lines. It then calculates the centroid of the detected lines to determine the vehicle's lateral offset, which is fed into the steering control logic.
*   **Object Detection & Classification:** Uses contour detection and color thresholding to identify traffic signs and parking lot boundaries. It returns the object's color and coordinates to the main controller, which determines the appropriate action (e.g., "steer left to pass green pillar").
*   **Parking Spot Analysis:** Once the parking lot is identified, this module analyzes the space between the magenta boundaries to calculate the entry path for the parallel parking maneuver.

### 3. `sensor_drivers.py` (Low-Level Sensor Interface)
This module provides a clean interface for reading from all non-camera sensors.
*   **TCS3200 Driver:** Contains functions to read the raw frequency from the color sensor and convert it into normalized RGB values. It includes calibration routines to set baselines for the white track and black lines under specific lighting conditions.
*   **Ultrasonic Sensor Driver:** Manages the GPIO pulses required to trigger the HC-SR04 and measure the echo return time, converting it into a distance measurement in centimeters.
*   **Motor & Servo Control:** Provides high-level functions (e.g., `set_speed(speed)`, `set_steering_angle(angle)`) that abstract the low-level PWM calculations needed to control the L298N H-Bridge and the steering servo via the Pi's GPIO pins.

## 🔹 Build, Compile, and Upload Process

### Prerequisites
*   Hardware: Raspberry Pi 4, Pi Camera, TCS3200 Color Sensor, HC-SR04 Ultrasonic Sensor, DC Motor, L298N H-Bridge, Servo Motor.
*   Software: Raspberry Pi OS (Bullseye) with Python 3.7+.

### Step 1: Operating System and Dependency Setup
1.  Flash the latest Raspberry Pi OS (Lite or Desktop) onto a microSD card.
2.  Boot the Pi, connect to the internet, and update the system:
    ```bash
    sudo apt update && sudo apt upgrade -y
    ```
3.  Enable the camera interface and GPIO control using `sudo raspi-config`.

### Step 2: Software Installation and Configuration
1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/RoboTeamBZU/WRO-2025-Future-Engineers.git
    cd WRO-2025-Future-Engineers
    ```
2.  **Install Python Dependencies:**
    ```bash
    pip3 install -r requirements.txt
    ```
    The `requirements.txt` file includes: `opencv-python`, `numpy`, `RPi.GPIO`, and `picamera2`.
3.  **Hardware Wiring:** Connect all sensors and actuators to the Raspberry Pi's GPIO pins according to the detailed wiring diagram provided in the `/docs` folder.

### Step 3: Calibration and Testing
This is a critical phase that must be performed on a practice field that matches the competition specifications.
1.  **Sensor Calibration:** Run the calibration scripts in the `/calibration` directory.
    *   `calibrate_color_sensor.py`: Determines the reference values for the white track surface and the colored lines.
    *   `calibrate_camera_colors.py`: Sets the HSV color ranges for the blue/orange lines and red/green/magenta objects specific to the competition lighting and mat.
    *   `calibrate_steering.py`: Finds the PWM values for the servo that correspond to "straight," "full left," and "full right" turns.
2.  **Behavior Tuning:** Adjust parameters in the `config.py` file, such as motor speeds for corners and straights, and the steering response gain, to optimize lap times and reliability.

### Step 4: Deployment and Competition Execution
1.  **Final Deployment:** Once calibrated, the entire codebase resides on the Pi's SD card. The vehicle is now competition-ready.
2.  **Starting Procedure:** At the competition table:
    *   The judge will confirm the vehicle is switched off.
    *   The team places the vehicle in the randomly selected starting zone.
    *   A single switch is used to power on the Raspberry Pi.
    *   The system boots and automatically runs the main controller script (configured to start on boot via `systemd` or `crontab`), entering a waiting state.
3.  **Run Initiation:** Upon the judge's "Go" signal, a team member presses the single, designated start button (a physical push button connected to the Pi's GPIO), which begins the autonomous mission. The vehicle will then execute its programmed logic for the specific challenge, navigating the random track configuration entirely on its own.

This repository embodies a practical and effective approach to the WRO Future Engineers challenge, demonstrating a clear understanding of autonomous systems integration through the strategic use of a simplified, yet powerful, sensor and actuator suite.

Finally this idea began as a theory, and we are striving to put it into practice with as few mistakes as possible. We sincerely wish everyone the best of success in the competition.
