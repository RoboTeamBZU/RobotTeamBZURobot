# RoboTeam BZU - WRO 2025 Future Engineers

> **Note:** This repository is the newer version of our older repository [https://github.com/RoboTeamBZU/WRO-RoboTeamBZU.git](https://github.com/RoboTeamBZU/WRO-RoboTeamBZU.git).

## Team Information

- **Team Name:** RoboTeam BZU
- **Institution:** Birzeit University
- **Category:** Future Engineers – WRO 2025 Self-Driving Cars
- **Competition:** World Robot Olympiad 2025

## Project Overview

This repository contains the complete engineering documentation, source code, and design files for Team BZU's autonomous vehicle system developed for the WRO 2025 Future Engineers "Self-Driving Car Challenge." Our project demonstrates a sophisticated yet streamlined approach to autonomous navigation, implementing advanced computer vision, sensor fusion, and state-driven control logic to tackle the complex challenges presented in this competition.

### Competition Challenges

The WRO 2025 Future Engineers category presents two distinct challenges that our autonomous vehicle is designed to complete:

#### Open Challenge
The vehicle must autonomously complete three consecutive laps on a track with dynamically varying widths between 600mm and 1000mm. The track boundaries are marked with blue and orange lines, and the vehicle must maintain its course within these boundaries while optimizing speed and stability. The randomization of track width is determined immediately before each competition round, requiring our system to be robust and adaptive to different spatial constraints without any manual tuning.

#### Obstacle Challenge
Building upon the Open Challenge requirements, this advanced scenario introduces additional complexity through traffic sign navigation and precision parking. The vehicle must:

1. **Navigate Three Complete Laps:** Maintain lane discipline while processing real-time environmental data
2. **Interpret Traffic Signs:** Correctly identify and respond to red and green traffic signs (cylindrical pillars) positioned randomly along the track
   - **Red Signs:** Must be passed on the right side
   - **Green Signs:** Must be passed on the left side
3. **Execute Parallel Parking:** After completing three laps, identify a designated parking area marked with magenta boundaries and execute a parallel parking maneuver into a space of variable length

The entire system operates autonomously from start to finish, with all decision-making handled onboard without any external communication or human intervention once the competition run begins.

## System Architecture

Our vehicle implements a carefully designed architecture that balances performance, reliability, and compliance with competition regulations. The system uses a unified processing approach where all computational tasks are handled by a single powerful controller, ensuring tight integration and minimal latency between sensor inputs and actuator responses.

### Hardware Components

#### Processing & Control

**Raspberry Pi 4 (Main Controller)**
The Raspberry Pi 4 serves as the central processing unit and sole computational brain of our autonomous vehicle. This powerful single-board computer handles all aspects of the system including:
- Real-time computer vision processing using OpenCV
- Sensor data fusion from multiple input sources
- Finite state machine execution for behavioral control
- Motor control via PWM signal generation
- Decision-making algorithms for navigation and obstacle avoidance

The Pi runs a custom Python-based software stack that integrates all subsystems into a cohesive autonomous driving platform. Its quad-core ARM Cortex-A72 processor provides sufficient computational power for real-time image processing while maintaining low power consumption and heat generation.

#### Sensors & Perception

**Raspberry Pi Camera Module v2**
Our primary sensor for environmental perception and high-level navigation tasks. The camera provides 8-megapixel resolution images at up to 30 frames per second, which our computer vision algorithms process to perform:

- **Lane Detection:** Identifies blue and orange track boundary lines using color space transformation (RGB to HSV), Gaussian blur for noise reduction, and morphological operations. Contour detection algorithms locate the lane lines and calculate the vehicle's lateral position within the track boundaries. The centroid of detected lane markers serves as the primary reference point for steering corrections.

- **Traffic Sign Recognition:** Employs color-based object detection to identify red and green cylindrical pillars. The algorithm segments objects by their distinctive color profiles in HSV space, applying adaptive thresholding to handle varying lighting conditions. Position estimation calculates the sign's location relative to the vehicle's trajectory, determining whether to pass left or right.

- **Parking Lot Identification:** Scans for magenta-colored parking space boundaries that indicate the designated parking area. When detected, this triggers a state transition in the finite state machine from lap-running mode to parking maneuver mode.

**HC-SR04 Ultrasonic Distance Sensor**
Provides reliable distance measurements to objects and walls using ultrasonic echo timing. This sensor serves multiple critical functions:
- Collision avoidance by detecting proximity to track walls
- Distance measurement during parking maneuvers
- Validation of camera-based distance estimates through sensor fusion
- Emergency stop trigger if obstacles are detected at critically close range

The ultrasonic sensor's data is continuously monitored and integrated with visual data to create a more robust and reliable perception of the vehicle's surroundings.

**TCS3200 RGB Color Sensor**
A high-precision color sensor mounted on the underside of the vehicle, positioned to read the track surface directly beneath the chassis. This sensor provides:
- Fast and accurate detection of track line colors (blue and orange)
- Secondary validation of lane position independent of camera processing
- Improved reliability in challenging lighting conditions where camera performance may degrade
- Direct RGB value output that can be processed with minimal computational overhead

The TCS3200 acts as a complementary sensor to the camera, providing a redundant method for lane detection that increases overall system reliability.

#### Actuation Systems

**DC Geared Motor with L298N H-Bridge Driver**
Propulsion is provided by a single high-torque DC geared motor connected to the rear axle, ensuring both drive wheels are mechanically linked as required by competition regulations. The motor selection balances torque requirements for acceleration and hill climbing with speed requirements for competitive lap times.

The L298N H-Bridge motor driver interfaces between the Raspberry Pi's GPIO pins and the motor, enabling:
- Bidirectional control (forward and reverse)
- Variable speed control via PWM (Pulse Width Modulation)
- High current handling capacity for motor startup and sustained operation
- Protection circuits to prevent damage from back-EMF and short circuits

**Steering Servo Motor**
A precision hobby-grade servo motor controls the front wheel steering angle. The servo receives PWM signals from the Raspberry Pi that correspond to desired steering angles, allowing smooth and accurate directional control. The steering system is calibrated to provide:
- Maximum left turn angle for sharp corners
- Maximum right turn angle for opposite direction turns
- Precise center position for straight-line tracking
- Proportional control for gentle curves and lane corrections

#### Power System

A custom-designed power distribution system ensures stable operation of all electronic components. The system uses:
- High-capacity rechargeable battery pack (11.1V LiPo or similar)
- Dual-channel voltage regulator providing stable 5V for Raspberry Pi and sensors
- Separate power rail for motor driver to isolate high-current loads
- Power monitoring circuits to prevent over-discharge and system brownouts

## Software Architecture

The software is structured as a modular, event-driven system built in Python 3.7+, taking advantage of the extensive libraries available for the Raspberry Pi platform. The architecture emphasizes separation of concerns, making the codebase maintainable and allowing individual components to be tested and optimized independently.

### Core Modules

#### Main Controller (`main_controller.py`)

The heart of the autonomous system, this script orchestrates all other modules and implements the primary control logic:

**Finite State Machine (FSM)**
The vehicle's behavior is managed through a finite state machine with the following primary states:
- `INITIALIZATION`: System startup, sensor calibration check, and readiness verification
- `LANE_FOLLOWING`: Standard lap running mode, maintaining position within track boundaries
- `TRAFFIC_SIGN_DETECTION`: Active scanning and identification of red/green pillars
- `TRAFFIC_SIGN_RESPONSE`: Execution of passing maneuver (left or right) based on sign color
- `PARKING_SEARCH`: Scanning for magenta parking lot boundaries after lap completion
- `PARKING_APPROACH`: Positioning vehicle for parallel parking entry
- `PARALLEL_PARKING`: Execution of parking maneuver
- `MISSION_COMPLETE`: Final state after successful parking

State transitions are triggered by specific sensor inputs and internal conditions, creating a reactive system that responds appropriately to environmental changes.

**Sensor Fusion Logic**
Combines data from multiple sensors to create a more reliable understanding of the environment:
- Camera and TCS3200 data are weighted and combined for lane position estimation
- Ultrasonic distance measurements validate camera-based object detection
- Discrepancies between sensors trigger confidence evaluation and may result in conservative driving behavior

**Lap Counter**
Tracks completed laps by monitoring passage through the start/finish zone, identified by unique visual features or track markings captured by the camera system.

#### Computer Vision Module (`cv_processing.py`)

Handles all image processing and computer vision tasks:

**Lane Detection Pipeline**
1. Image acquisition from Pi Camera
2. Gaussian blur application to reduce noise
3. Color space conversion from RGB to HSV
4. Color thresholding to isolate blue and orange lane markers
5. Morphological operations (erosion and dilation) to clean up detected regions
6. Contour detection to identify lane line boundaries
7. Centroid calculation to determine lateral offset from track center
8. Steering error computation based on deviation from target position

**Object Detection and Classification**
- Color-based segmentation to isolate traffic signs and parking boundaries
- Contour analysis to filter out noise and identify significant objects
- Size and aspect ratio filtering to distinguish pillars from other colored objects
- Position calculation relative to vehicle center to determine passing strategy
- Confidence scoring based on multiple frame detections to avoid false positives

**Parking Space Analysis**
- Detection of magenta boundary markers indicating parking area
- Measurement of parking space length between markers
- Calculation of entry trajectory and required steering angles
- Monitoring of vehicle position during parking execution

#### Sensor Interface Module (`sensor_drivers.py`)

Provides abstraction layer for all hardware sensors:

**TCS3200 Driver**
- Frequency-to-RGB conversion functions
- Calibration data storage for white balance and color mapping
- Real-time color identification with noise filtering

**HC-SR04 Driver**
- GPIO trigger pulse generation
- Echo time measurement with timeout handling
- Distance calculation and moving average filtering

**Motor Control Interface**
- PWM signal generation for L298N H-Bridge control
- Speed ramping functions to prevent wheel slip
- Direction control with safety interlocks

**Servo Control Interface**
- Angle-to-PWM conversion based on calibration data
- Smooth steering transitions to avoid mechanical stress
- Limit checking to prevent exceeding maximum turn angles

### Configuration and Calibration

The `config.py` file contains all tunable parameters including:
- Motor speed settings for different track sections
- Steering gain coefficients for PID-style control
- Color threshold values for vision processing
- State transition conditions and timeouts
- Sensor fusion weighting factors

## Installation and Setup

### Prerequisites

- Raspberry Pi 4 (2GB RAM minimum, 4GB recommended)
- MicroSD card (16GB minimum, Class 10 or better)
- Raspberry Pi Camera Module v2
- TCS3200 RGB Color Sensor
- HC-SR04 Ultrasonic Distance Sensor
- L298N H-Bridge Motor Driver
- DC Geared Motor (6V-12V recommended)
- Steering Servo Motor (standard size, metal gear recommended)
- Power supply (11.1V LiPo battery or equivalent)
- Voltage regulators (5V output for Pi and sensors)

### Operating System Setup

1. **Flash Raspberry Pi OS:**
   - Download Raspberry Pi OS (Bullseye or newer)
   - Use Raspberry Pi Imager to write OS to microSD card
   - Boot the Pi and complete initial setup wizard

2. **System Update:**
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

3. **Enable Required Interfaces:**
   ```bash
   sudo raspi-config
   ```
   - Enable Camera interface
   - Enable I2C (if using I2C sensors)
   - Enable SPI (if required)
   - Set GPU memory to 128MB for camera processing

### Software Installation

1. **Clone Repository:**
   ```bash
   git clone https://github.com/RoboTeamBZU/RobotTeamBZURobot.git
   cd RobotTeamBZURobot
   ```

2. **Install Python Dependencies:**
   ```bash
   pip3 install -r requirements.txt
   ```
   
   Key dependencies include:
   - `opencv-python`: Computer vision library
   - `numpy`: Numerical computing
   - `RPi.GPIO`: GPIO pin control
   - `picamera2`: Pi Camera interface
   - `gpiozero`: High-level GPIO device interface

3. **Hardware Connections:**
   Connect all components according to the wiring diagram in `/docs/wiring_diagram.png`:
   - Camera: CSI camera connector
   - TCS3200: GPIO pins for S0, S1, S2, S3, OUT
   - HC-SR04: GPIO for Trigger and Echo
   - L298N: GPIO for IN1, IN2, ENA (PWM)
   - Servo: GPIO pin with PWM capability
   - Power: Ensure proper voltage regulation and grounding

### Calibration Procedures

Calibration must be performed on a track that matches official competition specifications. Navigate to the `/calibration` directory and run each script:

#### Color Sensor Calibration
```bash
python3 calibrate_color_sensor.py
```
This script guides you through:
- White balance calibration on the track surface
- Blue line color mapping
- Orange line color mapping
- Threshold optimization for reliable detection

#### Camera Color Calibration
```bash
python3 calibrate_camera_colors.py
```
Interactive tool for setting HSV color ranges:
- Blue track line detection range
- Orange track line detection range
- Red traffic sign detection range
- Green traffic sign detection range
- Magenta parking lot detection range

Test each color under actual competition lighting conditions and save the optimal values.

#### Steering Calibration
```bash
python3 calibrate_steering.py
```
Determines PWM values for:
- Full left turn (maximum safe angle)
- Center position (straight ahead)
- Full right turn (maximum safe angle)

The script will prompt you to adjust values until the wheels achieve proper positions.

#### Behavior Tuning

Edit `config.py` to optimize:
- `MAX_SPEED`: Maximum motor speed for straight sections
- `TURN_SPEED`: Reduced speed for corners
- `STEERING_GAIN`: Proportional gain for steering corrections
- `LAP_COUNT_TARGET`: Number of laps before parking (typically 3)

Test the vehicle on the practice track and iterate on these values to achieve optimal performance.

## Competition Deployment

### Pre-Competition Checklist

- [ ] All sensors calibrated and verified
- [ ] Battery fully charged
- [ ] Software configured to auto-start on boot
- [ ] Emergency stop button functional
- [ ] Wheels and steering mechanism moving freely
- [ ] Camera lens clean and properly focused
- [ ] All electrical connections secure

### Auto-Start Configuration

Configure the system to automatically run the main controller on boot:

**Using systemd (recommended):**
Create `/etc/systemd/system/roboteam.service`:
```ini
[Unit]
Description=RoboTeam BZU Autonomous Vehicle
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/RobotTeamBZURobot
ExecStart=/usr/bin/python3 main_controller.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable roboteam.service
```

### Competition Procedure

1. **Initial Setup:**
   - Power off the vehicle
   - Place in randomly selected starting zone as directed by judges
   - Verify vehicle is entirely within starting area

2. **System Boot:**
   - Power on vehicle using main power switch
   - Wait for Raspberry Pi to boot (LED indicators will show ready state)
   - System enters waiting state, ready for start signal

3. **Run Initiation:**
   - Upon judge's "Go" signal, press the designated start button
   - Vehicle immediately begins autonomous operation
   - No further human interaction permitted during run

4. **Mission Execution:**
   - Vehicle autonomously navigates the challenge
   - All decisions made onboard based on real-time sensor data
   - Completes laps and parking maneuver according to challenge requirements

## Technical Approach and Innovation

Our solution emphasizes reliability and robustness through several key design decisions:

### Sensor Fusion Strategy
Rather than relying on a single sensor, we combine multiple inputs to create a more confident perception of the environment. This redundancy allows the system to maintain performance even when individual sensors experience momentary issues.

### State-Driven Architecture
The finite state machine approach provides clear separation of behaviors and makes the system's operation transparent and debuggable. Each state has well-defined entry conditions, execution logic, and exit conditions.

### Computational Efficiency
All processing occurs on a single Raspberry Pi 4, demonstrating that sophisticated autonomous behavior doesn't require distributed computing or external processing. Optimized algorithms ensure real-time performance.

### Adaptive Control
The steering and speed control algorithms adjust their behavior based on the vehicle's current state and environmental conditions, providing smooth operation across varying track configurations.

## Project Philosophy

This project began as a theoretical concept and has been transformed into a practical working system through careful engineering and iterative development. We have strived to implement our design with as few compromises as possible, while remaining within the competition regulations and maintaining the accessibility of the technology we've used.

Our goal extends beyond simply competing—we aim to demonstrate that effective autonomous systems can be built with educational-grade components and open-source software, making this technology approachable for students and teams worldwide.

We sincerely wish everyone the best of success in the competition and hope that our documentation and approach can serve as a helpful reference for other teams pursuing similar challenges.
`

## License

This project is open-source and available for educational purposes. Please refer to the LICENSE file for specific terms and conditions.

## Contact

**Team BZU**  
Birzeit University  
Email: [Team Contact Email]  
GitHub: https://github.com/RoboTeamBZU

## Acknowledgments

We would like to thank Birzeit University for their support, our mentors for their guidance, and the WRO organization for creating this challenging and educational competition platform.
