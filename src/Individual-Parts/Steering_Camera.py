from flask import Flask, Response, render_template_string, request, jsonify
from picamera2 import Picamera2
import cv2
import time
import json
import threading
import numpy as np
import math
import board
import adafruit_mpu6050
import pigpio
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

app = Flask(__name__)

# -----------------------------
# Hardware Pin Configuration
# -----------------------------
SERVO_PIN = 18
IN1, IN2, ENA = 24, 23, 13

# Servo limits
SERVO_MAX_RIGHT = 40
SERVO_MAX_LEFT = -40

# Motor settings
DEFAULT_MOTOR_SPEED = 180

# PID Controller parameters
KP = 0.8
KI = 0.01
KD = 0.3

# Direction settings
SERVO_DIRECTION = 1
GYRO_DIRECTION = -1

# -----------------------------
# Camera Setup
# -----------------------------
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()
time.sleep(2)

# -----------------------------
# Steering State Controller (Vision-based target calculation)
# -----------------------------
class VisionSteeringController:
    def __init__(self):
        self.target_angle = 0.0  # Current target heading (accumulated)
        self.max_rate = 5.0      # Maximum degrees per frame to change
        self.deadzone = 50       # Ignore small differences (noise reduction)
        self.k_integral = 0.01   # How fast to accumulate angle
        self.lock = threading.Lock()
        
    def update(self, left_pixels, right_pixels):
        """
        Update target angle based on wall pixel difference
        More left pixels = turn right (increase angle)
        More right pixels = turn left (decrease angle)
        """
        with self.lock:
            difference = left_pixels - right_pixels
            
            # Apply deadzone to ignore small differences
            if abs(difference) < self.deadzone:
                difference = 0
            
            # Calculate rate of change (angular velocity)
            rate = difference * self.k_integral
            
            # Clamp rate to prevent sudden jumps
            rate = max(-self.max_rate, min(self.max_rate, rate))
            
            # Accumulate the angle (integral control)
            self.target_angle += rate
            
            return self.target_angle, rate, difference
    
    def get_target_angle(self):
        with self.lock:
            return self.target_angle
    
    def reset(self):
        """Reset to initial heading"""
        with self.lock:
            self.target_angle = 0.0
    
    def set_angle(self, angle):
        """Manually set target angle"""
        with self.lock:
            self.target_angle = angle
    
    def set_k_integral(self, value):
        with self.lock:
            self.k_integral = value
    
    def set_max_rate(self, value):
        with self.lock:
            self.max_rate = value
    
    def set_deadzone(self, value):
        with self.lock:
            self.deadzone = value

# -----------------------------
# Gyroscope Steering Controller (Hardware control)
# -----------------------------
class GyroSteeringController:
    def __init__(self):
        # Initialize pigpio for motors
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Run 'sudo pigpiod' first")
        
        for p in (ENA, IN1, IN2):
            self.pi.set_mode(p, pigpio.OUTPUT)
        
        self.pi.set_PWM_frequency(ENA, 2000)
        
        # Initialize servo
        Device.pin_factory = PiGPIOFactory()
        self.servo = AngularServo(
            SERVO_PIN,
            min_angle=SERVO_MAX_LEFT,
            max_angle=SERVO_MAX_RIGHT,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )
        
        # Initialize MPU6050
        i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        
        # State variables
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.last_error = 0.0
        self.is_moving = False
        self.current_speed = DEFAULT_MOTOR_SPEED
        self.current_servo_angle = 0.0
        self.running = False
        self.lock = threading.Lock()
        
        # Raw gyro data
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        # Calibrate gyro
        self.calibrate_gyro()
        
    def calibrate_gyro(self, samples=100):
        print("Calibrating gyroscope...")
        gyro_z_offset = 0.0
        for i in range(samples):
            gyro_z_offset += self.mpu.gyro[2]
            time.sleep(0.01)
        self.gyro_z_offset = gyro_z_offset / samples
        print(f"Gyro calibrated. Offset: {self.gyro_z_offset:.4f}")
    
    def run_motor(self, speed):
        forward = speed >= 0
        self.pi.write(IN1, 1 if forward else 0)
        self.pi.write(IN2, 0 if forward else 1)
        speed_value = min(255, abs(int(speed)))
        self.pi.set_PWM_dutycycle(ENA, speed_value)
        self.is_moving = True
        self.current_speed = speed_value
        
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.write(IN1, 0)
        self.pi.write(IN2, 0)
        self.is_moving = False
        self.current_speed = 0
        
    def update_heading(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Read raw gyro data
        gyro_data = self.mpu.gyro
        self.gyro_x = gyro_data[0]
        self.gyro_y = gyro_data[1]
        self.gyro_z = gyro_data[2]
        
        gyro_z_corrected = (self.gyro_z - self.gyro_z_offset) * GYRO_DIRECTION
        heading_change = math.degrees(gyro_z_corrected * dt)
        
        with self.lock:
            self.current_heading += heading_change
            self.current_heading = (self.current_heading + 180) % 360 - 180
        
    def calculate_steering_angle(self, target_heading):
        with self.lock:
            error = target_heading - self.current_heading
        
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        
        derivative = error - self.last_error
        self.last_error = error
        
        steering = (KP * error + KI * self.integral + KD * derivative) * SERVO_DIRECTION
        steering = max(min(steering, SERVO_MAX_RIGHT), SERVO_MAX_LEFT)
        
        return steering, error
    
    def get_current_heading(self):
        with self.lock:
            return self.current_heading
    
    def get_gyro_data(self):
        """Return raw gyro rotation rates"""
        return {
            'x': self.gyro_x,
            'y': self.gyro_y, 
            'z': self.gyro_z,
            'heading': self.get_current_heading()
        }
    
    def set_target_heading(self, heading):
        with self.lock:
            self.target_heading = heading
            self.integral = 0.0
            self.last_error = 0.0
        
    def start_continuous_steering(self, speed=DEFAULT_MOTOR_SPEED):
        """Start continuous steering mode"""
        self.running = True
        self.run_motor(speed)
        
    def stop_continuous_steering(self):
        """Stop continuous steering mode"""
        self.running = False
        self.stop_motor()
        self.servo.angle = 0
        self.current_servo_angle = 0
        
    def steering_loop(self, vision_controller):
        """Main steering loop - runs continuously"""
        print("Starting steering loop...")
        self.last_time = time.time()
        
        while self.running:
            # Update current heading from gyro
            self.update_heading()
            
            # Get target heading from vision controller
            target = vision_controller.get_target_angle()
            self.set_target_heading(target)
            
            # Calculate and apply steering
            steering_angle, error = self.calculate_steering_angle(target)
            self.servo.angle = steering_angle
            self.current_servo_angle = steering_angle
            
            time.sleep(0.02)  # 50Hz update rate
            
    def cleanup(self):
        self.stop_motor()
        self.servo.angle = 0
        self.servo.close()
        self.pi.stop()

# Global controllers
vision_steering = VisionSteeringController()
gyro_steering = GyroSteeringController()
steering_thread = None

# -----------------------------
# ROI Configuration
# -----------------------------
def load_roi_config(path="roi_config.json"):
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
        print("✓ Loaded ROI config")
        return cfg["zones"]
    except:
        print("⚠ Using default ROI config")
        return {
            "Corner_Top": 20,
            "Corner_Bottom": 40,
            "ignore_Top": 20,
            "Corner_LM": 0,
            "Corner_RM": 100,
            "Wall_Top": 30,
            "Wall_Bottom": 70,
            "Right_Wall_RM": 100,
            "Right_Wall_LM": 70,
            "Left_Wall_RM": 30,
            "Left_Wall_LM": 0,
        }

# Color detection for walls
COLOR_LOW  = (0, 0, 0)      # lower HSV bound (black walls)
COLOR_HIGH = (180, 255, 60)  # upper HSV bound

# -----------------------------
# Video Stream Generator
# -----------------------------
def gen_frames():
    zones = load_roi_config()

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, w, _ = frame.shape

        # ---- ROI pixel boundaries ----
        Ct = int(h * zones["Corner_Top"] / 100)
        Cb = int(h * zones["Corner_Bottom"] / 100)
        Wt = int(h * zones["Wall_Top"] / 100)
        Wb = int(h * zones["Wall_Bottom"] / 100)

        Clm = int(w * zones["Corner_LM"] / 100)
        Crm = int(w * zones["Corner_RM"] / 100)
        Rlm = int(w * zones["Right_Wall_LM"] / 100)
        Rrm = int(w * zones["Right_Wall_RM"] / 100)
        Llm = int(w * zones["Left_Wall_LM"] / 100)
        Lrm = int(w * zones["Left_Wall_RM"] / 100)

        # ------------------- Pixel Counting ---------------------
        mask = cv2.inRange(hsv, COLOR_LOW, COLOR_HIGH)

        # Extract ROIs
        corner_roi = mask[Ct:Cb, Clm:Crm]
        left_roi   = mask[Wt:Wb, Llm:Lrm]
        right_roi  = mask[Wt:Wb, Rlm:Rrm]

        # Count pixels
        corner_pixels = cv2.countNonZero(corner_roi)
        left_pixels   = cv2.countNonZero(left_roi)
        right_pixels  = cv2.countNonZero(right_roi)

        # ------------------- UPDATE TARGET HEADING -------------------
        target_angle, rate, difference = vision_steering.update(left_pixels, right_pixels)
        current_heading = gyro_steering.get_current_heading()
        gyro_data = gyro_steering.get_gyro_data()

        # ---------------- DRAW ROI boxes ------------------------
        cv2.rectangle(frame, (Clm, Ct), (Crm, Cb), (0, 255, 0), 2)      # Corner
        cv2.rectangle(frame, (Llm, Wt), (Lrm, Wb), (255, 0, 0), 2)      # Left
        cv2.rectangle(frame, (Rlm, Wt), (Rrm, Wb), (0, 0, 255), 2)      # Right

        # ---- Draw pixel count text on each ROI ----
        cv2.putText(frame, f"C:{corner_pixels}",
                    (Clm, Ct - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

        cv2.putText(frame, f"L:{left_pixels}",
                    (Llm, Wt - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255,0,0), 2)

        cv2.putText(frame, f"R:{right_pixels}",
                    (Rlm, Wt - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,0,255), 2)

        # ------------------- Display Steering Info -------------------
        info_y = 30
        line_height = 25
        
        # Left column - Vision data
        cv2.putText(frame, "VISION TARGET", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Target: {target_angle:.1f}°",
                    (10, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Rate: {rate:.2f}°/f",
                    (10, info_y + line_height * 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Diff: {difference}px",
                    (10, info_y + line_height * 3), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        
        # Right column - Gyro data
        right_x = w - 250
        cv2.putText(frame, "GYRO ACTUAL", (right_x, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Heading: {current_heading:.1f}°",
                    (right_x, info_y + line_height), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"Gyro X: {gyro_data['x']:.3f}",
                    (right_x, info_y + line_height * 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Gyro Y: {gyro_data['y']:.3f}",
                    (right_x, info_y + line_height * 3), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Gyro Z: {gyro_data['z']:.3f}",
                    (right_x, info_y + line_height * 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        
        # Bottom - Motor status
        bottom_y = h - 80
        cv2.putText(frame, f"Motor: {gyro_steering.current_speed}/255 | Servo: {gyro_steering.current_servo_angle:.1f}°",
                    (10, bottom_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        
        # Visual indicator of target and current heading
        center_x = w // 2
        indicator_y = h - 40
        arrow_length = 50
        
        # Draw compass circle
        cv2.circle(frame, (center_x, indicator_y), 60, (100, 100, 100), 2)
        
        # Draw target heading (cyan arrow)
        arrow_angle_rad = target_angle * 3.14159 / 180
        end_x = int(center_x + arrow_length * np.sin(arrow_angle_rad))
        end_y = int(indicator_y - arrow_length * np.cos(arrow_angle_rad))
        cv2.arrowedLine(frame, (center_x, indicator_y), (end_x, end_y), 
                       (0, 255, 255), 4, tipLength=0.3)
        cv2.putText(frame, "T", (end_x + 5, end_y + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw current heading (yellow arrow)
        current_angle_rad = current_heading * 3.14159 / 180
        end_x2 = int(center_x + (arrow_length - 5) * np.sin(current_angle_rad))
        end_y2 = int(indicator_y - (arrow_length - 5) * np.cos(current_angle_rad))
        cv2.arrowedLine(frame, (center_x, indicator_y), (end_x2, end_y2), 
                       (0, 255, 255), 3, tipLength=0.3)
        cv2.putText(frame, "A", (end_x2 + 5, end_y2 + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw center reference line (0° - North)
        cv2.line(frame, (center_x, indicator_y - 65), 
                (center_x, indicator_y - 52), (0, 255, 0), 2)
        cv2.putText(frame, "N", (center_x - 8, indicator_y - 68), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Status indicator
        status_color = (0, 255, 0) if gyro_steering.is_moving else (0, 0, 255)
        status_text = "RUNNING" if gyro_steering.is_moving else "STOPPED"
        cv2.circle(frame, (w - 80, 30), 12, status_color, -1)
        cv2.putText(frame, status_text, (w - 150, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

        # Encode frame to JPEG
        ret, buffer = cv2.imencode(".jpg", frame)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# -----------------------------
# Enhanced Frontend with Controls
# -----------------------------
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PiVision - Integrated Gyro Steering</title>
<script src="https://cdn.tailwindcss.com"></script>
<script src="https://unpkg.com/feather-icons"></script>
</head>
<body class="bg-gray-900 text-gray-100 min-h-screen">
<main class="container mx-auto px-4 py-8">
    <div class="max-w-6xl mx-auto">
        <h1 class="text-4xl font-bold text-indigo-400 mb-6 flex items-center">
            <i data-feather="compass" class="mr-3"></i>
            PiVision - Integrated Gyro Steering
        </h1>
        
        <div class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-6">
            <!-- Main Video Feed -->
            <div class="lg:col-span-2">
                <div class="relative bg-gray-800 rounded-xl overflow-hidden shadow-2xl border-2 border-gray-700">
                    <img src="{{ url_for('video_feed') }}" 
                         class="w-full h-auto object-cover" 
                         alt="Live camera feed">
                    <div class="absolute top-4 left-4 bg-black bg-opacity-80 text-white px-4 py-2 rounded-lg">
                        <div class="text-xs text-gray-300">Steering Mode</div>
                        <div class="text-xl font-bold text-green-400">Vision + Gyro</div>
                    </div>
                </div>
                
                <!-- Legend -->
                <div class="mt-4 bg-gray-800 p-4 rounded-lg grid grid-cols-2 gap-4">
                    <div>
                        <div class="flex items-center mb-2">
                            <div class="w-4 h-4 bg-cyan-400 mr-2"></div>
                            <span class="text-sm font-semibold">Target (T)</span>
                        </div>
                        <p class="text-xs text-gray-400">Vision calculated heading</p>
                    </div>
                    <div>
                        <div class="flex items-center mb-2">
                            <div class="w-4 h-4 bg-yellow-400 mr-2"></div>
                            <span class="text-sm font-semibold">Actual (A)</span>
                        </div>
                        <p class="text-xs text-gray-400">Gyroscope measured heading</p>
                    </div>
                </div>
            </div>
            
            <!-- Control Panel -->
            <div class="space-y-4">
                <!-- Motor Control -->
                <div class="bg-gradient-to-br from-gray-800 to-gray-900 p-5 rounded-lg border-2 border-gray-700 shadow-lg">
                    <h3 class="text-xl font-bold text-indigo-300 mb-4 flex items-center">
                        <i data-feather="zap" class="w-5 h-5 mr-2"></i>
                        Motor Control
                    </h3>
                    <div class="space-y-3">
                        <button id="start-btn" onclick="startSteering()" 
                                class="w-full bg-gradient-to-r from-green-600 to-green-700 hover:from-green-700 hover:to-green-800 text-white font-bold py-4 px-4 rounded-lg transition shadow-lg transform hover:scale-105">
                            <i data-feather="play" class="inline w-6 h-6 mr-2"></i>
                            START STEERING
                        </button>
                        <button id="stop-btn" onclick="stopSteering()" 
                                class="w-full bg-gradient-to-r from-red-600 to-red-700 hover:from-red-700 hover:to-red-800 text-white font-bold py-4 px-4 rounded-lg transition shadow-lg transform hover:scale-105">
                            <i data-feather="stop-circle" class="inline w-6 h-6 mr-2"></i>
                            STOP STEERING
                        </button>
                    </div>
                    
                    <div class="mt-5 pt-4 border-t border-gray-700">
                        <div class="flex justify-between mb-2">
                            <span class="text-gray-300 font-semibold">Motor Speed</span>
                            <span class="text-white font-bold text-lg" id="speed-value">180</span>
                        </div>
                        <input type="range" id="speed-slider" min="100" max="255" value="180" 
                               class="w-full h-3 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-green-500"
                               oninput="updateSpeedDisplay(this.value)">
                        <div class="flex justify-between text-xs text-gray-500 mt-1">
                            <span>Slow (100)</span>
                            <span>Fast (255)</span>
                        </div>
                    </div>
                </div>
                
                <!-- Vision Parameters -->
                <div class="bg-gray-800 p-5 rounded-lg border border-gray-700">
                    <h3 class="text-lg font-semibold text-cyan-300 mb-3 flex items-center">
                        <i data-feather="eye" class="w-5 h-5 mr-2"></i>
                        Vision Parameters
                    </h3>
                    <div class="space-y-4 text-sm">
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Integral Gain (k)</span>
                                <span class="text-white font-mono" id="k-value">0.010</span>
                            </div>
                            <input type="range" id="k-slider" min="1" max="50" value="10" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-cyan-500"
                                   oninput="updateKDisplay(this.value)" onchange="updateK(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Controls how fast target accumulates</div>
                        </div>
                        
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Max Rate (°/frame)</span>
                                <span class="text-white font-mono" id="rate-value">5.0</span>
                            </div>
                            <input type="range" id="rate-slider" min="1" max="200" value="50" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-cyan-500"
                                   oninput="updateRateDisplay(this.value)" onchange="updateRate(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Maximum turn rate limit</div>
                        </div>
                        
                        <div>
                            <div class="flex justify-between mb-1">
                                <span class="text-gray-400">Deadzone (pixels)</span>
                                <span class="text-white font-mono" id="dead-value">50</span>
                            </div>
                            <input type="range" id="dead-slider" min="0" max="200" value="50" 
                                   class="w-full h-2 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-cyan-500"
                                   oninput="updateDeadDisplay(this.value)" onchange="updateDead(this.value)">
                            <div class="text-xs text-gray-500 mt-1">Ignore small wall differences</div>
                        </div>
                    </div>
                </div>
                
                <!-- Quick Actions -->
                <div class="bg-gray-800 p-5 rounded-lg border border-gray-700">
                    <h3 class="text-lg font-semibold text-yellow-300 mb-3 flex items-center">
                        <i data-feather="target" class="w-5 h-5 mr-2"></i>
                        Quick Actions
                    </h3>
                    <div class="space-y-2">
                        <button onclick="resetSteering()" 
                                class="w-full bg-yellow-600 hover:bg-yellow-700 text-white font-semibold py-2 px-4 rounded-lg transition">
                            <i data-feather="rotate-ccw" class="inline w-4 h-4 mr-2"></i>
                            Reset Target to 0°
                        </button>
                    </div>
                </div>
                
                <!-- System Info -->
                <div class="bg-gray-800 p-4 rounded-lg border border-gray-700">
                    <h3 class="text-sm font-semibold text-gray-400 mb-2">System Information</h3>
                    <div class="text-xs text-gray-500 space-y-1">
                        <p>• PID: Kp=0.8, Ki=0.01, Kd=0.3</p>
                        <p>• Update Rate: 50Hz (20ms)</p>
                        <p>• Servo Range: ±40°</p>
                        <p>• Camera: 640×480 @ 30fps</p>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Info Cards -->
        <div class="grid grid-cols-1 md:grid-cols-3 gap-4">
            <div class="bg-gradient-to-br from-cyan-900 to-gray-800 p-5 rounded-lg border border-cyan-700">
                <h3 class="text-lg font-bold text-cyan-300 mb-2 flex items-center">
                    <i data-feather="camera" class="w-5 h-5 mr-2"></i>
                    Vision System
                </h3>
                <div class="text-sm text-gray-300 space-y-2">
                    <p>• Analyzes wall positions</p>
                    <p>• Calculates target heading</p>
                    <p>• More left pixels → turn right</p>
                    <p>• More right pixels → turn left</p>
                </div>
            </div>
            
            <div class="bg-gradient-to-br from-yellow-900 to-gray-800 p-5 rounded-lg border border-yellow-700">
                <h3 class="text-lg font-bold text-yellow-300 mb-2 flex items-center">
                    <i data-feather="activity" class="w-5 h-5 mr-2"></i>
                    Gyroscope System
                </h3>
                <div class="text-sm text-gray-300 space-y-2">
                    <p>• MPU6050 sensor</p>
                    <p>• Tracks actual heading</p>
                    <p>• Real-time orientation</p>
                    <p>• 50Hz update rate</p>
                </div>
            </div>
            
            <div class="bg-gradient-to-br from-purple-900 to-gray-800 p-5 rounded-lg border border-purple-700">
                <h3 class="text-lg font-bold text-purple-300 mb-2 flex items-center">
                    <i data-feather="settings" class="w-5 h-5 mr-2"></i>
                    PID Controller
                </h3>
                <div class="text-sm text-gray-300 space-y-2">
                    <p>• Compares target vs actual</p>
                    <p>• Calculates steering angle</p>
                    <p>• Controls servo position</p>
                    <p>• Minimizes heading error</p>
                </div>
            </div>
        </div>
    </div>
</main>

<script>
feather.replace();

// Display update functions (instant feedback)
function updateSpeedDisplay(value) {
    document.getElementById('speed-value').textContent = value;
}

function updateKDisplay(value) {
    const k = value / 1000;
    document.getElementById('k-value').textContent = k.toFixed(3);
}

function updateRateDisplay(value) {
    const rate = value / 10;
    document.getElementById('rate-value').textContent = rate.toFixed(1);
}

function updateDeadDisplay(value) {
    document.getElementById('dead-value').textContent = value;
}

// API call functions (send to backend)
function updateK(value) {
    const k = value / 1000;
    fetch('/update_param', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({param: 'k_integral', value: k})
    })
    .then(response => response.json())
    .then(data => console.log('Updated k_integral:', data));
}

function updateRate(value) {
    const rate = value / 10;
    fetch('/update_param', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({param: 'max_rate', value: rate})
    })
    .then(response => response.json())
    .then(data => console.log('Updated max_rate:', data));
}

function updateDead(value) {
    fetch('/update_param', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({param: 'deadzone', value: parseInt(value)})
    })
    .then(response => response.json())
    .then(data => console.log('Updated deadzone:', data));
}

function startSteering() {
    const speed = parseInt(document.getElementById('speed-slider').value);
    fetch('/start_steering', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({speed: speed})
    })
    .then(response => response.json())
    .then(data => {
        console.log('Started:', data);
        showNotification('Steering Started!', 'success');
    })
    .catch(error => {
        console.error('Error:', error);
        showNotification('Failed to start!', 'error');
    });
}

function stopSteering() {
    fetch('/stop_steering', {method: 'POST'})
    .then(response => response.json())
    .then(data => {
        console.log('Stopped:', data);
        showNotification('Steering Stopped!', 'warning');
    })
    .catch(error => {
        console.error('Error:', error);
        showNotification('Failed to stop!', 'error');
    });
}

function resetSteering() {
    fetch('/reset_steering', {method: 'POST'})
    .then(response => response.json())
    .then(data => {
        console.log('Reset:', data);
        showNotification('Target Reset to 0°', 'info');
    });
}

function showNotification(message, type) {
    const colors = {
        success: 'bg-green-600',
        error: 'bg-red-600',
        warning: 'bg-yellow-600',
        info: 'bg-blue-600'
    };
    
    const notification = document.createElement('div');
    notification.className = `fixed top-4 right-4 ${colors[type]} text-white px-6 py-3 rounded-lg shadow-lg z-50 transition-opacity`;
    notification.textContent = message;
    document.body.appendChild(notification);
    
    setTimeout(() => {
        notification.style.opacity = '0';
        setTimeout(() => notification.remove(), 300);
    }, 2000);
}

// Initialize feather icons after page load
document.addEventListener('DOMContentLoaded', function() {
    feather.replace();
});
</script>
</body>
</html>
"""

# -----------------------------
# Flask Routes
# -----------------------------
@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_steering', methods=['POST'])
def start_steering():
    global steering_thread
    
    data = request.get_json()
    speed = data.get('speed', DEFAULT_MOTOR_SPEED)
    
    if not gyro_steering.running:
        gyro_steering.start_continuous_steering(speed)
        steering_thread = threading.Thread(
            target=gyro_steering.steering_loop, 
            args=(vision_steering,),
            daemon=True
        )
        steering_thread.start()
        return jsonify({'status': 'started', 'speed': speed})
    else:
        return jsonify({'status': 'already_running', 'speed': speed})

@app.route('/stop_steering', methods=['POST'])
def stop_steering():
    gyro_steering.stop_continuous_steering()
    return jsonify({'status': 'stopped'})

@app.route('/reset_steering', methods=['POST'])
def reset_steering():
    vision_steering.reset()
    return jsonify({'status': 'ok', 'angle': vision_steering.get_target_angle()})

@app.route('/update_param', methods=['POST'])
def update_param():
    data = request.get_json()
    param = data.get('param')
    value = data.get('value')
    
    if param == 'k_integral':
        vision_steering.set_k_integral(value)
    elif param == 'max_rate':
        vision_steering.set_max_rate(value)
    elif param == 'deadzone':
        vision_steering.set_deadzone(value)
    
    return jsonify({'status': 'ok', 'param': param, 'value': value})

@app.route('/status', methods=['GET'])
def get_status():
    """API endpoint to get current system status"""
    gyro_data = gyro_steering.get_gyro_data()
    return jsonify({
        'is_running': gyro_steering.running,
        'is_moving': gyro_steering.is_moving,
        'current_speed': gyro_steering.current_speed,
        'target_heading': vision_steering.get_target_angle(),
        'current_heading': gyro_data['heading'],
        'gyro_x': gyro_data['x'],
        'gyro_y': gyro_data['y'],
        'gyro_z': gyro_data['z'],
        'servo_angle': gyro_steering.current_servo_angle
    })

# -----------------------------
if __name__ == "__main__":
    try:
        print("=" * 60)
        print("Integrated Vision + Gyroscope Steering Control")
        print("=" * 60)
        print("\nHardware:")
        print(f"  • Servo: GPIO {SERVO_PIN}")
        print(f"  • Motor: IN1={IN1}, IN2={IN2}, ENA={ENA}")
        print(f"  • Gyroscope: MPU6050 (I2C)")
        print("\nVision Parameters:")
        print(f"  • Integral gain (k): {vision_steering.k_integral}")
        print(f"  • Max rate: {vision_steering.max_rate}°/frame")
        print(f"  • Deadzone: {vision_steering.deadzone} pixels")
        print("\nPID Parameters:")
        print(f"  • KP: {KP}, KI: {KI}, KD: {KD}")
        print("\nStarting server on http://0.0.0.0:5000")
        print("=" * 60)
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        gyro_steering.cleanup()
        print("Cleanup complete")
