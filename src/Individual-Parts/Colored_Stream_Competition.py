#!/usr/bin/env python3
"""
Autonomous Wall-Following Robot with Obstacle Avoidance
Toggle switch on GPIO 25 to start/stop
"""

from picamera2 import Picamera2
import cv2
import time
import json
import numpy as np
import math
import board
import adafruit_mpu6050
import pigpio
from gpiozero import AngularServo, Device, Button
from gpiozero.pins.pigpio import PiGPIOFactory

# -----------------------------
# Hardware Configuration
# -----------------------------
SERVO_PIN = 18
IN1, IN2, ENA = 24, 23, 13
SWITCH_PIN = 25

SERVO_MAX_RIGHT = 35
SERVO_MAX_LEFT = -35
DEFAULT_MOTOR_SPEED = 220

# PID Parameters
KP = 0.8
KI = 0.01
KD = 0.3
SERVO_DIRECTION = 1
GYRO_DIRECTION = -1

# Vision Parameters
K_INTEGRAL = 0.01
MAX_RATE = 4.0
DEADZONE = 1000
LAP_LIMIT = 1080.0

# Obstacle Avoidance Parameters
AVOIDANCE_WEIGHT = 0.003
AVOIDANCE_THRESHOLD = 100
MAX_AVOIDANCE = 3.0

# -----------------------------
# Setup
# -----------------------------
print("Initializing...")
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()
time.sleep(2)

button = Button(SWITCH_PIN, bounce_time=0.3)
print("✓ Camera and switch ready")

# -----------------------------
# Vision Controller with Avoidance
# -----------------------------
class VisionController:
    def __init__(self):
        self.target_angle = 0.0
        self.is_active = False
        self.avoidance_history = []
        self.avoidance_smooth_window = 3
        
    def smooth_avoidance(self, current_bias):
        """Apply moving average smoothing to avoidance bias"""
        self.avoidance_history.append(current_bias)
        if len(self.avoidance_history) > self.avoidance_smooth_window:
            self.avoidance_history.pop(0)
        
        if len(self.avoidance_history) > 0:
            return sum(self.avoidance_history) / len(self.avoidance_history)
        return 0.0
        
    def update(self, left_pixels, right_pixels, avoidance_bias=0.0):
        if not self.is_active:
            return self.target_angle, 0.0, 0, 0.0
        
        difference = left_pixels - right_pixels
        
        if abs(difference) < DEADZONE:
            difference = 0
        
        rate = difference * K_INTEGRAL
        rate = max(-MAX_RATE, min(MAX_RATE, rate))
        
        # Smooth and apply avoidance bias
        smoothed_avoidance = self.smooth_avoidance(avoidance_bias)
        total_rate = rate + smoothed_avoidance
        total_rate = max(-MAX_RATE * 1.5, min(MAX_RATE * 1.5, total_rate))
        
        self.target_angle += total_rate
        
        return self.target_angle, total_rate, difference, smoothed_avoidance
    
    def reset(self):
        self.target_angle = 0.0
        self.is_active = False
        self.avoidance_history = []

# -----------------------------
# Gyro Controller
# -----------------------------
class GyroController:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Run 'sudo pigpiod' first")
        
        for p in (ENA, IN1, IN2):
            self.pi.set_mode(p, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(ENA, 2000)
        
        Device.pin_factory = PiGPIOFactory()
        self.servo = AngularServo(
            SERVO_PIN,
            min_angle=SERVO_MAX_LEFT,
            max_angle=SERVO_MAX_RIGHT,
            min_pulse_width=0.0012,
            max_pulse_width=0.0017
        )
        
        i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        
        self.cumulative_rotation = 0.0
        self.current_heading = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.last_error = 0.0
        self.running = False
        
        self.calibrate_gyro()
        
    def calibrate_gyro(self):
        print("Calibrating gyro...")
        offset = 0.0
        for i in range(100):
            offset += self.mpu.gyro[2]
            time.sleep(0.01)
        self.gyro_z_offset = offset / 100
        print(f"✓ Gyro ready (offset: {self.gyro_z_offset:.4f})")
    
    def run_motor(self, speed):
        self.pi.write(IN1, 1 if speed >= 0 else 0)
        self.pi.write(IN2, 0 if speed >= 0 else 1)
        self.pi.set_PWM_dutycycle(ENA, min(255, abs(int(speed))))
        
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.write(IN1, 0)
        self.pi.write(IN2, 0)
        
    def update_heading(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        gyro_z = (self.mpu.gyro[2] - self.gyro_z_offset) * GYRO_DIRECTION
        heading_change = math.degrees(gyro_z * dt)
        
        self.cumulative_rotation += heading_change
        self.current_heading = (self.current_heading + heading_change + 180) % 360 - 180
        
    def calculate_steering(self, target):
        error = target - self.cumulative_rotation
        
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        
        derivative = error - self.last_error
        self.last_error = error
        
        steering = (KP * error + KI * self.integral + KD * derivative) * SERVO_DIRECTION
        return max(min(steering, SERVO_MAX_RIGHT), SERVO_MAX_LEFT), error
        
    def cleanup(self):
        self.stop_motor()
        self.servo.angle = 0
        self.servo.close()
        self.pi.stop()

# -----------------------------
# ROI Config
# -----------------------------
def load_roi_config():
    try:
        with open("roi_config.json", "r") as f:
            return json.load(f)["zones"]
    except:
        return {
            "Corner_Top": 20,
            "Corner_Bottom": 40,
            "Left_Obstacle_LM": 0,
            "Left_Obstacle_RM": 50,
            "Right_Obstacle_LM": 50,
            "Right_Obstacle_RM": 100,
            "Right_Wall_Top": 30,
            "Left_Wall_Top": 30,
            "Wall_Bottom": 70,
            "Right_Wall_RM": 100,
            "Right_Wall_LM": 70,
            "Left_Wall_RM": 30,
            "Left_Wall_LM": 0,
        }

# Color detection
COLOR_LOW = (0, 0, 0)
COLOR_HIGH = (180, 255, 60)

RED_LOW = (159, 196, 64)
RED_HIGH = (188, 255, 142)

GREEN_LOW = (37, 139, 45)
GREEN_HIGH = (80, 255, 142)

# -----------------------------
# Main Loop
# -----------------------------
def main():
    vision = VisionController()
    gyro = GyroController()
    roi_zones = load_roi_config()
    
    mission_complete = False
    last_status_time = time.time()
    
    print("\n" + "="*60)
    print("🤖 AUTONOMOUS ROBOT WITH OBSTACLE AVOIDANCE READY")
    print("="*60)
    print(f"Switch GPIO {SWITCH_PIN}: Toggle ON to start 3-lap mission")
    print("Obstacle Detection:")
    print("  🔴 Red blocks → Pass on RIGHT")
    print("  🟢 Green blocks → Pass on LEFT")
    print("="*60 + "\n")
    
    try:
        while True:
            # Check switch state
            if button.is_pressed and not mission_complete:
                if not gyro.running:
                    # Start mission ONCE
                    print("🚀 STARTING MISSION WITH OBSTACLE AVOIDANCE")
                    vision.reset()
                    vision.is_active = True
                    gyro.cumulative_rotation = 0.0
                    gyro.current_heading = 0.0
                    gyro.running = True
                    gyro.run_motor(DEFAULT_MOTOR_SPEED)
                    gyro.last_time = time.time()
                    last_status_time = time.time()
                    print("✓ Running...\n")
            
            # Continue running if mission started
            if gyro.running:
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                h, w, _ = frame.shape
                
                # Extract ROI boundaries
                Ct = int(h * roi_zones["Corner_Top"] / 100)
                Cb = int(h * roi_zones["Corner_Bottom"] / 100)
                RWt = int(h * roi_zones["Right_Wall_Top"] / 100)
                LWt = int(h * roi_zones["Left_Wall_Top"] / 100)
                Wb = int(h * roi_zones["Wall_Bottom"] / 100)
                
                LOlm = int(w * roi_zones["Left_Obstacle_LM"] / 100)
                LOrm = int(w * roi_zones["Left_Obstacle_RM"] / 100)
                ROlm = int(w * roi_zones["Right_Obstacle_LM"] / 100)
                ROrm = int(w * roi_zones["Right_Obstacle_RM"] / 100)
                
                Rlm = int(w * roi_zones["Right_Wall_LM"] / 100)
                Rrm = int(w * roi_zones["Right_Wall_RM"] / 100)
                Llm = int(w * roi_zones["Left_Wall_LM"] / 100)
                Lrm = int(w * roi_zones["Left_Wall_RM"] / 100)
                
                # Wall detection
                wall_mask = cv2.inRange(hsv, COLOR_LOW, COLOR_HIGH)
                left_roi = wall_mask[LWt:Wb, Llm:Lrm]
                right_roi = wall_mask[RWt:Wb, Rlm:Rrm]
                
                left_pixels = cv2.countNonZero(left_roi)
                right_pixels = cv2.countNonZero(right_roi)
                
                # Obstacle detection (split ROIs)
                red_mask = cv2.inRange(hsv, RED_LOW, RED_HIGH)
                green_mask = cv2.inRange(hsv, GREEN_LOW, GREEN_HIGH)
                
                red_roi = red_mask[Ct:Cb, ROlm:ROrm]  # Right side only
                green_roi = green_mask[Ct:Cb, LOlm:LOrm]  # Left side only
                
                red_pixels = cv2.countNonZero(red_roi)
                green_pixels = cv2.countNonZero(green_roi)
                
                # Calculate avoidance bias
                avoidance_bias = 0.0
                
                if red_pixels > AVOIDANCE_THRESHOLD:
                    avoidance_bias += red_pixels * AVOIDANCE_WEIGHT
                
                if green_pixels > AVOIDANCE_THRESHOLD:
                    avoidance_bias -= green_pixels * AVOIDANCE_WEIGHT
                
                avoidance_bias = max(-MAX_AVOIDANCE, min(MAX_AVOIDANCE, avoidance_bias))
                
                # Update vision with avoidance
                target_angle, rate, diff, smoothed_avoid = vision.update(
                    left_pixels, right_pixels, avoidance_bias
                )
                
                gyro.update_heading()
                
                # Check if 3 laps completed
                if abs(gyro.cumulative_rotation) >= LAP_LIMIT - 10:
                    laps = gyro.cumulative_rotation / 360.0
                    print(f"\n🏁 MISSION COMPLETE! {laps:.2f} laps")
                    gyro.running = False
                    gyro.stop_motor()
                    gyro.servo.angle = 0
                    vision.is_active = False
                    mission_complete = True
                    print("✓ Robot stopped. Turn switch OFF then ON to restart.\n")
                    continue
                
                steering_angle, error = gyro.calculate_steering(target_angle)
                gyro.servo.angle = steering_angle
                
                # Status every 1 second
                current_time = time.time()
                if current_time - last_status_time >= 1.0:
                    last_status_time = current_time
                    laps = abs(gyro.cumulative_rotation) / 360.0
                    
                    # Build status message
                    status = f"Laps: {laps:.2f} | Rot: {gyro.cumulative_rotation:.1f}° | "
                    status += f"Err: {error:.1f}° | Servo: {steering_angle:.1f}°"
                    
                    # Add obstacle detection status
                    if red_pixels > AVOIDANCE_THRESHOLD:
                        status += f" | 🔴 RED: {red_pixels}px → RIGHT"
                    if green_pixels > AVOIDANCE_THRESHOLD:
                        status += f" | 🟢 GREEN: {green_pixels}px → LEFT"
                    
                    print(status)
            
            elif not button.is_pressed and mission_complete:
                # Button released after mission complete, allow restart
                mission_complete = False
                print("✓ Ready for new mission (turn switch ON)")
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\n\n⚠ Stopping...")
    finally:
        gyro.cleanup()
        picam2.stop()
        print("✓ Shutdown complete")

if __name__ == "__main__":
    main()
