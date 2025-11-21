#!/usr/bin/env python3
"""
Autonomous Wall-Following Robot - Simplified
Toggle switch on GPIO 25 to start/stop
"""

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
from gpiozero import AngularServo, Device, Button
from gpiozero.pins.pigpio import PiGPIOFactory

# -----------------------------
# Hardware Configuration
# -----------------------------
SERVO_PIN = 18
IN1, IN2, ENA = 24, 23, 13
SWITCH_PIN = 25

SERVO_MAX_RIGHT = 25
SERVO_MAX_LEFT = -25
DEFAULT_MOTOR_SPEED = 220

KP = 0.8
KI = 0.01
KD = 0.3
SERVO_DIRECTION = 1
GYRO_DIRECTION = -1

K_INTEGRAL = 0.01
MAX_RATE = 2.0
DEADZONE = 1000
LAP_LIMIT = 1080.0

# -----------------------------
# Setup
# -----------------------------
print("Initializing...")
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()
time.sleep(2)

button = Button(SWITCH_PIN, pull_up=False, bounce_time=0.3)
print("✓ Camera and switch ready")

# -----------------------------
# Vision Controller
# -----------------------------
class VisionController:
    def __init__(self):
        self.target_angle = 0.0
        self.is_active = False
        
    def update(self, left_pixels, right_pixels):
        if not self.is_active:
            return self.target_angle, 0.0, 0
        
        difference = left_pixels - right_pixels
        
        if abs(difference) < DEADZONE:
            difference = 0
        
        rate = difference * K_INTEGRAL
        rate = max(-MAX_RATE, min(MAX_RATE, rate))
        self.target_angle += rate
        
        return self.target_angle, rate, difference
    
    def reset(self):
        self.target_angle = 0.0
        self.is_active = False

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
            "Right_Wall_Top": 40,
            "Left_Wall_Top": 30,
            "Wall_Bottom": 70,
            "Right_Wall_RM": 100,
            "Right_Wall_LM": 70,
            "Left_Wall_RM": 30,
            "Left_Wall_LM": 0,
        }

# -----------------------------
# Main Loop
# -----------------------------
def main():
    vision = VisionController()
    gyro = GyroController()
    roi_zones = load_roi_config()
    
    mission_complete = False  # Track if mission finished
    
    print("\n" + "="*60)
    print("🤖 AUTONOMOUS ROBOT READY")
    print("="*60)
    print(f"Switch GPIO {SWITCH_PIN}: Toggle ON to start 3-lap mission")
    print("="*60 + "\n")
    
    try:
        while True:
            # Check switch state
            if button.is_pressed and not mission_complete:
                if not gyro.running:
                    # Start mission
                    print("🚀 STARTING MISSION")
                    vision.reset()
                    vision.is_active = True
                    gyro.cumulative_rotation = 0.0
                    gyro.current_heading = 0.0
                    gyro.running = True
                    gyro.run_motor(DEFAULT_MOTOR_SPEED)
                    gyro.last_time = time.time()
                    print("✓ Running...\n")
                
                # Main steering loop
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                h, w, _ = frame.shape
                
                RWt = int(h * roi_zones["Right_Wall_Top"] / 100)
                LWt = int(h * roi_zones["Left_Wall_Top"] / 100)
                Wb = int(h * roi_zones["Wall_Bottom"] / 100)
                Rlm = int(w * roi_zones["Right_Wall_LM"] / 100)
                Rrm = int(w * roi_zones["Right_Wall_RM"] / 100)
                Llm = int(w * roi_zones["Left_Wall_LM"] / 100)
                Lrm = int(w * roi_zones["Left_Wall_RM"] / 100)
                
                mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 60))
                left_roi = mask[LWt:Wb, Llm:Lrm]
                right_roi = mask[RWt:Wb, Rlm:Rrm]
                
                left_pixels = cv2.countNonZero(left_roi)
                right_pixels = cv2.countNonZero(right_roi)
                
                target_angle, rate, diff = vision.update(left_pixels, right_pixels)
                gyro.update_heading()
                
                # Check if 3 laps completed
                if abs(gyro.cumulative_rotation) >= LAP_LIMIT - 10:
                    laps = gyro.cumulative_rotation / 360.0
                    print(f"\n🏁 MISSION COMPLETE! {laps:.2f} laps")
                    gyro.running = False
                    gyro.stop_motor()
                    gyro.servo.angle = 0
                    vision.is_active = False
                    mission_complete = True  # Mark as complete, won't restart
                    print("✓ Robot stopped. Turn switch OFF then ON to restart.\n")
                    continue
                
                steering_angle, error = gyro.calculate_steering(target_angle)
                gyro.servo.angle = steering_angle
                
                # Status every ~1 second
                if int(gyro.cumulative_rotation * 10) % 500 == 0:
                    laps = abs(gyro.cumulative_rotation) / 360.0
                    print(f"Laps: {laps:.2f} | Rotation: {gyro.cumulative_rotation:.1f}° | "
                          f"Error: {error:.1f}° | Servo: {steering_angle:.1f}°")
                
            else:
                # Switch is OFF or mission complete
                if gyro.running:
                    print("\n🛑 STOPPED")
                    gyro.running = False
                    gyro.stop_motor()
                    gyro.servo.angle = 0
                    vision.is_active = False
                
                # If switch turned OFF after mission complete, allow restart
                if not button.is_pressed and mission_complete:
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
