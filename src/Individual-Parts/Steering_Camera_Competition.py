#!/usr/bin/env python3
"""
Autonomous Wall-Following Robot with Vision + Gyro Control
Runs standalone with GPIO button (pin 25) - no Flask or internet required
Press button to start 3-lap autonomous run
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
from signal import pause

# -----------------------------
# Hardware Pin Configuration
# -----------------------------
SERVO_PIN = 18
IN1, IN2, ENA = 24, 23, 13
BUTTON_PIN = 25  # GPIO pin for start button

# Servo limits
SERVO_MAX_RIGHT = 35
SERVO_MAX_LEFT = -35

# Motor settings
DEFAULT_MOTOR_SPEED = 220

# PID Controller parameters
KP = 0.8
KI = 0.01
KD = 0.3

# Direction settings
SERVO_DIRECTION = 1
GYRO_DIRECTION = -1

# Vision parameters
K_INTEGRAL = 0.01
MAX_RATE = 2.0
DEADZONE = 1000
LAP_LIMIT = 1080.0  # 3 laps

# -----------------------------
# Camera Setup
# -----------------------------
print("Initializing camera...")
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(preview_config)
picam2.start()
time.sleep(2)
print("✓ Camera ready")

# -----------------------------
# Vision Steering Controller
# -----------------------------
class VisionSteeringController:
    def __init__(self):
        self.target_angle = 0.0
        self.max_rate = MAX_RATE
        self.deadzone = DEADZONE
        self.k_integral = K_INTEGRAL
        self.lock = threading.Lock()
        self.is_active = False
        self.lap_limit = LAP_LIMIT
        self.lap_min = LAP_LIMIT - 10
        self.lap_max = LAP_LIMIT + 10
        
    def update(self, left_pixels, right_pixels):
        """Update target angle based on wall pixel difference"""
        with self.lock:
            if not self.is_active:
                return self.target_angle, 0.0, 0
            
            difference = left_pixels - right_pixels
            
            # Apply deadzone
            if abs(difference) < self.deadzone:
                difference = 0
            
            # Calculate rate of change
            rate = difference * self.k_integral
            
            # Clamp rate
            rate = max(-self.max_rate, min(self.max_rate, rate))
            
            # Accumulate angle
            self.target_angle += rate
            
            return self.target_angle, rate, difference
    
    def get_target_angle(self):
        with self.lock:
            return self.target_angle
    
    def should_stop(self, current_rotation):
        """Check if robot should stop based on lap completion"""
        with self.lock:
            return self.is_active and (self.lap_min <= abs(current_rotation) <= self.lap_max)
    
    def start(self):
        with self.lock:
            self.is_active = True
            print("✓ Vision steering activated")
    
    def stop(self):
        with self.lock:
            self.is_active = False
            print("✓ Vision steering deactivated")
    
    def reset(self):
        with self.lock:
            self.target_angle = 0.0
            self.is_active = False
            print("✓ Vision steering reset")

# -----------------------------
# Gyroscope Steering Controller
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
            min_pulse_width=0.0012,
            max_pulse_width=0.0017
        )
        
        # Initialize MPU6050
        i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        
        # State variables
        self.current_heading = 0.0
        self.cumulative_rotation = 0.0
        self.target_heading = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.last_error = 0.0
        self.is_moving = False
        self.current_speed = DEFAULT_MOTOR_SPEED
        self.current_servo_angle = 0.0
        self.running = False
        self.lock = threading.Lock()
        
        # Calibrate gyro
        self.calibrate_gyro()
        
    def calibrate_gyro(self, samples=100):
        print("Calibrating gyroscope...")
        gyro_z_offset = 0.0
        for i in range(samples):
            gyro_z_offset += self.mpu.gyro[2]
            time.sleep(0.01)
        self.gyro_z_offset = gyro_z_offset / samples
        print(f"✓ Gyro calibrated. Offset: {self.gyro_z_offset:.4f}")
    
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
        
        # Read gyro data
        gyro_data = self.mpu.gyro
        gyro_z = gyro_data[2]
        
        gyro_z_corrected = (gyro_z - self.gyro_z_offset) * GYRO_DIRECTION
        heading_change = math.degrees(gyro_z_corrected * dt)
        
        with self.lock:
            self.cumulative_rotation += heading_change
            self.current_heading = (self.current_heading + heading_change + 180) % 360 - 180
        
    def calculate_steering_angle(self, target_heading):
        with self.lock:
            error = target_heading - self.cumulative_rotation
        
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        
        derivative = error - self.last_error
        self.last_error = error
        
        steering = (KP * error + KI * self.integral + KD * derivative) * SERVO_DIRECTION
        steering = max(min(steering, SERVO_MAX_RIGHT), SERVO_MAX_LEFT)
        
        return steering, error
    
    def get_cumulative_rotation(self):
        with self.lock:
            return self.cumulative_rotation
    
    def set_target_heading(self, heading):
        with self.lock:
            self.target_heading = heading
        
    def start_continuous_steering(self, speed=DEFAULT_MOTOR_SPEED):
        """Start continuous steering mode"""
        self.running = True
        self.run_motor(speed)
        print(f"✓ Motor started at speed {speed}")
        
    def stop_continuous_steering(self):
        """Stop continuous steering mode"""
        self.running = False
        self.stop_motor()
        self.servo.angle = 0
        self.current_servo_angle = 0
        print("✓ Motor stopped")
        
    def steering_loop(self, vision_controller, roi_zones):
        """Main steering loop - runs continuously"""
        print("🚀 Starting autonomous steering loop...")
        self.last_time = time.time()
        
        while self.running:
            try:
                # Capture and process frame
                frame = picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                h, w, _ = frame.shape
                
                # Calculate ROI boundaries
                RWt = int(h * roi_zones["Right_Wall_Top"] / 100)
                LWt = int(h * roi_zones["Left_Wall_Top"] / 100)
                Wb = int(h * roi_zones["Wall_Bottom"] / 100)
                Rlm = int(w * roi_zones["Right_Wall_LM"] / 100)
                Rrm = int(w * roi_zones["Right_Wall_RM"] / 100)
                Llm = int(w * roi_zones["Left_Wall_LM"] / 100)
                Lrm = int(w * roi_zones["Left_Wall_RM"] / 100)
                
                # Create mask and count pixels
                mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 60))
                left_roi = mask[LWt:Wb, Llm:Lrm]
                right_roi = mask[RWt:Wb, Rlm:Rrm]
                
                left_pixels = cv2.countNonZero(left_roi)
                right_pixels = cv2.countNonZero(right_roi)
                
                # Update vision target
                target_angle, rate, diff = vision_controller.update(left_pixels, right_pixels)
                
                # Update heading from gyro
                self.update_heading()
                
                # Check if we should auto-stop
                current_rotation = self.get_cumulative_rotation()
                
                if vision_controller.should_stop(current_rotation):
                    laps = current_rotation / 360.0
                    print(f"\n🏁 Mission Complete! {laps:.2f} laps ({current_rotation:.1f}°)")
                    self.stop_continuous_steering()
                    vision_controller.stop()
                    break
                
                # Set target and calculate steering
                self.set_target_heading(target_angle)
                steering_angle, error = self.calculate_steering_angle(target_angle)
                self.servo.angle = steering_angle
                self.current_servo_angle = steering_angle
                
                # Print status every 50 frames (~1 second)
                if int(current_rotation * 10) % 500 == 0:
                    laps = abs(current_rotation) / 360.0
                    print(f"Laps: {laps:.2f} | Target: {target_angle:.1f}° | "
                          f"Actual: {current_rotation:.1f}° | Error: {error:.1f}° | "
                          f"Servo: {steering_angle:.1f}° | L/R: {left_pixels}/{right_pixels}")
                
                time.sleep(0.02)  # 50Hz update rate
                
            except Exception as e:
                print(f"⚠ Error in steering loop: {e}")
                self.stop_continuous_steering()
                vision_controller.stop()
                break
                
        print("✓ Steering loop ended")
        
    def cleanup(self):
        self.stop_motor()
        self.servo.angle = 0
        self.servo.close()
        self.pi.stop()
        print("✓ Hardware cleanup complete")

# -----------------------------
# ROI Configuration
# -----------------------------
def load_roi_config(path="roi_config.json"):
    try:
        with open(path, "r") as f:
            cfg = json.load(f)
        print("✓ Loaded ROI config from file")
        return cfg["zones"]
    except:
        print("⚠ Using default ROI config")
        return {
            "Corner_Top": 20,
            "Corner_Bottom": 40,
            "ignore_Top": 20,
            "Corner_LM": 0,
            "Corner_RM": 100,
            "Right_Wall_Top": 40,
            "Left_Wall_Top": 30,
            "Wall_Bottom": 70,
            "Right_Wall_RM": 100,
            "Right_Wall_LM": 70,
            "Left_Wall_RM": 30,
            "Left_Wall_LM": 0,
        }

# -----------------------------
# Main Control Logic
# -----------------------------
class AutonomousRobot:
    def __init__(self):
        self.vision_controller = VisionSteeringController()
        self.gyro_controller = GyroSteeringController()
        self.roi_zones = load_roi_config()
        self.steering_thread = None
        self.is_running = False
        
        # Setup button
        self.button = Button(BUTTON_PIN, pull_up=True, bounce_time=0.1)
        self.button.when_pressed = self.button_pressed
        
        print("\n" + "="*60)
        print("🤖 AUTONOMOUS WALL-FOLLOWING ROBOT")
        print("="*60)
        print(f"Button: GPIO {BUTTON_PIN}")
        print(f"Servo: GPIO {SERVO_PIN} (Range: {SERVO_MAX_LEFT}° to {SERVO_MAX_RIGHT}°)")
        print(f"Motor: IN1={IN1}, IN2={IN2}, ENA={ENA} (Speed: {DEFAULT_MOTOR_SPEED})")
        print(f"Mission: Complete 3 laps ({LAP_LIMIT}° rotation)")
        print("\n✅ Ready! Press button on GPIO 25 to start...")
        print("="*60 + "\n")
        
    def button_pressed(self):
        """Handle button press - start or stop"""
        if not self.is_running:
            self.start_mission()
        else:
            self.stop_mission()
    
    def start_mission(self):
        """Start autonomous mission"""
        if self.is_running:
            print("⚠ Mission already running!")
            return
        
        print("\n" + "🚀"*30)
        print("STARTING AUTONOMOUS MISSION")
        print("🚀"*30)
        
        # Reset controllers
        self.vision_controller.reset()
        self.gyro_controller.cumulative_rotation = 0.0
        self.gyro_controller.current_heading = 0.0
        
        # Activate vision steering
        self.vision_controller.start()
        
        # Start motor and steering loop
        self.gyro_controller.start_continuous_steering(DEFAULT_MOTOR_SPEED)
        
        self.steering_thread = threading.Thread(
            target=self.gyro_controller.steering_loop,
            args=(self.vision_controller, self.roi_zones),
            daemon=True
        )
        self.steering_thread.start()
        
        self.is_running = True
        print("✓ Mission started! Robot is now autonomous.")
        print("  Press button again to emergency stop.\n")
    
    def stop_mission(self):
        """Emergency stop"""
        if not self.is_running:
            return
        
        print("\n🛑 EMERGENCY STOP!")
        self.vision_controller.stop()
        self.gyro_controller.stop_continuous_steering()
        self.is_running = False
        print("✓ Mission stopped\n")
    
    def run(self):
        """Main run loop - just wait for button presses"""
        try:
            print("Waiting for button press...")
            pause()  # Wait indefinitely for button events
        except KeyboardInterrupt:
            print("\n\n⚠ Keyboard interrupt detected")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup before exit"""
        print("\n🔧 Shutting down...")
        self.stop_mission()
        self.gyro_controller.cleanup()
        picam2.stop()
        print("✓ Cleanup complete. Goodbye!")

# -----------------------------
# Entry Point
# -----------------------------
if __name__ == "__main__":
    try:
        robot = AutonomousRobot()
        robot.run()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
