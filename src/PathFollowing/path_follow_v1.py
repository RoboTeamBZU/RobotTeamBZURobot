from picamera2 import Picamera2
import cv2
import numpy as np
import time
import pigpio
import RPi.GPIO as GPIO
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# -----------------------------
# Pins
# -----------------------------
IN1, IN2, ENA = 16, 20, 12
SERVO_PIN = 2
SWITCH_PIN = 19
LED_RED, LED_GREEN, LED_BLUE = 13, 6, 5

# -----------------------------
# TUNING PARAMETERS
# -----------------------------
NORMAL_SPEED = 180
CORNER_SPEED = 150
SLOW_SPEED = 120

PROPORTIONAL_GAIN = 0.15  # How aggressively to steer (lower = smoother)
MAX_STEERING_ANGLE = 50

FRAME_CENTER_TOLERANCE = 20
ROI_HEIGHT_RATIO = 0.35  # Optimized for speed

# HSV thresholds
lower_white = np.array([0, 0, 130])
upper_white = np.array([255, 255, 255])
lower_dark = np.array([0, 0, 0])
upper_dark = np.array([255, 255, 40])

# Competition mode - disable display for speed
COMPETITION_MODE = True  # Set to False for debugging with display

# -----------------------------
# Camera setup - OPTIMIZED resolution
# -----------------------------
cam = Picamera2()
cam.configure(cam.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}  # Smaller = faster
))
cam.start()
time.sleep(2)  # Camera warmup

# -----------------------------
# Setup GPIO + pigpio + servo
# -----------------------------
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running")

GPIO.setmode(GPIO.BCM)
for p in (IN1, IN2, ENA): 
    pi.set_mode(p, pigpio.OUTPUT)
pi.set_PWM_frequency(ENA, 2000)

GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)
GPIO.setup(LED_BLUE, GPIO.OUT)
GPIO.output(LED_RED, GPIO.LOW)
GPIO.output(LED_GREEN, GPIO.LOW)
GPIO.output(LED_BLUE, GPIO.LOW)

Device.pin_factory = PiGPIOFactory()
servo = AngularServo(SERVO_PIN, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo.angle = 0

# -----------------------------
# Motor functions
# -----------------------------
def run_motor(value):
    forward = value >= 0
    pi.write(IN1, 1 if forward else 0)
    pi.write(IN2, 0 if forward else 1)
    pi.set_PWM_dutycycle(ENA, min(255, abs(int(value))))

def stop_motor(brake=False):
    pi.set_PWM_dutycycle(ENA, 0)
    pi.write(IN1, 1 if brake else 0)
    pi.write(IN2, 1 if brake else 0)

# -----------------------------
# Main loop
# -----------------------------
print("Version 1: Basic Path Following")
print("Waiting for switch...")
while GPIO.input(SWITCH_PIN) == 1:
    time.sleep(0.1)

GPIO.output(LED_BLUE, GPIO.HIGH)
print("Switch ON - running")

try:
    while True:
        # Check switch - can pause mid-run
        if GPIO.input(SWITCH_PIN) == 1:
            stop_motor()
            servo.angle = 0
            GPIO.output(LED_BLUE, GPIO.LOW)
            time.sleep(0.05)
            continue
        else:
            GPIO.output(LED_BLUE, GPIO.HIGH)

        # Capture frame
        frame = cam.capture_array()
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]

        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Process ROI
        roi_top = int(height * (1 - ROI_HEIGHT_RATIO))
        roi = mask_white[roi_top:, :]
        M = cv2.moments(roi)

        # Default values
        direction = "NO PATH"
        steering_angle = 0
        motor_speed = NORMAL_SPEED
        color_text = (0, 0, 255)

        if M["m00"] > 500:  # Path found
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Calculate error from center
            error = cx - (width // 2)
            
            # Proportional steering
            steering_angle = error * PROPORTIONAL_GAIN
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Classify direction for display
            if error > FRAME_CENTER_TOLERANCE:
                direction = "RIGHT"
                color_text = (0, 255, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "LEFT"
                color_text = (255, 255, 0)
            else:
                direction = "STRAIGHT"
                color_text = (0, 255, 0)
            
            # Speed adjustment based on steering
            if abs(steering_angle) > 40:
                motor_speed = CORNER_SPEED
            elif abs(steering_angle) > 25:
                motor_speed = (NORMAL_SPEED + CORNER_SPEED) // 2
            
            if not COMPETITION_MODE:
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                print(f"Dir: {direction:8} | Angle: {steering_angle:5.1f}° | Speed: {motor_speed}")
        
        else:
            # Path lost - this shouldn't happen often with good camera placement
            direction = "LOST"
            steering_angle = 0
            motor_speed = SLOW_SPEED
            if not COMPETITION_MODE:
                print("⚠️ Path lost - check camera angle")

        # Apply commands
        servo.angle = steering_angle
        run_motor(motor_speed)

        # Display (only if not in competition mode)
        if not COMPETITION_MODE:
            cv2.line(frame, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
            cv2.line(frame, (0, roi_top), (width, roi_top), (255, 255, 255), 1)
            cv2.putText(frame, f"{direction}", (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_text, 2)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("V1: Basic", display)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    cam.stop()
    stop_motor(brake=True)
    servo.angle = 0
    GPIO.output(LED_BLUE, GPIO.LOW)
    pi.stop()
    GPIO.cleanup()
    if not COMPETITION_MODE:
        cv2.destroyAllWindows()
