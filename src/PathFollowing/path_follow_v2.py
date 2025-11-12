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
NORMAL_SPEED = 190
CORNER_SPEED = 140
APPROACH_SPEED = 160

PROPORTIONAL_GAIN = 0.12
MAX_STEERING_ANGLE = 55
FRAME_CENTER_TOLERANCE = 20
ROI_HEIGHT_RATIO = 0.4

# Corner detection thresholds
PATH_LOST_THRESHOLD = 300  # Minimum white pixels to consider "path found"
CORNER_MEMORY_FRAMES = 8   # How many frames to remember direction
PATH_NARROW_THRESHOLD = 0.4  # Path width ratio for corner warning

# HSV thresholds
lower_white = np.array([0, 0, 130])
upper_white = np.array([255, 255, 255])
lower_dark = np.array([0, 0, 0])
upper_dark = np.array([255, 255, 40])

COMPETITION_MODE = True

# -----------------------------
# Camera setup
# -----------------------------
cam = Picamera2()
cam.configure(cam.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
))
cam.start()
time.sleep(2)

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
# Corner memory variables
# -----------------------------
frames_since_path = 0
last_known_steering = 0
last_direction = "STRAIGHT"

# -----------------------------
# Main loop
# -----------------------------
print("Version 2: Corner Detection with Memory")
print("Waiting for switch...")
while GPIO.input(SWITCH_PIN) == 1:
    time.sleep(0.1)

GPIO.output(LED_BLUE, GPIO.HIGH)
print("Switch ON - running")

try:
    while True:
        # Check switch
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
        mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)

        # Process ROI
        roi_top = int(height * (1 - ROI_HEIGHT_RATIO))
        roi = mask_white[roi_top:, :]
        roi_h, roi_w = roi.shape
        M = cv2.moments(roi)
        white_pixel_count = cv2.countNonZero(roi)

        # Default values
        direction = "NO PATH"
        steering_angle = 0
        motor_speed = NORMAL_SPEED
        color_text = (0, 0, 255)

        # CORNER DETECTION LOGIC
        if M["m00"] < PATH_LOST_THRESHOLD:
            # PATH LOST - likely in a corner
            frames_since_path += 1
            
            if frames_since_path < CORNER_MEMORY_FRAMES:
                # Continue last known turn direction (MEMORY MODE)
                direction = f"CORNER-{last_direction}"
                steering_angle = last_known_steering * 1.2  # Increase turn slightly
                steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
                motor_speed = CORNER_SPEED
                color_text = (255, 0, 255)  # Purple for corner mode
                
                if not COMPETITION_MODE:
                    print(f"🔄 Corner mode: {direction} | Angle: {steering_angle:.1f}°")
            else:
                # Lost for too long - STOP
                direction = "LOST"
                stop_motor(brake=True)
                steering_angle = 0
                motor_speed = 0
                if not COMPETITION_MODE:
                    print("❌ Path lost too long - STOPPED")

        else:
            # PATH FOUND - normal operation
            frames_since_path = 0
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Calculate error from center
            error = cx - (width // 2)
            
            # Check path width for approaching corner
            path_width_pixels = cv2.countNonZero(roi.sum(axis=0) > 0)
            path_width_ratio = path_width_pixels / roi_w
            
            # Proportional steering
            steering_angle = error * PROPORTIONAL_GAIN
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Save this steering for corner memory
            if abs(steering_angle) > 10:
                last_known_steering = steering_angle
            
            # Classify direction
            if error > FRAME_CENTER_TOLERANCE:
                direction = "RIGHT"
                last_direction = "RIGHT"
                color_text = (0, 255, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "LEFT"
                last_direction = "LEFT"
                color_text = (255, 255, 0)
            else:
                direction = "STRAIGHT"
                last_direction = "STRAIGHT"
                color_text = (0, 255, 0)
            
            # Speed adjustment based on path width (corner warning)
            if path_width_ratio < PATH_NARROW_THRESHOLD:
                motor_speed = APPROACH_SPEED
                direction += "-NARROW"
                if not COMPETITION_MODE:
                    print(f"⚠️ Approaching corner (width: {path_width_ratio:.2f})")
            elif abs(steering_angle) > 40:
                motor_speed = CORNER_SPEED
            elif abs(steering_angle) > 25:
                motor_speed = APPROACH_SPEED
            
            if not COMPETITION_MODE:
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

        # Apply commands
        servo.angle = steering_angle
        run_motor(motor_speed)

        # Display
        if not COMPETITION_MODE:
            cv2.line(frame, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
            cv2.line(frame, (0, roi_top), (width, roi_top), (255, 255, 255), 1)
            cv2.putText(frame, f"{direction}", (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_text, 2)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("V2: Corner Memory", display)
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
