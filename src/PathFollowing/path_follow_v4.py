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
# AGGRESSIVE TUNING PARAMETERS
# -----------------------------
NORMAL_SPEED = 210        # Fast on straights
CORNER_SPEED = 170        # Maintain speed in corners
APPROACH_SPEED = 190      # Slight slowdown before corners
EXIT_SPEED = 200          # Accelerate out of corners

PROPORTIONAL_GAIN = 0.18  # More aggressive steering
DERIVATIVE_GAIN = 0.05    # Predict path changes
MAX_STEERING_ANGLE = 60
FRAME_CENTER_TOLERANCE = 15
ROI_HEIGHT_RATIO = 0.45   # Look further ahead

# Advanced corner detection
PATH_LOST_THRESHOLD = 250
CORNER_MEMORY_FRAMES = 10
PATH_NARROW_RATIO = 0.35
WALL_DANGER_THRESHOLD = 1000

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
# State tracking variables
# -----------------------------
last_error = 0
frames_since_path = 0
last_steering = 0
corner_mode = False
frames_in_corner = 0
last_direction = "STRAIGHT"

# -----------------------------
# Main loop
# -----------------------------
print("Version 4: Aggressive Racing Mode")
print("⚡ HIGH SPEED - Ensure camera is properly positioned!")
print("Waiting for switch...")
while GPIO.input(SWITCH_PIN) == 1:
    time.sleep(0.1)

GPIO.output(LED_BLUE, GPIO.HIGH)
print("Switch ON - RACING!")

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
        roi_dark = mask_dark[roi_top:, :]
        roi_h, roi_w = roi.shape
        
        # Multi-zone detection
        zone_h = roi_h // 3
        far_zone = roi[:zone_h, :]
        mid_zone = roi[zone_h:2*zone_h, :]
        near_zone = roi[2*zone_h:, :]
        
        far_white = cv2.countNonZero(far_zone)
        mid_white = cv2.countNonZero(mid_zone)
        near_white = cv2.countNonZero(near_zone)
        
        # Wall detection
        left_wall = cv2.countNonZero(roi_dark[:, :roi_w//3])
        right_wall = cv2.countNonZero(roi_dark[:, 2*roi_w//3:])
        wall_bias = (right_wall - left_wall) / 1000.0
        
        # Calculate moments
        M = cv2.moments(roi)
        white_total = cv2.countNonZero(roi)
        
        # Default values
        direction = "LOST"
        steering_angle = 0
        motor_speed = NORMAL_SPEED
        color_text = (0, 0, 255)
        
        # --- CORNER PREDICTION & DETECTION ---
        corner_ahead = (far_white < 150 and mid_white > 200)
        path_narrow = False
        if white_total > 0:
            path_width = cv2.countNonZero(roi.sum(axis=0) > 0)
            path_narrow = (path_width / roi_w) < PATH_NARROW_RATIO
        
        # --- PATH FOLLOWING WITH PD CONTROL ---
        if M["m00"] > PATH_LOST_THRESHOLD:
            # PATH FOUND
            frames_since_path = 0
            corner_mode = False
            
            cx = int(M["m10"] / M["m00"])
            error = cx - (width // 2)
            
            # PD Controller: Proportional + Derivative
            derivative = error - last_error
            steering_angle = (error * PROPORTIONAL_GAIN + 
                            derivative * DERIVATIVE_GAIN)
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            last_error = error
            last_steering = steering_angle
            
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
            
            # ADAPTIVE SPEED CONTROL
            if corner_ahead or path_narrow:
                motor_speed = APPROACH_SPEED
                direction = f"{direction}-APPROACHING"
                frames_in_corner = 0
            elif abs(steering_angle) > 45:
                motor_speed = CORNER_SPEED
                frames_in_corner += 1
            elif frames_in_corner > 0 and frames_in_corner < 5:
                # Just exited corner - accelerate!
                motor_speed = EXIT_SPEED
                frames_in_corner += 1
            else:
                motor_speed = NORMAL_SPEED
                frames_in_corner = 0
            
            # Wall avoidance adjustment
            if abs(wall_bias) > 0.6:
                steering_angle += wall_bias * 15
                steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
                motor_speed = min(motor_speed, CORNER_SPEED)
            
            if not COMPETITION_MODE:
                cy = int(M["m01"] / M["m00"]) + roi_top
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
        
        else:
            # PATH LOST - CORNER MODE
            frames_since_path += 1
            corner_mode = True
            
            if frames_since_path < CORNER_MEMORY_FRAMES:
                # Continue last direction with increased angle
                direction = f"CORNER-{last_direction}"
                
                # Maintain or increase steering based on last known
                if abs(last_steering) < 30:
                    steering_angle = last_steering * 1.5
                else:
                    steering_angle = last_steering * 1.2
                
                steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
                motor_speed = CORNER_SPEED
                color_text = (255, 0, 255)
                
                # Use wall info to refine turn
                if left_wall > WALL_DANGER_THRESHOLD:
                    steering_angle = min(steering_angle, -20)  # Force right
                elif right_wall > WALL_DANGER_THRESHOLD:
                    steering_angle = max(steering_angle, 20)   # Force left
                
                if not COMPETITION_MODE:
                    print(f"🔄 Corner: {direction} | Angle: {steering_angle:.1f}° | Frame: {frames_since_path}")
            
            else:
                # Lost too long - use walls
                if left_wall > right_wall + 300:
                    steering_angle = -40
                    direction = "EMERGENCY-RIGHT"
                elif right_wall > left_wall + 300:
                    steering_angle = 40
                    direction = "EMERGENCY-LEFT"
                else:
                    steering_angle = 0
                    direction = "STOPPED"
                    stop_motor(brake=True)
                    if not COMPETITION_MODE:
                        print("❌ Lost path completely")
                
                motor_speed = CORNER_SPEED * 0.7
                color_text = (0, 0, 255)

        # Apply commands
        servo.angle = steering_angle
        run_motor(motor_speed)

        # Display
        if not COMPETITION_MODE:
            # Status text
            status = f"{direction} | S:{int(motor_speed)} | A:{steering_angle:.0f}"
            cv2.putText(frame, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_text, 2)
            
            # Visual aids
            cv2.line(frame, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
            cv2.line(frame, (0, roi_top), (width, roi_top), (255, 255, 255), 1)
            
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("V4: Racing Mode", display)
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