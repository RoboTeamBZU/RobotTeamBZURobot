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
IN1, IN2, ENA = 24, 23, 13
SERVO_PIN = 18
SWITCH_PIN = 25
LED_RED, LED_GREEN, LED_BLUE = 16, 6, 5

# -----------------------------
# TUNING PARAMETERS
# -----------------------------
NORMAL_SPEED = 240
CORNER_SPEED = 240
APPROACH_SPEED = 240
DANGER_SPEED = 200

PROPORTIONAL_GAIN = 0.13
WALL_INFLUENCE_GAIN = 0.08
MAX_STEERING_ANGLE = 55
FRAME_CENTER_TOLERANCE = 20
ROI_HEIGHT_RATIO = 0.4

# Wall detection
WALL_THRESHOLD = 800  # Dark pixels indicating wall presence

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
# Main loop
# -----------------------------
print("Version 3: Wall-Aware Multi-Zone Detection")
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
        roi_dark = mask_dark[roi_top:, :]
        roi_h, roi_w = roi.shape
        
        # --- MULTI-ZONE PATH DETECTION ---
        # Divide ROI into 3 horizontal zones
        zone_height = roi_h // 3
        far_zone = roi[:zone_height, :]          # 40-60cm ahead
        mid_zone = roi[zone_height:2*zone_height, :]  # 30-40cm
        near_zone = roi[2*zone_height:, :]       # 20-30cm
        
        far_white = cv2.countNonZero(far_zone)
        mid_white = cv2.countNonZero(mid_zone)
        near_white = cv2.countNonZero(near_zone)
        
        # --- WALL DETECTION ---
        # Split into left and right thirds
        left_wall_roi = roi_dark[:, :roi_w//3]
        right_wall_roi = roi_dark[:, 2*roi_w//3:]
        
        left_wall_pixels = cv2.countNonZero(left_wall_roi)
        right_wall_pixels = cv2.countNonZero(right_wall_roi)
        
        # Calculate wall pressure (normalized)
        wall_bias = (right_wall_pixels - left_wall_pixels) / 1000.0
        wall_bias = max(-1.0, min(1.0, wall_bias))  # Clamp to -1 to 1
        
        # Calculate moments
        M = cv2.moments(roi)
        
        # Default values
        direction = "NO PATH"
        steering_angle = 0
        motor_speed = NORMAL_SPEED
        color_text = (0, 0, 255)

        # --- CORNER PREDICTION ---
        corner_approaching = False
        if far_white < 200 and mid_white > 300:
            corner_approaching = True
            motor_speed = APPROACH_SPEED
            direction = "CORNER AHEAD"
            color_text = (255, 128, 0)  # Orange

        # --- PATH FOLLOWING ---
        if M["m00"] > 400:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Calculate path error
            error = cx - (width // 2)
            path_error = error / (width // 2)  # Normalize to -1 to 1
            
            # COMBINE path following with wall avoidance
            steering_angle = (path_error * PROPORTIONAL_GAIN * 100 + 
                            wall_bias * WALL_INFLUENCE_GAIN * 100)
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Classify direction
            if error > FRAME_CENTER_TOLERANCE:
                direction = "RIGHT"
                color_text = (0, 255, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "LEFT"
                color_text = (255, 255, 0)
            else:
                direction = "STRAIGHT"
                color_text = (0, 255, 0)
            
            # Add wall warning to direction
            if abs(wall_bias) > 0.5:
                wall_side = "L" if wall_bias < 0 else "R"
                direction += f"-WALL{wall_side}"
            
            # Speed adjustment
            if corner_approaching:
                motor_speed = APPROACH_SPEED
            elif abs(steering_angle) > 40:
                motor_speed = CORNER_SPEED
            elif abs(wall_bias) > 0.7:  # Very close to wall
                motor_speed = DANGER_SPEED
            
            if not COMPETITION_MODE:
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                print(f"Dir: {direction:15} | Steer: {steering_angle:5.1f}° | "
                      f"Wall: L{left_wall_pixels:4} R{right_wall_pixels:4} | "
                      f"Speed: {motor_speed}")
        
        else:
            # Path lost - use wall information
            if left_wall_pixels > right_wall_pixels + WALL_THRESHOLD:
                steering_angle = -35  # Turn away from left wall
                direction = "WALL-LEFT"
            elif right_wall_pixels > left_wall_pixels + WALL_THRESHOLD:
                steering_angle = 35   # Turn away from right wall
                direction = "WALL-RIGHT"
            else:
                steering_angle = 0
                direction = "SEARCHING"
            
            motor_speed = CORNER_SPEED
            color_text = (255, 0, 255)

        # Apply commands
        servo.angle = steering_angle
        run_motor(motor_speed)

        # Display
        if not COMPETITION_MODE:
            # Draw zones
            cv2.line(frame, (0, roi_top + zone_height), (width, roi_top + zone_height), (100, 100, 100), 1)
            cv2.line(frame, (0, roi_top + 2*zone_height), (width, roi_top + 2*zone_height), (100, 100, 100), 1)
            
            # Draw wall detection zones
            cv2.line(frame, (roi_w//3, roi_top), (roi_w//3, height), (128, 128, 128), 1)
            cv2.line(frame, (2*roi_w//3, roi_top), (2*roi_w//3, height), (128, 128, 128), 1)
            
            cv2.line(frame, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
            cv2.line(frame, (0, roi_top), (width, roi_top), (255, 255, 255), 1)
            cv2.putText(frame, f"{direction}", (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_text, 2)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("V3: Wall-Aware", display)
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