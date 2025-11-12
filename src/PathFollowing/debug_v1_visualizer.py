from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# -----------------------------
# VISUALIZATION ONLY - NO MOVEMENT
# Shows what V1 (Basic Path Following) would do
# -----------------------------

SWITCH_PIN = 19

# Camera setup
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()
time.sleep(2)

# HSV thresholds
lower_white = np.array([0, 0, 130])
upper_white = np.array([255, 255, 255])
lower_dark = np.array([0, 0, 0])
upper_dark = np.array([255, 255, 40])

# Parameters
FRAME_CENTER_TOLERANCE = 30
ROI_HEIGHT_RATIO = 0.35
PROPORTIONAL_GAIN = 0.15
MAX_STEERING_ANGLE = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("="*60)
print("VERSION 1: BASIC PATH FOLLOWING - DEBUG MODE")
print("="*60)
print("NO MOTORS WILL MOVE - Visualization only")
print("Press 'q' to quit")
print("="*60)

try:
    while True:
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
        
        # Create visualization
        display = frame.copy()
        
        # Draw ROI boundary
        cv2.rectangle(display, (0, roi_top), (width, height), (255, 255, 0), 2)
        cv2.putText(display, "ROI (Region of Interest)", (10, roi_top - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Draw center line
        cv2.line(display, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
        cv2.putText(display, "Target", (width // 2 + 10, roi_top + 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Default values
        direction = "NO PATH"
        steering_angle = 0
        motor_speed = 180
        status_color = (0, 0, 255)
        arrow_color = (0, 0, 255)
        
        if M["m00"] > 500:
            # PATH FOUND
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Draw centroid
            cv2.circle(display, (cx, cy), 10, (0, 255, 0), -1)
            cv2.putText(display, "Path Center", (cx + 15, cy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Calculate error
            error = cx - (width // 2)
            
            # Draw error line
            cv2.line(display, (width // 2, cy), (cx, cy), (255, 0, 255), 3)
            cv2.putText(display, f"Error: {error}px", 
                       (min(cx, width//2) + 10, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            
            # Calculate steering
            steering_angle = error * PROPORTIONAL_GAIN
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Classify direction
            if error > FRAME_CENTER_TOLERANCE:
                direction = "TURN RIGHT"
                status_color = (0, 255, 255)
                arrow_color = (0, 200, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "TURN LEFT"
                status_color = (255, 255, 0)
                arrow_color = (200, 255, 0)
            else:
                direction = "GO STRAIGHT"
                status_color = (0, 255, 0)
                arrow_color = (0, 255, 0)
            
            # Speed adjustment
            if abs(steering_angle) > 40:
                motor_speed = 150
            elif abs(steering_angle) > 25:
                motor_speed = 165
            else:
                motor_speed = 180
            
            # Draw steering arrow
            arrow_length = int(abs(steering_angle) * 3)
            arrow_x = width // 2 + int(steering_angle * 5)
            cv2.arrowedLine(display, (width // 2, height - 30), 
                          (arrow_x, height - 30), arrow_color, 5, tipLength=0.3)
        
        else:
            # NO PATH
            direction = "PATH LOST!"
            status_color = (0, 0, 255)
            motor_speed = 120
            cv2.putText(display, "X NO WHITE PATH DETECTED X", 
                       (width // 4, height // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        
        # ===== COMMAND DISPLAY =====
        # Background for text
        cv2.rectangle(display, (0, 0), (width, 150), (0, 0, 0), -1)
        
        # Title
        cv2.putText(display, "V1: BASIC PATH FOLLOWING", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Direction command
        cv2.putText(display, f"COMMAND: {direction}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        
        # Steering angle
        cv2.putText(display, f"Steering: {steering_angle:.1f} degrees", (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Motor speed
        cv2.putText(display, f"Speed: {motor_speed}", (10, 125),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Algorithm info
        cv2.putText(display, "Algorithm: Proportional Control", (350, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        cv2.putText(display, f"Gain: {PROPORTIONAL_GAIN}", (350, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        
        # Show white mask in corner
        mask_display = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_display, (160, 120))
        display[height-130:height-10, width-170:width-10] = mask_small
        cv2.rectangle(display, (width-170, height-130), (width-10, height-10), (255, 255, 255), 2)
        cv2.putText(display, "White Mask", (width-165, height-135),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Terminal output
        print(f"\r{direction:15} | Steering: {steering_angle:6.1f}° | Speed: {motor_speed:3} | "
              f"Path pixels: {int(M['m00']):5}  ", end='')
        
        # Display
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imshow("V1 Debug - Basic Path Following", display_bgr)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nStopped")
finally:
    cam.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("\nVisualization complete")
