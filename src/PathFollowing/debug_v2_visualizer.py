from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# -----------------------------
# VISUALIZATION ONLY - NO MOVEMENT
# Shows what V2 (Corner Memory) would do
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

# Parameters
FRAME_CENTER_TOLERANCE = 20
ROI_HEIGHT_RATIO = 0.4
PROPORTIONAL_GAIN = 0.12
MAX_STEERING_ANGLE = 55
PATH_LOST_THRESHOLD = 300
CORNER_MEMORY_FRAMES = 8
PATH_NARROW_THRESHOLD = 0.4

GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Memory variables
frames_since_path = 0
last_known_steering = 0
last_direction = "STRAIGHT"

print("="*60)
print("VERSION 2: CORNER MEMORY - DEBUG MODE")
print("="*60)
print("NO MOTORS WILL MOVE - Visualization only")
print("Watch how memory system handles corners!")
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
        roi_h, roi_w = roi.shape
        M = cv2.moments(roi)
        white_pixel_count = cv2.countNonZero(roi)
        
        # Create visualization
        display = frame.copy()
        
        # Draw ROI
        cv2.rectangle(display, (0, roi_top), (width, height), (255, 255, 0), 2)
        cv2.line(display, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
        
        # Default values
        direction = "NO PATH"
        steering_angle = 0
        motor_speed = 190
        status_color = (0, 0, 255)
        mode = "NORMAL"
        
        # CORNER DETECTION LOGIC
        if M["m00"] < PATH_LOST_THRESHOLD:
            # PATH LOST
            frames_since_path += 1
            mode = "CORNER MEMORY"
            
            if frames_since_path < CORNER_MEMORY_FRAMES:
                # Use memory
                direction = f"CORNER-{last_direction}"
                steering_angle = last_known_steering * 1.2
                steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
                motor_speed = 140
                status_color = (255, 0, 255)  # Purple
                
                # Draw memory indicator
                memory_bar_width = int((frames_since_path / CORNER_MEMORY_FRAMES) * 300)
                cv2.rectangle(display, (170, 130), (470, 160), (100, 100, 100), -1)
                cv2.rectangle(display, (170, 130), (170 + memory_bar_width, 160), (255, 0, 255), -1)
                cv2.putText(display, f"MEMORY: {frames_since_path}/{CORNER_MEMORY_FRAMES}", 
                           (180, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
            else:
                # Lost too long
                direction = "LOST - STOPPED"
                steering_angle = 0
                motor_speed = 0
                status_color = (0, 0, 255)
                mode = "EMERGENCY STOP"
        
        else:
            # PATH FOUND
            frames_since_path = 0
            mode = "NORMAL"
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Draw centroid
            cv2.circle(display, (cx, cy), 10, (0, 255, 0), -1)
            
            # Calculate error
            error = cx - (width // 2)
            cv2.line(display, (width // 2, cy), (cx, cy), (255, 0, 255), 3)
            
            # Check path width
            path_width_pixels = cv2.countNonZero(roi.sum(axis=0) > 0)
            path_width_ratio = path_width_pixels / roi_w
            
            # Draw path width indicator
            cv2.rectangle(display, (10, height - 50), (10 + int(path_width_ratio * 300), height - 30), 
                         (0, 255, 0) if path_width_ratio > PATH_NARROW_THRESHOLD else (255, 165, 0), -1)
            cv2.putText(display, f"Path Width: {path_width_ratio:.2f}", (10, height - 55),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            if path_width_ratio < PATH_NARROW_THRESHOLD:
                cv2.putText(display, "NARROW - CORNER AHEAD!", (320, height - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
            
            # Calculate steering
            steering_angle = error * PROPORTIONAL_GAIN
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Save to memory
            if abs(steering_angle) > 10:
                last_known_steering = steering_angle
            
            # Classify direction
            if error > FRAME_CENTER_TOLERANCE:
                direction = "TURN RIGHT"
                last_direction = "RIGHT"
                status_color = (0, 255, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "TURN LEFT"
                last_direction = "LEFT"
                status_color = (255, 255, 0)
            else:
                direction = "GO STRAIGHT"
                last_direction = "STRAIGHT"
                status_color = (0, 255, 0)
            
            # Speed adjustment
            if path_width_ratio < PATH_NARROW_THRESHOLD:
                motor_speed = 160
                direction += " (SLOWING)"
            elif abs(steering_angle) > 40:
                motor_speed = 140
            elif abs(steering_angle) > 25:
                motor_speed = 165
            
            # Draw steering arrow
            arrow_x = width // 2 + int(steering_angle * 5)
            cv2.arrowedLine(display, (width // 2, height - 80), 
                          (arrow_x, height - 80), status_color, 5, tipLength=0.3)
        
        # ===== COMMAND DISPLAY =====
        cv2.rectangle(display, (0, 0), (width, 120), (0, 0, 0), -1)
        
        # Title with mode
        cv2.putText(display, f"V2: CORNER MEMORY [{mode}]", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Direction
        cv2.putText(display, f"COMMAND: {direction}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        # Details
        cv2.putText(display, f"Steering: {steering_angle:.1f}° | Speed: {motor_speed}", (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Memory state
        memory_info = f"Last: {last_direction} @ {last_known_steering:.1f}°"
        cv2.putText(display, memory_info, (350, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        
        # Show mask
        mask_display = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_display, (160, 120))
        display[height-130:height-10, width-170:width-10] = mask_small
        cv2.rectangle(display, (width-170, height-130), (width-10, height-10), (255, 255, 255), 2)
        
        # Terminal output
        print(f"\r{mode:15} | {direction:20} | Steer: {steering_angle:6.1f}° | "
              f"Speed: {motor_speed:3} | Memory: {frames_since_path}/{CORNER_MEMORY_FRAMES}  ", end='')
        
        # Display
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imshow("V2 Debug - Corner Memory", display_bgr)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nStopped")
finally:
    cam.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("\nVisualization complete")
