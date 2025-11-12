from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# -----------------------------
# VISUALIZATION ONLY - NO MOVEMENT
# Shows what V4 (Aggressive Racing) would do
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
FRAME_CENTER_TOLERANCE = 15
ROI_HEIGHT_RATIO = 0.45
PROPORTIONAL_GAIN = 0.18
DERIVATIVE_GAIN = 0.05
MAX_STEERING_ANGLE = 60
PATH_LOST_THRESHOLD = 250
CORNER_MEMORY_FRAMES = 10
WALL_DANGER_THRESHOLD = 1000

GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# State variables
last_error = 0
frames_since_path = 0
last_steering = 0
corner_mode = False
frames_in_corner = 0
last_direction = "STRAIGHT"

print("="*60)
print("VERSION 4: AGGRESSIVE RACING MODE - DEBUG MODE")
print("="*60)
print("NO MOTORS WILL MOVE - Visualization only")
print("Watch PD control and corner acceleration!")
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
        far_white = cv2.countNonZero(far_zone)
        mid_white = cv2.countNonZero(mid_zone)
        
        # Wall detection
        left_wall = cv2.countNonZero(roi_dark[:, :roi_w//3])
        right_wall = cv2.countNonZero(roi_dark[:, 2*roi_w//3:])
        
        # Calculate moments
        M = cv2.moments(roi)
        white_total = cv2.countNonZero(roi)
        
        # Create visualization
        display = frame.copy()
        
        # Draw ROI
        cv2.rectangle(display, (0, roi_top), (width, height), (255, 255, 0), 3)
        cv2.line(display, (width // 2, roi_top), (width // 2, height), (255, 0, 0), 2)
        
        # Default values
        direction = "LOST"
        steering_angle = 0
        motor_speed = 210
        status_color = (0, 0, 255)
        mode = "NORMAL"
        p_component = 0
        d_component = 0
        
        # Corner prediction
        corner_ahead = (far_white < 150 and mid_white > 200)
        
        # PATH FOLLOWING WITH PD CONTROL
        if M["m00"] > PATH_LOST_THRESHOLD:
            # PATH FOUND
            frames_since_path = 0
            corner_mode = False
            mode = "RACING"
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_top
            
            # Draw centroid
            cv2.circle(display, (cx, cy), 12, (0, 255, 0), -1)
            
            # Calculate error
            error = cx - (width // 2)
            derivative = error - last_error
            
            # Draw error line
            cv2.line(display, (width // 2, cy), (cx, cy), (255, 0, 255), 4)
            
            # PD Controller
            p_component = error * PROPORTIONAL_GAIN
            d_component = derivative * DERIVATIVE_GAIN
            steering_angle = p_component + d_component
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            last_error = error
            last_steering = steering_angle
            
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
            
            # CORNER STATE MACHINE
            if corner_ahead:
                motor_speed = 190
                direction = f"{direction} [APPROACHING]"
                frames_in_corner = 0
                mode = "APPROACH"
            elif abs(steering_angle) > 45:
                motor_speed = 170
                frames_in_corner += 1
                mode = "CORNERING"
            elif frames_in_corner > 0 and frames_in_corner < 5:
                motor_speed = 200
                direction = f"{direction} [ACCELERATING]"
                frames_in_corner += 1
                mode = "EXIT BOOST"
                status_color = (0, 255, 128)
            else:
                motor_speed = 210
                frames_in_corner = 0
                mode = "RACING"
            
            # Draw PD components
            # P component
            p_arrow_x = width // 2 + int(p_component * 5)
            cv2.arrowedLine(display, (width // 2, height - 120), 
                          (p_arrow_x, height - 120), (255, 200, 0), 3, tipLength=0.3)
            cv2.putText(display, f"P: {p_component:.1f}", 
                       (width // 2 - 50, height - 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)
            
            # D component
            d_arrow_x = width // 2 + int(d_component * 20)
            cv2.arrowedLine(display, (width // 2, height - 90), 
                          (d_arrow_x, height - 90), (255, 0, 200), 3, tipLength=0.3)
            cv2.putText(display, f"D: {d_component:.1f}", 
                       (width // 2 - 50, height - 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 200), 1)
            
            # Total
            total_arrow_x = width // 2 + int(steering_angle * 5)
            cv2.arrowedLine(display, (width // 2, height - 60), 
                          (total_arrow_x, height - 60), (0, 255, 0), 6, tipLength=0.3)
            cv2.putText(display, f"Total: {steering_angle:.1f}°", 
                       (width // 2 - 60, height - 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        else:
            # PATH LOST - CORNER MODE
            frames_since_path += 1
            corner_mode = True
            mode = "CORNER MEMORY"
            
            if frames_since_path < CORNER_MEMORY_FRAMES:
                direction = f"CORNER-{last_direction}"
                
                if abs(last_steering) < 30:
                    steering_angle = last_steering * 1.5
                else:
                    steering_angle = last_steering * 1.2
                
                steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
                motor_speed = 170
                status_color = (255, 0, 255)
                
                # Wall refinement
                if left_wall > WALL_DANGER_THRESHOLD:
                    steering_angle = min(steering_angle, -20)
                    direction += " [WALL!]"
                elif right_wall > WALL_DANGER_THRESHOLD:
                    steering_angle = max(steering_angle, 20)
                    direction += " [WALL!]"
                
                # Memory bar
                memory_progress = frames_since_path / CORNER_MEMORY_FRAMES
                bar_width = int(memory_progress * 300)
                cv2.rectangle(display, (170, 170), (470, 200), (50, 50, 50), -1)
                cv2.rectangle(display, (170, 170), (170 + bar_width, 200), (255, 0, 255), -1)
                cv2.putText(display, f"MEMORY: {frames_since_path}/{CORNER_MEMORY_FRAMES}", 
                           (180, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            else:
                # Emergency - use walls
                if left_wall > right_wall + 300:
                    steering_angle = -40
                    direction = "EMERGENCY RIGHT"
                elif right_wall > left_wall + 300:
                    steering_angle = 40
                    direction = "EMERGENCY LEFT"
                else:
                    steering_angle = 0
                    direction = "EMERGENCY STOP"
                    motor_speed = 0
                
                motor_speed = max(motor_speed, 120)
                status_color = (0, 0, 255)
                mode = "EMERGENCY"
        
        # ===== COMMAND DISPLAY =====
        cv2.rectangle(display, (0, 0), (width, 165), (0, 0, 0), -1)
        
        # Title with mode
        mode_color = (0, 255, 128) if mode == "EXIT BOOST" else (255, 255, 255)
        cv2.putText(display, f"V4: RACING MODE [{mode}]", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
        
        # Direction
        cv2.putText(display, f"COMMAND: {direction}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Speed prominently displayed
        speed_color = (0, 255, 128) if motor_speed >= 200 else (255, 255, 255)
        cv2.putText(display, f"SPEED: {motor_speed}", (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, speed_color, 3)
        
        # Steering
        cv2.putText(display, f"Steering: {steering_angle:.1f}°", (250, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # PD values
        pd_info = f"P:{p_component:.1f} + D:{d_component:.1f}"
        cv2.putText(display, pd_info, (10, 135),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Corner state
        corner_info = f"Corner frames: {frames_in_corner} | Lost frames: {frames_since_path}"
        cv2.putText(display, corner_info, (10, 155),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        
        # Show mask
        mask_display = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_display, (130, 100))
        display[height-110:height-10, width-145:width-15] = mask_small
        cv2.rectangle(display, (width-145, height-110), (width-15, height-10), (255, 255, 255), 2)
        cv2.putText(display, "Path", (width-140, height-115),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Terminal output
        print(f"\r{mode:15} | {direction:25} | Speed: {motor_speed:3} | "
              f"Steer: {steering_angle:6.1f}° | P:{p_component:5.1f} D:{d_component:5.1f}  ", end='')
        
        # Display
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imshow("V4 Debug - Aggressive Racing Mode", display_bgr)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\nStopped")
finally:
    cam.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("\nVisualization complete")
