"""
Fisheye Camera V3 Debug - Wall-Aware Multi-Zone Detection
NO MOTORS - Visualization only for testing camera angle and ROI
"""

from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# -----------------------------
# Camera Configuration
# -----------------------------
SWITCH_PIN = 19

# Fisheye calibration (replace with YOUR calibration!)
K = np.array([[280.0, 0.0, 320.0],
              [0.0, 280.0, 240.0],
              [0.0, 0.0, 1.0]])

D = np.array([[-0.15], [0.02], [0.0], [0.0]])

# Try to load your calibration
try:
    data = np.load('fisheye_calibration.npz')
    K = data['K']
    D = data['D']
    print("✓ Loaded fisheye calibration")
except:
    print("⚠️  Using default calibration - CALIBRATE YOUR CAMERA!")

# -----------------------------
# ADJUSTABLE PARAMETERS - TUNE THESE!
# -----------------------------
ROI_TOP_OFFSET = 0.45        # 0.0=bottom, 1.0=top (ADJUST THIS to move ROI up/down)
ROI_HEIGHT_RATIO = 0.4       # How tall the ROI is
ROI_WIDTH_CROP = 0.15        # Crop edges (fisheye distortion)
ZOOM_LEVEL = 0.7             # 0.6-0.8 recommended

# Detection parameters
PROPORTIONAL_GAIN = 0.13
WALL_INFLUENCE_GAIN = 0.08
MAX_STEERING_ANGLE = 40
FRAME_CENTER_TOLERANCE = 20
WALL_THRESHOLD = 800

# HSV thresholds
lower_white = np.array([0, 0, 130])
upper_white = np.array([255, 255, 255])
lower_dark = np.array([0, 0, 0])
upper_dark = np.array([255, 255, 40])

# Display settings
TEXT_OVERLAY = False  # Set True to show text on video (blocks view)
SHOW_ZONES = True     # Show zone lines
SHOW_MASKS = True     # Show white/dark masks in corners

# -----------------------------
# Fisheye Undistortion
# -----------------------------
class FisheyeUndistorter:
    def __init__(self, K, D, image_size=(640, 480), zoom=0.7):
        self.K_new = K.copy()
        self.K_new[0, 0] *= zoom
        self.K_new[1, 1] *= zoom
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), self.K_new, image_size, cv2.CV_16SC2
        )
    
    def undistort(self, image):
        return cv2.remap(image, self.map1, self.map2, 
                        cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

# -----------------------------
# Camera Setup
# -----------------------------
cam = Picamera2()
cam.configure(cam.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
cam.start()
time.sleep(2)

undistorter = FisheyeUndistorter(K, D, image_size=(640, 480), zoom=ZOOM_LEVEL)

GPIO.setmode(GPIO.BCM)
GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("="*70)
print("FISHEYE V3 DEBUG - Wall-Aware Multi-Zone")
print("="*70)
print(f"ROI Position: {ROI_TOP_OFFSET:.2f} (0=bottom, 1=top)")
print("Adjust ROI_TOP_OFFSET in code to move ROI up/down")
print("Press 'q' to quit")
print("="*70)

try:
    while True:
        # Capture and undistort
        frame_raw = cam.capture_array()
        if frame_raw.ndim == 3 and frame_raw.shape[2] == 4:
            frame_raw = frame_raw[:, :, :3]
        
        frame = undistorter.undistort(frame_raw)
        
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)

        # Calculate ROI position (moved up by offset)
        roi_height_pixels = int(height * ROI_HEIGHT_RATIO)
        roi_top = int(height * ROI_TOP_OFFSET)
        roi_bottom = roi_top + roi_height_pixels
        
        # Clamp to frame boundaries
        roi_top = max(0, min(roi_top, height - 10))
        roi_bottom = min(height, max(roi_bottom, roi_top + 10))
        
        # Crop sides (fisheye edges)
        crop_left = int(width * ROI_WIDTH_CROP)
        crop_right = width - crop_left
        
        # Extract ROI
        roi = mask_white[roi_top:roi_bottom, crop_left:crop_right]
        roi_dark = mask_dark[roi_top:roi_bottom, crop_left:crop_right]
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
        wall_bias = max(-1.0, min(1.0, wall_bias))
        
        # Path detection
        M = cv2.moments(roi)
        
        # Create display
        display = frame.copy()
        
        # Draw ROI boundary (BRIGHT for visibility)
        cv2.rectangle(display, (crop_left, roi_top), 
                     (crop_right, roi_bottom), (0, 255, 255), 3)  # Cyan, thick
        
        if SHOW_ZONES:
            # Draw zone lines
            zone1_y = roi_top + zone_h
            zone2_y = roi_top + 2*zone_h
            cv2.line(display, (crop_left, zone1_y), (crop_right, zone1_y), (100, 255, 100), 2)
            cv2.line(display, (crop_left, zone2_y), (crop_right, zone2_y), (100, 255, 100), 2)
            
            # Zone labels (small, top-left)
            cv2.putText(display, "FAR", (crop_left + 5, roi_top + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
            cv2.putText(display, "MID", (crop_left + 5, zone1_y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
            cv2.putText(display, "NEAR", (crop_left + 5, zone2_y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
            
            # Wall zones
            left_x = crop_left + roi_w // 3
            right_x = crop_left + 2 * roi_w // 3
            cv2.line(display, (left_x, roi_top), (left_x, roi_bottom), (128, 128, 255), 2)
            cv2.line(display, (right_x, roi_top), (right_x, roi_bottom), (128, 128, 255), 2)
        
        # Center line
        cv2.line(display, (width // 2, roi_top), (width // 2, roi_bottom), (255, 0, 0), 2)
        
        # Corner prediction
        corner_ahead = (far_white < 200 and mid_white > 300)
        
        direction = "NO PATH"
        steering_angle = 0
        status_color = (0, 0, 255)
        
        if M["m00"] > 400:
            # PATH FOUND
            cx_roi = int(M["m10"] / M["m00"])
            cy_roi = int(M["m01"] / M["m00"])
            cx = cx_roi + crop_left
            cy = cy_roi + roi_top
            
            # Draw centroid
            cv2.circle(display, (cx, cy), 10, (0, 255, 0), -1)
            
            # Calculate error
            error = cx_roi - (roi_w // 2)
            path_error = error / (roi_w // 2)
            
            # Combined steering
            path_component = path_error * PROPORTIONAL_GAIN * 100
            wall_component = wall_bias * WALL_INFLUENCE_GAIN * 100
            steering_angle = path_component + wall_component
            steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))
            
            # Direction
            if error > FRAME_CENTER_TOLERANCE:
                direction = "RIGHT"
                status_color = (0, 255, 255)
            elif error < -FRAME_CENTER_TOLERANCE:
                direction = "LEFT"
                status_color = (255, 255, 0)
            else:
                direction = "STRAIGHT"
                status_color = (0, 255, 0)
            
            if abs(wall_bias) > 0.5:
                wall_side = "L" if wall_bias < 0 else "R"
                direction += f"-W{wall_side}"
            
            if corner_ahead:
                direction += " CORNER!"
                cv2.putText(display, "CORNER AHEAD", (width//2 - 80, roi_top - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)
        else:
            # PATH LOST
            if left_wall > right_wall + WALL_THRESHOLD:
                steering_angle = -35
                direction = "WALL-LEFT"
            elif right_wall > left_wall + WALL_THRESHOLD:
                steering_angle = 35
                direction = "WALL-RIGHT"
            else:
                direction = "LOST"
            status_color = (255, 0, 255)
        
        # Text overlay (optional, moved to edges)
        if TEXT_OVERLAY:
            # Background box at top
            cv2.rectangle(display, (0, 0), (width, 100), (0, 0, 0), -1)
            cv2.putText(display, f"V3: {direction}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.putText(display, f"Steer: {steering_angle:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(display, f"Zones: F{far_white} M{mid_white} N{near_white}", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        
        # Show masks in bottom corners (small)
        if SHOW_MASKS:
            mask_white_display = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
            mask_dark_display = cv2.cvtColor(mask_dark, cv2.COLOR_GRAY2BGR)
            mask_white_small = cv2.resize(mask_white_display, (120, 90))
            mask_dark_small = cv2.resize(mask_dark_display, (120, 90))
            
            # Bottom right - white mask
            display[height-100:height-10, width-130:width-10] = mask_white_small
            cv2.rectangle(display, (width-130, height-100), (width-10, height-10), (255, 255, 255), 1)
            cv2.putText(display, "Path", (width-125, height-105),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
            # Bottom left - dark mask
            display[height-100:height-10, 10:130] = mask_dark_small
            cv2.rectangle(display, (10, height-100), (130, height-10), (255, 255, 255), 1)
            cv2.putText(display, "Walls", (15, height-105),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        
        # Terminal output (doesn't block view)
        print(f"\r{direction:20} | Steer:{steering_angle:5.1f}° | "
              f"Zones F:{far_white:4} M:{mid_white:4} N:{near_white:4} | "
              f"Walls L:{left_wall:4} R:{right_wall:4}  ", end='')
        
        # Display
        display_bgr = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
        cv2.imshow("Fisheye V3 Debug - Adjust ROI", display_bgr)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\n\nStopped")
finally:
    cam.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("\nVisualization complete")
