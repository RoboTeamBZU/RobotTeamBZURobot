import cv2
import numpy as np
from picamera2 import Picamera2
import time

class WRONavigator:
    """
    Navigation system for WRO Future Engineers square track
    Uses camera vision to detect walls, lines, and navigate corners
    """
    
    def __init__(self, debug=True):
        self.debug = debug
        
        # Initialize camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)  # Camera warm-up
        
        # Navigation parameters
        self.frame_width = 640
        self.frame_height = 480
        
        # Define ROI (Region of Interest) zones
        # Bottom zone for immediate steering, middle for lookahead
        self.steering_zone = (int(self.frame_height * 0.7), self.frame_height)
        self.lookahead_zone = (int(self.frame_height * 0.4), int(self.frame_height * 0.7))
        
        # Color ranges in HSV
        self.color_ranges = {
            'blue': ([100, 100, 50], [130, 255, 255]),    # Blue corner line
            'orange': ([5, 100, 100], [25, 255, 255]),    # Orange corner line
            'black_wall': ([0, 0, 0], [180, 255, 60])     # Black walls
        }
        
        # State tracking
        self.in_corner = False
        self.corner_direction = None  # 'left' or 'right'
        
    def undistort_fisheye(self, frame):
        """
        Remove fisheye distortion (optional, adjust parameters for your lens)
        """
        # For now, return original - you can calibrate this later
        return frame
    
    def detect_walls(self, frame, zone):
        """
        Detect black walls on left and right sides
        Returns: (left_wall_x, right_wall_x, center_offset)
        """
        y_start, y_end = zone
        roi = frame[y_start:y_end, :]
        
        # Convert to HSV and detect black
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        black_mask = cv2.inRange(hsv, np.array(self.color_ranges['black_wall'][0]), 
                                      np.array(self.color_ranges['black_wall'][1]))
        
        # Find wall edges
        height, width = black_mask.shape
        left_wall_x = None
        right_wall_x = None
        
        # Scan from left
        for x in range(0, width // 3):
            if np.sum(black_mask[:, x]) > height * 0.3 * 255:
                left_wall_x = x
                break
        
        # Scan from right
        for x in range(width - 1, 2 * width // 3, -1):
            if np.sum(black_mask[:, x]) > height * 0.3 * 255:
                right_wall_x = x
                break
        
        # Calculate center offset
        if left_wall_x is not None and right_wall_x is not None:
            track_center = (left_wall_x + right_wall_x) // 2
            frame_center = width // 2
            center_offset = track_center - frame_center
        else:
            center_offset = 0
        
        return left_wall_x, right_wall_x, center_offset
    
    def detect_center_line(self, frame, zone):
        """
        Detect dotted center line for straight sections
        Returns: line center x position
        """
        y_start, y_end = zone
        roi = frame[y_start:y_end, :]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Detect darker spots (dotted line)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get centroids of dots
            centers = []
            for cnt in contours:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    centers.append(cx)
            
            if centers:
                return int(np.mean(centers))
        
        return None
    
    def detect_corner_lines(self, frame):
        """
        Detect blue and orange corner lines
        Returns: ('left', 'right', or None) and confidence
        """
        # Focus on middle-bottom area
        roi = frame[self.lookahead_zone[0]:self.steering_zone[1], :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Detect blue (left turn indicator)
        blue_mask = cv2.inRange(hsv, np.array(self.color_ranges['blue'][0]), 
                                     np.array(self.color_ranges['blue'][1]))
        blue_pixels = cv2.countNonZero(blue_mask)
        
        # Detect orange (right turn indicator)
        orange_mask = cv2.inRange(hsv, np.array(self.color_ranges['orange'][0]), 
                                       np.array(self.color_ranges['orange'][1]))
        orange_pixels = cv2.countNonZero(orange_mask)
        
        # Threshold for detection
        threshold = 500
        
        if blue_pixels > threshold and blue_pixels > orange_pixels:
            return 'left', blue_pixels
        elif orange_pixels > threshold and orange_pixels > blue_pixels:
            return 'right', orange_pixels
        
        return None, 0
    
    def calculate_steering(self, frame):
        """
        Main navigation logic - calculate steering angle
        Returns: steering_angle (-100 to 100), speed (0-100), state
        """
        # Detect walls
        left_wall, right_wall, wall_offset = self.detect_walls(frame, self.steering_zone)
        
        # Detect corner lines
        corner_dir, corner_confidence = self.detect_corner_lines(frame)
        
        # Detect center line
        center_line_x = self.detect_center_line(frame, self.steering_zone)
        
        # State machine for navigation
        if corner_dir and corner_confidence > 1000:
            # Corner detected - initiate turn
            self.in_corner = True
            self.corner_direction = corner_dir
            
            if corner_dir == 'left':
                steering = -60  # Turn left
                speed = 40
                state = "CORNER_LEFT"
            else:
                steering = 60   # Turn right
                speed = 40
                state = "CORNER_RIGHT"
        
        elif self.in_corner:
            # Continue through corner
            if self.corner_direction == 'left':
                steering = -50
            else:
                steering = 50
            speed = 45
            state = f"IN_CORNER_{self.corner_direction.upper()}"
            
            # Exit corner when we see straight ahead again
            if abs(wall_offset) < 50 and center_line_x:
                self.in_corner = False
                state = "EXITING_CORNER"
        
        else:
            # Straight section - follow center line or walls
            if center_line_x:
                # Follow center line
                frame_center = self.frame_width // 2
                offset = center_line_x - frame_center
                steering = int(offset * 0.3)  # P-controller
                speed = 60
                state = "FOLLOWING_LINE"
            else:
                # Follow walls (keep centered)
                steering = int(wall_offset * 0.25)
                speed = 55
                state = "WALL_FOLLOWING"
        
        # Clamp values
        steering = max(-100, min(100, steering))
        speed = max(0, min(100, speed))
        
        return steering, speed, state, {
            'left_wall': left_wall,
            'right_wall': right_wall,
            'wall_offset': wall_offset,
            'center_line': center_line_x,
            'corner': corner_dir
        }
    
    def draw_debug_info(self, frame, steering, speed, state, debug_data):
        """
        Draw debug visualization on frame
        """
        debug_frame = frame.copy()
        h, w = debug_frame.shape[:2]
        
        # Draw ROI zones
        cv2.rectangle(debug_frame, (0, self.steering_zone[0]), 
                     (w, self.steering_zone[1]), (0, 255, 0), 2)
        cv2.rectangle(debug_frame, (0, self.lookahead_zone[0]), 
                     (w, self.lookahead_zone[1]), (255, 0, 0), 2)
        
        # Draw center line
        cv2.line(debug_frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
        
        # Draw detected center line
        if debug_data['center_line']:
            cx = debug_data['center_line']
            cv2.line(debug_frame, (cx, self.steering_zone[0]), 
                    (cx, self.steering_zone[1]), (0, 255, 255), 3)
        
        # Draw wall positions
        if debug_data['left_wall']:
            cv2.line(debug_frame, (debug_data['left_wall'], self.steering_zone[0]),
                    (debug_data['left_wall'], self.steering_zone[1]), (255, 0, 255), 2)
        if debug_data['right_wall']:
            cv2.line(debug_frame, (debug_data['right_wall'], self.steering_zone[0]),
                    (debug_data['right_wall'], self.steering_zone[1]), (255, 0, 255), 2)
        
        # Draw info text
        info_y = 30
        cv2.putText(debug_frame, f"State: {state}", (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Steering: {steering}", (10, info_y + 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_frame, f"Speed: {speed}", (10, info_y + 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw steering indicator
        center_x = w // 2
        steer_x = center_x + int(steering * 2)
        cv2.arrowedLine(debug_frame, (center_x, h - 50), 
                       (steer_x, h - 50), (0, 0, 255), 3, tipLength=0.3)
        
        return debug_frame
    
    def run(self):
        """
        Main navigation loop
        """
        print("Starting WRO Navigation System...")
        print("Press 'q' to quit, 's' to save frame")
        
        try:
            while True:
                # Capture frame
                frame = self.picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                # Calculate navigation
                steering, speed, state, debug_data = self.calculate_steering(frame)
                
                # TODO: Send steering and speed to your motor controller
                # Example: motor_controller.set_steering(steering)
                #          motor_controller.set_speed(speed)
                
                # Debug visualization
                if self.debug:
                    debug_frame = self.draw_debug_info(frame, steering, speed, state, debug_data)
                    cv2.imshow('WRO Navigation', debug_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('s'):
                        cv2.imwrite(f'debug_frame_{int(time.time())}.jpg', debug_frame)
                        print("Frame saved!")
                
                # Print telemetry
                print(f"\r{state} | Steering: {steering:3d} | Speed: {speed:3d}", end='')
                
        finally:
            self.picam2.stop()
            cv2.destroyAllWindows()
            print("\nNavigation stopped.")

# Main execution
if __name__ == "__main__":
    navigator = WRONavigator(debug=True)
    navigator.run()
