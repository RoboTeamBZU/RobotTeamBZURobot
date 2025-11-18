import cv2
import numpy as np
from picamera2 import Picamera2
import json
import time

class ROIConfigurator:
    """
    Interactive tool to configure Region of Interest zones for WRO navigation
    Allows visual adjustment of detection zones with live camera feed
    """
    
    def __init__(self):
        # Initialize camera
        print("Initializing camera...")
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)
        
        self.frame_width = 640
        self.frame_height = 480
        
        # ROI zones (in percentages, then convert to pixels)
        self.zones = {
            'steering_top': 70,      # Top of steering zone (% from top)
            'steering_bottom': 100,  # Bottom of steering zone
            'lookahead_top': 40,     # Top of lookahead zone
            'lookahead_bottom': 70,  # Bottom of lookahead zone
            'ignore_top': 20,        # Ignore top portion (sky/background)
            'left_margin': 10,       # Left edge margin (%)
            'right_margin': 90       # Right edge margin (%)
        }
        
        # Color detection ranges (HSV)
        self.color_ranges = {
            'blue_h_low': 100,
            'blue_h_high': 130,
            'blue_s_low': 100,
            'blue_s_high': 255,
            'blue_v_low': 50,
            'blue_v_high': 255,
            
            'orange_h_low': 5,
            'orange_h_high': 25,
            'orange_s_low': 100,
            'orange_s_high': 255,
            'orange_v_low': 100,
            'orange_v_high': 255,
            
            'black_h_low': 0,
            'black_h_high': 180,
            'black_s_low': 0,
            'black_s_high': 255,
            'black_v_low': 0,
            'black_v_high': 60
        }
        
        # Current mode
        self.mode = 'zones'  # 'zones', 'blue', 'orange', 'black'
        self.show_masks = False
        self.paused = False
        self.current_frame = None
        
        print("ROI Configurator initialized!")
        print("\nControls:")
        print("  1 - Adjust ROI Zones")
        print("  2 - Adjust Blue detection")
        print("  3 - Adjust Orange detection")
        print("  4 - Adjust Black wall detection")
        print("  m - Toggle mask view")
        print("  p - Pause/Unpause")
        print("  s - Save configuration")
        print("  l - Load configuration")
        print("  r - Reset to defaults")
        print("  q - Quit")
    
    def create_trackbars(self):
        """Create OpenCV trackbars based on current mode"""
        cv2.destroyAllWindows()
        cv2.namedWindow('ROI Configurator')
        
        if self.mode == 'zones':
            cv2.createTrackbar('Steering Top %', 'ROI Configurator', 
                              self.zones['steering_top'], 100, self.on_trackbar)
            cv2.createTrackbar('Steering Bottom %', 'ROI Configurator', 
                              self.zones['steering_bottom'], 100, self.on_trackbar)
            cv2.createTrackbar('Lookahead Top %', 'ROI Configurator', 
                              self.zones['lookahead_top'], 100, self.on_trackbar)
            cv2.createTrackbar('Lookahead Bottom %', 'ROI Configurator', 
                              self.zones['lookahead_bottom'], 100, self.on_trackbar)
            cv2.createTrackbar('Ignore Top %', 'ROI Configurator', 
                              self.zones['ignore_top'], 100, self.on_trackbar)
            cv2.createTrackbar('Left Margin %', 'ROI Configurator', 
                              self.zones['left_margin'], 100, self.on_trackbar)
            cv2.createTrackbar('Right Margin %', 'ROI Configurator', 
                              self.zones['right_margin'], 100, self.on_trackbar)
        
        elif self.mode == 'blue':
            cv2.createTrackbar('H Low', 'ROI Configurator', 
                              self.color_ranges['blue_h_low'], 180, self.on_trackbar)
            cv2.createTrackbar('H High', 'ROI Configurator', 
                              self.color_ranges['blue_h_high'], 180, self.on_trackbar)
            cv2.createTrackbar('S Low', 'ROI Configurator', 
                              self.color_ranges['blue_s_low'], 255, self.on_trackbar)
            cv2.createTrackbar('S High', 'ROI Configurator', 
                              self.color_ranges['blue_s_high'], 255, self.on_trackbar)
            cv2.createTrackbar('V Low', 'ROI Configurator', 
                              self.color_ranges['blue_v_low'], 255, self.on_trackbar)
            cv2.createTrackbar('V High', 'ROI Configurator', 
                              self.color_ranges['blue_v_high'], 255, self.on_trackbar)
        
        elif self.mode == 'orange':
            cv2.createTrackbar('H Low', 'ROI Configurator', 
                              self.color_ranges['orange_h_low'], 180, self.on_trackbar)
            cv2.createTrackbar('H High', 'ROI Configurator', 
                              self.color_ranges['orange_h_high'], 180, self.on_trackbar)
            cv2.createTrackbar('S Low', 'ROI Configurator', 
                              self.color_ranges['orange_s_low'], 255, self.on_trackbar)
            cv2.createTrackbar('S High', 'ROI Configurator', 
                              self.color_ranges['orange_s_high'], 255, self.on_trackbar)
            cv2.createTrackbar('V Low', 'ROI Configurator', 
                              self.color_ranges['orange_v_low'], 255, self.on_trackbar)
            cv2.createTrackbar('V High', 'ROI Configurator', 
                              self.color_ranges['orange_v_high'], 255, self.on_trackbar)
        
        elif self.mode == 'black':
            cv2.createTrackbar('H Low', 'ROI Configurator', 
                              self.color_ranges['black_h_low'], 180, self.on_trackbar)
            cv2.createTrackbar('H High', 'ROI Configurator', 
                              self.color_ranges['black_h_high'], 180, self.on_trackbar)
            cv2.createTrackbar('S Low', 'ROI Configurator', 
                              self.color_ranges['black_s_low'], 255, self.on_trackbar)
            cv2.createTrackbar('S High', 'ROI Configurator', 
                              self.color_ranges['black_s_high'], 255, self.on_trackbar)
            cv2.createTrackbar('V Low', 'ROI Configurator', 
                              self.color_ranges['black_v_low'], 255, self.on_trackbar)
            cv2.createTrackbar('V High', 'ROI Configurator', 
                              self.color_ranges['black_v_high'], 255, self.on_trackbar)
    
    def on_trackbar(self, val):
        """Trackbar callback - updates values in real-time"""
        pass  # Values are read directly in process_frame
    
    def read_trackbars(self):
        """Read current trackbar values"""
        if self.mode == 'zones':
            self.zones['steering_top'] = cv2.getTrackbarPos('Steering Top %', 'ROI Configurator')
            self.zones['steering_bottom'] = cv2.getTrackbarPos('Steering Bottom %', 'ROI Configurator')
            self.zones['lookahead_top'] = cv2.getTrackbarPos('Lookahead Top %', 'ROI Configurator')
            self.zones['lookahead_bottom'] = cv2.getTrackbarPos('Lookahead Bottom %', 'ROI Configurator')
            self.zones['ignore_top'] = cv2.getTrackbarPos('Ignore Top %', 'ROI Configurator')
            self.zones['left_margin'] = cv2.getTrackbarPos('Left Margin %', 'ROI Configurator')
            self.zones['right_margin'] = cv2.getTrackbarPos('Right Margin %', 'ROI Configurator')
        
        elif self.mode == 'blue':
            self.color_ranges['blue_h_low'] = cv2.getTrackbarPos('H Low', 'ROI Configurator')
            self.color_ranges['blue_h_high'] = cv2.getTrackbarPos('H High', 'ROI Configurator')
            self.color_ranges['blue_s_low'] = cv2.getTrackbarPos('S Low', 'ROI Configurator')
            self.color_ranges['blue_s_high'] = cv2.getTrackbarPos('S High', 'ROI Configurator')
            self.color_ranges['blue_v_low'] = cv2.getTrackbarPos('V Low', 'ROI Configurator')
            self.color_ranges['blue_v_high'] = cv2.getTrackbarPos('V High', 'ROI Configurator')
        
        elif self.mode == 'orange':
            self.color_ranges['orange_h_low'] = cv2.getTrackbarPos('H Low', 'ROI Configurator')
            self.color_ranges['orange_h_high'] = cv2.getTrackbarPos('H High', 'ROI Configurator')
            self.color_ranges['orange_s_low'] = cv2.getTrackbarPos('S Low', 'ROI Configurator')
            self.color_ranges['orange_s_high'] = cv2.getTrackbarPos('S High', 'ROI Configurator')
            self.color_ranges['orange_v_low'] = cv2.getTrackbarPos('V Low', 'ROI Configurator')
            self.color_ranges['orange_v_high'] = cv2.getTrackbarPos('V High', 'ROI Configurator')
        
        elif self.mode == 'black':
            self.color_ranges['black_h_low'] = cv2.getTrackbarPos('H Low', 'ROI Configurator')
            self.color_ranges['black_h_high'] = cv2.getTrackbarPos('H High', 'ROI Configurator')
            self.color_ranges['black_s_low'] = cv2.getTrackbarPos('S Low', 'ROI Configurator')
            self.color_ranges['black_s_high'] = cv2.getTrackbarPos('S High', 'ROI Configurator')
            self.color_ranges['black_v_low'] = cv2.getTrackbarPos('V Low', 'ROI Configurator')
            self.color_ranges['black_v_high'] = cv2.getTrackbarPos('V High', 'ROI Configurator')
    
    def draw_zones(self, frame):
        """Draw ROI zones on frame"""
        h, w = frame.shape[:2]
        
        # Convert percentages to pixels
        ignore_y = int(h * self.zones['ignore_top'] / 100)
        lookahead_top_y = int(h * self.zones['lookahead_top'] / 100)
        lookahead_bottom_y = int(h * self.zones['lookahead_bottom'] / 100)
        steering_top_y = int(h * self.zones['steering_top'] / 100)
        steering_bottom_y = int(h * self.zones['steering_bottom'] / 100)
        left_x = int(w * self.zones['left_margin'] / 100)
        right_x = int(w * self.zones['right_margin'] / 100)
        
        # Draw ignore zone (red)
        cv2.rectangle(frame, (0, 0), (w, ignore_y), (0, 0, 255), 2)
        cv2.putText(frame, "IGNORE", (10, ignore_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Draw lookahead zone (blue)
        cv2.rectangle(frame, (left_x, lookahead_top_y), 
                     (right_x, lookahead_bottom_y), (255, 0, 0), 2)
        cv2.putText(frame, "LOOKAHEAD", (left_x + 10, lookahead_top_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # Draw steering zone (green)
        cv2.rectangle(frame, (left_x, steering_top_y), 
                     (right_x, steering_bottom_y), (0, 255, 0), 3)
        cv2.putText(frame, "STEERING", (left_x + 10, steering_top_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw center line
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
        
        # Draw margins
        cv2.line(frame, (left_x, 0), (left_x, h), (255, 255, 255), 1)
        cv2.line(frame, (right_x, 0), (right_x, h), (255, 255, 255), 1)
        
        return frame
    
    def apply_color_detection(self, frame):
        """Apply color detection and show masks"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        if self.mode == 'blue':
            lower = np.array([self.color_ranges['blue_h_low'], 
                            self.color_ranges['blue_s_low'], 
                            self.color_ranges['blue_v_low']])
            upper = np.array([self.color_ranges['blue_h_high'], 
                            self.color_ranges['blue_s_high'], 
                            self.color_ranges['blue_v_high']])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            pixel_count = cv2.countNonZero(mask)
            
            cv2.putText(result, f"Blue Pixels: {pixel_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            return result if self.show_masks else frame
        
        elif self.mode == 'orange':
            lower = np.array([self.color_ranges['orange_h_low'], 
                            self.color_ranges['orange_s_low'], 
                            self.color_ranges['orange_v_low']])
            upper = np.array([self.color_ranges['orange_h_high'], 
                            self.color_ranges['orange_s_high'], 
                            self.color_ranges['orange_v_high']])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            pixel_count = cv2.countNonZero(mask)
            
            cv2.putText(result, f"Orange Pixels: {pixel_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            
            return result if self.show_masks else frame
        
        elif self.mode == 'black':
            lower = np.array([self.color_ranges['black_h_low'], 
                            self.color_ranges['black_s_low'], 
                            self.color_ranges['black_v_low']])
            upper = np.array([self.color_ranges['black_h_high'], 
                            self.color_ranges['black_s_high'], 
                            self.color_ranges['black_v_high']])
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            pixel_count = cv2.countNonZero(mask)
            
            cv2.putText(result, f"Black Pixels: {pixel_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            return result if self.show_masks else frame
        
        return frame
    
    def process_frame(self, frame):
        """Process frame with current settings"""
        self.read_trackbars()
        
        if self.mode == 'zones':
            frame = self.draw_zones(frame)
        else:
            frame = self.apply_color_detection(frame)
        
        # Add mode indicator
        mode_text = f"Mode: {self.mode.upper()} | Press 1-4 to switch | M: mask | P: pause | S: save"
        cv2.putText(frame, mode_text, (10, frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if self.paused:
            cv2.putText(frame, "PAUSED", (frame.shape[1]//2 - 50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        return frame
    
    def save_config(self, filename='roi_config.json'):
        """Save current configuration to file"""
        config = {
            'zones': self.zones,
            'color_ranges': self.color_ranges
        }
        with open(filename, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"\n✓ Configuration saved to {filename}")
    
    def load_config(self, filename='roi_config.json'):
        """Load configuration from file"""
        try:
            with open(filename, 'r') as f:
                config = json.load(f)
            self.zones = config['zones']
            self.color_ranges = config['color_ranges']
            print(f"\n✓ Configuration loaded from {filename}")
            self.create_trackbars()  # Recreate trackbars with loaded values
        except FileNotFoundError:
            print(f"\n✗ Configuration file {filename} not found")
    
    def run(self):
        """Main loop"""
        self.create_trackbars()
        
        try:
            while True:
                if not self.paused:
                    # Capture new frame
                    frame = self.picam2.capture_array()
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self.current_frame = frame.copy()
                else:
                    # Use last captured frame
                    frame = self.current_frame.copy()
                
                # Process and display
                display_frame = self.process_frame(frame)
                cv2.imshow('ROI Configurator', display_frame)
                
                # Handle keypresses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('1'):
                    self.mode = 'zones'
                    print("\nMode: ROI Zones")
                    self.create_trackbars()
                elif key == ord('2'):
                    self.mode = 'blue'
                    print("\nMode: Blue Detection")
                    self.create_trackbars()
                elif key == ord('3'):
                    self.mode = 'orange'
                    print("\nMode: Orange Detection")
                    self.create_trackbars()
                elif key == ord('4'):
                    self.mode = 'black'
                    print("\nMode: Black Wall Detection")
                    self.create_trackbars()
                elif key == ord('m'):
                    self.show_masks = not self.show_masks
                    print(f"\nMask view: {'ON' if self.show_masks else 'OFF'}")
                elif key == ord('p'):
                    self.paused = not self.paused
                    print(f"\n{'Paused' if self.paused else 'Resumed'}")
                elif key == ord('s'):
                    self.save_config()
                elif key == ord('l'):
                    self.load_config()
                elif key == ord('r'):
                    print("\nResetting to defaults...")
                    self.__init__()
                    self.create_trackbars()
        
        finally:
            self.picam2.stop()
            cv2.destroyAllWindows()
            print("\nConfigurator closed.")

if __name__ == "__main__":
    print("=" * 60)
    print("WRO ROI Configuration Tool")
    print("=" * 60)
    configurator = ROIConfigurator()
    configurator.run()
