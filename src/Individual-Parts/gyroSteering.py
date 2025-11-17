import time
import board
import adafruit_mpu6050
import pigpio
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# Pin Configuration
SERVO_PIN = 18

# Servo limits (degrees from center)
SERVO_MAX_RIGHT = 40
SERVO_MAX_LEFT = -40

# PID Controller parameters
KP = 0.8  # Proportional gain
KI = 0.01  # Integral gain
KD = 0.3  # Derivative gain

class GyroSteering:
    def __init__(self):
        # Initialize pigpio factory for servo
        Device.pin_factory = PiGPIOFactory()
        
        # Initialize servo with range -40 to +40 degrees
        self.servo = AngularServo(
            SERVO_PIN,
            min_angle=SERVO_MAX_LEFT,
            max_angle=SERVO_MAX_RIGHT,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )
        
        # Initialize MPU6050
        i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        
        # Steering variables
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.last_time = time.time()
        
        # PID variables
        self.integral = 0.0
        self.last_error = 0.0
        
        print("Gyro Steering System Initialized")
        print(f"Servo range: {SERVO_MAX_LEFT}° to {SERVO_MAX_RIGHT}°")
        
    def calibrate_gyro(self, samples=100):
        """Calibrate gyro by taking multiple samples at rest"""
        print(f"Calibrating gyro... Keep device still!")
        gyro_z_offset = 0.0
        
        for i in range(samples):
            gyro_z_offset += self.mpu.gyro[2]
            time.sleep(0.01)
            
        self.gyro_z_offset = gyro_z_offset / samples
        print(f"Calibration complete. Offset: {self.gyro_z_offset:.4f} rad/s")
        
    def update_heading(self):
        """Update current heading based on gyro data"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Get gyro Z-axis (yaw rate) and subtract offset
        gyro_z = self.mpu.gyro[2] - self.gyro_z_offset
        
        # Integrate gyro rate to get heading change
        heading_change = math.degrees(gyro_z * dt)
        self.current_heading += heading_change
        
        # Normalize heading to -180 to +180 range
        self.current_heading = (self.current_heading + 180) % 360 - 180
        
    def calculate_steering_angle(self):
        """Calculate servo angle using PID controller"""
        # Calculate error (shortest angular distance)
        error = self.target_heading - self.current_heading
        
        # Normalize error to -180 to +180
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        # PID calculations
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)  # Anti-windup
        
        derivative = error - self.last_error
        self.last_error = error
        
        # PID output
        steering = KP * error + KI * self.integral + KD * derivative
        
        # Constrain to servo limits
        steering = max(min(steering, SERVO_MAX_RIGHT), SERVO_MAX_LEFT)
        
        return steering, error
        
    def set_target_heading(self, target):
        """Set new target heading"""
        self.target_heading = target
        # Reset integral term for new target
        self.integral = 0.0
        self.last_error = 0.0
        print(f"\nNew target heading: {target}°")
        
    def steer_to_heading(self, target_heading, tolerance=2.0, timeout=10.0):
        """Steer to target heading and wait until reached"""
        self.set_target_heading(target_heading)
        start_time = time.time()
        
        print(f"Steering to {target_heading}°...")
        
        while True:
            self.update_heading()
            steering_angle, error = self.calculate_steering_angle()
            
            # Apply steering
            self.servo.angle = steering_angle
            
            # Print status
            print(f"Current: {self.current_heading:6.1f}° | "
                  f"Target: {self.target_heading:6.1f}° | "
                  f"Error: {error:6.1f}° | "
                  f"Servo: {steering_angle:6.1f}°", end='\r')
            
            # Check if target reached
            if abs(error) < tolerance:
                print(f"\nTarget reached! Final heading: {self.current_heading:.1f}°")
                self.servo.angle = 0  # Center servo
                return True
                
            # Check timeout
            if time.time() - start_time > timeout:
                print(f"\nTimeout! Could not reach target. Current: {self.current_heading:.1f}°")
                self.servo.angle = 0  # Center servo
                return False
                
            time.sleep(0.02)  # 50Hz update rate
            
    def cleanup(self):
        """Clean up resources"""
        self.servo.angle = 0
        self.servo.close()
        print("\nSteering system shut down")

def main():
    steering = GyroSteering()
    
    try:
        # Calibrate gyro
        steering.calibrate_gyro()
        time.sleep(1)
        
        # Reset heading to 0
        steering.current_heading = 0.0
        steering.last_time = time.time()
        print("\nHeading reset to 0°. You can now test rotations!")
        print("=" * 60)
        
        # Interactive mode
        while True:
            user_input = input("\nEnter target angle (-180 to 180) or 'q' to quit: ").strip()
            
            if user_input.lower() == 'q':
                break
                
            try:
                target = float(user_input)
                
                if -180 <= target <= 180:
                    steering.steer_to_heading(target)
                else:
                    print("Please enter a value between -180 and 180")
                    
            except ValueError:
                print("Invalid input. Please enter a number or 'q' to quit")
                
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
        
    finally:
        steering.cleanup()

if __name__ == "__main__":
    main()
