import time
import math
import board
import adafruit_mpu6050
import pigpio
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# Pin Configuration
SERVO_PIN = 18
IN1, IN2, ENA = 24, 23, 13

# Servo limits
SERVO_MAX_RIGHT = 20
SERVO_MAX_LEFT = -20

# Motor settings
DEFAULT_MOTOR_SPEED = 200

# PID Controller parameters
KP = 0.8
KI = 0.01
KD = 0.3

# Direction settings
SERVO_DIRECTION = 1
GYRO_DIRECTION = -1

class GyroSteering:
    def __init__(self):
        # Initialize pigpio for motors
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Run 'sudo pigpiod' first")
        
        for p in (ENA, IN1, IN2):
            self.pi.set_mode(p, pigpio.OUTPUT)
        
        self.pi.set_PWM_frequency(ENA, 2000)
        
        # Initialize servo
        Device.pin_factory = PiGPIOFactory()
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
        
        # State variables
        self.current_heading = 0.0
        self.cumulative_rotation = 0.0  # Tracks total rotation (no limits)
        self.target_heading = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.last_error = 0.0
        self.is_moving = False
        self.current_speed = 0
        
    def calibrate_gyro(self, samples=100):
        gyro_z_offset = 0.0
        for i in range(samples):
            gyro_z_offset += self.mpu.gyro[2]
            time.sleep(0.01)
        self.gyro_z_offset = gyro_z_offset / samples
    
    def run_motor(self, value):
        forward = value >= 0
        self.pi.write(IN1, 1 if forward else 0)
        self.pi.write(IN2, 0 if forward else 1)
        speed = min(255, abs(int(value)))
        self.pi.set_PWM_dutycycle(ENA, speed)
        self.is_moving = True
        self.current_speed = speed
        
    def stop_motor(self):
        self.pi.set_PWM_dutycycle(ENA, 0)
        self.pi.write(IN1, 0)
        self.pi.write(IN2, 0)
        self.is_moving = False
        self.current_speed = 0
        
    def update_heading(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        gyro_z = (self.mpu.gyro[2] - self.gyro_z_offset) * GYRO_DIRECTION
        heading_change = math.degrees(gyro_z * dt)
        
        # Update both heading (normalized) and cumulative rotation (unlimited)
        self.cumulative_rotation += heading_change
        self.current_heading = (self.current_heading + heading_change + 180) % 360 - 180
        
    def calculate_steering_angle(self, error):
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        
        derivative = error - self.last_error
        self.last_error = error
        
        raw_steering = (KP * error + KI * self.integral + KD * derivative) * SERVO_DIRECTION
        steering = max(min(raw_steering, SERVO_MAX_RIGHT), SERVO_MAX_LEFT)
        
        return steering
        
    def steer_to_heading(self, target_heading, speed=DEFAULT_MOTOR_SPEED, tolerance=2.0, timeout=10.0):
        """Steer to normalized heading (-180 to 180)"""
        self.target_heading = target_heading
        self.integral = 0.0
        self.last_error = 0.0
        start_time = time.time()
        
        self.run_motor(speed)
        
        while True:
            self.update_heading()
            
            # Calculate shortest path error
            error = self.target_heading - self.current_heading
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360
            
            steering_angle = self.calculate_steering_angle(error)
            self.servo.angle = steering_angle
            
            print(f"Current: {self.current_heading:6.1f}° | "
                  f"Target: {self.target_heading:6.1f}° | "
                  f"Error: {error:6.1f}° | "
                  f"Servo: {steering_angle:6.1f}° | "
                  f"Speed: {self.current_speed}/255", end='\r')
            
            if abs(error) < tolerance:
                print(f"\nTarget reached! Final heading: {self.current_heading:.1f}°")
                self.stop_motor()
                self.servo.angle = 0
                return True
                
            if time.time() - start_time > timeout:
                print(f"\nTimeout! Current: {self.current_heading:.1f}°")
                self.stop_motor()
                self.servo.angle = 0
                return False
                
            time.sleep(0.02)
    
    def rotate_degrees(self, degrees, speed=DEFAULT_MOTOR_SPEED, tolerance=2.0, timeout=30.0):
        """Rotate by a specific number of degrees (can be > 360 or < -360)"""
        target_rotation = self.cumulative_rotation + degrees
        self.integral = 0.0
        self.last_error = 0.0
        start_time = time.time()
        
        self.run_motor(speed)
        
        while True:
            self.update_heading()
            
            # Error based on cumulative rotation (no normalization)
            error = target_rotation - self.cumulative_rotation
            
            steering_angle = self.calculate_steering_angle(error)
            self.servo.angle = steering_angle
            
            print(f"Rotated: {self.cumulative_rotation:7.1f}° | "
                  f"Target: {target_rotation:7.1f}° | "
                  f"Remaining: {error:6.1f}° | "
                  f"Servo: {steering_angle:6.1f}° | "
                  f"Speed: {self.current_speed}/255", end='\r')
            
            if abs(error) < tolerance:
                print(f"\nRotation complete! Total rotated: {self.cumulative_rotation:.1f}°")
                self.stop_motor()
                self.servo.angle = 0
                return True
                
            if time.time() - start_time > timeout:
                print(f"\nTimeout! Rotated: {self.cumulative_rotation:.1f}°")
                self.stop_motor()
                self.servo.angle = 0
                return False
                
            time.sleep(0.02)
            
    def cleanup(self):
        self.stop_motor()
        self.servo.angle = 0
        self.servo.close()
        self.pi.stop()

def main():
    steering = GyroSteering()
    motor_speed = DEFAULT_MOTOR_SPEED
    
    try:
        steering.calibrate_gyro()
        time.sleep(1)
        steering.current_heading = 0.0
        steering.cumulative_rotation = 0.0
        steering.last_time = time.time()
        
        print("\nControls:")
        print("  Enter number for cumulative rotation (e.g., 200, 360, 1000, -500)")
        print("  Enter 'h [angle]' for heading mode (e.g., 'h 90' for northeast)")
        print("  Enter 'q' to quit\n")
        
        while True:
            user_input = input("Enter command: ").strip()
            
            if user_input.lower() == 'q':
                break
            
            # Check for heading mode
            if user_input.lower().startswith('h '):
                try:
                    target = float(user_input.split()[1])
                    if -180 <= target <= 180:
                        steering.steer_to_heading(target, speed=motor_speed)
                    else:
                        print("Heading must be between -180 and 180")
                except (ValueError, IndexError):
                    print("Invalid heading command. Use format: h [angle]")
                continue
                
            # Default: cumulative rotation mode
            try:
                degrees = float(user_input)
                steering.rotate_degrees(degrees, speed=motor_speed)
            except ValueError:
                print("Invalid input")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        steering.cleanup()

if __name__ == "__main__":
    main()
