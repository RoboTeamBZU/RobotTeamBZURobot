"""
Simple VL53L0X TOF Path Following
Uses: Right, Left, Back, Front sensors (in that order)
No threading, constant speed, simple logic
"""

import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import pigpio
import RPi.GPIO as GPIO
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# ============================================================================
# HARDWARE PINS (Your exact setup)
# ============================================================================
IN1, IN2, ENA = 24, 23, 13
SERVO_PIN = 18
SWITCH_PIN = 25
LED_BLUE = 6

# Sensor SHDN pins (matches your working code)
XSHUT_PINS = [
    board.D22,  # Right sensor
    board.D27,  # Left sensor
    board.D17,  # Back sensor
    board.D26   # Front sensor
]

SENSOR_NAMES = ['Right', 'Left', 'Back', 'Front']

# ============================================================================
# SIMPLE TUNING PARAMETERS
# ============================================================================
CONSTANT_SPEED = 170            # One speed for everything!
STEERING_GAIN = 0.05            # How much to steer based on wall difference
DEAD_ZONE = 80                  # Ignore small differences (8cm)
EMERGENCY_STOP_DISTANCE = 150   # Stop if front wall < 15cm (in mm)

# Sensor reading thresholds
SENSOR_MIN_VALID = 30           # Minimum valid reading (3cm)
SENSOR_MAX_VALID = 2000         # Maximum valid reading (200cm)
SENSOR_ERROR_VALUE = 250        # Your sensors return 250mm when out of range

# Global variables for sensor history
last_good_distances = {'Right': 500, 'Left': 500, 'Back': 500, 'Front': 500}

# ============================================================================
# STARTUP SEQUENCE FOR AUTONOMOUS MODE
# ============================================================================
def startup_sequence():
    """Visual indicator that system has booted and is ready"""
    # Setup GPIO for LED
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_BLUE, GPIO.OUT)
    
    print("\n" + "="*60)
    print("🤖 AUTONOMOUS ROBOT STARTUP")
    print("="*60)
    
    # Blink LED 3 times to indicate "System Ready"
    print("System initializing...")
    for i in range(3):
        GPIO.output(LED_BLUE, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(LED_BLUE, GPIO.LOW)
        time.sleep(0.3)
    
    print("✓ System ready!")
    print("✓ Waiting for start switch (GPIO 25)...")
    print("="*60 + "\n")

# ============================================================================
# SETUP SENSORS (Exact copy of your working code)
# ============================================================================
def setup_sensors():
    """Initialize VL53L0X sensors (your exact working code)"""
    print("Initializing sensors...")
    
    i2c = board.I2C()
    
    # Setup SHDN pins
    xshut = []
    for pin in XSHUT_PINS:
        power_pin = DigitalInOut(pin)
        power_pin.switch_to_output(value=False)
        xshut.append(power_pin)
    
    print("All sensors shut down")
    time.sleep(0.1)
    
    # Power on and initialize one by one
    vl53 = []
    for i, power_pin in enumerate(xshut):
        power_pin.value = True
        print(f"Turned ON {SENSOR_NAMES[i]} sensor")
        
        sensor = VL53L0X(i2c)
        
        if i < len(xshut) - 1:
            sensor.set_address(i + 0x30)
            print(f"  → Address: 0x{i + 0x30:02X}")
        else:
            print(f"  → Address: 0x29 (default)")
        
        vl53.append(sensor)
        time.sleep(0.1)
    
    print("✓ All sensors ready!\n")
    return vl53

# ============================================================================
# SETUP MOTOR & SERVO (Your exact servo code)
# ============================================================================
def setup_motor_servo():
    """Initialize motor and servo"""
    # pigpio for motor
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running")
    
    # Motor pins
    for p in (IN1, IN2, ENA):
        pi.set_mode(p, pigpio.OUTPUT)
    pi.set_PWM_frequency(ENA, 2000)
    
    # Switch and LED
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LED_BLUE, GPIO.OUT)
    GPIO.output(LED_BLUE, GPIO.LOW)
    
    # Servo (your exact working setup)
    Device.pin_factory = PiGPIOFactory()
    servo = AngularServo(SERVO_PIN, min_pulse_width=0.0006, max_pulse_width=0.0023)
    servo.angle = 0
    
    return pi, servo

def run_motor(pi, speed):
    """Run motor at constant speed"""
    pi.write(IN1, 1)  # Always forward
    pi.write(IN2, 0)
    pi.set_PWM_dutycycle(ENA, speed)

def stop_motor(pi):
    """Stop motor with brake"""
    pi.set_PWM_dutycycle(ENA, 0)
    pi.write(IN1, 1)
    pi.write(IN2, 1)

# ============================================================================
# READ SENSORS
# ============================================================================
def read_sensors(vl53):
    """Read all 4 sensors: Right, Left, Back, Front with intelligent filtering"""
    global last_good_distances
    distances = {}
    
    for i, sensor in enumerate(vl53):
        name = SENSOR_NAMES[i]
        try:
            distance = sensor.range
            
            # Check if reading is valid
            # Your sensors return 250mm when out of range or error
            if distance == SENSOR_ERROR_VALUE:
                # Sensor out of range - use last good value
                distances[name] = last_good_distances[name]
                print(f"⚠️  {name} out of range (250mm error), using last: {last_good_distances[name]}mm")
            elif SENSOR_MIN_VALID <= distance <= SENSOR_MAX_VALID:
                # Valid reading
                distances[name] = distance
                last_good_distances[name] = distance  # Save for future
            else:
                # Weird reading - use last good value
                distances[name] = last_good_distances[name]
        except:
            # Sensor read error - use last good value
            distances[name] = last_good_distances[name]
        
        time.sleep(0.02)  # Small delay between reads
    
    return distances

# ============================================================================
# SIMPLE STEERING LOGIC
# ============================================================================
def calculate_steering(distances):
    """
    Smart steering with corner detection
    Positive angle = RIGHT, Negative angle = LEFT (your servo orientation)
    """
    left = distances['Left']
    right = distances['Right']
    front = distances['Front']
    
    # CORNER DETECTION: Front wall close + one side much farther
    if front < 300:  # Approaching corner
        # Check if one side is significantly more open
        if right > left + 200:
            # Right side more open = turn right
            print("→ RIGHT CORNER detected")
            return 35  # Turn right
        elif left > right + 200:
            # Left side more open = turn left
            print("← LEFT CORNER detected")
            return -35  # Turn left
    
    # NORMAL WALL CENTERING
    diff = left - right
    
    # Sanity check: if difference is too large, one sensor probably failed
    if abs(diff) > 800:
        print(f"⚠️  Large diff detected: {diff}mm - maintaining course")
        return 0  # Keep going straight, don't make crazy turns
    
    # Dead zone - ignore small differences
    if abs(diff) < DEAD_ZONE:
        return 0
    
    # Simple proportional steering
    angle = diff * STEERING_GAIN
    
    # Limit to servo range: -40 (left) to +40 (right)
    angle = max(-40, min(40, angle))
    
    return angle

# ============================================================================
# MAIN LOOP
# ============================================================================
def main():
    print("="*60)
    print("SIMPLE TOF PATH FOLLOWING")
    print("="*60)
    
    # Setup hardware
    vl53 = setup_sensors()
    pi, servo = setup_motor_servo()
    
    # Startup sequence (blinks LED 3 times)
    startup_sequence()
    
    # Now wait for switch
    print("Press switch (GPIO 25 to GND) to start...")
    while GPIO.input(SWITCH_PIN) == 1:
        time.sleep(0.1)
    
    GPIO.output(LED_BLUE, GPIO.HIGH)
    print("\n🚗 RUNNING!\n")
    print("Right | Left  | Back  | Front | Diff  | Steering | Action")
    print("-"*70)
    
    try:
        while True:
            # Check switch
            if GPIO.input(SWITCH_PIN) == 1:
                stop_motor(pi)
                servo.angle = 0
                GPIO.output(LED_BLUE, GPIO.LOW)
                time.sleep(0.05)
                continue
            else:
                GPIO.output(LED_BLUE, GPIO.HIGH)
            
            # Read all sensors
            distances = read_sensors(vl53)
            
            # Check for emergency (front wall too close)
            if distances['Front'] < EMERGENCY_STOP_DISTANCE:
                stop_motor(pi)
                servo.angle = 0
                diff = distances['Left'] - distances['Right']
                print(f"{distances['Right']:4} | {distances['Left']:4} | "
                      f"{distances['Back']:4} | {distances['Front']:4} | "
                      f"{diff:5} |    0.0° | ⚠️  EMERGENCY STOP!")
                time.sleep(0.5)
                continue
            
            # Calculate steering
            target_steering = calculate_steering(distances)
            steering_angle = smooth_steering(target_steering)  # Apply smoothing
            diff = distances['Left'] - distances['Right']
            
            # Apply steering (smoothed angle)
            servo.angle = steering_angle
            
            # Run at constant speed
            run_motor(pi, CONSTANT_SPEED)
            
            # Determine action text
            if abs(steering_angle) < 5:
                action = "STRAIGHT"
            elif steering_angle > 0:
                action = f"RIGHT {abs(steering_angle):.0f}°"  # Positive = RIGHT
            else:
                action = f"LEFT {abs(steering_angle):.0f}°"   # Negative = LEFT
            
            # Show target vs actual for debugging smoothing
            if abs(target_steering - steering_angle) > 2:
                action += f" (→{target_steering:.0f}°)"  # Show where it's going
            
            # Print status
            print(f"{distances['Right']:4} | {distances['Left']:4} | "
                  f"{distances['Back']:4} | {distances['Front']:4} | "
                  f"{diff:5} | {steering_angle:6.1f}° | {action}")
            
            time.sleep(0.1)  # 10Hz loop
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    
    finally:
        print("\nShutting down...")
        stop_motor(pi)
        servo.angle = 0
        GPIO.output(LED_BLUE, GPIO.LOW)
        pi.stop()
        GPIO.cleanup()
        print("Done!")

# ============================================================================
# RUN
# ============================================================================
if __name__ == "__main__":
    main()
