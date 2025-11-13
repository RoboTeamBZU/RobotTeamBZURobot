"""
Simple Corner Detection + Gyro Turns ONLY
No wall following - just detect corners and turn 90° precisely
"""

import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import adafruit_mpu6050
import pigpio
import RPi.GPIO as GPIO
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# ============================================================================
# HARDWARE PINS
# ============================================================================
IN1, IN2, ENA = 24, 23, 13
SERVO_PIN = 18
SWITCH_PIN = 25
LED_BLUE = 6

XSHUT_PINS = [board.D22, board.D27, board.D17, board.D26]  # Right, Left, Back, Front
SENSOR_NAMES = ['Right', 'Left', 'Back', 'Front']

# ============================================================================
# TUNING
# ============================================================================
SPEED = 220

# ============================================================================
# MEDIAN FILTER
# ============================================================================
class MedianFilter:
    """Keep last N readings and return median to filter out spikes"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.readings = {}
    
    def update(self, sensor_name, value):
        """Add new reading and return filtered value"""
        if sensor_name not in self.readings:
            self.readings[sensor_name] = []
        
        self.readings[sensor_name].append(value)
        
        if len(self.readings[sensor_name]) > self.window_size:
            self.readings[sensor_name].pop(0)
        
        sorted_vals = sorted(self.readings[sensor_name])
        return sorted_vals[len(sorted_vals) // 2]

sensor_filter = MedianFilter(window_size=5)

# ============================================================================
# SETUP
# ============================================================================
def setup_sensors():
    """Initialize TOF sensors"""
    print("Setting up sensors...")
    i2c = board.I2C()
    
    xshut = [DigitalInOut(pin) for pin in XSHUT_PINS]
    for p in xshut:
        p.switch_to_output(value=False)
    time.sleep(0.1)
    
    vl53 = []
    for i, p in enumerate(xshut):
        p.value = True
        time.sleep(0.2)
        sensor = VL53L0X(i2c)
        if i < len(xshut) - 1:
            sensor.set_address(i + 0x30)
        vl53.append(sensor)
    
    print("✓ Sensors ready")
    return vl53

def setup_motor_servo():
    """Initialize motor and servo"""
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running")
    
    for p in (IN1, IN2, ENA):
        pi.set_mode(p, pigpio.OUTPUT)
    pi.set_PWM_frequency(ENA, 2000)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LED_BLUE, GPIO.OUT)
    GPIO.output(LED_BLUE, GPIO.LOW)
    
    Device.pin_factory = PiGPIOFactory()
    servo = AngularServo(SERVO_PIN, min_pulse_width=0.0006, max_pulse_width=0.0023)
    servo.angle = 0
    
    print("✓ Motor & servo ready")
    return pi, servo

def setup_mpu6050():
    """Initialize gyro"""
    try:
        i2c = board.I2C()
        mpu = adafruit_mpu6050.MPU6050(i2c)
        print("✓ Gyro ready")
        return mpu
    except:
        print("⚠️  No gyro found - cannot run without gyro!")
        return None

# ============================================================================
# MOTOR CONTROL
# ============================================================================
def run_motor(pi, speed):
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.set_PWM_dutycycle(ENA, speed)

def stop_motor(pi):
    pi.set_PWM_dutycycle(ENA, 0)
    pi.write(IN1, 1)
    pi.write(IN2, 1)

# ============================================================================
# SENSOR READING WITH FILTERING
# ============================================================================
def read_sensors(vl53):
    """Read all 4 sensors with median filtering"""
    distances = {}
    for i, sensor in enumerate(vl53):
        name = SENSOR_NAMES[i]
        try:
            raw = sensor.range
            if 10 <= raw <= 1500:
                filtered = sensor_filter.update(name, raw)
                distances[name] = filtered
            else:
                distances[name] = 1500
        except:
            distances[name] = 1500
        time.sleep(0.1)
    return distances

# ============================================================================
# CORNER DETECTION
# ============================================================================
def detect_corner(distances):
    """
    Detect corner using filtered sensor data
    Returns: 'left', 'right', or None
    """
    r, l, f = distances['Right'], distances['Left'], distances['Front']
    
    # Corner: front wall close AND one side clearly open
    if f < 200:
        if l > 600 and r < 350:
            print(f"  → Corner LEFT detected (L={l}, R={r}, F={f})")
            return 'left'
        elif r > 600 and l < 350:
            print(f"  → Corner RIGHT detected (L={l}, R={r}, F={f})")
            return 'right'
    
    return None

# ============================================================================
# GYRO-CONTROLLED TURNING (Progressive Servo Method)
# ============================================================================
def turn_90_progressive(pi, servo, mpu, direction):
    """
    Turn 90° by progressively increasing servo angle until gyro confirms rotation
    This is smoother and more controlled than fixed max steering
    """
    if not mpu:
        print("⚠️  No gyro - cannot turn!")
        return
    
    print(f"\n🔄 Turning {direction.upper()} 90° (progressive method)...")
    
    target_rotation = -90 if direction == 'left' else 90
    
    # Start with small steering angle
    current_servo_angle = 0
    max_servo_angle = 40
    servo_increment = 5  # Increase steering by 5° every loop if needed
    
    run_motor(pi, SPEED)
    
    accumulated_rotation = 0
    last_time = time.time()
    start_time = time.time()
    
    print("Rotation | Servo Angle | Yaw Rate")
    print("-" * 45)
    
    while abs(accumulated_rotation) < abs(target_rotation) * 0.95:
        # Safety timeout
        if time.time() - start_time > 5.0:
            print("\n⚠️  Timeout!")
            break
        
        # Read gyro
        gyro = mpu.gyro
        yaw_rate = gyro[2] * 57.2958  # rad/s to deg/s
        
        # Time integration
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        accumulated_rotation += yaw_rate * dt
        
        # Progressive steering: increase angle if not turning fast enough
        if abs(yaw_rate) < 30 and abs(current_servo_angle) < max_servo_angle:
            # Not turning fast enough - increase steering
            current_servo_angle += servo_increment if direction == 'right' else -servo_increment
            current_servo_angle = max(-max_servo_angle, min(max_servo_angle, current_servo_angle))
            servo.angle = current_servo_angle
        
        print(f"{accumulated_rotation:7.1f}° | {current_servo_angle:6.1f}° | {yaw_rate:+7.1f}°/s", end='\r')
        time.sleep(0.02)
    
    stop_motor(pi)
    servo.angle = 0
    time.sleep(0.5)
    
    print(f"\n✓ Turn complete: {accumulated_rotation:.1f}° in {time.time()-start_time:.1f}s\n")

# ============================================================================
# ALTERNATIVE: Fixed Angle Method (Original)
# ============================================================================
def turn_90_fixed(pi, servo, mpu, direction):
    """
    Turn 90° with fixed maximum steering (original method)
    Simpler but less smooth
    """
    if not mpu:
        print("⚠️  No gyro - cannot turn!")
        return
    
    print(f"\n🔄 Turning {direction.upper()} 90° (fixed angle method)...")
    
    # Set to maximum steering immediately
    servo.angle = -40 if direction == 'left' else 40
    target_rotation = -90 if direction == 'left' else 90
    
    run_motor(pi, SPEED)
    
    accumulated_rotation = 0
    last_time = time.time()
    start_time = time.time()
    
    print("Rotation | Yaw Rate")
    print("-" * 30)
    
    while abs(accumulated_rotation) < abs(target_rotation) * 0.95:
        if time.time() - start_time > 5.0:
            print("\n⚠️  Timeout!")
            break
        
        gyro = mpu.gyro
        yaw_rate = gyro[2] * 57.2958
        
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        accumulated_rotation += yaw_rate * dt
        
        print(f"{accumulated_rotation:7.1f}° | {yaw_rate:+7.1f}°/s", end='\r')
        time.sleep(0.02)
    
    stop_motor(pi)
    servo.angle = 0
    time.sleep(0.5)
    
    print(f"\n✓ Turn complete: {accumulated_rotation:.1f}° in {time.time()-start_time:.1f}s\n")

# ============================================================================
# MAIN
# ============================================================================
def main():
    print("="*60)
    print("CORNER DETECTION + GYRO TURNS")
    print("No wall following - just straight until corner detected")
    print("="*60)
    
    vl53 = setup_sensors()
    pi, servo = setup_motor_servo()
    mpu = setup_mpu6050()
    
    if not mpu:
        print("\n❌ MPU6050 required! Exiting...")
        return
    
    # CHOOSE YOUR TURNING METHOD HERE:
    turn_method = turn_90_progressive  # Options: turn_90_progressive or turn_90_fixed
    
    print(f"\nUsing: {turn_method.__name__}")
    print("Waiting for switch...")
    while GPIO.input(SWITCH_PIN) == 1:
        time.sleep(0.1)
    
    GPIO.output(LED_BLUE, GPIO.HIGH)
    print("\n🚗 RUNNING!\n")
    print("Right | Left | Front | Status")
    print("-"*50)
    
    try:
        while True:
            # Check switch
            if GPIO.input(SWITCH_PIN) == 1:
                stop_motor(pi)
                servo.angle = 0
                GPIO.output(LED_BLUE, GPIO.LOW)
                time.sleep(0.05)
                continue
            
            GPIO.output(LED_BLUE, GPIO.HIGH)
            
            # Read sensors with filtering
            d = read_sensors(vl53)
            
            # Check for corner
            corner = detect_corner(d)
            if corner:
                stop_motor(pi)
                time.sleep(0.2)
                turn_method(pi, servo, mpu, corner)
                continue
            
            # Just go straight - no wall following
            servo.angle = 0
            run_motor(pi, SPEED)
            
            print(f"{d['Right']:4} | {d['Left']:4} | {d['Front']:4} | STRAIGHT")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nStopped")
    
    finally:
        stop_motor(pi)
        servo.angle = 0
        GPIO.output(LED_BLUE, GPIO.LOW)
        pi.stop()
        GPIO.cleanup()
        print("Done!")

if __name__ == "__main__":
    main()
