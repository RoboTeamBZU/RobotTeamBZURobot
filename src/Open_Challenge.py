#!/usr/bin/env python3
import time
import os
import math
import traceback
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import adafruit_mpu6050
import pigpio
import RPi.GPIO as GPIO
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory
from picamera2 import Picamera2
import cv2
import numpy as np

# ============================================================================
# CONFIG (DO NOT change GPIOs without telling me)
# ============================================================================
IN1, IN2, ENA = 24, 23, 13        # motor driver pins (unchanged)
SERVO_PIN = 18                   # steering servo pin (unchanged)
SWITCH_PIN = 25                  # start/stop switch (unchanged)
LED_BLUE = 6

XSHUT_PINS = [board.D22, board.D27, board.D17, board.D26]  # Right, Left, Back, Front
SENSOR_NAMES = ['Right', 'Left', 'Back', 'Front']

# Motion & steering
SPEED = 200           # motor PWM (0-255). Reduce for initial testing.
MAX_STEER = 40        # degrees clamp for servo
DEAD_ZONE = 80        # mm difference considered centered

# PD controller (tune during practice)
PD_KP = 0.03
PD_KD = 0.007
PD_DT_MIN = 0.01

# Corner detection thresholds (mm)
FRONT_CLOSE = 180
SIDE_OPEN = 600
SIDE_CLOSE = 350

# Turn parameters
TURN_TARGET_DEG = 90
TURN_TOLERANCE = 0.95
TURN_TIMEOUT = 4.0  # seconds

# Camera
CAM_W, CAM_H = 640, 480
CAM_CAL_FILE = "camera_cal.npz"  # optional undistort file; create with calibration script

# Sensor timing
SENSOR_SAMPLE_DELAY = 0.02

# Gyro calibration
GYRO_CAL_DURATION = 1.0

# ============================================================================
# HARDWARE SETUP
# ============================================================================
def setup_sensors():
    """Initialize VL53L0X sensors (XSHUT sequencing)."""
    print("Setting up TOF sensors...")
    i2c = board.I2C()
    xshut = [DigitalInOut(pin) for pin in XSHUT_PINS]
    for p in xshut:
        p.switch_to_output(value=False)
    time.sleep(0.1)

    vl53 = []
    for i, p in enumerate(xshut):
        p.value = True
        time.sleep(0.12)
        try:
            sensor = VL53L0X(i2c)
            if i < len(xshut) - 1:
                sensor.set_address(i + 0x30)
            vl53.append(sensor)
        except Exception as e:
            print(f"⚠️  VL53 init failed for {SENSOR_NAMES[i]}: {e}")
            vl53.append(None)
    print("✓ TOF Sensors ready")
    return vl53

def setup_motor_servo():
    """Initialize pigpio, motor pins, servo, switch and LED."""
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Start with: sudo systemctl enable --now pigpiod")

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
    """Initialize MPU6050 (gyro)."""
    try:
        i2c = board.I2C()
        mpu = adafruit_mpu6050.MPU6050(i2c)
        print("✓ MPU6050 ready")
        return mpu
    except Exception as e:
        print(f"⚠️  MPU6050 not found: {e}")
        return None

# ============================================================================
# MOTOR CONTROL
# ============================================================================
def run_motor(pi, speed):
    """Run forward at given duty cycle (0-255)."""
    pi.write(IN1, 1)
    pi.write(IN2, 0)
    pi.set_PWM_dutycycle(ENA, int(max(0, min(255, speed))))

def stop_motor(pi):
    pi.set_PWM_dutycycle(ENA, 0)
    pi.write(IN1, 1)
    pi.write(IN2, 1)

# ============================================================================
# SENSOR READS
# ============================================================================
def read_sensors(vl53):
    """Read all 4 TOF sensors; clamp out-of-range to 1500 mm."""
    distances = {}
    for i, sensor in enumerate(vl53):
        name = SENSOR_NAMES[i]
        if sensor is None:
            distances[name] = 1500
            continue
        try:
            d = sensor.range
            distances[name] = d if 10 <= d <= 1500 else 1500
        except Exception:
            distances[name] = 1500
        time.sleep(SENSOR_SAMPLE_DELAY)
    return distances

# ============================================================================
# PD CONTROLLER
# ============================================================================
class PDController:
    def __init__(self, kp=PD_KP, kd=PD_KD):
        self.kp = kp
        self.kd = kd
        self.last_error = 0.0
        self.last_time = None

    def update(self, error):
        t = time.time()
        if self.last_time is None:
            dt = PD_DT_MIN
        else:
            dt = max(PD_DT_MIN, t - self.last_time)
        d_error = (error - self.last_error) / dt
        output = self.kp * error + self.kd * d_error
        self.last_error = error
        self.last_time = t
        return output

# ============================================================================
# CAMERA (Picamera2) + OPTIONAL UNDISTORT
# ============================================================================
def init_camera():
    print("Initializing camera...")
    try:
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration(main={"size": (CAM_W, CAM_H)})
        picam2.configure(camera_config)
        picam2.start()
        time.sleep(0.5)  # Give camera time to warm up
        
        remap = None
        if os.path.exists(CAM_CAL_FILE):
            try:
                data = np.load(CAM_CAL_FILE)
                K = data['K']
                D = data['D']
                try:
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (CAM_W, CAM_H), cv2.CV_16SC2)
                except Exception:
                    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (CAM_W, CAM_H), cv2.CV_16SC2)
                remap = (map1, map2)
                print("✓ Camera calibration loaded")
            except Exception as e:
                print(f"⚠️ Failed loading camera calibration: {e}")
                traceback.print_exc()
        print("✓ Camera ready (Picamera2)")
        return picam2, remap
    except Exception as e:
        print(f"⚠️ Camera initialization failed: {e}")
        print("  Robot will run using TOF sensors only")
        traceback.print_exc()
        return None, None

# ============================================================================
# SIMPLE WALL DETECTION (camera)
# ============================================================================
def detect_walls_from_camera(frame):
    """
    Detect left & right wall contours in a horizontal ROI and return lateral error (px).
    Positive error -> lane center is right of image center (robot is too far left),
    Negative error -> lane center is left of image center.
    Returns None on failure.
    """
    if frame is None:
        return None
    h, w = frame.shape[:2]
    y1 = int(h * 0.35)
    y2 = int(h * 0.65)
    roi = frame[y1:y2, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    clahe = cv2.createCLAHE(clipLimit=3.0)
    enhanced = clahe.apply(blurred)
    _, th = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.bitwise_not(th)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    min_area = (w * (y2-y1)) * 0.01
    large = [c for c in contours if cv2.contourArea(c) > min_area]
    if not large:
        return None

    boxes = [cv2.boundingRect(c) for c in large]
    boxes = sorted(boxes, key=lambda b: b[0])
    left_box = boxes[0]
    right_box = boxes[-1] if len(boxes) > 1 else None

    left_cx = left_box[0] + left_box[2] // 2
    if right_box is not None:
        right_cx = right_box[0] + right_box[2] // 2
        lane_center_px = (left_cx + right_cx) / 2.0
        error_px = lane_center_px - (w / 2.0)
        # Return pixel-space error (PD controller tuned accordingly)
        return error_px
    else:
        # If only one wall is visible, we can't compute lane center reliably -> fallback
        return None

# ============================================================================
# CORNER DETECTION & TURNING (gyro)
# ============================================================================
def detect_corner(distances):
    """Use front & side TOFs to decide whether a 90° turn is required."""
    r, l, f = distances['Right'], distances['Left'], distances['Front']
    if f < FRONT_CLOSE:
        if l > SIDE_OPEN and r < SIDE_CLOSE:
            return 'left'
        elif r > SIDE_OPEN and l < SIDE_CLOSE:
            return 'right'
    return None

def calibrate_gyro(mpu, duration=GYRO_CAL_DURATION):
    """Average gyro z readings for duration seconds to compute bias."""
    if not mpu:
        return 0.0
    print(f"Calibrating gyro for {duration:.1f}s. Keep robot still.")
    t_end = time.time() + duration
    samples = []
    while time.time() < t_end:
        try:
            samples.append(mpu.gyro[2])
        except Exception:
            pass
        time.sleep(0.01)
    bias = float(np.mean(samples)) if samples else 0.0
    print(f"✓ Gyro bias (rad/s): {bias:.6f}  (~{bias*57.2958:.3f} °/s)")
    return bias

def turn_90(pi, servo, mpu, bias_z, direction, speed=SPEED):
    """Perform ~90° turn by integrating gyro z; with timeout and bias compensation."""
    if not mpu:
        print("⚠️ No MPU - cannot perform gyro turn")
        return False
    print(f"\n🔄 Turning {direction.upper()} {TURN_TARGET_DEG}° ...")
    servo.angle = -MAX_STEER if direction == 'left' else MAX_STEER
    target = -TURN_TARGET_DEG if direction == 'left' else TURN_TARGET_DEG

    run_motor(pi, speed)
    last_t = time.time()
    start_t = last_t
    rotation = 0.0
    while abs(rotation) < abs(target) * TURN_TOLERANCE:
        if time.time() - start_t > TURN_TIMEOUT:
            print("\n⚠️ Turn timeout - stopping.")
            stop_motor(pi)
            servo.angle = 0
            return False
        try:
            gz = mpu.gyro[2]
            yaw_rate = (gz - bias_z) * 57.2958  # deg/s
        except Exception:
            yaw_rate = 0.0
        now = time.time()
        dt = now - last_t
        last_t = now
        rotation += yaw_rate * dt
        time.sleep(0.01)
    stop_motor(pi)
    servo.angle = 0
    time.sleep(0.2)
    print(f"✓ Turn done: {rotation:.1f}°")
    return True

# ============================================================================
# UTILITY
# ============================================================================
def apply_steering(servo, angle):
    """Safe servo set with clamp."""
    servo.angle = max(-MAX_STEER, min(MAX_STEER, angle))

# ============================================================================
# MAIN
# ============================================================================
def main():
    print("="*60)
    print("WRO 2025 - Open Challenge - Simple Wall Following + Gyro 90° turns")
    print("="*60)

    # Init hardware
    vl53 = setup_sensors()
    pi, servo = setup_motor_servo()
    mpu = setup_mpu6050()
    pd = PDController(kp=PD_KP, kd=PD_KD)

    # Camera
    picam2, remap = init_camera()

    # Calibrate gyro bias if possible
    gyro_bias = calibrate_gyro(mpu) if mpu else 0.0

    # Wait for Start button (SWITCH_PIN). According to WRO, robot must wait after power on.
    print("\nWaiting for Start button (press to RUN)...")
    while GPIO.input(SWITCH_PIN) == 1:
        time.sleep(0.05)

    GPIO.output(LED_BLUE, GPIO.HIGH)
    print("\n🚗 RUNNING!\n")
    print("Right | Left | Front | Steering | Action")
    print("-" * 55)

    try:
        while True:
            # If switch released, pause motors
            if GPIO.input(SWITCH_PIN) == 1:
                stop_motor(pi)
                servo.angle = 0
                GPIO.output(LED_BLUE, GPIO.LOW)
                time.sleep(0.05)
                continue
            GPIO.output(LED_BLUE, GPIO.HIGH)

            # Read TOF sensors
            distances = read_sensors(vl53)

            # Capture camera frame and compute error (pixel space)
            frame = camera_capture_frame(picam2, remap)
            cam_error = detect_walls_from_camera(frame) if frame is not None else None

            # Safety: emergency front stop
            if distances['Front'] < 80:
                print("‼️ FRONT TOO CLOSE - stopping")
                stop_motor(pi)
                servo.angle = 0
                time.sleep(0.2)
                continue

           # Corner detection via TOF (reliable short-range)
            corner = detect_corner(distances)
            if corner and mpu:
                stop_motor(pi)
                time.sleep(0.12)
                success = turn_90(pi, servo, mpu, gyro_bias, corner)
                pd.last_error = 0.0
                pd.last_time = None
                if not success:
                    print("⚠️ Turn failed - brief pause")
                    time.sleep(0.5)
                continue

            # Steering: prefer camera error; fallback to TOF diff
            if cam_error is not None:
                control_out = pd.update(cam_error)
            else:
                diff = distances['Right'] - distances['Left']
                if abs(diff) < DEAD_ZONE:
                    control_out = 0.0
                else:
                    control_out = pd.update(diff)

            # Map control_out to servo angle directly (tune PD gains to produce reasonable range)
            angle = max(-MAX_STEER, min(MAX_STEER, control_out))
            apply_steering(servo, angle)
            run_motor(pi, SPEED)

           # Status print
            if abs(angle) < 5:
                action = "STRAIGHT"
            elif angle > 0:
                action = f"RIGHT {abs(angle):.0f}°"
            else:
                action = f"LEFT {abs(angle):.0f}°"

            print(f"{distances['Right']:4} | {distances['Left']:4} | {distances['Front']:4} | "
                  f"{angle:6.1f}° | {action}")
            
            # ← ADD THESE 3 LINES HERE:
            if cam_error is not None:
                print(f"  📷 Camera working: {cam_error:.1f}px error")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    finally:
        stop_motor(pi)
        servo.angle = 0
        GPIO.output(LED_BLUE, GPIO.LOW)
        pi.stop()
        GPIO.cleanup()
        try:
            picam2.stop()
        except Exception:
            pass
        print("Done!")

if __name__ == "__main__":
    main()
