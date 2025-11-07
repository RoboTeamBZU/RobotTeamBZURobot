# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
"""
Example of how to use the adafruit_vl53l0x library to change the assigned address of
multiple VL53L0X sensors on the same I2C bus. This example only focuses on 2 VL53L0X
sensors, but can be modified for more. BE AWARE: a multitude of sensors may require
more current than the on-board 3V regulator can output (typical current consumption during
active range readings is about 19 mA per sensor).
"""
import time
import board
from digitalio import DigitalInOut
from adafruit_vl53l0x import VL53L0X
import os

# declare the singleton variable for the default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# declare the digital output pins connected to the "SHDN" pin on each VL53L0X sensor
xshut = [
    DigitalInOut(board.D22), # GPIO 22
    DigitalInOut(board.D27), # GPIO 27
    DigitalInOut(board.D17), # GPIO 17
    DigitalInOut(board.D26), # GPIO 26
    # add more VL53L0X sensors by defining their SHDN pins here
]

for power_pin in xshut:
    # make sure these pins are a digital output, not a digital input
    power_pin.switch_to_output(value=False)
    print(f"shut down sensor{power_pin}")
    # These pins are active when Low, meaning:
    #   if the output signal is LOW, then the VL53L0X sensor is off.
    #   if the output signal is HIGH, then the VL53L0X sensor is on.

# all VL53L0X sensors are now off

# initialize a list to be used for the array of VL53L0X sensors
vl53 = []

# Names for the sensors in the same order as xshut
SENSOR_NAMES = ['Right', 'Left','Back',  'Front']

# now change the addresses of the VL53L0X sensors
for i, power_pin in enumerate(xshut):
    # turn on the VL53L0X to allow hardware check
    power_pin.value = True
    print(f"turned ON sensor{power_pin}")
    # instantiate the VL53L0X sensor on the I2C bus & insert it into the "vl53" list
    vl53.insert(i, VL53L0X(i2c))  # also performs VL53L0X hardware check
    # no need to change the address of the last VL53L0X sensor
    if i < len(xshut) - 1:
        # default address is 0x29. Change that to something else
        vl53[i].set_address(i + 0x30)  # address assigned should NOT be already in use

# there is a helpful list of pre-designated I2C addresses for various I2C devices at
# https://learn.adafruit.com/i2c-addresses/the-list
# According to this list 0x30-0x34 are available, although the list may be incomplete.

def clear_screen():
    """Clear the terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')

def draw_car_with_distances(distances_dict):
    """
    Draw a car representation with distance readings on each side
    distances_dict should have keys: 'Front', 'Back', 'Left', 'Right'
    """
    clear_screen()
    
    front = distances_dict.get('Front', '---')
    back = distances_dict.get('Back', '---')
    left = distances_dict.get('Left', '---')
    right = distances_dict.get('Right', '---')
    
    # Create the car ASCII art with distances
    print("\n")
    print(f"                  {front:^10}")
    print("                ┌──────────┐")
    print("                │  FRONT   │")
    print("              ┌─┴──────────┴─┐")
    print(f"  {left:<10}  │              │  {right:>10}")
    print(f"              │      🚗       │")
    print(f"  LEFT        │              │        RIGHT")
    print("              └─┬──────────┬─┘")
    print("                │   BACK   │")
    print("                └──────────┘")
    print(f"                  {back:^10}")
    print("\n")
    print("Press Ctrl+C to stop")

def detect_range():
    """continuously read and display sensor ranges"""
    try:
        while True:
            distances_dict = {}
            
            for index, sensor in enumerate(vl53):
                try:
                    distance = sensor.range
                    distances_dict[SENSOR_NAMES[index]] = f"{distance}mm"
                except OSError:
                    distances_dict[SENSOR_NAMES[index]] = "ERROR"
                time.sleep(0.05)
            
            draw_car_with_distances(distances_dict)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n\nStopping sensor readings...")

print(
    "Multiple VL53L0X sensors' addresses are assigned properly\n"
    "Starting car visualization with distance readings..."
)
time.sleep(2)
detect_range()