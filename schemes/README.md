Electromechanical Parts
====

| Component                | Model / Type                             | Purpose                                              | Notes                               |
| ------------------------ | ---------------------------------------- | ---------------------------------------------------- | ----------------------------------- |
| MicroComputer            | **Raspberry Pi 4 Model B (4GB)**         | High-level processing, autonomy & startup automation | Boots robot system, systemd control |
| Motor Drivers            | **L298N**                                | Drives DC motors and handles direction/PWM control   | TB6612 recommended for efficiency   |
| Motors                   | **2× DC Gear Motors**                    | Drive wheels + speed feedback                        | Quadrature encoder                  |
| Sensors                  | **4× ToF VL53L0X**                       | Distance measurement for navigation                  | Front + left + right                |
| Battery                  | **Li-Po Battery Pack** (2S/3S)           | Main power source                                    | Ensure proper voltage rating        |
| Regulator                | **Step-Down Module**                     | Powers rasp pi, sensors, servos                      | Stable 5V output                    |
| Motor Power Switch       | Rocker Switch                            | Main power switch                                    | For safe power cycling              |
| Kill Switch              | Toggle                                   | Emergency stop                                       | Linked to ESP32 or RPi GPIO         |
| Servo (If used)          | SG90                                     | Steering / mechanism actuation                       | (If your design uses it)            |
| Chassis                  | 3D-Printed                               | Robot frame                                          | Lightweight optimized base          |
| Wheels                   | 4x Rubber Drive Wheels                   | Motion                                               | Compatible with motor shaft         |
| Wiring                   | Dupont jumper wires                      | Electrical connections                               | Male-Male & Male-Female             |
| Breadboard               | Mini breadboard                          | Circuit integration                                  | Neat prototyping                    |
