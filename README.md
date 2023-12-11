# Balancing-Robot
Buddy, a balancing robot for ECE 4180 Finals Project

Group members: Sidhant Gulati, Muath Alsubh. Fan Han Hoon, Daniel Niemann

Brief Description of Design Project:
Our project is a balancing robot that uses an IMU to figure out its orientation and automatically corrects its
position so that it does not fall over. We plan to use MBED multi-threading and RTOS.
A balancing robot requires fast error calculation and a PID controller to balance it. This can run on one of 
the threads while all other functions such as Ultrasonic sensor, LEDs, Speakers, etc.

List of Features
1) 3D printed Chassis
2) IMU
3) Two regular geared motors
4) Motor Driver
5) Battery
6) Mbed
7) LEDs
8) Power Switch
9) HC-SR04 Sonar Sensor
10) Adafruit Bluefruit LE UART Friend (BLE)

## Assembly Table for Balancing Robot Project

Below is the list of components required for the balancing robot project, along with their quantities, purposes, and any relevant notes.

| Part Name                                 | Quantity | Purpose                            | Notes/Remarks                                  |
|-------------------------------------------|----------|------------------------------------|------------------------------------------------|
| LSM9DS0 Sensor                            | 1        | Motion sensing                     | Gyroscope and accelerometer                    |
| mbed Microcontroller                      | 1        | Main control unit                  | Program with PID and filter logic              |
| Motor (e.g., DC, stepper)                 | 2        | Movement control                   | Ensure compatible with control signals         |
| Motor Driver (e.g., H-Bridge)             | 1        | Interface between MCU and motors   | Match with motor specifications                |
| Power Supply                              | 1        | Provide power                      | Voltage and current must match system requirements |
| Chassis                                   | 1        | Structural component               | Custom design or pre-fabricated                |
| Wheels                                    | 2        | Locomotion                         | Match with motor shafts                        |
| Battery                                   | 1        | Portable power source              | Ensure sufficient capacity and voltage         |
| Cables/Wires                              | Varied   | Connections                        | Lengths as needed for neat wiring              |
| Soldering Supplies                        | N/A      | For electrical connections         | Soldering iron, solder, etc.                   |
| Screw Set                                 | Varied   | For mechanical assembly            | Assorted sizes as per design                   |
| Adafruit Bluefruit LE UART Friend (BLE)   | 1        | Bluetooth connectivity             | For wireless communication/control             |
| HC-SR04 Sonar Sensor                      | 1        | Distance measurement               | Ultrasonic distance sensor                     |

### Additional Components and Tools
- **Prototyping Board (Breadboard)**: For initial testing of the circuit before final soldering.
- **Programming Tools**: USB cables, computer with necessary software and drivers.
- **Testing Equipment**: Multimeter, oscilloscope (if necessary).
- **Mechanical Tools**: Screwdrivers, pliers, wrench set for assembly.
- **Safety Equipment**: Safety glasses, gloves for soldering.

These components are essential for building a fully functional balancing robot capable of advanced tasks like wireless control and environmental interaction. Ensure compatibility and proper interfacing with all parts for optimal performance.



List of Features if we have enough time.
1) Ultrasonic Sensor
2) Multiple Modes – Dancing, Explorer, 
Tracking
3) Bluetooth Control
4) Speakers
5) Line Follower

List any closely related projects (URLs) and describe how your project will be different or improved.
https://www.youtube.com/watch?v=swEe17M_1KQ
https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
If you need any parts or software not available in the lab or in your parts kit, what is your plan to obtain the 
parts quickly (within 10 days)? – With the current virus issues, I would recommend using only what is 
already available in the lab this term, since ordering and shipping may be disrupted.
We plan on 3D printing the chassis
We are ordering a new IMU sensor
We need soldering services from the HIVE as well as Prototyping board
