# Balancing-Robot
Buddy, a balancing robot for ECE 4180 Finals Project

Group members: Sidhant Gulati, Muath Alsubh. Fan Han Hoon, Daniel Niemann

## Overview
This repository contains all the necessary code, documentation, and resources for building a self-balancing robot. The robot uses an LSM9DS0 sensor for motion sensing and an mbed microcontroller for processing and control. Additional features include Bluetooth connectivity for remote control and an HC-SR04 Sonar Sensor for distance measurement.

## Features
- Self-balancing capabilities using gyroscope and accelerometer data.
- Bluetooth control via Adafruit Bluefruit LE UART Friend.
- Obstacle detection using HC-SR04 Sonar Sensor.
- Customizable and expandable design.


## Brief Description of Design Project:
The Self-Balancing Robot project is an advanced robotics initiative aimed at designing and constructing a robot that can autonomously maintain its balance in real-time. Utilizing a combination of motion-sensing technology, precision control algorithms, and mechanical engineering, this robot represents a confluence of various fields of STEM.

## Design Objectives

- Stability: To engineer a robot that can stand upright on two wheels, adjusting its position dynamically to counteract forces that might cause it to topple.
- Control: Integration of a PID (Proportional, Integral, Derivative) control system to process real-time data from motion sensors and adjust motor responses accordingly.
- Connectivity and Interaction: Implement Bluetooth technology for remote control capabilities and a sonar sensor for obstacle detection and navigation.

## Components
The following components are required to build the robot:

| Part Name                                 | Quantity | Purpose                            | Notes/Remarks                                  |
|-------------------------------------------|----------|------------------------------------|------------------------------------------------|
| LSM9DS0 Sensor                            | 1        | Motion sensing                     | Gyroscope and accelerometer                    |
| mbed Microcontroller                      | 1        | Main control unit                  | Program with PID and filter logic              |
| Motor (e.g., DC, stepper)                 | 2        | Movement control                   | Ensure compatible with control signals         |
| Motor Driver (e.g., H-Bridge)             | 1        | Interface between MCU and motors   | Match with motor specifications                |
| Power Supply                              | 1        | Provide power                      | Voltage and current must match system requirements |
| Chassis                                   | 1        | Structural component               | Custom design or pre-fabricated                |
| Wheels                                    | 2        | Locomotion                         | Match with motor shafts                        |
| Battery                                   | 4        | Portable power source              | Ensure sufficient capacity and voltage         |
| Cables/Wires                              | Varied   | Connections                        | Lengths as needed for neat wiring              |
| Soldering Supplies                        | N/A      | For electrical connections         | Soldering iron, solder, etc.                   |
| Screw Set                                 | Varied   | For mechanical assembly            | Assorted sizes as per design                   |
| Adafruit Bluefruit LE UART Friend (BLE)   | 1        | Bluetooth connectivity             | For wireless communication/control             |
| HC-SR04 Sonar Sensor                      | 1        | Distance measurement               | Ultrasonic distance sensor                     |

## Encountered Challenges

Throughout the development of our Balancing Robot, we faced several significant challenges:

    IMU Selection: Initially, we experimented with two different Inertial Measurement Units (IMUs) that did not meet our project requirements. Finding the right IMU that provided accurate and reliable data was crucial for effective motion sensing and stability.

    Servo Motor Drift: We encountered issues with servo motor drift, a common problem in precision control applications. The drift in the motors made it particularly challenging to maintain the robot's balance, necessitating careful selection and calibration of motors.

    PID Tuning Process: One of the most time-consuming aspects was tuning the PID (Proportional, Integral, Derivative) controller. With over 200 iterations, we worked meticulously to find the optimal settings that allowed for stable and responsive balancing behavior.

## Implemented Improvements

In response to these challenges, we made several key improvements:

    Optimized IMU: After testing different IMUs, we settled on the LSM9DS0 sensor, which provided the necessary accuracy and consistency for our control algorithms.

    Motor Selection and Calibration: We addressed the servo motor drift issue by selecting higher-quality motors and implementing rigorous calibration procedures to minimize drift and enhance control precision.

    Systematic PID Tuning: To streamline the PID tuning process, we adopted a systematic approach, adjusting one parameter at a time and methodically testing the impact of each change. This strategy, while time-consuming, was critical in achieving the desired balance and stability.

These challenges have been instrumental in our learning journey and have significantly contributed to the overall success and reliability of our Balancing Robot project.



https://www.youtube.com/watch?v=swEe17M_1KQ
https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
If you need any parts or software not available in the lab or in your parts kit, what is your plan to obtain the 
parts quickly (within 10 days)? – With the current virus issues, I would recommend using only what is 
already available in the lab this term, since ordering and shipping may be disrupted.
We plan on 3D printing the chassis
We are ordering a new IMU sensor
We need soldering services from the HIVE as well as Prototyping board
