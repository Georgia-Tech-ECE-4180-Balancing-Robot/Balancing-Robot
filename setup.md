# Setup and Installation Guide for the Balancing Robot

## Prerequisites
Before starting, ensure you have the following:
- An mbed account for accessing the mbed compiler or Keil Studio.
- Basic soldering and wiring skills.
- Familiarity with C/C++ programming for embedded systems.

## Hardware Assembly
1. **Assemble the Chassis**: Construct the frame of the robot using the chassis kit. Securely attach the wheels to the motors.

2. **Mount the Motors**: Attach the motors to the chassis, ensuring they are firmly in place and correctly aligned.

3. **Connect the Motor Drivers**: Solder the motor drivers to the motors. Make sure the connections are strong and insulated to avoid short circuits.

4. **Install the LSM9DS0 Sensor**: Securely mount the LSM9DS0 sensor on the robot. Ensure it's firmly attached and positioned correctly according to its datasheet specifications.

5. **Install the HC-SR04 Sensor**: Mount the HC-SR04 sensor on the front of the robot for obstacle detection.

6. **Setup the Power Supply**: Connect the battery to the power supply circuit. Make sure to follow proper polarity and secure all connections.

7. **Final Wiring**: Connect all components (motors, sensors, Bluetooth module) to the mbed microcontroller according to the circuit diagram provided in the project repository.

## Software Configuration
1. **mbed Compiler / Keil Studio Setup**:
   - Log in to your mbed account and set up your environment in the mbed compiler or Keil Studio.
   - Import the provided project code from the repository.

2. **Code Customization**:
   - Review the code and make any necessary customizations specific to your hardware setup (e.g., pin assignments, sensor calibration).

3. **Compile and Upload**:
   - Compile the code in the mbed compiler or Keil Studio.
   - Connect the mbed microcontroller to your computer via USB.
   - Upload the compiled binary to the microcontroller.

## Testing and Calibration
1. **Basic Functionality Test**:
   - Power on the robot.
   - Test basic movements to ensure motors and sensors are functioning correctly.

2. **Calibrate the Sensors**:
   - Follow the instructions in the code comments to calibrate the LSM9DS0 and HC-SR04 sensors.

3. **PID Tuning**:
   - Begin with PID tuning. This process can be time-consuming but is crucial for stable balancing.
   - Adjust Kp, Ki, and Kd values iteratively, testing the robot's response after each adjustment.

4. **Bluetooth Connectivity Test**:
   - Pair the robot with a Bluetooth-enabled device.
   - Test remote control features through the Adafruit Bluefruit LE UART Friend module.

## Troubleshooting
- If the robot is not balancing properly, revisit the PID tuning process and sensor calibration.
- Ensure all connections are secure and free of short circuits.
- Check battery charge levels before each testing session.


## Additional Resources
- [mbed Documentation](https://os.mbed.com/docs/mbed-os/v6.15/introduction/index.html)
- [Keil Studio](https://www.keil.com/)

Congratulations on setting up your Balancing Robot! For more advanced features and project enhancements, explore the additional resources and documentation in our GitHub repository.
