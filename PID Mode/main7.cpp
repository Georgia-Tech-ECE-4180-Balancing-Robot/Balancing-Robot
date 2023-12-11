#include "mbed.h"
#include "LSM9DS0.h"
#include "Motor.h"
#include "rtos.h"
RawSerial pc(USBTX, USBRX);     // Serial connection to PC
RawSerial  dev(p28,p27); // tx, rx - Adafruit BLE
LSM9DS0 imu(p9, p10, 0x6B, 0x1D); // Adjust the pins and address as needed
Timer t;
Motor leftMotor(p21, p6, p5); // pwm, fwd, rev
Motor rightMotor(p22, p8, p7); // pwm, fwd, rev
Timer sonar;
DigitalOut trigger(p30);
DigitalIn echo(p29);
int correction;
Thread sonarThread;
Thread bluetoothThread;
// debug
// PID parameters
double Kp = 0.0248;  // Proportional gain
double Ki = 0.000014 * 30;  // Integral gain
double Kd = 0.002; // Derivative gain

// PID variables
double setPoint = 0;  // The desired tilt angle, probably 0 for a balancing robot
double error1, previousError = 0, integral = 0, derivative, output, gyroRate, compAngle, alpha = 0.95;
//vars for IMU 
float accBias, gyroBias, t1, motorBias;
float accAngle, oldAngle;
void calibrate()
{       
    for(int i=0; i<100; i++) {                                 // Read a thousand values
        imu.readAccel();                                        // Read the Accelerometer
        //imu.readGyro();                                         // Read the gyro
        accBias=accBias+ (-1) * atan2(imu.ay,imu.az)*180/3.142-90; // Like this, 0 is upright, forward is negative, backward positive
        //gyroBias=gyroBias+imu.gx; 
    }
    accBias=accBias/100;                                       // Convert sum to average
    //gyroBias=gyroBias/100;                                     // Convert sum to average
    // #ifdef DEBUG
    // pc.printf("bias: %f\n\r", accBias);
    // #endif
}

void sonarFunction() {
    // Put your sonar code here
    while (true) {
        // Sonar distance measurement logic
        // Trigger sonar to send a ping
    trigger = 1;
    // myled = 1;
    // myled2 = 0;
    sonar.reset();
    wait_us(10.0);
    trigger = 0;
    // myled = 0;

    // Wait for echo high
    while (echo == 0) {};
    // myled2 = echo;

    // Echo high, so start timer
    sonar.start();

    // Wait for echo low
    while (echo == 1) {};

    // Stop timer and read value
    sonar.stop();
    // myled2 = 0;

    // Subtract software overhead timer delay and scale to cm
    int distance = (sonar.read_us() - correction) / 58.0;
    if(distance <= 20){
        rightMotor.speed(0);
        leftMotor.speed(0);
    }
    printf(" %d cm \n\r", distance);
    wait(0.1); // Adjust the frequency as needed
    }
}

void bluetoothFunction() {
    
    // dev.baud(9600); // Set baud rate for the Bluetooth module
    while (true) {
        if (dev.getc() == '!') {
            if (dev.getc()=='B') { //button data
            char c = dev.getc();
            switch (c) {
                case '1':
                    // Full speed forward
                    leftMotor.speed(1);
                    rightMotor.speed(1);
                    break;
                case '2':
                    // Full speed backward
                    rightMotor.speed(-1);
                    leftMotor.speed(1);
                    break;
                case '3':
                    // Standard speed/mode
                    rightMotor.speed(0);
                    leftMotor.speed(0);

                    break;
                case '4':
                    // spinning
                    rightMotor.speed(1);
                    leftMotor.speed(-1);
                // Add more cases as needed
            }
            }
        }
        wait(0.1); // Adjust the frequency as needed
    }
}

int main() {
    // pc.baud(9600);  // Set baud rate for serial communication, cannot with rawserial
    sonarThread.start(sonarFunction);
    bluetoothThread.start(bluetoothFunction);
    uint16_t status = imu.begin();    // Initialize the LSM9DS0
    calibrate();
    pc.printf("LSM9DS0 WHO_AM_I's returned: 0x");
    pc.printf("%x\n",status);
    pc.printf("Should be 0x49D4\n");
    pc.printf("It's alive!\n");
    pc.printf("\n\r");
    wait(0.5);
    t.start();

    while (true) {
        imu.readAccel(); // Read accelerometer
        // imu.readGyro();  // Read gyroscope
        // Getting the angle
        // Print accelerometer data
        accAngle= (-1) * atan2(imu.ay,imu.az)*180/3.142-90-accBias;
        // compAngle = accAngle;
        

        #ifdef DEBUG
        pc.printf("Accel - X: %f, Y: %f, Z: %f\n\r", imu.ax, imu.ay, imu.az);
        pc.printf("Angle: %f\n\r\n", accAngle);
        
        // Print gyroscope data
        //pc.printf("Gyro - X: %f, Y: %f, Z: %f\n\r", imu.gx, imu.gy, imu.gz);

        // Print magnetometer data
        //pc.printf("Mag - X: %f, Y: %f, Z: %f\n\r", imu.mx, imu.my, imu.mz);
        #endif
        //PID calculations
        // float t1 = t.read();
        // Calculate time delta
        // float t2 = t.read();
        // // time diff
        // dt = t2 - t1;
        // t1 = t2;

        // convert gyroscope data to degrees per second
        //gyroRate = (((-1) * atan2(imu.gy, imu.gx) * 180 / 3.142 - gyroBias)) / 10;
        //float gyroAngleChange = gyroRate * t1;
        //pc.printf("Change: %.4f\n\n\r", gyroAngleChange);
        // Comp filter

        // to make it run better, i've used some shortcuts
        //compAngle = accAngle; // gyroAngleChange * alpha; // ?
        // error1 = setPoint - compAngle; // Assuming pitch is what you're balancing on
        t1 = t.read();

        if(abs(accAngle) < 30) {
            integral += -accAngle * t1; // scaling
        } 
        derivative = (-accAngle - previousError) / t1; // scaling
        
        output = Kp * -accAngle + Ki * integral - Kd * derivative; // final output
        previousError = -accAngle; // storing previous error
        t.reset();

        //t.reset();
        #ifdef DEBUG
        pc.printf("Error: %f\n\r", accAngle);
        // Add a delay between reads for readability
        wait(0.1);
        #endif
        // Motor control logic based on PID output
        // leftMotor = output > 0 ? output : 0;   // Example control logic
        // rightMotor = output > 0 ? output : 0;  // Example control logic
        output = output;
        if(output > 1) { output = 1; }
        else if(output < -1) { output = -1; }
    
        leftMotor.speed(output);
        rightMotor.speed(output);
        
        // wait(0.0018);
        // Debug output
        // P is error
        pc.printf("PID: P ( %.4f )   ||   I ( %.4f )   ||  D (%.4f) \n\r", -accAngle * Kp, integral, derivative);
        // pc.printf("Error: %.4f, Gryo: X\n\r", error1);
        pc.printf("Pitch: %.4f, Motor Output: %f\n\r", -accAngle, output);
        //wait(0.7);  // delay for readibility
        //wait(0.00009); // for a consistent sampling rate
    }
}
