#include "mbed.h"
#include "LSM9DS0.h"
#include "Motor.h"
Serial pc(USBTX, USBRX);     // Serial connection to PC
LSM9DS0 imu(p9, p10, 0x6B, 0x1D); // Adjust the pins and address as needed
Timer t;
Motor leftMotor(p21, p6, p5); // pwm, fwd, rev
Motor rightMotor(p22, p8, p7); // pwm, fwd, rev
// debug
// PID parameters
double Kp = 0.05;  // Proportional gain
double Ki = 0.025;  // Integral gain
double Kd = 0.002; // Derivative gain

// PID variables
double setPoint = 0;  // The desired tilt angle, probably 0 for a balancing robot
double error1, previousError = 0, integral = 0, derivative, output;
//vars for IMU 
float accBias, gyroBias;
float motorSpeed;
float g=0.028;
float accAngle, accAngle2;
void calibrate()
{
    for(int i=0; i<100; i++) {                                 // Read a thousand values
        imu.readAccel();                                        // Read the Accelerometer
        imu.readGyro();                                         // Read the gyro
        accBias=accBias+ (-1) * atan2(imu.ay,imu.az)*180/3.142-90; // Like this, 0 is upright, forward is negative, backward positive
        gyroBias=gyroBias+imu.gx; 
    }
    accBias=accBias/100;                                       // Convert sum to average
    gyroBias=gyroBias/100;                                     // Convert sum to average
    // #ifdef DEBUG
    // pc.printf("bias: %f\n\r", accBias);
    // #endif
}

int main() {
    pc.baud(9600);  // Set baud rate for serial communication
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
        imu.readGyro();  // Read gyroscope
        //imu.readMag();   // Read magnetometer
        // Getting the angle
        // Print accelerometer data
        accAngle= (-1) * atan2(imu.ay,imu.az)*180/3.142-90-accBias;
        //#ifdef DEBUG
        pc.printf("Accel - X: %f, Y: %f, Z: %f\n\r", imu.ax, imu.ay, imu.az);
        pc.printf("Angle: %f\n\r\n", accAngle);
        
        // Print gyroscope data
        //pc.printf("Gyro - X: %f, Y: %f, Z: %f\n\r", imu.gx, imu.gy, imu.gz);

        // Print magnetometer data
        //pc.printf("Mag - X: %f, Y: %f, Z: %f\n\r", imu.mx, imu.my, imu.mz);
        //#endif


        //PID calculations
         float t1 = t.read();
        // error1 =  setPoint-(accAngle+0.3) ; // Assuming pitch is what you're balancing on
        // integral += error1 * t1;
        // derivative = (error1 - previousError) / t1;
        // output = Kp * error1 + Ki * integral + Kd * derivative;
        // pc.printf("integral: %f\n\r",integral);
        // pc.printf("Derivative: %f\n\r",derivative);
        // pc.printf("Output: %f\n\r",output);
        // previousError = error1;

        // t.reset();
        // //#ifdef DEBUG
        // pc.printf("Error: %f\n\r", error1);
        // Add a delay between reads for readability
        //wait(0.1);
       // #endif
        // Motor control logic based on PID output
        // leftMotor = output > 0 ? output : 0;   // Example control logic
        // rightMotor = output > 0 ? output : 0;  // Example control logic
  
        // Motor control logic based on PID output
accAngle2=accAngle+6.2;

if (abs(accAngle2)<=15){
 motorSpeed = -0.01 * accAngle;
}
else if (abs(accAngle2)<=45){
 motorSpeed= -0.015*accAngle;}

 else if (abs(accAngle2)<=90){
motorSpeed= -0.03*accAngle;
 }
// Set motor speeds in both cases

leftMotor.speed(motorSpeed*1.5);
rightMotor.speed(motorSpeed*1.5);

        

        // Debug output
        pc.printf("Pitch: %.4f, Motor Output: %f\n\r", accAngle, motorSpeed);
        t.reset();

    }
}
