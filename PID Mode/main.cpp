#include "mbed.h"
#include "LSM9DS0.h"
#include "Motor.h"

#define PI 3.141592653

#define Kp 6.00
#define Ki 0.001
#define Kd 0.50

Serial pc(USBTX, USBRX);     // Serial connection to PC
LSM9DS0 imu(p9, p10, 0x6B, 0x1D); // Adjust the pins and address as needed
Timer t;
Motor leftMotor(p21, p6, p5); // pwm, fwd, rev
Motor rightMotor(p22, p8, p7); // pwm, fwd, rev

double accAngle;
double gyroAngle;

double oldTime;
double newTime;
double deltaTime;

double setPoint = 102.5;
double errorVal = 0.00;
double errorOld;

double Pval, Ival, Dval;

double output;

void calibrate()
{

    double angle;

    imu.readAccel();
    imu.readGyro();

    angle = (180/PI) * atan2(-imu.ay , sqrt(imu.ax*imu.ax + imu.az*imu.az));

    gyroAngle = angle;
}

int main() {
    pc.baud(9600);  // Set baud rate for serial communication
    uint16_t status = imu.begin();    // Initialize the LSM9DS0
    //calibrate();
    pc.printf("LSM9DS0 WHO_AM_I's returned: 0x");
    pc.printf("%x\n",status);
    pc.printf("Should be 0x49D4\n");
    pc.printf("It's alive!\n");
    pc.printf("\n\r");
    wait(0.5);
    t.start();
    newTime = t.read();

    while (true) {

        // Timey Wimey wibbly Wobbly
        oldTime = newTime;
        newTime = t.read();
        deltaTime = oldTime - newTime;

        imu.readAccel(); // Read accelerometer
        //imu.readGyro();  // Read gyroscope
        
        // Calculate angle from Accel
        accAngle = atan2(-imu.ay , imu.az) * 180/PI;

        // Calculate angle from Gyro
        //gyroAngle = gyroAngle + (deltaTime * (imu.gx));

        // PID Controller

        // Print accel angle
        pc.printf("Angle: %f ", accAngle);

        errorVal = setPoint - accAngle;

        // Print error angle
        pc.printf("Error: %f ", errorVal);

        Pval = errorVal;

        if(abs(errorVal) < 50)
        {
            Ival -= (errorVal * deltaTime);
        }

        Dval = ((errorVal - errorOld)/deltaTime);

        output = (Kp * Pval) + (Ki * Ival) + (Kd * Dval);     

        if((abs(errorVal) < 0.01) || (errorVal*errorOld < 0))
        {
            Ival = 0;
        }

        if(output > 255) {
            output = 255; 
        }
        else if(output < -255) {
            output = -255;
        }

        pc.printf("P: %f I: %f D: %f Out: %f\n\r", Pval, Ival, Dval, output);

        leftMotor.speed(output/255);
        rightMotor.speed(output/255);
       
    }
}
