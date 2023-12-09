// Most of the Credit goes to jimblom
// Modifications by Allen Wild
#ifndef _LSM9DS0_H__
#define _LSM9DS0_H__

#include "mbed.h"

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G          0x0F
#define CTRL_REG1_G         0x20
#define CTRL_REG2_G         0x21
#define CTRL_REG3_G         0x22
#define CTRL_REG4_G         0x23
#define CTRL_REG5_G         0x24
#define REFERENCE_G         0x25
#define STATUS_REG_G        0x27
#define OUT_X_L_G           0x28
#define OUT_X_H_G           0x29
#define OUT_Y_L_G           0x2A
#define OUT_Y_H_G           0x2B
#define OUT_Z_L_G           0x2C
#define OUT_Z_H_G           0x2D
#define FIFO_CTRL_REG_G     0x2E
#define FIFO_SRC_REG_G      0x2F
#define INT1_CFG_G          0x30
#define INT1_SRC_G          0x31
#define INT1_THS_XH_G       0x32
#define INT1_THS_XL_G       0x33
#define INT1_THS_YH_G       0x34
#define INT1_THS_YL_G       0x35
#define INT1_THS_ZH_G       0x36
#define INT1_THS_ZL_G       0x37
#define INT1_DURATION_G     0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM       0x05
#define OUT_TEMP_H_XM       0x06
#define STATUS_REG_M        0x07
#define OUT_X_L_M           0x08
#define OUT_X_H_M           0x09
#define OUT_Y_L_M           0x0A
#define OUT_Y_H_M           0x0B
#define OUT_Z_L_M           0x0C
#define OUT_Z_H_M           0x0D
#define WHO_AM_I_XM         0x0F
#define INT_CTRL_REG_M      0x12
#define INT_SRC_REG_M       0x13
#define INT_THS_L_M         0x14
#define INT_THS_H_M         0x15
#define OFFSET_X_L_M        0x16
#define OFFSET_X_H_M        0x17
#define OFFSET_Y_L_M        0x18
#define OFFSET_Y_H_M        0x19
#define OFFSET_Z_L_M        0x1A
#define OFFSET_Z_H_M        0x1B
#define REFERENCE_X         0x1C
#define REFERENCE_Y         0x1D
#define REFERENCE_Z         0x1E
#define CTRL_REG0_XM        0x1F
#define CTRL_REG1_XM        0x20
#define CTRL_REG2_XM        0x21
#define CTRL_REG3_XM        0x22
#define CTRL_REG4_XM        0x23
#define CTRL_REG5_XM        0x24
#define CTRL_REG6_XM        0x25
#define CTRL_REG7_XM        0x26
#define STATUS_REG_A        0x27
#define OUT_X_L_A           0x28
#define OUT_X_H_A           0x29
#define OUT_Y_L_A           0x2A
#define OUT_Y_H_A           0x2B
#define OUT_Z_L_A           0x2C
#define OUT_Z_H_A           0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT_GEN_1_REG       0x30
#define INT_GEN_1_SRC       0x31
#define INT_GEN_1_THS       0x32
#define INT_GEN_1_DURATION  0x33
#define INT_GEN_2_REG       0x34
#define INT_GEN_2_SRC       0x35
#define INT_GEN_2_THS       0x36
#define INT_GEN_2_DURATION  0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F

/**
 * LSM9DS0 Class - driver for the 9 DoF IMU
 */
class LSM9DS0
{
public:

    /// gyro_scale defines the possible full-scale ranges of the gyroscope:
    enum gyro_scale
    {
        G_SCALE_245DPS,     // 00: +/- 245 degrees per second
        G_SCALE_500DPS,     // 01: +/- 500 dps
        G_SCALE_2000DPS,    // 10: +/- 2000 dps
    };

    /// accel_scale defines all possible FSR's of the accelerometer:
    enum accel_scale
    {
        A_SCALE_2G, // 000: +/- 2g
        A_SCALE_4G, // 001: +/- 4g
        A_SCALE_6G, // 010: +/- 6g
        A_SCALE_8G, // 011: +/- 8g
        A_SCALE_16G // 100: +/- 16g
    };

    /// mag_scale defines all possible FSR's of the magnetometer:
    enum mag_scale
    {
        M_SCALE_2GS,    // 00: +/- 2Gs
        M_SCALE_4GS,    // 01: +/- 4Gs
        M_SCALE_8GS,    // 10: +/- 8Gs
        M_SCALE_12GS,   // 11: +/- 12Gs
    };

    /// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
    enum gyro_odr
    {                           // ODR (Hz) --- Cutoff
        G_ODR_95_BW_125  = 0x0, //   95         12.5
        G_ODR_95_BW_25   = 0x1, //   95          25
        // 0x2 and 0x3 define the same data rate and bandwidth
        G_ODR_190_BW_125 = 0x4, //   190        12.5
        G_ODR_190_BW_25  = 0x5, //   190         25
        G_ODR_190_BW_50  = 0x6, //   190         50
        G_ODR_190_BW_70  = 0x7, //   190         70
        G_ODR_380_BW_20  = 0x8, //   380         20
        G_ODR_380_BW_25  = 0x9, //   380         25
        G_ODR_380_BW_50  = 0xA, //   380         50
        G_ODR_380_BW_100 = 0xB, //   380         100
        G_ODR_760_BW_30  = 0xC, //   760         30
        G_ODR_760_BW_35  = 0xD, //   760         35
        G_ODR_760_BW_50  = 0xE, //   760         50
        G_ODR_760_BW_100 = 0xF, //   760         100
    };

    /// accel_oder defines all possible output data rates of the accelerometer:
    enum accel_odr
    {
        A_POWER_DOWN,   // Power-down mode (0x0)
        A_ODR_3125,     // 3.125 Hz (0x1)
        A_ODR_625,      // 6.25 Hz (0x2)
        A_ODR_125,      // 12.5 Hz (0x3)
        A_ODR_25,       // 25 Hz (0x4)
        A_ODR_50,       // 50 Hz (0x5)
        A_ODR_100,      // 100 Hz (0x6)
        A_ODR_200,      // 200 Hz (0x7)
        A_ODR_400,      // 400 Hz (0x8)
        A_ODR_800,      // 800 Hz (9)
        A_ODR_1600      // 1600 Hz (0xA)
    };
    /// accel_oder defines all possible output data rates of the magnetometer:
    enum mag_odr
    {
        M_ODR_3125, // 3.125 Hz (0x00)
        M_ODR_625,  // 6.25 Hz (0x01)
        M_ODR_125,  // 12.5 Hz (0x02)
        M_ODR_25,   // 25 Hz (0x03)
        M_ODR_50,   // 50 (0x04)
        M_ODR_100,  // 100 Hz (0x05)
    };

    // We'll store the gyro, accel, and magnetometer readings in a series of
    // public class variables. Each sensor gets three variables -- one for each
    // axis. Call readGyro(), readAccel(), and readMag() first, before using
    // these variables!
    // These values are the RAW signed 16-bit readings from the sensors.
    int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
    int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
    int16_t mx_raw, my_raw, mz_raw; // x, y, and z axis readings of the magnetometer
    int16_t temperature_raw;

	// floating-point values of scaled data in real-world units
	float gx, gy, gz;
	float ax, ay, az;
	float mx, my, mz;
	float temperature_c, temperature_f; // temperature in celcius and fahrenheit

    float abias[3];
    float gbias[3];

    
    /**  LSM9DS0 -- LSM9DS0 class constructor
    *  The constructor will set up a handful of private variables, and set the
    *  communication mode as well.
    *  Input:
    *   - interface = Either MODE_SPI or MODE_I2C, whichever you're using
    *               to talk to the IC.
    *   - gAddr = If MODE_I2C, this is the I2C address of the gyroscope.
    *               If MODE_SPI, this is the chip select pin of the gyro (CSG)
    *   - xmAddr = If MODE_I2C, this is the I2C address of the accel/mag.
    *               If MODE_SPI, this is the cs pin of the accel/mag (CSXM)
	*/
    LSM9DS0(PinName sda, PinName scl, uint8_t gAddr, uint8_t xmAddr);
    
    /**  begin() -- Initialize the gyro, accelerometer, and magnetometer.
    *  This will set up the scale and output rate of each sensor. It'll also
    *  "turn on" every sensor and every axis of every sensor.
    *  Input:
    *   - gScl = The scale of the gyroscope. This should be a gyro_scale value.
    *   - aScl = The scale of the accelerometer. Should be a accel_scale value.
    *   - mScl = The scale of the magnetometer. Should be a mag_scale value.
    *   - gODR = Output data rate of the gyroscope. gyro_odr value.
    *   - aODR = Output data rate of the accelerometer. accel_odr value.
    *   - mODR = Output data rate of the magnetometer. mag_odr value.
    *  Output: The function will return an unsigned 16-bit value. The most-sig
    *       bytes of the output are the WHO_AM_I reading of the accel. The
    *       least significant two bytes are the WHO_AM_I reading of the gyro.
    *  All parameters have a defaulted value, so you can call just "begin()".
    *  Default values are FSR's of: +/- 245DPS, 2g, 2Gs; ODRs of 95 Hz for 
    *  gyro, 100 Hz for accelerometer, 100 Hz for magnetometer.
    *  Use the return value of this function to verify communication.
	*/
    uint16_t begin(gyro_scale gScl = G_SCALE_245DPS, 
                accel_scale aScl = A_SCALE_2G, mag_scale mScl = M_SCALE_2GS,
                gyro_odr gODR = G_ODR_95_BW_125, accel_odr aODR = A_ODR_50, 
                mag_odr mODR = M_ODR_50);
    
    /**  readGyro() -- Read the gyroscope output registers.
    *  This function will read all six gyroscope output registers.
    *  The readings are stored in the class' gx_raw, gy_raw, and gz_raw variables. Read
    *  those _after_ calling readGyro().
	*/
    void readGyro();
    
    /**  readAccel() -- Read the accelerometer output registers.
    *  This function will read all six accelerometer output registers.
    *  The readings are stored in the class' ax_raw, ay_raw, and az_raw variables. Read
    *  those _after_ calling readAccel().
	*/
    void readAccel();
    
    /**  readMag() -- Read the magnetometer output registers.
    *  This function will read all six magnetometer output registers.
    *  The readings are stored in the class' mx_raw, my_raw, and mz_raw variables. Read
    *  those _after_ calling readMag().
	*/
    void readMag();
    
    /**  readTemp() -- Read the temperature output register.
    *  This function will read two temperature output registers.
    *  The combined readings are stored in the class' temperature variables. Read
    *  those _after_ calling readTemp().
	*/
    void readTemp();
    
    /**  setGyroScale() -- Set the full-scale range of the gyroscope.
    *  This function can be called to set the scale of the gyroscope to 
    *  245, 500, or 200 degrees per second.
    *  Input:
    *   - gScl = The desired gyroscope scale. Must be one of three possible
    *       values from the gyro_scale enum.
	*/
    void setGyroScale(gyro_scale gScl);
    
    /**  setAccelScale() -- Set the full-scale range of the accelerometer.
    *  This function can be called to set the scale of the accelerometer to
    *  2, 4, 6, 8, or 16 g's.
    *  Input:
    *   - aScl = The desired accelerometer scale. Must be one of five possible
    *       values from the accel_scale enum.
	*/
    void setAccelScale(accel_scale aScl);
    
    /**  setMagScale() -- Set the full-scale range of the magnetometer.
    *  This function can be called to set the scale of the magnetometer to
    *  2, 4, 8, or 12 Gs.
    *  Input:
    *   - mScl = The desired magnetometer scale. Must be one of four possible
    *       values from the mag_scale enum.
	*/
    void setMagScale(mag_scale mScl);
    
    /**  setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    *  Input:
    *   - gRate = The desired output rate and cutoff frequency of the gyro.
    *       Must be a value from the gyro_odr enum (check above, there're 14).
	*/
    void setGyroODR(gyro_odr gRate);
    
    /**  setAccelODR() -- Set the output data rate of the accelerometer
    *  Input:
    *   - aRate = The desired output rate of the accel.
    *       Must be a value from the accel_odr enum (check above, there're 11).
	*/
    void setAccelODR(accel_odr aRate);
    
    /**  setMagODR() -- Set the output data rate of the magnetometer
    *  Input:
    *   - mRate = The desired output rate of the mag.
    *       Must be a value from the mag_odr enum (check above, there're 6).
	*/
    void setMagODR(mag_odr mRate);
    
    /**  configGyroInt() -- Configure the gyro interrupt output.
    *  Triggers can be set to either rising above or falling below a specified
    *  threshold. This function helps setup the interrupt configuration and 
    *  threshold values for all axes.
    *  Input:
    *   - int1Cfg = A 8-bit value that is sent directly to the INT1_CFG_G
    *       register. This sets AND/OR and high/low interrupt gen for each axis
    *   - int1ThsX = 16-bit interrupt threshold value for x-axis
    *   - int1ThsY = 16-bit interrupt threshold value for y-axis
    *   - int1ThsZ = 16-bit interrupt threshold value for z-axis
    *   - duration = Duration an interrupt holds after triggered. This value
    *       is copied directly into the INT1_DURATION_G register.
    *  Before using this function, read about the INT1_CFG_G register and
    *  the related INT1* registers in the LMS9DS0 datasheet.
	*/
    void configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX = 0,
                          uint16_t int1ThsY = 0, uint16_t int1ThsZ = 0, 
                          uint8_t duration = 0);
                          
    void calcBias();

	/**  return a comass heading (in degrees) using X/Y magnetometer data */
	float calcHeading();


private:    
    /**  xmAddress and gAddress store the I2C address
    *  for each sensor.
	*/
    uint8_t xmAddress, gAddress;
    
    /**  gScale, aScale, and mScale store the current scale range for each 
    *  sensor. Should be updated whenever that value changes.
	*/
    gyro_scale gScale;
    accel_scale aScale;
    mag_scale mScale;
    
    /**  gRes, aRes, and mRes store the current resolution for each sensor. 
    *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
    *  This value is calculated as (sensor scale) / (2^15).
	*/
    float gRes, aRes, mRes;
    
    /**  initGyro() -- Sets up the gyroscope to begin reading.
    *  This function steps through all five gyroscope control registers.
    *  Upon exit, the following parameters will be set:
    *   - CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
    *       95 Hz ODR, 12.5 Hz cutoff frequency.
    *   - CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
    *       set to 7.2 Hz (depends on ODR).
    *   - CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
    *       active high). Data-ready output enabled on DRDY_G.
    *   - CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
    *       address. Scale set to 245 DPS. SPI mode set to 4-wire.
    *   - CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
	*/
    void initGyro();
    
    /**  initAccel() -- Sets up the accelerometer to begin reading.
    *  This function steps through all accelerometer related control registers.
    *  Upon exit these registers will be set as:
    *   - CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
    *   - CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
    *       all axes enabled.
    *   - CTRL_REG2_XM = 0x00: +/- 2g scale. 773 Hz anti-alias filter BW.
    *   - CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	*/
    void initAccel();
    
    /**  initMag() -- Sets up the magnetometer to begin reading.
    *  This function steps through all magnetometer-related control registers.
    *  Upon exit these registers will be set as:
    *   - CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
    *   - CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
    *       requests don't latch. Temperature sensor disabled.
    *   - CTRL_REG6_XM = 0x00: +/- 2 Gs scale.
    *   - CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
    *   - INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	*/
    void initMag();
    
    /**  gReadByte() -- Reads a byte from a specified gyroscope register.
    *  Input:
    *   - subAddress = Register to be read from.
    *  Output:
    *   - An 8-bit value read from the requested address.
	*/
    uint8_t gReadByte(uint8_t subAddress);
    
    /**  gWriteByte() -- Write a byte to a register in the gyroscope.
    *  Input:
    *   - subAddress = Register to be written to.
    *   - data = data to be written to the register.
	*/
    void gWriteByte(uint8_t subAddress, uint8_t data);
    
    /**  xmReadByte() -- Read a byte from a register in the accel/mag sensor
    *  Input:
    *   - subAddress = Register to be read from.
    *  Output:
    *   - An 8-bit value read from the requested register.
	*/
    uint8_t xmReadByte(uint8_t subAddress);
    
    /**  xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
    *  Input:
    *   - subAddress = Register to be written to.
    *   - data = data to be written to the register.
	*/
    void xmWriteByte(uint8_t subAddress, uint8_t data);
    
    /**  calcgRes() -- Calculate the resolution of the gyroscope.
    *  This function will set the value of the gRes variable. gScale must
    *  be set prior to calling this function.
	*/
    void calcgRes();
    
    /**  calcmRes() -- Calculate the resolution of the magnetometer.
    *  This function will set the value of the mRes variable. mScale must
    *  be set prior to calling this function.
	*/
    void calcmRes();
    
    /**  calcaRes() -- Calculate the resolution of the accelerometer.
    *  This function will set the value of the aRes variable. aScale must
    *  be set prior to calling this function.
	*/
    void calcaRes();
    
    
    ///////////////////
    // I2C Functions //
    ///////////////////
	I2C i2c;

    
    /**  I2CwriteByte() -- Write a byte out of I2C to a register in the device
    *  Input:
    *   - address = The 7-bit I2C address of the slave device.
    *   - subAddress = The register to be written to.
    *   - data = Byte to be written to the register.
	*/
    void I2CwriteByte(char address, char subAddress, char data);
    
    /**  I2CreadByte() -- Read a single byte from a register over I2C.
    *  Input:
    *   - address = The 7-bit I2C address of the slave device.
    *   - subAddress = The register to be read from.
    *  Output:
    *   - The byte read from the requested address.
	*/
    uint8_t I2CreadByte(char address, char subAddress);
};

#endif // _LSM9DS0_H //
