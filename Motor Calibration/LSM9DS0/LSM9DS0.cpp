#include "LSM9DS0.h"
#include "math.h"

LSM9DS0::LSM9DS0(PinName sda, PinName scl, uint8_t gAddr, uint8_t xmAddr) : i2c(sda, scl)
{
	// xmAddress and gAddress will store the 7-bit I2C address, if using I2C.
	xmAddress = xmAddr;
	gAddress = gAddr;
}

uint16_t LSM9DS0::begin(gyro_scale gScl, accel_scale aScl, mag_scale mScl, 
						gyro_odr gODR, accel_odr aODR, mag_odr mODR)
{
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	mScale = mScl;
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable
	
	
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t gTest = gReadByte(WHO_AM_I_G);		// Read the gyro WHO_AM_I
	uint8_t xmTest = xmReadByte(WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I
	
	// Gyro initialization stuff:
	initGyro(); // This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale(gScale); // Set the gyro range
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale(aScale); // Set the accel range.
	
	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR); // Set the magnetometer output data rate.
	setMagScale(mScale); // Set the magnetometer's range.
	
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xmTest << 8) | gTest;
}

void LSM9DS0::initGyro()
{

	gWriteByte(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	gWriteByte(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	gWriteByte(CTRL_REG3_G, 0x88);	//Interrupt enabled on both INT_G  and I2_DRDY
	gWriteByte(CTRL_REG4_G, 0x00); // Set scale to 245 dps
	gWriteByte(CTRL_REG5_G, 0x00); //Init default values
	
}

void LSM9DS0::initAccel()
{
	xmWriteByte(CTRL_REG0_XM, 0x00);  
	xmWriteByte(CTRL_REG1_XM, 0x57); // 50Hz data rate, x/y/z all enabled												 
	xmWriteByte(CTRL_REG2_XM, 0x00); // Set scale to 2g
	xmWriteByte(CTRL_REG3_XM, 0x04); // Accelerometer data ready on INT1_XM (0x04)

}

void LSM9DS0::initMag()
{	
	xmWriteByte(CTRL_REG5_XM, 0x94); // Mag data rate - 100 Hz, enable temperature sensor
	xmWriteByte(CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS
	xmWriteByte(CTRL_REG7_XM, 0x00); // Continuous conversion mode
	xmWriteByte(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
	xmWriteByte(INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
}

void LSM9DS0::readAccel()
{
	uint16_t data = 0;

	//Get x
	data = xmReadByte(OUT_X_H_A);
	data <<= 8;
	data |= xmReadByte(OUT_X_L_A);
	ax_raw = data;
	ax = ax_raw * aRes;

	//Get y
	data=0;
	data = xmReadByte(OUT_Y_H_A);
	data <<= 8;
	data |= xmReadByte(OUT_Y_L_A);
	ay_raw = data;
	ay = ay_raw * aRes;

	//Get z
	data=0;
	data = xmReadByte(OUT_Z_H_A);
	data <<= 8;
	data |= xmReadByte(OUT_Z_L_A);
	az_raw = data;
	az = az_raw * aRes;
}

void LSM9DS0::readMag()
{
	uint16_t data = 0;  

	//Get x
	data = xmReadByte(OUT_X_H_M);
	data <<= 8;
	data |= xmReadByte(OUT_X_L_M);
	mx_raw = data;
	mx = mx_raw * mRes;

	//Get y
	data = xmReadByte(OUT_Y_H_M);
	data <<= 8;
	data |= xmReadByte(OUT_Y_L_M);
	my_raw = data;
	my = my_raw * mRes;

	//Get z
	data = xmReadByte(OUT_Z_H_M);
	data <<= 8;
	data |= xmReadByte(OUT_Z_L_M);
	mz_raw = data;
	mz = mz_raw * mRes;
}

void LSM9DS0::readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	
	temp[0] = xmReadByte(OUT_TEMP_L_XM);
	temp[1] = xmReadByte(OUT_TEMP_H_XM);
	
	// Temperature is a 12-bit signed integer	
	temperature_raw = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4;

	temperature_c = (float)temperature_raw / 8.0 + 25;
	temperature_f = temperature_c * 1.8 + 32;
}


void LSM9DS0::readGyro()
{	
	uint16_t data = 0;

	//Get x
	data = gReadByte(OUT_X_H_G);
	data <<= 8;
	data |= gReadByte(OUT_X_L_G);
	gx_raw = data;
	gx = gx_raw * gRes;

	//Get y
	data = gReadByte(OUT_Y_H_G);
	data <<= 8;
	data |= gReadByte(OUT_Y_L_G);
	gy_raw = data;
	gy = gy_raw * gRes;

	//Get z
	data = gReadByte(OUT_Z_H_G);
	data <<= 8;
	data |= gReadByte(OUT_Z_L_G);
	gz_raw = data;
	gz = gz_raw * gRes;
}

void LSM9DS0::setGyroScale(gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void LSM9DS0::setAccelScale(accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS0::setMagScale(mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(CTRL_REG6_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void LSM9DS0::setGyroODR(gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp);
}
void LSM9DS0::setAccelODR(accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_XM, temp);
}
void LSM9DS0::setMagODR(mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(CTRL_REG5_XM, temp);
}

void LSM9DS0::configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration)
{
	gWriteByte(INT1_CFG_G, int1Cfg);
	gWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(INT1_DURATION_G, 0x00);
}

void LSM9DS0::calcgRes()
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (gScale)
	{
		case G_SCALE_245DPS:
			gRes = 245.0 / 32768.0;
			break;
		case G_SCALE_500DPS:
			gRes = 500.0 / 32768.0;
			break;
		case G_SCALE_2000DPS:
			gRes = 2000.0 / 32768.0;
			break;
	}
}

void LSM9DS0::calcaRes()
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	aRes = aScale == A_SCALE_16G ? 16.0 / 32768.0 : 
		   (((float) aScale + 1.0) * 2.0) / 32768.0;
}

void LSM9DS0::calcmRes()
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	mRes = mScale == M_SCALE_2GS ? 2.0 / 32768.0 : 
		   (float) (mScale << 2) / 32768.0;
}

#define R2D 57.295779513F
// calculate compass heading, assuming readMag() has been called already
float LSM9DS0::calcHeading()
{
	if (my > 0)
		return 90.0 - atan(mx / my)*R2D;
	else if (my < 0)
		return 270.0 - atan(mx / my)*R2D;
	else if (mx < 0)
		return 180.0;
	else
		return 0.0;
}
	
void LSM9DS0::calcBias()
{  
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int samples, ii;

	// First get gyro bias
	uint8_t c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c | 0x40); // Enable gyro FIFO  
	wait_ms(20);					   // Wait for change to take effect
	gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
	wait_ms(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (gReadByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

	for(ii = 0; ii < samples ; ii++)
	{
		// Read the gyro data stored in the FIFO
		data[0] = gReadByte(OUT_X_L_G);
		data[1] = gReadByte(OUT_X_H_G);
		data[2] = gReadByte(OUT_Y_L_G);
		data[3] = gReadByte(OUT_Y_H_G);
		data[4] = gReadByte(OUT_Z_L_G);
		data[5] = gReadByte(OUT_Z_H_G);

		gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
	}  

	gyro_bias[0] /= samples; // average the data
	gyro_bias[1] /= samples; 
	gyro_bias[2] /= samples; 

	gbias[0] = (float)gyro_bias[0]*gRes;	// Properly scale the data to get deg/s
	gbias[1] = (float)gyro_bias[1]*gRes;
	gbias[2] = (float)gyro_bias[2]*gRes;

	c = gReadByte(CTRL_REG5_G);
	gWriteByte(CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO  
	wait_ms(20);
	gWriteByte(FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode

	//  Now get the accelerometer biases
	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c | 0x40);		// Enable accelerometer FIFO  
	wait_ms(20);								  // Wait for change to take effect
	xmWriteByte(FIFO_CTRL_REG, 0x20 | 0x1F);	// Enable accelerometer FIFO stream mode and set watermark at 32 samples
	wait_ms(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (xmReadByte(FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

	for(ii = 0; ii < samples ; ii++)
	{
		// Read the accelerometer data stored in the FIFO
		data[0] = xmReadByte(OUT_X_L_A);
		data[1] = xmReadByte(OUT_X_H_A);
		data[2] = xmReadByte(OUT_Y_L_A);
		data[3] = xmReadByte(OUT_Y_H_A);
		data[4] = xmReadByte(OUT_Z_L_A);
		data[5] = xmReadByte(OUT_Z_H_A);
		accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1./aRes); // Assumes sensor facing up!
	}  

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples; 
	accel_bias[2] /= samples; 

	abias[0] = (float)accel_bias[0]*aRes; // Properly scale data to get gs
	abias[1] = (float)accel_bias[1]*aRes;
	abias[2] = (float)accel_bias[2]*aRes;

	c = xmReadByte(CTRL_REG0_XM);
	xmWriteByte(CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO  
	wait_ms(20);
	xmWriteByte(FIFO_CTRL_REG, 0x00);		  // Enable accelerometer bypass mode
}

void LSM9DS0::gWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	I2CwriteByte(gAddress, subAddress, data);
}

void LSM9DS0::xmWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	return I2CwriteByte(xmAddress, subAddress, data);
}

uint8_t LSM9DS0::gReadByte(uint8_t subAddress)
{
	return I2CreadByte(gAddress, subAddress);
}

uint8_t LSM9DS0::xmReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address.
	return I2CreadByte(xmAddress, subAddress);
}

void LSM9DS0::I2CwriteByte(char address, char subAddress, char data)
{	
	char cmd[2] = {subAddress, data};
	i2c.write(address<<1, cmd, 2);

}

uint8_t LSM9DS0::I2CreadByte(char address, char subAddress)
{
	char data; // store the register data
	i2c.write(address<<1, &subAddress, 1, true);
	i2c.read(address<<1, &data, 1);
	
	return data;

}