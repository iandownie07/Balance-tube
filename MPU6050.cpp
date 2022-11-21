//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Include the header file for this class

#include <functional>
#include<stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "MPU6050.h"

float dt; //Loop time (recalculated with each loop)

float _accel_angle[3];
float _gyro_angle[3];
float _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

float a[3]; //Temporary storage variables used in _update()
float g[3]; //Temporary storage variables used in _update()




typedef struct
{
	void *getGyro;
	void *getAccel;

} queue_entry_t;		


void  getGyroRaw(float gyro[3])
{
    uint8_t buffer[6];
    
    //Gyro data
    uint8_t reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDRESS, &reg, sizeof(reg), false );
    i2c_read_blocking(I2C_PORT, MPU6050_ADDRESS, buffer, sizeof(buffer), false );
    
    int16_t X = (buffer[0] << 8) | buffer[1];
    int16_t Y = (buffer[2] << 8) | buffer[3];
    int16_t Z = (buffer[4] << 8) | buffer[5];
	
	gyro[0] = (float)X;
    gyro[1] = (float)Y;
	gyro[2] = (float)Z;    
}

void  getAccelRaw(float accelerometer[3])
{
    uint8_t buffer[6];
    // reading the accelerometer data
    uint8_t reg = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDRESS, &reg, sizeof(reg), false );
    i2c_read_blocking(I2C_PORT, MPU6050_ADDRESS, buffer, sizeof(buffer), false );

	int16_t X = (buffer[0] << 8) | buffer[1];
    int16_t Y = (buffer[2] << 8) | buffer[3];
    int16_t Z = (buffer[4] << 8) | buffer[5];
    
    accelerometer[0] = (float)X;
    accelerometer[1] = (float)Y;
    accelerometer[2] = (float)Z;   
}
 
void getGyro(float gyro[3]) {
	getGyroRaw(gyro); //Store raw values into variables
	gyro[0] = round((gyro[0] - G_OFF_X) * 1000.0 / GYRO_SENS) / 1000.0; //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
	gyro[1] = round((gyro[1] - G_OFF_Y) * 1000.0 / GYRO_SENS) / 1000.0; // Pitch
	gyro[2] = round((gyro[2] - G_OFF_Z) * 1000.0 / GYRO_SENS) / 1000.0;

	//return gyro;
}


void getAccel(float accelerometer[3]) {
	getAccelRaw(accelerometer); //Store raw values into variables
	accelerometer[0] = round((accelerometer[0] - A_OFF_X) * 1000.0 / ACCEL_SENS) / 1000.0; //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
	accelerometer[1] = round((accelerometer[1] - A_OFF_Y) * 1000.0 / ACCEL_SENS) / 1000.0;
	accelerometer[2] = round((accelerometer[2] - A_OFF_Z) * 1000.0 / ACCEL_SENS) / 1000.0;

	//return accelerometer;
}

void  getOffsets(float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off) {
	float gyro_off[3]; //Temporary storage
	float accel_off[3];

	*gr_off = 0, *gp_off = 0, *gy_off = 0; //Initialize the offsets to zero
	*ax_off = 0, *ay_off = 0, *az_off = 0; //Initialize the offsets to zero

	for (int i = 0; i < 10000; i++) { //Use loop to average offsets
		getGyroRaw(gyro_off); //Raw gyroscope values
		*gr_off = *gr_off + gyro_off[0], *gp_off = *gp_off + gyro_off[1], *gy_off = *gy_off + gyro_off[2]; //Add to sum

		getAccelRaw(accel_off); //Raw accelerometer values
		*ax_off = *ax_off + accel_off[0], *ay_off = *ay_off + accel_off[1], *az_off = *az_off + accel_off[2]; //Add to sum
	}

	*gr_off = *gr_off / 10000, *gp_off = *gp_off / 10000, *gy_off = *gy_off / 10000; //Divide by number of loops (to average)
	*ax_off = *ax_off / 10000, *ay_off = *ay_off / 10000, *az_off = *az_off / 10000;

	*az_off = *az_off - ACCEL_SENS; //Remove 1g from the value calculated to compensate for gravity)
}

int  getAngle(int axis, float *result) {
	if (axis >= 0 && axis <= 2) { //Check that the axis is in the valid range
		*result = _angle[axis]; //Get the result
		return 0;
	}
	else {
		std::cout << "ERR (MPU6050.cpp:getAngle()): 'axis' must be between 0 and 2 (for roll, pitch or yaw)\n"; //Print error message
		*result = 0; //Set result to zero
		return 1;
	}
}

void  _update() { //Main update function - runs continuously
	sleep_ms(20);
	dt = 0.009; //Loop time (recalculated with each loop)
	uint32_t start_time = time_us_32(); //Read current time into start variable
	bool calc_yaw = false;
	bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter

	while (1) { //Loop forever

		getGyro(g); //Get the data from the sensors
		getAccel(a);

		//_angle[0] = atan2(a[1], a[2]) * RAD_T_DEG; 

		//_angle[1] = atan2(-a[0], sqrt(a[1]*a[1] + a[2]*a[2])) * RAD_T_DEG;
		
		//X (roll) axis
		_accel_angle[0] = atan2(a[2], a[1]) * RAD_T_DEG - 90; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
		_gyro_angle[0] = _angle[0] + g[0]*dt; //Use roll axis (X axis)

		//Y (pitch) axis
		_accel_angle[1] = atan2(a[2], a[0]) * RAD_T_DEG - 90; //Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
		_gyro_angle[1] = _angle[1] + g[1]*dt; //Use pitch axis (Y axis)

		//Z (yaw) axis
		if (calc_yaw) {
			_gyro_angle[2] = _angle[2] + g[2]*dt; //Use yaw axis (Z axis)
		}


		if (_first_run) { //Set the gyroscope angle reference point if this is the first function run
			for (int i = 0; i <= 1; i++) {
				_gyro_angle[i] = _accel_angle[i]; //Start off with angle from accelerometer (absolute angle since gyroscope is relative)
			}
			_gyro_angle[2] = 0; //Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
			_first_run = 0;
		}

		float asum = abs(a[0]) + abs(a[1]) + abs(a[2]); //Calculate the sum of the accelerations
		float gsum = abs(g[0]) + abs(g[1]) + abs(g[2]); //Calculate the sum of the gyro readings

		for (int i = 0; i <= 1; i++) { //Loop through roll and pitch axes
			if (abs(_gyro_angle[i] - _accel_angle[i]) > 5) { //Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
				_gyro_angle[i] = _accel_angle[i];
			}

			//Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
			if (asum > 0.1 && asum < 3 && gsum > 0.3) { //Check that th movement is not very high (therefore providing inacurate angles)
				_angle[i] = (1 - TAU)*(_gyro_angle[i]) + (TAU)*(_accel_angle[i]); //Calculate the angle using a complementary filter
			}
			else if (gsum > 0.3) { //Use the gyroscope angle if the acceleration is high
				_angle[i] = _gyro_angle[i];
			}
			else if (gsum <= 0.3) { //Use accelerometer angle if not much movement
				_angle[i] = _accel_angle[i];
			}
		}

		//The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
		if (calc_yaw) { //Only calculate the angle when we want it to prevent large drift
			_angle[2] = _gyro_angle[2];
		}
		else {
			_angle[2] = 0;
			_gyro_angle[2] = 0;
		}

		uint32_t finish_time = time_us_32(); //Save time to end clock
		//static uint32_t dt = absolute_time_diff_us(start_time, finish_time)/1e6; //Calculate new dt
		//std::cout << "Current dt: " << dt << "\n";
		start_time = time_us_32(); //Save time to start clock
		
	}
}

void  MPU6050_Reset()
{
    int status;

	
	
	uint8_t reg[] = {0x6B, 0x00};
	uint8_t reg1[] = {0x1B, GYRO_CONFIG};
	uint8_t reg2[] = {0x1A, 0b00000011};
	uint8_t reg3[] = {0x19, 0b00000100};
	uint8_t reg4[] = {0x1C, ACCEL_CONFIG};
	
    i2c_write_blocking(I2C_PORT, MPU6050_ADDRESS, reg, sizeof(reg), false );
}
