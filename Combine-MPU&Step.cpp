//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code
#include <random>
#include<stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "MPU6050.h"
#include "pico_stepper.hpp"


#define SCL 19
#define SDA 18



float accelerometer[3], gyro[3];


const PicoStepperConf conf = {
 pin1: 13,
 pin2: 11,
 pin3: 12,
 pin4: 10,
 total_steps: 200,
 initial_speed: 50
};

PicoStepper stepper(conf);


int main() {
   
    //intialize stdio
    stdio_init_all();
    //intialize i2c1
    i2c_init(I2C_PORT, 100*1000);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    
    // need to enable the pullups
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);

    MPU6050_Reset();

	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

	sleep_ms(2000); //Wait for the MPU6050 to stabilize


	//Calculate the offsets
	//std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	//getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	//std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";


	//Read the current yaw angle
	//device.calc_yaw = true;
	multicore_launch_core1(_update);

	while(1)
	{
		getAngle(0, &gr);
		getAngle(1, &gp);
		getAngle(2, &gy);
		std::cout << "Current angle around the roll axis: " << gr << "\n";
		std::cout << "Current angle around the pitch axis: " << gp << "\n";
		std::cout << "Current angle around the yaw axis: " << gy << "\n";

		// tilted up gives a negative angle
		if (gp < 0) // I need to know if a positive gp means it slopes up or down. If it slopes up, I need to give stepper.step a positive value
		{
			stepper.step(200); // positive takes it anti-clockwise => we want more elevator force
		}
		else if (gp > 0) 
		{
			stepper.step(-200); // negative takes it clockwise => we want less elevator force
		}
		
	

	sleep_ms(2000);
	}

	return 0;
}


