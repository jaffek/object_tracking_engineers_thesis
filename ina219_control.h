#ifndef INA219_CONTROL_H
#define  INA219_CONTROL_H

#include <unistd.h>				
#include <fcntl.h>				
#include <sys/ioctl.h>			
#include <linux/i2c-dev.h>	
#include <iostream>

using namespace std;
	
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 // INA219 module control //
// open I2C port
int open_I2C();

// I2C slave connection
void adress_I2C(int adress, int &device_I2C_id);

// INA219 calibration
void ina_219_calibration(int &device_I2C_id);

// INA219 configuration
void ina_219_configuration(int &device_I2C_id);

// Read and calculate supply voltage
float supply_voltage_calc(int &device_I2C_id);

// Read and calculate current
float current_calc(int &device_I2C_id);

#endif
