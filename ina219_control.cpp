#include "ina219_control.h"

int open_I2C()
{
	char *filename = (char*)"/dev/i2c-1";
	int device_I2C_id = open(filename, O_RDWR);
	if(device_I2C_id < 0)
		cout << "Error: Cannot start I2C connection!" << endl;
	return device_I2C_id;
}

void adress_I2C(int adress, int &device_I2C_id)
{
	if(device_I2C_id >= 0)
		if (ioctl(device_I2C_id, I2C_SLAVE, adress) < 0)
			cout << "Error: Cannot connect to slave!" << endl;
}

void ina_219_calibration(int &device_I2C_id)
{
	if(device_I2C_id >= 0)
	{
		unsigned char buffer[60] = {0};
		buffer[0] = 0x05;
		buffer[1] = 0b00010000;
		buffer[2] = 0b00000000;
		int number_of_bytes = 3;
		if (write(device_I2C_id, buffer, number_of_bytes) != number_of_bytes)		
			cout << "Error: Cannot write bytes to slave - calibration!" << endl;
	}
}

void ina_219_configuration(int &device_I2C_id)
{
	if(device_I2C_id >= 0)
	{
		unsigned char buffer[60] = {0};
		buffer[0] = 0x00;
		buffer[1] = 0b00111001;
		buffer[2] = 0b10011111;
		int number_of_bytes = 3;			
		if (write(device_I2C_id, buffer, number_of_bytes) != number_of_bytes)		
			cout << "Error: Cannot write bytes to slave - configuration!" << endl;
	}
}

float supply_voltage_calc(int &device_I2C_id)
{
	if(device_I2C_id < 0)
		return 0;
	else
	{
		float supply_voltage;
		unsigned char buffer[60] = {0};
		unsigned char buffer1[60] = {0};
		buffer[0] = 0x02;
		if(write(device_I2C_id, buffer, 1) != 1)
		{
			cout << "Error: Cannot write byte to slave - bus voltage!" << endl;
			return 0;
		}
		if(read(device_I2C_id, buffer, 2) != 2)		
		{
			cout << "Error: Cannot read bytes from slave - bus voltage!" << endl;
			return 0;
		}
		else
		{
			int received_bytes = buffer[0]<<8 | buffer[1]; 
			received_bytes = received_bytes >> 3;
			float bus_voltage = float(received_bytes)*0.004;
			supply_voltage = bus_voltage;
		}
		
		buffer1[0] = 0x01;
		if(write(device_I2C_id, buffer1, 1) != 1)
		{
			cout << "Error: Cannot read bytes from slave - shunt voltage!" << endl;
			return 0;
		}
		if (read(device_I2C_id, buffer1, 2) != 2)	
		{
			cout << "Error: Cannot read bytes from slave - shunt voltage!" << endl;
			return 0;
		}
		else
		{
			int received_bytes = buffer1[0]<<8 | buffer1[1]; 
			float shunt_voltage = float(received_bytes)*0.01/1000;
			supply_voltage = supply_voltage + shunt_voltage;
		}
		return supply_voltage;
	}
}

float current_calc(int &device_I2C_id)
{
	if(device_I2C_id < 0)
		return 0;
	else
	{
		unsigned char buffer[60] = {0};
		if(write(device_I2C_id, buffer, 1) != 1)
		{
			cout << "Error: Cannot write byte to slave - current!" << endl;
			return 0;
		}
		if (read(device_I2C_id, buffer, 2) != 2)		
		{
			cout << "Error: Cannot read bytes from slave - current!" << endl;
			return 0;
		}
		else
		{
			int received_bytes = buffer[0]<<8 | buffer[1]; 
			float current = float(received_bytes)*0.0001;
			return current;
		}
	}
}
