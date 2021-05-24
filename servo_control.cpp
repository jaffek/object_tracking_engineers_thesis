#include "servo_control.h"

int serial_open()
{
	int descriptor; 
	descriptor = serialOpen("/dev/ttyS0",9600); 
	return descriptor;
}

void servo_on_off(int &descriptor, unsigned char on_off, unsigned char channel)
{
	unsigned char servo_code = 0;
	servo_code = 0b11000000 | channel;
	serialPutchar(descriptor,servo_code);
	serialPutchar(descriptor,on_off);
}

void move_servos(int &descriptor, unsigned char channel, unsigned int position, unsigned char velocity)
{
	unsigned char servo_code = 0;
	unsigned char higher_byte = 0;
	unsigned char lower_byte = 0;
	servo_code = 0b11100000 | channel;
	higher_byte = (position >> 6) & 0b01111111;
	lower_byte = position & 0b00111111;
	serialPutchar(descriptor, servo_code);
	serialPutchar(descriptor, higher_byte);
	serialPutchar(descriptor, lower_byte);
	serialPutchar(descriptor, velocity);
}

unsigned int check_position(int &descriptor, unsigned char channel)
{
	unsigned int position = 0;
	unsigned char higher_byte = 0;
	unsigned char lower_byte = 0;
	unsigned char servo_code = 0b10100000 | channel;
	serialPutchar(descriptor, servo_code);
	while(serialDataAvail(descriptor) != 2);
	higher_byte = serialGetchar(descriptor);
	lower_byte = serialGetchar(descriptor);
	position = higher_byte;
	position = higher_byte << 6;
	position = position | lower_byte;
	return position;
}

void initial_position(int &descriptor, int velocity)
{
	unsigned int servo1_pos = 4000;		// servo movement range corresponds to the range: 0-8000 (0-180 degree)
	unsigned int servo2_pos = 4000;		// servo movement range corresponds to the range: 0-8000 (0-180 degree)
	move_servos(descriptor,1,servo1_pos,50);
	move_servos(descriptor,2,servo2_pos,50);
	while(check_position(descriptor,1) != servo1_pos);
	while(check_position(descriptor,2) != servo2_pos);
}
