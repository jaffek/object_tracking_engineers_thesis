#ifndef SERVO_CONTROL_H
#define  SERVO_CONTROL_H
#include <wiringSerial.h>
#include <wiringPi.h> 

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 // servo control //
// open serial port
int serial_open();

// servo activation/deactivation
void servo_on_off(int &descriptor, unsigned char on_off, unsigned char channel);

// servo move command
void move_servos(int &descriptor, unsigned char channel, unsigned int position, unsigned char velocity);

// checking servo position
unsigned int check_position(int &descriptor, unsigned char channel);

// default position command
void initial_position(int &descriptor, int velocity);

#endif
