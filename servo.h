/*
 * servo.h
 *  Driver module for servo, with RpiCar S
 *  Created on: 30 Oct 2021
 *      Author: carlos
 */
#ifndef SERVO_H_
#define SERVO_H_
#include <time.h>
#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>  // para el uint8_t --> typedef unsigned char uint8_t;
#include "PCA9685.h"

#define MIN_PULSE_WIDTH 600    // the shortest pulse sent to a servo(0 degree)
#define MAX_PULSE_WIDTH 2400   // the longest pulse sent to a servo (180 degrees)
#define DEFAULT_PULSE_WIDTH 1500   // default pulse width when servo is attached (90 degrees)

#define	FREQUENCY 			60

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */

#define PWM_CHANNEL 0x00 // el servo motor se controla con channel 0 (0-15) en address 0x40
//#define RANGE 1024  // max range of the PWM signal

class Servo{
public:
	Servo (uint8_t channel=PWM_CHANNEL, uint8_t offset=0, bool lock=true, uint8_t address=PCA9685_I2C_ADDRESS);
	~Servo();
	//void servoCtrl(int dutyCycle);
	void setup();
	float getFrequency();
	void setFrequency(float frequency);
	uint8_t getOffset();
	void setOffset(uint8_t offset);

	void write_angle(uint8_t angle);
	uint8_t angle_to_analog(uint8_t angle);

	float frequency;
	uint8_t channel; //  1 - 16

	bool lock=true;
	uint8_t address;
	PCA9685 pwm = PCA9685(PCA9685_I2C_ADDRESS);

	uint8_t offset;
	void test();
	void install();

};
#endif /* SERVO_H_ */
