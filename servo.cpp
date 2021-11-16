/*
 * servo.cpp
 *  Created on: 30 Oct 2021
 *      Author: carlos
*/
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <bcm2835.h>
#include "servo.h"
#include "PCA9685.h"


Servo :: Servo(uint8_t channel=PWM_CHANNEL, uint8_t offst = 0, bool lck = true, uint8_t address=0x40){
	if (channel < 0 || channel > 16)
	printf ("Servo channel  is not in range (0, 15)\n");
	this->channel=channel; //se indica con this ya que tienen mismo nombre
	this->offset=offst;
	this->lock=lck;
		printf("servo 1.2 set servo FREQUENCY = 60 \n");
	this->frequency = FREQUENCY;

		printf("servo 1.3 inicializar i2c pwm --> PCA9685.setup() \n");
	this->pwm.setup(); // --> PCA9685.setup() para inicializar el control del I2C PWM

/* Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
 Clock divider is set to 16. With a divider of 16 and a RANGE of 1024, in MARKSPACE mode, the pulse repetition frequency will be
1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
	    bcm2835_gpio_fsel(PWM_CHANNEL, RPI_BPLUS_GPIO_J8_12); // pin18 en rpi3b+
	    bcm2835_pwm_set_clock (BCM2835_PWM_CLOCK_DIVIDER_16);
		bcm2835_pwm_set_mode (PWM_CHANNEL, 1, true); //0 == balanced mode; Set true if you want Mark-Space mode.
		bcm2835_pwm_set_range (PWM_CHANNEL, RANGE); RANGE=1024 (
*/
		printf("servo 1.4 write_angle(90)\n");
	this->write_angle(90);
		printf(" ********* servo configurado a 90 grados :) ***********\n");
}

void Servo :: setup()
{
	this->pwm.setup();
}

float Servo :: getFrequency()
{
	return this->frequency;
}

void Servo :: setFrequency(float frequency)
{
	this->frequency = frequency;
	this->pwm.frequency = frequency;
}

uint8_t Servo :: getOffset()
{
	return this->offset;
}

void Servo :: setOffset(uint8_t offset)
{
	this->offset = offset;
}

void Servo :: write_angle(uint8_t angle)
{
	if (this->lock)
	{
		if (angle > 180)
			angle = 180;
		if (angle < 0)
			angle = 0;
	}
	else
	{
		if (angle <0 || angle>180){
			printf("Servo %i turn angle %i is not in (0, 180).", channel, angle);
		}
	}
	//uint8_t val = this->angle_to_analog(angle);
	//val += offset;
	uint8_t val = this->angle_to_analog(angle-offset);
		printf(" el valor analogico a setear en las ruedas delanteras es  %i y el offset es  %i *_* \n", val, offset);
	this->pwm.write(this->channel, 0, val);
	printf("el canal es %i y el angulo es %d \n", channel, angle);
	offset=val;
	//http://circuitden.com/blog/16
}

uint8_t Servo :: angle_to_analog(uint8_t angle) {
	//https://www.airspayce.com/mikem/bcm2835/group__pwm.html#ga7e2eddac472b6d81e0ba7dbf165672cb
	uint8_t pulse_wide = this->pwm.map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	uint8_t analog_value = int(float(pulse_wide) / 1000000 * this->frequency * 4096);
	printf("el valor analogico es %d   **  \n", analog_value);
	return analog_value;

		//https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
	//printf("el valor analogico introducido es %d   **  \n", pulse_wide);
	//return pulse_wide;

	    //http://circuitden.com/blog/16
	//int pulse = (int)abs(((angle - offset)*33)/180) + 10;
	//return pulse;
}

Servo :: ~Servo() {
	printf("~Servo() Borramos objeto Servo\n");
}

void Servo :: test(){
//	Servo mi_servo = Servo();
	printf(" *** INICIO PCA9685 SETUP *** \n");
	this->pwm.setup();
	printf(" *** FIN SETUP *** \n");

	this->write_angle(90);
	bcm2835_delay(5);
	this->write_angle(0);
	bcm2835_delay(5);
	this->write_angle(180);

}

int main_servo(void){
	return 0;
}


