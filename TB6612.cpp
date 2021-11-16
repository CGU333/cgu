  /*
 * TB6612.cpp
 *  Created on: 30 Oct 2021
 *      Author: carlos
   */
/*
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /
#include <fcntl.h>
#include <syslog.h>
#include <inttypes.h>
#include <errno.h>
#include <math.h>
*/
#include <bcm2835.h>
#include "TB6612.h"
#include <unistd.h>
#define BCM2835_GPIO_FSEL_OUTP 0x01 //output
#define RPI_BPLUS_GPIO_J8_11 17 //Motor A  (DC)
#define RPI_BPLUS_GPIO_J8_12 18 //Motor B  (DC)
// PWM output on RPi Plug P1 pin 11 and 12 (which is GPIO pin 17 and 18)  in alt fun 5. controlled by PWM channel 0
#define PWM_CHANNEL 0
// This controls the max range of the PWM signal
#define RANGE 1024
/* GPIO configuration
 * http://circuitden.com/blog/16
 * https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
 * https://www.airspayce.com/mikem/bcm2835/group__gpio.html#gac69a029acceb17691826650d8f188cd8
 */
    // Clase para el control del motor a partir del PWM
Motor :: Motor(uint8_t direction_channel, float pwm=0, bool offset=true){
		//this->_debug_("Debug on")
		this->direction_channel = direction_channel;
		this->pwm = pwm;
		this->offset = offset;
		this->forward_offset = this->offset;
		this->backward_offset = not this->forward_offset;

		if (!bcm2835_init())
	        exit(0);
	 	 // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
	    // Clock divider is set to 16.
	    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
	    // the pulse repetition frequency will be
	    // 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
	    bcm2835_gpio_fsel(direction_channel, BCM2835_GPIO_FSEL_OUTP); // pin18 en rpi3b+ BCM2835_GPIO_FSEL_OUTP
}

int Motor :: getspeed(){
	return this->speed;
}
void Motor :: setspeed(int velocidad){
	//''' Set Speed with giving value '''
	if (velocidad < 0 || velocidad >100)
		//raise ValueError('speed ranges fron 0 to 100, not "{0}"'.format(speed));  //???
		printf("la velocidad debe estar en un rango de 0 a 100)");
	this->speed = velocidad;
//	this->pwm(this->speed);
}

void Motor :: forward(){
	bcm2835_gpio_write(this->direction_channel, this->forward_offset);
//	this->speed = velocidad;
	printf("Motor hacia adelante");
}
void Motor :: backward(){
	bcm2835_gpio_write(this->direction_channel, this->backward_offset);
//	this->speed = velocidad;
	printf("Motor hacia atras");
}
void Motor :: stop(){
	this->speed = 0;
	printf("Motor parado");
}

//@property
bool Motor :: getoffset(){
	return this->offset;
}

//@offset.setter
void Motor :: setoffset(bool value){
	//''' Set offset for much user-friendly '''
		if ((value!=1) || (value!= 0))
			//	raise ValueError('offset value must be Bool value, not"{0}"'.format(value));
			printf("Offset debe ser un valor booleano (0 o 1)");
		this->forward_offset = value;
		this->backward_offset = not this->forward_offset;
	//	this->_debug_('Set offset to %d' % self._offset);
}

uint8_t Motor :: getpwm(){
	return this->pwm;
}

//@pwm.setter
void Motor :: setpwm(uint8_t pwm){
	//this->_debug_('pwm set')
	this->pwm = pwm;
}

Motor :: ~Motor() {
	printf("Borramos objeto");
	bcm2835_close();
}

//Funcion para testear los motores traseros
void test(){
	printf("********************************************");
	printf("*                                          *");
	printf("*           SunFounder TB6612              *");
	printf("*                                          *");
	printf("*          Connect MA to BCM17             *");
	printf("*          Connect MB to BCM18             *");
	printf("*         Connect PWMA to BCM27            *");
	printf("*         Connect PWMB to BCM22            *");
	printf("*                                          *");
	printf("********************************************");
	//GPIO.setmode(GPIO.BCM)
	bcm2835_gpio_fsel(27,BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(22,BCM2835_GPIO_FSEL_OUTP);
	//a = GPIO.PWM(27, 60)
	//b = GPIO.PWM(22, 60)
	//a.start(0)
	//b.start(0)
/*
	void a_speed(int value){
		//a.ChangeDutyCycle(value)
	}
	void b_speed(int dvalue){
		//b.ChangeDutyCycle(value)
	}
	//	https://www.airspayce.com/mikem/bcm2835/
	//   https://en.wikipedia.org/wiki/I%C2%B2C
	//    https://www.airspayce.com/mikem/bcm2835/group__i2c.html#ga5d01c1483ae12ff682068d0a56b6529a
	//   https://www.airspayce.com/mikem/bcm2835/group__i2c.html#gabb58ad603bf1ebb4eac2d439820809be
*/
	Motor motorA(23);  // motor1 DC en canal 23  (address=0x40)
	Motor motorB(24);  // motor 2 DC en canal 24 (address=0x40)
//	motorA.setpwm = a_speed;
//	motorB.setpwm = b_speed;
	motorA.forward();
	int i=0;
	for (i=0;i<101;i++)
	{
		motorA.setspeed(i);
		sleep(1);
	}
	for (i=100;i<-1;i--)
	{
		motorA.setspeed(i);
		sleep(1);
	}

	motorA.backward();
	for (i=0;i<101;i++)
	{
		motorA.setspeed(i);
			sleep(1);
	}
	for (i=100;i<-1;i--)
	{
		motorA.setspeed(i);
			sleep(1);
	}

	motorB.forward();
	for (i=0;i<101;i++)
	{
		motorB.setspeed(i);
		sleep(1);
	}
	for (i=100;i<-1;i--)
	{
		motorB.setspeed(i);
		sleep(1);
	}

	motorB.backward();
	for (i=0;i<101;i++)
	{
		motorB.setspeed(i);
		sleep(1);
	}
	for (i=100;i<-1;i--)
	{
		motorB.setspeed(i);
		sleep(1);
	}
}

//  https://www.airspayce.com/mikem/bcm2835/group__constants.html#ga63c029bd6500167152db4e57736d0939
int main_TB6612(void){
	test();
	return 0;
}

/*
#include < bcm2835.h>
//P1插座第11脚
#define PIN RPI_GPIO_P1_11
int main(int argc, char **argv)
{
 if (!bcm2835_init())
 return 1;

// 输出方式
bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);

while (1)
{
 bcm2835_gpio_write(PIN, HIGH);
bcm2835_delay(100);

bcm2835_gpio_write(PIN, LOW);
bcm2835_delay(100);
}
bcm2835_close();
return 0;
}
*/



