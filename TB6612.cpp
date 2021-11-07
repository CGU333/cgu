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
#define BCM2835_GPIO_FSEL_OUTP 0x0b001 //output
// PWM output on RPi Plug P1 pin 12 (which is GPIO pin 18)
// in alt fun 5.
// Note that this is the _only_ PWM pin available on the RPi IO headers
#define PIN RPI_GPIO_P1_12
// and it is controlled by PWM channel 0
#define PWM_CHANNEL 0
// This controls the max range of the PWM signal
#define RANGE 1024

    // Clase para el control del motor a partir del PWM
Motor :: Motor(uint8_t direction_channel, float pwm=0, bool offset=true){
		//this->_debug_("Debug on")
		this->direction_channel = direction_channel;
		this->pwm = pwm;
		this->offset = offset;
		this->forward_offset = this->offset;
		this->backward_offset = not this->forward_offset;
		this->speed = 0;
		//this->_debug_('setup motor direction channel at %s' % direction_channel)
		//this->_debug_('setup motor pwm channel')# self._debug_('setup motor pwm channel as %s ' % self._pwm.__name__)

	//GPIO.setwarnings(False) // ***lectura de ficheros
	//GPIO.setmode(GPIO.BCM) // ***lectura de ficheros
	//GPIO.setup(self.direction_channel, GPIO.OUT) // ***lectura de ficheros

//https://www.airspayce.com/mikem/bcm2835/group__gpio.html#gaf866b136c0a9fd4cca4065ce51eb4495
//https://www.airspayce.com/mikem/bcm2835/group__gpio.html#gac69a029acceb17691826650d8f188cd8
//https://www.airspayce.com/mikem/bcm2835/group__constants.html#ga485a5a69e864565b0cd18b33ae1f59f2
//https://www.airspayce.com/mikem/bcm2835/pwm_8c-example.html
		// Example program for bcm2835 library how to use PWM to control GPIO pins
		if (!bcm2835_init())
	        exit(0);
	 // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
	    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_ALT5);

	    // Clock divider is set to 16.
	    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
	    // the pulse repetition frequency will be
	    // 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
	    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);
	    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);
	    bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);

	    // Vary the PWM m/s ratio between 1/RANGE and (RANGE-1)/RANGE
	    // over the course of a a few seconds
	    int direction = 1; // 1 is increase, -1 is decrease
	    int data = 1;
	    while (1)
	    {
	        if (data == 1)
	            direction = 1;   // Switch to increasing
	        else if (data == RANGE-1)
	            direction = -1;  // Switch to decreasing
	        data += direction;
	        bcm2835_pwm_set_data(PWM_CHANNEL, data);
	        bcm2835_delay(1);
	    }

	    bcm2835_close();
}
/*
void Motor:: _debug_(self,message){
	if self._DEBUG :
		print(self._DEBUG_INFO,message)
}
*/
//@property
int Motor :: getspeed(){
	return this->speed;
}
void Motor :: setspeed(int velocidad){
	//''' Set Speed with giving value '''
	if (velocidad < 0 || velocidad >100)
		//raise ValueError('speed ranges fron 0 to 100, not "{0}"'.format(speed));  //???
		printf("la velocidad debe estar en un rango de 0 a 100)");
	//if ()!callable(this->pwm)
		//raise ValueError('pwm is not callable, please set Motor.pwm to a pwm control function with only 1 veriable speed');
	//this->debug_('Set speed to: %s' % speed);
	this->speed = velocidad;
	//this->pwm(this->speed);

}

void Motor :: forward(){
	//GPIO.output(self.direction_channel, self.forward_offset) // ***lectura de ficheros
	//this->speed = velocidad;
	printf("Motor hacia adelante");
}
void Motor :: backward(){
	//GPIO.output(self.direction_channel, self.backward_offset) // ***lectura de ficheros
	//this->speed = velocidad;
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
/*
//@property
void Motor :: getdebug(debug){
	return this->_DEBUG;
}

//@debug.setter
void Motor :: setdebug(debug){
	''' Set if debug information shows '''
	if debug in (True, False):
		this->_DEBUG = debug
	else:
		raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

	if this->_DEBUG:
		print(this->_DEBUG_INFO, "Set debug on")
	else:
		print(this->_DEBUG_INFO, "Set debug off")
}
*/
//@property
float Motor :: getpwm(){
	return this->pwm;
}

//@pwm.setter
void Motor :: setpwm(float pwm){
	//this->_debug_('pwm set')
	this->pwm = pwm;
}

Motor :: ~Motor() {
	printf("Borramos objeto");
}

//Funcion para testear los motores traseros: girar hacia adelante y luego hacia atras
void test(){
	printf("********************************************");
	printf("*                                          *");
	printf("*           SunFounder TB6612              *");
	printf("*                                          *");
	printf("*          Connect MA to BCM17             *");
	printf("*          Connect MB to BCM18             *");
	printf("*         Connect PWMA to BCM27            *");
	printf("*         Connect PWMB to BCM12            *");
	printf("*                                          *");
	printf("********************************************");
	//GPIO.setmode(GPIO.BCM)
	//GPIO.setup((27, 22), GPIO.OUT)
	//a = GPIO.PWM(27, 60)
	//b = GPIO.PWM(22, 60)
	//a.start(0)
	//b.start(0)
/*
	void a_speed(int value){
		//a.ChangeDutyCycle(value)  // ***lectura de ficheros
	}
	void b_speed(int dvalue){
		//b.ChangeDutyCycle(value)  // ***lectura de ficheros
	}
//	https://www.airspayce.com/mikem/bcm2835/    ************************************
	//   https://en.wikipedia.org/wiki/I%C2%B2C
	//https://www.airspayce.com/mikem/bcm2835/group__i2c.html#ga5d01c1483ae12ff682068d0a56b6529a
		//   https://www.airspayce.com/mikem/bcm2835/group__i2c.html#gabb58ad603bf1ebb4eac2d439820809be
*/
	Motor motorA(23);
	Motor motorB(24);
//	motorA.debug = True;
//	motorB.debug = True;
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
