/*
 * servo.cpp
 *  Created on: 30 Oct 2021
 *      Author: carlos
*/
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
//#include <bcm2835.h>
#include "servo.h"
//#include "PCA9685.h"

Servo :: Servo(uint8_t channel, uint8_t offst, bool lck, uint8_t address){ // address= 0x40
	 //tal como esta conectado el servo motor en el sunfounder picar, el canal es el cero (0)
	this->channel=channel; //se indica con this ya que tienen mismo nombre
	this->offset=offst;
	this->lock=lck;
		printf("servo 1.2 set servo FREQUENCY = 60 \n");
	this->frequency = FREQUENCY;
		printf("servo 1.3 inicializar i2c pwm --> PCA9685.setup() \n");
	this->setup(); // --> PCA9685.setup() para inicializar el control del I2C PWM

		printf("servo 1.4 write_angle(90)\n");
	this->write_angle(90);
	sleep(0.5);
	printf(" ********* servo configurado a 90 grados :) ***********\n");
}

void Servo :: setup(){
	this->pwm.setup();
}

float Servo :: getFrequency(){
	return this->frequency;
}

void Servo :: setFrequency(float frequency){
	this->frequency = frequency;
	this->pwm.frequency = frequency;
}

uint8_t Servo :: getOffset(){
	return this->offset;
}

void Servo :: setOffset(uint8_t offset){
	this->offset = offset;
}

//https://qastack.mx/raspberrypi/53854/driving-pwm-output-frequency
void Servo :: write_angle(uint8_t angle){
		if (angle > 180)
			angle = 180;
		if (angle < 0)
			angle = 0;
	//uint8_t val = ( (angle-offset) * ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180) + MIN_PULSE_WIDTH);
	int val = this->angle_to_analog(angle);
	val+=this->offset;
	this->pwm.write_PWM(this->channel, 0, val);
	this->offset=val;
	printf("write_angle() -> el canal es %i y el angulo es %d \n", channel, angle);
	printf(" el valor analogico a setear en las ruedas delanteras es  %d y el offset es  %d *_* \n", val, offset);
	//http://circuitden.com/blog/16
}

uint8_t Servo :: angle_to_analog(uint8_t angle) {
	//https://www.airspayce.com/mikem/bcm2835/group__pwm.html#ga7e2eddac472b6d81e0ba7dbf165672cb
	//float pulse_wide = this->pwm.map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
	float pulse_wide = 10.3*(float)angle + 546; // tiempo de pulso    http://circuitden.com/blog/16
	int analog_value = int(float(pulse_wide) / 10000000 * this->frequency * 4096);
	printf(" el  ANGULO es ***** angulo seteado : %d   **  \n", angle);
	printf(" el  PULSO es ****** pulso : %f   **  \n", pulse_wide);
	printf(" el valor ANALOGICO es : ******  %d   **  \n", analog_value);

	//http://circuitden.com/blog/16
	//uint8_t pulse_wide = ((angle * 1800 / 180 ) + 600); //pulso 0.6 ~ 2.4 ; 600<pulse<2400
	//uint8_t analog_value = pulse_wide * 4096 * 60 / 10; //

	//float pulse_wide = 10.3*(float)angle + 546; // tiempo de pulso
	//pulse = (int)abs(((angle - prevangle)*33)/180) + 10; //pulsos a recorrer para cada tiempo de pulso

	//https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
	return analog_value;
}

Servo :: ~Servo() {
	printf("~Servo() Borramos objeto Servo bcm2835_close(); \n");
	this->pwm.~PCA9685();
}

void Servo :: test(){
	printf("\n ** seteando a 0 **\n");
	delay(1000);
	this->write_angle(0);
	printf("\n ** seteado a 0 **\n");
	delay(1000);

	printf("%d\n",180);
	this->write_angle(180);
	delay(1000);

	printf("%d\n",0);
	delay(1000);
	this->write_angle(0);
	delay(1000);

	printf("%d\n",180);
	this->write_angle(180);
	delay(1000);
}
/*
int angle=0;
for(angle=0; angle <181;angle+=10){
}
*/


int main_servo(void){
	return 0;
}


/* Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
 Clock divider is set to 16. With a divider of 16 and a RANGE of 1024, in MARKSPACE mode, the pulse repetition frequency will be
1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
	    bcm2835_gpio_fsel(18, RPI_BPLUS_GPIO_J8_12); // pin18 en rpi3b+
	    bcm2835_pwm_set_clock (BCM2835_PWM_CLOCK_DIVIDER_16);
		bcm2835_pwm_set_mode (PWM_CHANNEL, 1, true); //0 == balanced mode; Set true if you want Mark-Space mode.
		bcm2835_pwm_set_range (PWM_CHANNEL, RANGE); RANGE=1024 (
*/
