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
/* GPIO
 * http://circuitden.com/blog/16
 * https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
 * https://www.airspayce.com/mikem/bcm2835/group__gpio.html#gac69a029acceb17691826650d8f188cd8
 */
    // Clase para el control del motor a partir del PWM
Motor :: Motor(uint8_t direction_channel, uint8_t pwm, bool offset){
		//this->_debug_("Debug on")
		this->direction_channel = direction_channel;
		this->pwm = pwm;
		this->offset = offset;
		this->forward_offset = this->offset;
		this->backward_offset = not this->forward_offset;
		this->speed=0;
		bcm2835_gpio_fsel(direction_channel,BCM2835_GPIO_FSEL_OUTP);
		printf("***  Motor DC    INICIALIZADO***");
}

int Motor :: getspeed(){
	return this->speed;
}
void Motor :: setspeed(uint8_t velocidad){
	//''' Set Speed with giving value '''
	if (velocidad < 0 || velocidad >100)
		//raise ValueError('speed ranges fron 0 to 100, not "{0}"'.format(speed));  //???
		printf("la velocidad debe estar en un rango de 0 a 100)");
	this->speed = velocidad;
	this->setpwm(this->direction_channel, velocidad);
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

void Motor :: setpwm(uint8_t direction_channel, uint8_t pwm){
	//this->_debug_('pwm set')
	this->pwm.write_pulse(this->direction_channel, pwm);
}

Motor :: ~Motor() {
	printf("Borramos objeto");
	bcm2835_close();
}
//https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
//Funcion para testear los motores traseros
void Motor ::test(){
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
	//GPIO.setmode(GPIO.BCM) //bcm2835 utiliza nomenclatura pines wiringpi
	Motor motorA(23);  // motorDC 1  en canal 23  (address=0x40)
	Motor motorB(24);  // motorDC 2  en canal 24 (address=0x40)
	bcm2835_gpio_fsel(27,BCM2835_GPIO_FSEL_OUTP); // el pin es 27
	bcm2835_gpio_fsel(22,BCM2835_GPIO_FSEL_OUTP); // el pin es 22
	motorA.setpwm(27, 60);
	motorB.setpwm(22, 60);

	  printf("*****probando esto********");
        bcm2835_gpio_write(27, 0);
        bcm2835_gpio_write(22, 0);
        bcm2835_delay(1000);
        sleep(1);

    printf("*****Y AHORA ADELANTE Y ATRAS********");
    motorA.forward();
	int i=0;
	for (i=0;i<101;i++){
		motorA.setspeed(i);
		delay(1000);
	}
	for (i=100;i>-1;i--){
		motorA.setspeed(i);
		delay(1000);
	}
	motorA.backward();
	for (i=0;i<101;i++){
		motorA.setspeed(i);
		delay(1000);
	}
	for (i=100;i>-1;i--){
		motorA.setspeed(i);
		delay(1000);
	}
	motorB.forward();
	for (i=0;i<101;i++){
		motorB.setspeed(i);
		delay(1000);
	}
	for (i=100;i>-1;i--){
		motorB.setspeed(i);
		delay(1000);
	}
	motorB.backward();
	for (i=0;i<101;i++){
		motorB.setspeed(i);
		delay(1000);
	}
	for (i=100;i<-1;i--){
		motorB.setspeed(i);
		sleep(1);
	}
}
/*
	//	https://www.airspayce.com/mikem/bcm2835/
	//   https://en.wikipedia.org/wiki/I%C2%B2C
	//    https://www.airspayce.com/mikem/bcm2835/group__i2c.html#ga5d01c1483ae12ff682068d0a56b6529a
	//   https://www.airspayce.com/mikem/bcm2835/group__i2c.html#gabb58ad603bf1ebb4eac2d439820809be
*/
//  https://www.airspayce.com/mikem/bcm2835/group__constants.html#ga63c029bd6500167152db4e57736d0939
int main_TB6612(void){
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


/*
 * #include <bcm2835.h>

int main() {
    bcm2835_init();
    bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_OUTP);
    for(int v = 0;;v = !v) {
        bcm2835_gpio_write(18, v);
        bcm2835_delay(1000);
    }
    bcm2835_close();
}
*/




