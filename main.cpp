/*
 * main.cpp
 *
 *  Created on: 5 Nov 2021
 *      Author: carlos
 */

#include "back_wheels.h"
#include "front_wheels.h"
#include "PCA9685.h"
#include "servo.h"
#include "TB6612.h"
#include "../src/PCF8591.h"
	//https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
int main(void) {

	printf(" *** INICIANDO PCA9685 SETUP *** \n");
	Servo miservo = Servo();
	printf(" *** INICIANDO SERVO test *** \n");
	miservo.test();
	printf("\n ***  test SERVO FINALIZADO*** \n");

/*
	printf(" *** INICIANDO MOTORES test *** \n");
	Motor motorA = Motor(23);  // motorDC 1  en canal 23  (address=0x40)
	Motor motorB = Motor(24);  // motorDC 2  en canal 24 (address=0x40)

	bcm2835_gpio_fsel(27,BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(22,BCM2835_GPIO_FSEL_OUTP);
	motorA.setpwm(27, 60);
	motorB.setpwm(22, 60);

		for(int v = 0;;v = !v) {
			bcm2835_gpio_write(27, v);
			bcm2835_gpio_write(22, v);
			bcm2835_delay(1000);
	    }
*/
	return 0;
}

