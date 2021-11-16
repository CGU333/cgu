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

int main(void) {
//	Servo miservo = Servo(0,0,1,0x40);
//	miservo.test();
PCA9685 miPCA = PCA9685();
	//main_PCA9685();

	//Front_Wheels misRuedasDelanteras(0); //cnhl=0 para ruedas delanteras
	//misRuedasDelanteras.wheel.pwm.setup(); //dentro de Front_Wheels > Servo > PCA9685
	//misRuedasDelanteras.test();
	return 0;
}

