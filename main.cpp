/*
 * man.cpp
 *
 *  Created on: 5 Nov 2021
 *      Author: carlos
 */

#include "back_wheels.h"
#include "front_wheels.h"
#include "PCA9685.h"
#include "servo.h"
#include "TB6612.h"

int main(void) {
	// Creamos un objeto de tipo Servo
	printf("main 1\n");
	//Servo miServo(1, 0, 1, 0x40);
	Front_Wheels misRuedasDelanteras(0); //cnhl=0 para ruedas delanteras
	printf("main 2\n");
	misRuedasDelanteras.wheel.pwm.setup(); //dentro de Front_Wheels > Servo > PCA9685
	//misRuedasDelanteras.test();
	return 0;
}

