/*
 * PCA9685.cpp
 *
 *  Created on: 29 Oct 2021
 *      Author: carlos
 */
using namespace std; //para evitar poner std:: en cada definicion de vectores

#include <bcm2835.h>
#include "PCA9685.h"
#include <unistd.h>
#include <stdio.h>      /* printf */
#include <math.h>       /* floor */

/*
* https://www.airspayce.com/mikem/bcm2835/pwm_8c-example.html
* https://programmerclick.com/article/1519244875/
*/

// control del PWM
PCA9685 :: PCA9685(){ //addr=0x40 (PWM)
		this->addr = 0x40;        //se podria pasar la direccion 0x48 directamente al crear el metodo pero al pasarlo desde el .h esta mas encapsulado
		this->sendBuf[5] = {'0'};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		this->errCode = 0;
		this->T = 0 ; //para empezar asi
		this->frequency = 60;
}

void PCA9685 :: setup(){
		//ahora codigo correspondiente al metodo SETUP de python
		printf("PCA9685 setup() 1 Init...\n");
		bcm2835_init();
		printf("PCA9685 setup() 2 i2c begin\n");
		if(!bcm2835_i2c_begin())
		{
			printf("bcm2835_i2c_begin failed");
			exit(0);
		}
		bcm2835_i2c_setSlaveAddress(this->addr); //direccion I2C = 0x40
		bcm2835_i2c_set_baudrate(100000);  //frecuencia por el I2C = 100MHz

		printf("PCA9685 setup() 3 writes varios\n");
        this->write_all_value(0, 0);
        this->write_byte_data(_MODE2, _OUTDRV);
        this->write_byte_data(_MODE1, _ALLCALL);
        sleep(1);

        printf("PCA9685  setup() 4 writes varios 2\n");
        uint8_t mode1 = read_byte_data(_MODE1);
        mode1 = mode1 & ~_SLEEP;
        write_byte_data(_MODE1, mode1);
        sleep(1);

}

uint8_t PCA9685 :: read_byte_data(uint8_t addr){ // channel = 0...
						//si lo pasamos como canal en lugar de como valor de array, se declararia como: unint8_t addr
						// al pasar el parametro como entero, llamaremos a la funcion asi: read(0), read(1), etc

		bcm2835_i2c_setSlaveAddress(addr); // establecido I2C en  0x40
		//pasar el canal como el valor entero de un array (0,1,2,...) ademas de encapsular, facilita la forma de indicar las direcciones
		sendBuf[0] = addr ; //para leer, primero se comprueba no hay error al escribir en el canal indicado
		if((errCode = bcm2835_i2c_write(sendBuf,1)))  //lectura de tipo char
			printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		char byte;
		if((errCode = bcm2835_i2c_read(&byte,1))) //lectura de un char byte en el canal indicado
			printf("bcm2835_i2c_read failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		return static_cast<uint8_t>(byte);  // return = 0-255
}

void PCA9685 :: write_byte_data(uint8_t addr, uint8_t d){
	try {
		bcm2835_i2c_setSlaveAddress(addr);
		sendBuf[0] = addr;
		sendBuf[1] = d;
		if((errCode = bcm2835_i2c_write(sendBuf,2)))
			printf("bcm2835_i2c_write failed, errCode = 0x%x\n", errCode);
	}
	catch (...) {
		printf("Error write_byte_data(...)\n");
	}
}

float PCA9685 :: get_freq(){
	return (this->frequency);
}

//https://programmerclick.com/article/1519244875/
void PCA9685 :: set_freq(float freq){
		// Adafruit servo driver:
		//http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
	freq *= 0.9;  //Correct for overshoot in the frequency setting (see issue #11).
	this->frequency=freq;
	T =static_cast<uint32_t>(1/freq*1000000);
	double osc_clock = 25000000;
	float prescaleval = static_cast<unsigned char>(osc_clock/4096/freq-1);
	uint8_t prescale = static_cast<uint8_t>(floor(prescaleval+0.5));
	uint8_t oldmode = read_byte_data(_MODE1);
	uint8_t newmode = (oldmode&0x7f) | 0x10;
	write_byte_data(_MODE1, newmode); // go to sleep

	write_byte_data(_PRESCALE, prescale); // set the prescaler
	oldmode &= 0xef;
	write_byte_data(_MODE1, oldmode);
	delay(2);
	write_byte_data(_MODE1, (oldmode|0xa1));  //  This sets the MODE1 register to turn on auto increment.

}

void PCA9685 :: write(uint8_t channel, uint32_t pulseWidth){
	uint16_t off = static_cast<uint16_t> ((pulseWidth* 4096.0/T)*1.01);
	write(channel,0,off);
}

void PCA9685 :: write(uint8_t channel, uint16_t on, uint16_t off){
	bcm2835_i2c_setSlaveAddress(addr);
	sendBuf[0] = LED0_ON_L+4*channel;
	sendBuf[1] = on & 0x00FF;
	sendBuf[2] = on >> 8;
	sendBuf[3] = off & 0x00FF;
	sendBuf[4] = off >> 8;
	if((errCode = bcm2835_i2c_write(sendBuf,5)))
		printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
}

void PCA9685 :: write_all_value (uint16_t on, uint16_t off){
	try {
		bcm2835_i2c_setSlaveAddress(addr);
		sendBuf[0] = _ALL_LED_ON_L;
		sendBuf[1] = on & 0x00FF;
		sendBuf[2] = on >> 8;
		sendBuf[3] = off & 0x00FF;
		sendBuf[4] = off >> 8;
		if((errCode = bcm2835_i2c_write(sendBuf,5)))
			printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	}
	catch (...) {
		printf("Error write_all_value(...)\n");
	}
}

uint8_t PCA9685 :: map(int x, int in_min, int in_max, int out_min, int out_max){
    //'''To map the value from arange to another'''
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

PCA9685 :: ~PCA9685(){
	//bcm2835_i2c_end();
	//bcm2835_close();
}

int main_PCA9685(void){
	printf("main_pca9685\n");
	PCA9685 pwm;  // addr = 0x40
	pwm.setup();
	uint8_t i;
	try {
		for (i=0;i<16;i++) {
        sleep(0.5);
        printf("\nChannel %d\n", i);
        sleep(0.5);
        uint8_t j;
        	for (j=0; j< 4096; j++) {
            pwm.write(i, 0, j);
            printf("PWM value: %d", j);
            sleep(0.0003);
        	}
		}
	}
	catch (...) {
		pwm.~PCA9685();
	}

    return 0;
}

