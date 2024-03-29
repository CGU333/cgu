#ifndef PCA9685_H_
#define PCA9685_H_
/*
 * PCA9685.h
 *
 *  Created on: 29 Oct 2021
 *      Author: carlos
 */

#include <unistd.h>
#include <ctime>
#include <cstdio>
#include <cstdint>  // para el uint8_t --> typedef unsigned char uint8_t;
#include <bcm2835.h>  //equivalente en Python a importar SMBUS para los pines de entrada/salida.
				//existe opcion para habilitar esta biblioteca en Buildroot
				//En el futuro se sustituira por ficheros linux
#include <vector>  //añadimos esta libreria para poder utilizar vectores ya que en c++ es mejor que los arrays.
					// vectores en c++ vienen con varias funciones que podemos utilizar. es un equivalente a usar listas
using namespace std; //para evitar poner std:: en cada definicion de vectores

#define _MODE1            0x00
#define _PRESCALE         0xFE
#define _MODE2            0x01

#define _SUBADR1          0x02
#define _SUBADR2          0x03
#define _SUBADR3          0x04

#define LED0_ON_L         0x06
#define LED0_ON_H         0x07
#define LED0_OFF_L        0x08
#define LED0_OFF_H        0x09
#define _ALL_LED_ON_L     0xFA
#define _ALL_LED_ON_H     0xFB
#define _ALL_LED_OFF_L    0xFC
#define _ALL_LED_OFF_H    0xFD

#define _RESTART          0x80
#define _SLEEP            0x10
#define _EXTCLK           0x40
#define _ALLCALL          0x01
#define _INVRT            0x10
#define _OUTDRV           0x04

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define _MIN_PULSE_WIDTH 500    // the shortest pulse sent to a servo(0 degree)
#define _MAX_PULSE_WIDTH 2500   // the longest pulse sent to a servo (180 degrees)
#define _DEFAULT_PULSE_WIDTH 1500   // default pulse width when servo is attached (90 degrees)

//https://programmerclick.com/article/1519244875/
//http://circuitden.com/blog/16
// https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/i2c.html

// libreria i2cdev https://github.com/jrowberg/i2cdevlib


// Clase para el control del PWM
class PCA9685{
	public:
			PCA9685(uint8_t address=PCA9685_I2C_ADDRESS);
			PCA9685(uint8_t address, float freq);

			void setup(void);
			void inicializar_i2c();
			void restart_i2c(void);

			void set_PWM_freq(float freq);
			float get_PWM_freq();

			void write (uint8_t registro, uint32_t pulseWidth);
			void write (uint8_t registro, uint16_t on, uint16_t off);

			void write_all_value (uint16_t on, uint16_t off);
			uint8_t map(int x, int in_min, int in_max, int out_min, int out_max);
			~PCA9685();
			int main_PCA9685(void);

			float frequency;

	private:
		uint8_t addr;
		uint32_t T;  //PWM Period[us]
		char sendBuf[5]={0}; // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		uint8_t errCode=0;
		uint16_t _min_pulse_width;
		uint16_t _max_pulse_width;

		uint8_t read_byte_data(uint8_t registro);
		void write_byte_data(uint8_t registro, uint8_t d);
		//void check_i2c_issues();  //este metodo solo busca errores en i2c. no necesario por el momento



};


#endif /* PCA9685_H_ */
