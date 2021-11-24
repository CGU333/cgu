#ifndef TB6612_H_
#define TB6612_H_
using namespace std;  //para evitar poner std:: en cada definicion de vectores
#include <unistd.h>
#include "PCA9685.h"
#include <time.h>
#include <ctime>
#include <cstdio>
#include <cstdint>     // para el uint8_t --> typedef unsigned char uint8_t;
#include <bcm2835.h>  //equivalente en Python a importar SMBUS para los pines de entrada/salida.
				//existe opcion para habilitar esta biblioteca en Buildroot
				//En el futuro se sustituira por ficheros linux
#include <vector>     //añadimos esta libreria para poder utilizar vectores ya que en c++ es mejor que los arrays.
					  // vectores en c++ vienen con varias funciones que podemos utilizar. es un equivalente a usar listas
//Clase para control de los 2 motores DC traseros
class Motor{
	public:
		Motor(uint8_t direction_channel, uint8_t pwm=0, bool offset=true);
		void setup();
		int getspeed();
		void setspeed(uint8_t velocidad);
		void forward();
		void backward();
		void stop();
		bool getoffset();
		void setoffset(bool offset);
		//uint8_t getpwm();
		void setpwm(uint8_t direction_channel, uint8_t value); //se configuraran 2 motores DC
		~Motor();

		uint8_t direction_channel; // PWM0 o PWM1
		int forward_offset;
		int backward_offset;
		int speed;
		PCA9685 pwm = PCA9685(0x40);
		bool offset;
		void test();
};

#endif /* TB6612_H_ */
