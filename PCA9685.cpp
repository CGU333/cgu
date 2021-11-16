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
* https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
* https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/Adafruit_PWMServoDriver.h
*/

// control del PWM
PCA9685 :: PCA9685(uint8_t address){ //addr = 0x40
		this->addr = address;        //se podria pasar la direccion 0x48 directamente al crear el metodo pero al pasarlo desde el .h esta mas encapsulado
		this->sendBuf[5] = {'0'};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		this->errCode = 0;
	    this->_min_pulse_width = _MIN_PULSE_WIDTH;
	    this->_max_pulse_width = _MAX_PULSE_WIDTH;
		this->frequency = 60;
		this->T = 1/this->frequency ;
}


PCA9685 :: PCA9685(uint8_t address, float freq){
	this->addr = address;
	this->sendBuf[5] = {'0'};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
	this->errCode = 0;
	this->_min_pulse_width = _MIN_PULSE_WIDTH;
    this->_max_pulse_width = _MAX_PULSE_WIDTH;
	this->frequency = freq;
	this->T = 1/freq ;

}


void PCA9685 :: setup(void){
	inicializar_i2c();               // bcm2835_i2c_begin()
	restart_i2c();                   // i2c registros inicializar
	set_PWM_freq(this->frequency);   // i2c registros set freq
}


void PCA9685 :: inicializar_i2c(){
	//Inicializacion  Bus I2C
		printf("PCA9685 inicializar_i2c() 1.1 Init...\n");
	if(!bcm2835_init())
		printf("PCA9685 inicializar_i2c() -> !!! bcm2835_init() failed\n");
	else
		printf("PCA9685 inicializar_i2c() 1.2 i2c starting\n");
	if(!bcm2835_i2c_begin()){
		printf("PCA9685 inicializar_i2c() -> !!! bcm2835_i2c_begin() failed");
		exit(0);
	}
	bcm2835_i2c_setSlaveAddress(this->addr); //direccion I2C = 0x40
	bcm2835_i2c_set_baudrate(100000);  //frecuencia por el I2C = 100MHz (100000MHz standard RPI3B+)
}

void PCA9685 :: restart_i2c(void){

	printf("PCA9685 restart_i2c() 1.3.1 I2C  RESET  \n");
this->write_byte_data(_MODE1, _RESTART);
bcm2835_delay(10);
// Set to sleep first, to turn off internal oscillator
this->write_byte_data(_MODE1, _SLEEP);
bcm2835_delay(2); //mili segundos
// Write logic 1 to both SLEEP bit and EXTCLK bit, the switch is now made. Enable the external oscillator
this->write_byte_data(_MODE1, _EXTCLK | _SLEEP);
   // Turn on oscillator now
this->write_byte_data(_MODE1, 0x00); //reset I2C
bcm2835_delay(2); //mili segundos

/*this->write_all_value(0, 0);
	printf("PCA9685 restart_i2c() 1.3.2 *** sleep para configurar el prescaler *** write_byte_data(_MODE2, _OUTDRV --> %i, %i\n",_MODE2 ,_OUTDRV);
	this->write_byte_data(_MODE2, _OUTDRV); // go to sleep
bcm2835_delay(2); //mili segundos
	printf("PCA9685 restart_i2c() 1.3.3 ** configurar prescaler ** write_byte_data(_MODE1, _ALLCALL --> %i, %i\n",_MODE1 ,_ALLCALL);
this->write_byte_data(_MODE1, _ALLCALL); // set the prescaler (El Bus I2C debe estar en modo SLEEP para configurar el PRESCALER)
bcm2835_delay(5); //5 mili segundos para que se recupere

uint8_t sleep = this->read_byte_data(_MODE1);
uint8_t awake = sleep & ~_SLEEP; //con el "&" se despierta (bit a low) y con un "|" se pone modo sleep
	printf("PCA9685 restart_i2c() 1.3.4 ** despertando ** 1.3.4 write_byte_data(_MODE1, mode1) --> %i, %i\n",_MODE1 ,awake);
this->write_byte_data(_MODE1, awake);
bcm2835_delay(5); //mili segundos
*/
	printf("PCA9685 setup() 1.4 set freq  \n");

}

//https://programmerclick.com/article/1519244875/
void PCA9685 :: set_PWM_freq(float freq){
		// Adafruit servo driver:
		//http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
//freq *= 0.9;  //Correct for overshoot in the frequency setting (see issue #11).
	if (freq < 1)
	    freq = 1;
	if (freq > 3500)
	    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)
	this->frequency=freq;
//	T =(1/freq*1000000);

	float prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= this->frequency;
	prescaleval -= 1;

	uint8_t prescale = floor(prescaleval + 0.5);

	uint8_t oldmode = this->read_byte_data(_MODE1);
	uint8_t newmode = (oldmode & ~_RESTART) | _SLEEP; //sleep: (_buffer[0] & 0x7F) | 0x10)
	this->write_byte_data(_MODE1, newmode);           // go to sleep
	bcm2835_delay(2);
	this->write_byte_data(_PRESCALE, prescale);       // set the prescaler --> PRE_SCALE can only be set when SLEEP is logic 1
	this->write_byte_data(_MODE1, oldmode);           //reset
	bcm2835_delay(2);
	this->write_byte_data(_MODE1, oldmode|0xa0);      //a1 = auto incremento (0x80 en python)
	bcm2835_delay(2);
}


float PCA9685 :: get_PWM_freq(){
	return (this->frequency);
}


/*
void PCA9685 :: write(uint8_t channel, uint32_t pulseWidth){
	uint16_t off = ((pulseWidth* 4096.0/T)*1.01);
	//uint16_t off = static_cast<uint16_t> ((pulseWidth* 4096.0/T)*1.01);
	this->write(channel,0,off);

	//int off = (int)abs(((angle - prevangle)*33)/180) + 10;
	//http://circuitden.com/blog/16
}
*/
//https://www.airspayce.com/mikem/bcm2835/group__pwm.html#ga7e2eddac472b6d81e0ba7dbf165672cb

//escribir en el canal(0-15) el valor del PWM(0-4095)
void PCA9685 :: write(uint8_t channel, uint16_t on, uint16_t off){
	bcm2835_i2c_setSlaveAddress(this->addr);
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
		printf("write all value (0,0)\n");
		bcm2835_i2c_setSlaveAddress(this->addr);
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
    // setPwm(pin, 0, angle * ((_max_pulse_width - _min_pulse_width) / _max_servo_degree) + _min_pulse_width);
}

PCA9685 :: ~PCA9685(){
		printf("~PCA9685() Borrar objeto PCA9685\n");
	bcm2835_i2c_end();
		printf("~PCA9685() Finalizado I2C \n");
	bcm2835_close();
		printf("~PCA9685() Finalizado bcm2835  \n");

}

int PCA9685 :: main_PCA9685(void){
		printf("main_pca9685\n");
	PCA9685 pwm = PCA9685(0x40);  //I2C addr = 0x40 para el control de ruedas
	pwm.setup();
	uint8_t i;
	try {
		for (i=0;i<15;i++) {
        sleep(0.5);
        	printf("\nChannel %d\n", i);
        sleep(1);
        uint8_t j;
        	for (j=0; j< 4096; j++) {
            pwm.write(i, 0, j);
            sleep(1);
        	}
		}
	}
	catch (...) {
		pwm.~PCA9685();
	}

    return 0;
}


uint8_t PCA9685 :: read_byte_data(uint8_t registro){

		bcm2835_i2c_setSlaveAddress(this->addr); // establecido I2C en  0x40
		//pasar el canal como el valor entero de un array (0,1,2,...) ademas de encapsular, facilita la forma de indicar las direcciones
		sendBuf[0] =  registro; // en este caso se indica asi ya que channel = _MODE1 = 0x00 que es el canal que queremos
		if((errCode = bcm2835_i2c_write(sendBuf,1)))  //escritura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		char byte;
		if((errCode = bcm2835_i2c_read(&byte,1))) //lectura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_read failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		return byte;  // return = 0-255
}
//https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/i2c.html
void PCA9685 :: write_byte_data(uint8_t registro, uint8_t data){
	printf("PCA9685::write_byte_data(...)\n");
	try {
		bcm2835_i2c_setSlaveAddress(this->addr);
		sendBuf[0] = registro;
		sendBuf[1] = data;
		if((errCode = bcm2835_i2c_write(sendBuf,2)))
			printf("(write_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	}
	catch (...) {
		printf("Error write_byte_data(...)\n");
	}
		printf("PCA9685::write_byte_data: address %p\n", this->addr);
		printf("PCA9685::write_byte_data: registro %i\n", registro);
		printf("PCA9685::write_byte_data: sendbuf %i\n",sendBuf[1] );
		printf("PCA9685::write_byte_data: data %i\n",data );
}


/*
 * bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_ALTn)	Selecciona la función alternativa n del pin.
bcm2835_pwm_set_clock(div)	Configura el divisor para el reloj de 19.2MHz.
bcm2835_pwm_set_mode(canal, ms, activo)	Configura el modo del canal PWM y/o lo activa.
bcm2835_pwm_set_range(canal, range)	Configura el rango del pulso en un canal PWM.
bcm2835_pwm_set_data(canal, v)	Configura la anchura de pulso del canal PWM.
 *
 */



