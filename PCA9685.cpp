using namespace std; //para evitar poner std:: en cada definicion de vectores

#include <bcm2835.h>
#include "PCA9685.h"
#include <unistd.h>
#include <stdio.h>      /* printf */
#include <math.h>       /* floor */
#include <time.h>
/*
 *
 * // https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
//https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/i2c.html
 *
* https://www.airspayce.com/mikem/bcm2835/pwm_8c-example.html
* https://programmerclick.com/article/1519244875/
* https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
* https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/Adafruit_PWMServoDriver.h
*
* //https://github.com/jrowberg/i2cdevlib/blob/master/RaspberryPi_bcm2835/I2Cdev/I2Cdev.h
//https://github.com/jrowberg/i2cdevlib/blob/master/RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp
*/

PCA9685 :: PCA9685(uint8_t address){ //addr = 0x40
		this->addr = address;        //se podria pasar la direccion 0x48 directamente al crear el metodo pero al pasarlo desde el .h esta mas encapsulado
	//	this->Buffer[5] = {'0'};   // Inicializamos todos los valores a 0 (3 es la longitud total del array de chars)
		this->errCode = 0;
		this->frequency = 60;
		this->T= 1/this->frequency;
}

void PCA9685 :: setup(void){
	inicializar_i2c();               // bcm2835_i2c_begin()
	//restart_i2c();                   // i2c registros inicializar

	printf("FIN SETUP I2C PCA9685 ****\n");
}

void PCA9685 :: inicializar_i2c(){
	//Inicializacion  Bus I2C
		printf("PCA9685 inicializar_i2c() 1.1 Init...\n");
	if(!bcm2835_init()){
		printf("PCA9685 inicializar_i2c() -> !!! bcm2835_init() failed\n");
		exit(0);
	}
	else
		printf("PCA9685 inicializar_i2c() 1.2 i2c starting\n");
	if(!bcm2835_i2c_begin()){
		printf("PCA9685 inicializar_i2c() -> !!! bcm2835_i2c_begin() failed");
		exit(0);
	}
	bcm2835_i2c_setSlaveAddress(this->addr); //direccion I2C = 0x40
	bcm2835_i2c_set_baudrate(100000);  //frecuencia por el I2C = 100KHz (100000Hz standard RPI3B+)
	//this->write_byte_data(_MODE1, 0x00); //reset I2C
	this->write_byte_data(_MODE1, 0x00); //reset I2C
	set_PWM_freq(this->frequency);

	//https://www.airspayce.com/mikem/bcm2835/group__constants.html#ga63c029bd6500167152db4e57736d0939
	//bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_ALT5);  // pin_18 como Alt Fun5 para establecer PWM0
	//bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(18, RPI_BPLUS_GPIO_J8_12);  // pin_18 como Alt Fun5 para establecer PWM0 en rpi3b+
		printf("\n***  PCA9685 (SERVO PWM) Inicializado en pin 18 como ALT FUN 5 (PWM0)***\n");
	this->write_PWM(0, 0, 4096);  //  pulso I2c
	sleep(0.5);
	bcm2835_delay(20);
    this->write_PWM(0, 0, 0);
    sleep(0.5);
     printf("***  PCA9685  I2C  INICIALIZADO***\n");
}

void PCA9685 :: restart_i2c(void){
	printf("PCA9685 restart_i2c() 1.3.1 I2C  RESET  \n");
	this->write_byte_data(_MODE1, _RESTART);
	delay(2);
	// Set to sleep first, to turn off internal oscillator
	this->write_byte_data(_MODE1, _SLEEP);
	delay(20); //mili segundos
	// Write logic 1 to both SLEEP bit and EXTCLK bit, the switch is now made. Enable the external oscillator
	this->write_byte_data(_MODE1, _EXTCLK | _SLEEP);
   // Turn on oscillator now
	this->write_byte_data(_MODE1, 0x00); //reset I2C
	delay(2);                   //mili segundos

/*
this->write_all_value(0, 0);
	printf("PCA9685 restart_i2c() 1.3.2 *** sleep para configurar el prescaler *** write_byte_data(_MODE2, _OUTDRV --> %i, %i\n",_MODE2 ,_OUTDRV);
	this->write_byte_data(_MODE2, _OUTDRV); // go to sleep
sleep(0.5); //mili segundos
	printf("PCA9685 restart_i2c() 1.3.3 ** configurar prescaler ** write_byte_data(_MODE1, _ALLCALL --> %i, %i\n",_MODE1 ,_ALLCALL);
this->write_byte_data(_MODE1, _ALLCALL); // set the prescaler (El Bus I2C debe estar en modo SLEEP para configurar el PRESCALER)
sleep(0.5); //5 mili segundos para que se recupere
uint8_t sleep = this->read_byte_data(_MODE1);
uint8_t awake = sleep & ~_SLEEP; //con el "&" se despierta (bit a low) y con un "|" se pone modo sleep
	printf("PCA9685 restart_i2c() 1.3.4 ** despertando ** 1.3.4 write_byte_data(_MODE1, mode1) --> %i, %i\n",_MODE1 ,awake);
this->write_byte_data(_MODE1, awake);

*/
}

//https://programmerclick.com/article/1519244875/
void PCA9685 :: set_PWM_freq(float freq){
		// Adafruit servo driver: http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver
//freq *= 0.9;  //Correct for overshoot in the frequency setting (see issue #11).
	this->frequency=freq;
	float prescaleval = (25000000 / 4096) / freq - 1; // prescaleval = 100,72 OK
	//uint8_t prescale = prescaleval; //match.floor(prescaleval + 0.5); prescale = clock/4096*rate=100,75; prescale=100 ;  Rate=60, Clock =25Mhz
	uint8_t prescale = floor(prescaleval + 0.5);

	//DCONFIGURAR PRESCALER
	uint8_t oldmode = this->read_byte_data(_MODE1);
	uint8_t newmode = ((oldmode & 0x7F) | 0x10); //sleep: (_Buffer[0] & ~_RESTART) | _SLEEP) = 0x10
	this->write_byte_data(_MODE1, newmode);           // go to sleep
	this->write_byte_data(_PRESCALE, prescale);       // set the prescaler --> PRE_SCALE can only be set when SLEEP is logic 1
	this->write_byte_data(_MODE1, oldmode);           //reset
	bcm2835_delay(5);
	this->write_byte_data(_MODE1, oldmode|0xa1);      //a1 = auto incremento (0x80 en python)  = 0x20

	//PROBAR ESTO
	/* https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/Adafruit_PWMServoDriver.cpp
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  write8(PCA9685_PRESCALE, prescale); // set the prescaler

  delay(5);
  // clear the SLEEP bit to start
  write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
	 */
}

float PCA9685 :: get_PWM_freq(){
	return (this->frequency);
}

void PCA9685 :: write_pulse(uint8_t channel, uint32_t pulseWidth){
	uint16_t off = ((pulseWidth* 4096.0/T)*1.01);
	//uint16_t off = static_cast<uint16_t> ((pulseWidth* 4096.0/T)*1.01);
	this->write_PWM(channel,0,off);
	//int off = (int)abs(((angle - prevangle)*33)/180) + 10;
	//http://circuitden.com/blog/16
}

// canal(entre 0 y 15)
// valor del PWM(entre 0 y 4095)
void PCA9685 :: write_PWM(uint8_t channel, uint16_t on, uint16_t off){
	bcm2835_i2c_setSlaveAddress(this->addr);
	char buffer[5];
	buffer[0] = LED0_ON_L+4*channel;
	buffer[1] = on & 0x00FF;
	buffer[2] = on >> 8;
	buffer[3] = off & 0x00FF;
	buffer[4] = off >> 8;
	if((errCode = bcm2835_i2c_write(buffer,5)))
		printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
}

void PCA9685 :: write_all_value (uint16_t on, uint16_t off){
	char buffer[5];
	try {
		printf("write all value (0,0)\n");
		bcm2835_i2c_setSlaveAddress(this->addr);
		buffer[0] = _ALL_LED_ON_L;
		buffer[1] = on & 0x00FF;
		buffer[2] = on >> 8;
		buffer[3] = off & 0x00FF;
		buffer[4] = off >> 8;
		if((errCode = bcm2835_i2c_write(buffer,5)))
			printf("bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
	}
	catch (...) {
		printf("Error write_all_value(...)\n");
	}
}

uint8_t PCA9685 :: map(int x, int in_min, int in_max, int out_min, int out_max){
     //return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
          // es lo mismo que:
     //return( angle * ((_max_pulse_width - _min_pulse_width) / _max_servo_degree) + _min_pulse_width) );
     	 // es lo mismo que:
     //return (x * 1800 / 180 + 600);
 	 	 // es lo mismo que:
     return (x * 10.3 + 546); //pTE PROBAR
}



char sendBuf[256];
char recvBuf[256];
// READBYTE --> ALTERNATIVA PARA READ BYTE DATA
//https://github.com/jrowberg/i2cdevlib/blob/master/RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp
int8_t PCA9685 ::readByte(uint8_t registro, uint8_t *data) {
  bcm2835_i2c_setSlaveAddress(this->addr);
  sendBuf[0] = registro;
  uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1);
  data[0] = (uint8_t) recvBuf[0];
  return response == BCM2835_I2C_REASON_OK;
}
uint8_t PCA9685 :: read_byte_data(uint8_t registro){
		bcm2835_i2c_setSlaveAddress(this->addr); // establecido I2C en  0x40
		char Buffer[1];
		char byte;
		Buffer[0] = registro; //  channel = _MODE1 = 0x00 que es el canal que queremos
		//dejo esta configuracion, ya que funciona OK en PCF8591
		if((errCode = bcm2835_i2c_write(Buffer,1)))  //escritura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		if((errCode = bcm2835_i2c_read(&byte,1))) //lectura de un char byte en el canal indicado
			printf("(read_byte_data) bcm2835_i2c_read failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);
		return byte;  // return = 0-255
		/*
		 se descarta  configuracion bcm2835_i2c_read_register_rs segun indicaciones de la web
		https://www.airspayce.com/mikem/bcm2835/group__i2c.html#ga5d01c1483ae12ff682068d0a56b6529a
		 */
}
//https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/i2c.html
void PCA9685 :: write_byte_data(uint8_t registro, uint8_t data){
	printf("PCA9685::  **write_byte_data(...)**\n");
	char buffer[2];
		bcm2835_i2c_setSlaveAddress(this->addr);
		buffer[0] = registro;
		buffer[1] = data;
		if((errCode = bcm2835_i2c_write(buffer,2)))
			printf("(write_byte_data) bcm2835_i2c_write failed at %s%d, errCode = 0x%x\n",__FILE__,__LINE__, errCode);

	printf("PCA9685::write_byte_data: address 0x%02x\n", this->addr); // tambien es posible con %p
	printf("PCA9685::write_byte_data: registro 0x%02x\n", registro);  // tambien es posible con %i
	//printf("PCA9685::write_byte_data: Buffer %d\n",buffer[1] );
	printf("PCA9685::write_byte_data: data %i\n",data );
}

PCA9685 :: ~PCA9685(){
		printf("~PCA9685() Borrar objeto PCA9685\n");
	bcm2835_i2c_end();
		printf("~PCA9685() Finalizado I2C \n");
	bcm2835_close();
		printf("~PCA9685() Finalizado bcm2835  \n");
}

/*
            //https://www.perlmonks.org/?node_id=1166558
#include <stdio.h>
#include <bcm2835.h>

// PWM output on RPi Plug P1 pin 12 (which is GPIO pin 18)
// in alt fun 5.
// Note that this is the _only_ PWM pin available on the RPi IO header
+s
#define PIN RPI_GPIO_P1_12
// and it is controlled by PWM channel 0
#define PWM_CHANNEL 0
// This controls the max range of the PWM signal
#define RANGE 1024

int pwm_prepare();
void pwm_clean();
void pwm_set_led_intensity (int level);

int pwm_prepare(){
    if (!bcm2835_init())
        return 1;
    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be o
+utput there
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_ALT5);

    // Clock divider is set to 16.
    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
    // the pulse repetition frequency will be
    // 1.2MHz/1024 = 1171.875Hz
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16); //Configura el divisor para el reloj de 19.2MHz.
    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1); //Configura el modo del canal PWM y/o lo activa.
    bcm2835_pwm_set_range(PWM_CHANNEL, RANGE); //	Configura el rango del pulso en un canal PWM.
    return 0;
}

void pwm_clean(){
    bcm2835_close();
    return;
}

void pwm_set_led_intensity (int level) {

        if (0 <= level && level <= 1024) {

                bcm2835_pwm_set_data(0,level); //Configura la anchura de pulso del canal PWM.
        }
        return;
}
*/

/*
 * //https://www.airspayce.com/mikem/bcm2835/group__pwm.html#ga7e2eddac472b6d81e0ba7dbf165672cb
      //https://franciscomoya.gitbooks.io/taller-de-raspberry-pi/content/es/c/gpio.html
#include <bcm2835.h>
#include <stdio.h>
int main(int argc, char* argv[])
{
    if (argc < 5) {
        printf("Usage: %s divisor rango min max\n", argv[0]);
        exit(0);
    }
    int div = atoi(argv[1]);
    int range = atoi(argv[2]);
    int min = atoi(argv[3]);
    int max = atoi(argv[4]);

    bcm2835_init();
    bcm2835_gpio_fsel(18, BCM2835_GPIO_FSEL_ALT5);
    bcm2835_pwm_set_clock(div);
    bcm2835_pwm_set_mode(0, 1, 1);
    bcm2835_pwm_set_range(0, range);

    for(;;) {
        bcm2835_pwm_set_data(0, min);
        bcm2835_delay(1000);
        bcm2835_pwm_set_data(0, max);
        bcm2835_delay(1000);
    }
    bcm2835_close();
    return 0;
}
 */


