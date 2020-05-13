#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define __Main_1


#ifdef __Main_1 
	/*__________________Main code________________*/
	#include <Servo.h>  //add '<' and '>' before and after servo.h
	/* define baud rate as BAUD before this (can be defined externaly)*/
	#define BAUD  9600
	#define UBRRVAL F_CPU/8/BAUD-1
	#define RX_BUFFER_SIZE 32
	#define RXB_MASK 0x1f

	#define MPU_ADDR 0x68

	char rx_buffer[RX_BUFFER_SIZE];

	uint8_t rx_pointer;
	uint8_t ind;
	uint8_t received = 0;

	#undef FDEV_SETUP_STREAM
	#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }

	float accelerometer_x;
	float accAngleX;
	float accelerometer_y;		
	float accelerometer_z;
	float temperature;
	float gyro_x;
	float	gyro_y;
	float gyro_z;


	int v;
	int ADC_ON=0;
	int frist_time=0;
	int PAG = 0;
	int Equi_ON =0;
	int GA_ON=0;
	char str[20];

	
 
	int servoPin = 10;
	Servo servo;  
	int servoAngle = 0;   // servo position in degrees


	static int uputc(char,FILE*);

	static int uputc(char c,FILE *stream)
	{
		if (c == '\n')
			uputc('\r',stream);
		loop_until_bit_is_set(UCSR0A, UDRE0);	/* wait until we can send a new byte */
		UDR0 = (uint8_t) c;
		return 0;
	}

	void uputs(const char *c,FILE *stream)
	{
		do {
			uputc(*c++,stream);
		} while (*c != 0);
	}

	char* convert_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
	sprintf(str, "%6d", i);
	return str;
	}

	static FILE mystdout = FDEV_SETUP_STREAM(uputc, NULL,_FDEV_SETUP_WRITE);

	void clear_buffer()
	{
		static uint8_t i;
		for (i = 0; i < RX_BUFFER_SIZE; i++)
			rx_buffer[i] = '\0';
		rx_pointer = 0;
	}

	void init_adc(void)
	{
		
		ADMUX = (1<<REFS0) | (1 << MUX1); // PIN A2
		//ADCSRA &= ~(1 << ADIF);
		ADCSRA = (~(1 << ADIF))|(1 << ADATE)|(1 << ADSC)|(1 << ADPS2);
		//ADCSRA |= (1 << ADPS2)| (1 << ADPS1)| (1 << ADPS0);
		ADCSRA |= (1 << ADIE);
	}
	void init_UART(void)
	{
		/* uart config */
		UCSR0A = (1 << U2X0); /* this sets U2X0 to 1 */
		UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
		UCSR0C = (3 << UCSZ00); /* this sets UCSZ00 to 3 */

		/* baudrate setings (variable set by macros) */
		UBRR0H = (UBRRVAL) >> 8;
		UBRR0L = UBRRVAL;
	}

	void init_AG(void)
	{
		Wire.begin();
		Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
		Wire.write(0x6B); // PWR_MGMT_1 register
		
		Wire.write(0); // set to zero (wakes up the MPU-6050)
		printf("Esta ok!!\n\r");
		printf("Esta ok!!!\n\r");
	}

	void Active_ADC(void)
	{
		ADC_ON = 1;
		ADCSRA |=(1<<ADEN);
	}

	void OFF_ADC(void)
	{
		ADC_ON = 1;
		ADCSRA &= ~(1<<ADEN);
	}

	void ON_AG(void)
	{
		if (frist_time==1)
		{
			Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
			Wire.write(0x3B);
			Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
			frist_time = 0; 	
			
			printf("Active\n\r");
		}
	}

	void Controle(void)
	{
		if (rx_buffer[rx_pointer-1]=='\r' && rx_pointer>0)
		{
			rx_buffer[rx_pointer-1]='\0';
			if (strncmp(rx_buffer,"ON",2) == 0)
			{
				puts("OLA");
				Active_ADC();
				clear_buffer();
			}
			else if (strncmp(rx_buffer,"OFF",3) == 0)
			{
				printf("The ADC is OFF");
				OFF_ADC();
			}
			else if (strncmp(rx_buffer,"GAON",4) == 0)
			{
				frist_time=1;
				ON_AG();
				GA_ON=1;
			}
			else if (strncmp(rx_buffer,"PAG",3) == 0)
			{
				PAG=1-PAG;
			}
			else if (strncmp(rx_buffer,"EQON",4) == 0)
			{
				Equi_ON=1-Equi_ON;
			}
		}
		else
		{
			clear_buffer();
		}
		
	}

	void Manuel(void)
	{
		puts("Controles:\n\r");
		puts(" a - activestes the ADC;\n\r");
	}

	ISR(ADC_vect)
	{
		ADCSRA &= ~(1 << ADIF);
		v=(ADCL | ADCH<<8);
	}

	//uart receive isr
	ISR(USART0_RX_vect)
	{
		uint8_t c = UDR0;

		rx_buffer[rx_pointer++] = (char) c;	
	}

	int main(void)
	{
		float tempC ;
		byte precision = 4;
		char floatBuffer[20];
		char printBuffer[80];
		int atual_ang =0;

		stdout=&mystdout;
		init_adc();
		init_UART();
		init_AG();
		servo.attach(servoPin);
		/* pin config */
		// DDRB = (1 << DDB5);
		// PORTB=(1<<PB5);
		// DDRF = 0b00000000;
		

		/* ADC cfg */
		sei();							  /* enable interrupts */

		Manuel();

		while(1) 
		{
			Controle();
			if (v >0 && ADC_ON==1)
			{
				tempC = (float)v/1023*5.0;
				dtostrf(tempC, precision+3, precision, floatBuffer);
				sprintf(printBuffer, "With %%.%df precision, x = %s", precision, floatBuffer);
				puts(printBuffer);
				v=0;
			}
			else if ( GA_ON == 1 )
			{
				Wire.beginTransmission(MPU_ADDR);
				Wire.write(0x3B);
				Wire.endTransmission(false);
				Wire.requestFrom(MPU_ADDR, 7*2, true);
				// "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
				accelerometer_x = (int)(Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
				accelerometer_y = (int)(Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
				accelerometer_z = (int)(Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
				accAngleX = (atan(accelerometer_y / sqrt(pow(accelerometer_x, 2) + pow(accelerometer_z, 2))) * 180 / PI) - 0.58;
				if (PAG == 1)
				{
					printf("aX = ");
					dtostrf(accelerometer_x, precision+3, precision, str);
					printf(str);
					printf(" | aY = ");
					dtostrf(accelerometer_y, precision+3, precision, str);
					printf(str);
					printf(" | aZ = ");
					dtostrf(accelerometer_z, precision+3, precision, str);
					printf(str);
					printf("\n\r");
					printf("AngleX = ");
					dtostrf(accAngleX, precision+3, precision, str);
					printf(str);
					printf("\n\r");
				}
				if (Equi_ON == 1)
				{
					atual_ang = (int)accAngleX+92;
					//printf("Aqui ok mesmo\n\r");
					if (abs(atual_ang-90)<=5)
					{
						//printf("1\n\r");
						servo.write(atual_ang);
					}
					else if (atual_ang<80)
					{
						//printf("2\n\r");
						for(servoAngle = 180-atual_ang; servoAngle > 90; servoAngle--)  //move the micro servo from 0 degrees to 180 degrees
						{           
												
							servo.write(servoAngle); 
							// dtostrf(servoAngle, 3, 0, str);  
							// printf(str);           
							_delay_ms(5);                
						}
					}
					else if (atual_ang>100)
					{
						//printf("3\n\r");
						for(servoAngle = 180-atual_ang; servoAngle < 90; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
						{           
												
							servo.write(servoAngle); 
							// dtostrf(servoAngle, 3, 0, str);  
							// printf(str);           
							_delay_ms(5);                
						}
					}
				}
			}
			clear_buffer();
			_delay_ms(100);
			
		
		}
	}
#endif
/*_____________________________________________*/
#ifdef __Main_2
//(c) Michael Schoeffler 2017, http://www.mschoeffler.de


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
float temperature; // variables for temperature data
float accAngleX;
char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(float i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6f", i);
  return tmp_str;
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = (Wire.read()<<8 | Wire.read())/16384.0; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  accAngleX = (atan(accelerometer_y / sqrt(pow(accelerometer_x, 2) + pow(accelerometer_z, 2))) * 180 / PI) - 0.58;
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = ");
  Serial.print(accelerometer_x);
  Serial.print(" | aY = "); 
  Serial.print(accelerometer_y);
  Serial.print(" | aZ = "); 
  Serial.print(accelerometer_z);
  Serial.print(" | angX = "); 
  Serial.print(accAngleX);
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); 
  Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); 
  Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); 
  Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); 
  Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  // delay
  delay(1000);
}
#endif

#ifdef __Main_3 // https://www.intorobotics.com/tutorial-how-to-control-the-tower-pro-sg90-servo-with-arduino-uno/
#include <Servo.h>  //add '<' and '>' before and after servo.h
 
int servoPin = 10;
 
Servo servo;  
 
int servoAngle = 0;   // servo position in degrees
 
void setup()
{
  Serial.begin(9600);  
  servo.attach(servoPin);
}


 
void loop()
{
//control the servo's direction and the position of the motor
 
   servo.write(0);      // Turn SG90 servo Left to 45 degrees
   delay(1000);          // Wait 1 second
   servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);          // Wait 1 second
   servo.write(180);     // Turn SG90 servo Right to 135 degrees
   delay(1000);          // Wait 1 second
   servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);
 
//end control the servo's direction and the position of the motor
 
 
//control the servo's speed  
 
//if you change the delay value (from example change 50 to 10), the speed of the servo changes
  for(servoAngle = 0; servoAngle < 180; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
  {                                  
    servo.write(servoAngle);              
    delay(50);                  
  }
 
  for(servoAngle = 180; servoAngle > 0; servoAngle--)  //now move back the micro servo from 0 degrees to 180 degrees
  {                                
    servo.write(servoAngle);          
    delay(10);      
  }
  //end control the servo's speed  
}
#endif