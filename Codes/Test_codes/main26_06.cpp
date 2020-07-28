#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include<Servo.h>
#include<PID_v1.h>
#include "wiring_private.h"

#define __Main_ok




#ifdef __Main_1 
	/*__________________Main code________________*/
	#include <Servo.h>  //add '<' and '>' before and after servo.h
	#include<PID_v1.h>
	/* define baud rate as BAUD before this (can be defined externaly)*/
	#define BAUD  9600
	#define F_OSC 16000000
	#define UBRRVAL F_CPU/8/BAUD-1
	#define RX_BUFFER_SIZE 32
	#define RXB_MASK 0x1f

	#define MPU_ADDR 0x68
	
	char rx_buffer[RX_BUFFER_SIZE];

	uint8_t rx_pointer;
	uint8_t ind;
	uint8_t received = 0;

	float Kp = 2;                                                    //Initial Proportional Gain
	float Ki = 0.01;                                                      //Initial Integral Gain
	float Kd = 0;                                                    //Intitial Derivative Gain
	double Setpoint, Input, Output, ServoOutput;                                 

	PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.

	#undef FDEV_SETUP_STREAM
	#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }


	
	float accelerometer_x;
	float accAngleX;
	float accelerometer_y;		
	float accelerometer_z;
	float temperature;
	float gyro_x;
	float gyro_y;
	float gyro_z;


	int v;
	int ADC_ON = 0;
	int frist_time = 0;
	int PAG = 0;
	int Equi_ON = 0;
	int GA_ON = 0;
	char str[20];
	int ang = 90;
	int show_ang= 0;
	int dezenas = 0;
	int centenas = 0;
	int unidades = 0;
	int centezimas = 0;
	int DONT = 0;
	int ang_cent;
	int delay_me = 20;
 
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

	
	void numero(int i)
	{
		if (i == 1)
			PORTF = 0b10011111;
		else if (i == 2)
			PORTF = 0b00100101;
		else if (i == 3)
			PORTF = 0b00001101;
		else if (i == 4)
			PORTF = 0b10011001;
		else if (i == 5)
			PORTF = 0b01001001;
		else if (i == 6)
			PORTF = 0b01000001;
		else if (i == 7)
			PORTF = 0b00011111;
		else if (i == 8)
			PORTF = 0b00000001;
		else if (i == 9)
			PORTF = 0b00001001;
		else if (i == 0)
			PORTF = 0b00000011;
		else
			printf("ERROR");
	}
//			 __A__
//		F	|     | B
//			|__G__|
//		E	|     | C
//			|__D__|
	void code_numeros(int c,int d, int u, int f, int de)
	{
		//Pin PF7->A
		//Pin PF6->B
		//Pin PF5->C
		//Pin PF4->D
		//Pin PF3->E
		//Pin PF2->F
		//Pin PF1->G
		PORTK = 0b00001000;
		numero(c);
		_delay_ms(1.4);
		PORTK = 0b00000010;
		numero(u);
		PORTF &= ~(1<<PORTF0);
		_delay_ms(1.4);
		PORTK = 0b00000001;
		numero(f);
		_delay_ms(1.4);
		PORTK = 0b00000100;
		numero(d);
		_delay_ms(0.4);
		//PORTK = 0b00001111;
		//PORTK = 0b00000000;
		
	}
	void Show_val(int atual_ang,int de)
	{
		if (DONT == 0)
		{
			dezenas = (atual_ang%100)/10;
			centenas = (atual_ang-dezenas*10)/100;
			unidades = atual_ang-centenas*100-dezenas*10;
			centezimas=0;
		}
		
		// dtostrf(centenas, 3, 0, str);
		// printf(str);
		// printf("\n\r");
		code_numeros(centenas,dezenas,unidades,centezimas,de);
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
			else if (strncmp(rx_buffer,"GA",2) == 0)
			{
				frist_time=1;
				ON_AG();
				GA_ON=1;
			}
			else if (strncmp(rx_buffer,"PAG",3) == 0)
			{
				PAG=1-PAG;
			}
			else if (strncmp(rx_buffer,"EQ",2) == 0)
			{
				Equi_ON=1-Equi_ON;
				printf("OKOK\n\r");
			}
			else if (strncmp(rx_buffer,"SERVOS",6) == 0)
			{
				servo.write(ang);
			}
			else if (strncmp(rx_buffer,"SERVO+",6) == 0)
			{
				ang = ang+5;
				servo.write(ang);
			}
			else if (strncmp(rx_buffer,"SERVO-",6) == 0)
			{
				ang = ang-5;
				servo.write(ang);
				
			}
		}
		else if (!(PINB & (1 << PINB0)))
		{
				//printf("Ola");
				ang = ang-5;
				servo.write(ang);
				_delay_ms(105);
		}
		else if (!(PINB & (1 << PINB1)))
		{
				ang = ang+5;
				servo.write(ang);
				_delay_ms(105);
		}
		else if(!(PINB & (1 << PINB2)))
		{
				frist_time=1;
				ON_AG();
				GA_ON=1;
				_delay_ms(10);
				Equi_ON=1;
				_delay_ms(100);
				ang_cent=ang;
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

	void PID_set()
	{
		Input =0;
		myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
  		
		  myPID.SetOutputLimits(-35,35);                                     //Set Output limits to -80 and 80 degrees.
  	}


	ISR(ADC_vect)
	{
		ADCSRA &= ~(1 << ADIF);
		v=(ADCL | ADCH<<8);
	}

	

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
		unsigned long lastTime =0; 
		unsigned long now = 0;
   		unsigned long timeChange =0;
		

		stdout=&mystdout;
		init_adc();
		PID_set();
		init_UART();
		init_AG();
		servo.attach(servoPin);
		/* pin config */
		// DDRB = (1 << DDB5);
		// PORTB=(1<<PB5);
		// DDRF = 0b00000000;
		DDRB &= ~(1<<DDB0 |1<<DDB1 | 1<<DDB2);
		DDRF = 0b11111111;
		DDRK = 0b00001111;
		/* ADC cfg */
		sei();							  /* enable interrupts */

		Manuel();
		
		//Show_val(ang);
		//_delay_ms(1000);
		while(1) 
		{	
			Controle();  // tirar o '//' da linha 489

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
				//ang_cent=ang;
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
				/*if (Equi_ON == 1)
				{
					atual_ang = (int)accAngleX+ang;
					ang_cent=ang;
					if (abs(atual_ang-show_ang) > 3)
					{
						show_ang = atual_ang;
						DONT=0;
					}
					else
					{
						DONT=1;
					}
					Show_val(show_ang,delay_me);
					_delay_ms(5);
					
					
					//printf("Aqui ok mesmo\n\r");
					// if (abs(atual_ang-90)<=5)
					// {
					// 	//printf("1\n\r");
					// 	servo.write(atual_ang);
					// }
					servo.detach();
					if (atual_ang<(ang_cent-5))
					{
						servo.attach(servoPin);	
						printf("2\n\r");
						for(servoAngle = atual_ang; servoAngle < ang_cent+15; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
						{           
									
							servo.write(servoAngle); 
							// dtostrf(servoAngle, 3, 0, str);  
							// printf(str);  
							Show_val(atual_ang,delay_me);        
							_delay_ms(5);                
						}
					}
					else if (atual_ang>(ang_cent+5))
					{
						servo.attach(servoPin);
						printf("3\n\r");
						for(servoAngle = atual_ang; servoAngle > ang_cent-10; servoAngle--)  //move the micro servo from 0 degrees to 180 degrees
						{           
												
							servo.write(servoAngle); 
							// dtostrf(servoAngle, 3, 0, str);  
							// printf(str);
							Show_val(atual_ang,delay_me);          
							_delay_ms(5);                
						}
					}
				}*/
				Equi_ON=1;
				if(Equi_ON == 1)
				{
					_delay_ms(200);
					Input = accAngleX;
					dtostrf(Input, 3, 2, str);  
					printf("Input = ");
					printf(str);          	                                 
					printf("\n");
					Setpoint = 10;
					now = millis();
   					timeChange = (now - lastTime);
					dtostrf(now, 3, 10, str);  
					printf("Time = ");
					printf(str);
					printf("\n");
					printf("%d",now);
					lastTime=now;
					

 					bool p = myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
					ServoOutput=90+Output;                                            // 150 degrees is my horizontal
					dtostrf(Output, 3, 2, str);  
					printf("Output = ");
					printf(str);          	                                 
					printf("\n");
					dtostrf((int)p, 3, 2, str);  
					printf("bool = ");
					printf(str);          	                                 
					printf("\n");  
					//servo.write(ServoOutput);                                        //Writes value of Output to servo 
	
				}
			}
				
			else
				{
					Show_val(ang,delay_me);
				}
			clear_buffer();
			//_delay_ms(20);
			
		
		}
	}
#endif

#ifdef __Main_ok
/* define baud rate as BAUD before this (can be defined externaly)*/
#define BAUD  9600
#define F_OSC 16000000
#define UBRRVAL F_CPU/8/BAUD-1
#define RX_BUFFER_SIZE 32
#define RXB_MASK 0x1f

#define MPU_ADDR 0x68



#undef FDEV_SETUP_STREAM
#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }
char rx_buffer[RX_BUFFER_SIZE];

int size_p =350;
double p_velo[350];
double p_pos[350];
double p_ang[350];
double pc_velo[350];
double pc_pos[350];
int i_pos=0;
int i_vel=0;
int i_ang=0;

long time;
long period=70;
long PID_total;
long PID_p, PID_i, PID_d;
int distance_previous_error;

uint8_t rx_pointer;
uint8_t ind;
uint8_t received = 0;


const int servoPin = 10;                                               //Servo Pin

float Kp = 1.5;                                                    //Initial Proportional Gain
float Ki = 1.5;                                                      //Initial Integral Gain
float Kd = 65;
                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
																	
Servo myServo;                                                       //Initialize Servo.
const int echoPin = 7;
const int trigPin = 9;
unsigned long lastTime =0; 
unsigned long now = 0;
unsigned long timeChange =0;
int pulse_60=0;
char str[20];
int over_f=0;
int s_over=0;

int FT =0;
float T1,T2;
int distance;

float accelerometer_x;
float accAngleX;
float accelerometer_y;		
float accelerometer_z;
float temperature;
float gyro_x;
float gyro_y;
float gyro_z;

int data_save=0;
int ADC_ON = 0;
int frist_time = 0;
int PAG = 0;
int Equi_ON = 0;
int GA_ON = 0;
int ang = 90;
int show_ang= 0;
int dezenas = 0;
int centenas = 0;
int unidades = 0;
int centezimas = 0;
int DONT = 0;
int ang_cent;
int count_show=0;

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


void clear_buffer()
{
	static uint8_t i;
	for (i = 0; i < RX_BUFFER_SIZE; i++)
		rx_buffer[i] = '\0';
	rx_pointer = 0;
}

void set_timers()
{


	/*// timer que mando um pulso para o triger com 10 us
	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR2A = 159;// = (16*10^6) / (100000*1) - 1 (must be <256) para o triger do sonar
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS21 bit for 1 prescaler
	TCCR2B |= (1 << CS20);   
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);*/


	// timer responsavel em medir o tempo do sinal do echo
	TCCR4A = 0; //Disable compare interrupts
	TCCR4B = 0;
	TCNT4 = 0; //SETTING INTIAL TIMER VALUE

	TCCR4B|=(1<<ICES1); //SETTING FIRST CAPTURE ON RISING EDGE
	TIMSK4|=(1<<ICIE1); //ENABLING INPUT CAPTURE
	TIMSK4|=(1<<TOIE1);
	TIFR4 &= ~(1 << ICF1); //Clear the flag
	TCCR4B|=(1<<CS10);
	TCCR4B &= ~(1<<CS12);
	TCCR4B &= ~(1<<CS11);//STARTING TIMER WITH NO PRESCALER


	TCCR0A = 0; //Disable compare interrupts
	TCCR0B = 0;
	TCNT0 = 0; //SETTING INTIAL TIMER VALUE

	TCCR0B|=(1<<ICES1); //SETTING FIRST CAPTURE ON RISING EDGE
	TIMSK0|=(1<<TOIE1);
	TIFR0 &= ~(1 << ICF1); //Clear the flag
	TCCR0B|=(1<<CS10);
	TCCR0B |= (1<<CS11);
	TCCR0B &= ~(1<<CS12);//STARTING TIMER WITH NO PRESCALER
	sei();
}

void set_sonar()
{
	DDRH &=  ~(1<<DDH4);  //input (echo)
	DDRH |=  1<<DDH6;    // Output (triger)
}

void init_AG(void)
	{
		Wire.begin();
		Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
		Wire.write(0x6B); // PWR_MGMT_1 register
		
		Wire.write(0); // set to zero (wakes up the MPU-6050)
	    Wire.endTransmission(true);
	}
	void numero(int i)
	{
		if (i == 1)
			PORTF = 0b10011111;
		else if (i == 2)
			PORTF = 0b00100101;
		else if (i == 3)
			PORTF = 0b00001101;
		else if (i == 4)
			PORTF = 0b10011001;
		else if (i == 5)
			PORTF = 0b01001001;
		else if (i == 6)
			PORTF = 0b01000001;
		else if (i == 7)
			PORTF = 0b00011111;
		else if (i == 8)
			PORTF = 0b00000001;
		else if (i == 9)
			PORTF = 0b00001001;
		else if (i == 0)
			PORTF = 0b00000011;
		else
			printf("ERROR\n");
	}
//			 __A__
//		F	|     | B
//			|__G__|
//		E	|     | C
//			|__D__|
	void code_numeros(int c,int d, int u, int f, int delay_m)
	{
		//Pin PF7->A
		//Pin PF6->B
		//Pin PF5->C
		//Pin PF4->D
		//Pin PF3->E
		//Pin PF2->F
		//Pin PF1->G
		for (int de=0;de<=delay_m;de=de+5)
		{
			PORTK = 0b00001000;
			numero(c);
			_delay_ms(1.5);
			PORTK = 0b00000010;
			numero(u);
			PORTF &= ~(1<<PORTF0);
			_delay_ms(1.5);
			PORTK = 0b00000001;
			numero(f);
			_delay_ms(1.5);
			PORTK = 0b00000100;
			numero(d);
			_delay_ms(0.5);
			//PORTK = 0b00001111;
			//PORTK = 0b00000000;
		}
		
	}


	void Show_val(int atual_ang,int de)
	{
		if (DONT == 0)
		{
			
    		dezenas = (atual_ang%1000)/100;
			centenas = (atual_ang-dezenas*100)/1000;
			unidades = (atual_ang-centenas*1000-dezenas*100)/10;
			centezimas=atual_ang-centenas*1000-dezenas*100-unidades*10;
		}
		
		// printf("dezenas = ");
		// dtostrf(dezenas, 3, 0, str);
		// printf(str);
		// printf("\n\r");
		
		// printf("centenas = ");
		// dtostrf(centenas, 3, 0, str);
		// printf(str);
		// printf("\n\r");
		// printf("unidades = ");
		// dtostrf(unidades, 3, 0, str);
		// printf(str);
		// printf("\n\r");
		// printf("decimas = ");
		// dtostrf(centezimas, 3, 0, str);
		// printf(str);
		// printf("\n\r");
		code_numeros(centenas,dezenas,unidades,centezimas,de);
	}
void readAngle() 
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR, 7*2, true);
	// "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
	accelerometer_x =  (int)(Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)  fazer isto com shitf a
	accelerometer_y = (int)(Wire.read()<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
	accelerometer_z = (int)(Wire.read()<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
	accAngleX = (atan(accelerometer_y/16384.0 / sqrt(pow(accelerometer_x/16384.0, 2) + pow(accelerometer_z/16384.0, 2))) * 180 / PI) - 0.58;
	if (i_ang<=size_p && data_save ==1)
	{
		p_ang[i_ang]=(double)(accAngleX+90);
		i_ang=i_ang+1;
	}
}

float readPosition(float duration) 
{                                                           

	float cm;
	//printf("OLADQ\n");
	cm = (duration+65535*(float)s_over)/(928);
	if (i_pos<=size_p && data_save ==1)
	{
		p_pos[i_pos]= (double)(duration+65535*(float)s_over)/(928);
		pc_pos[i_pos]=p_pos[i_pos];
		i_pos=i_pos+1;
	}
	FT = 0;
	return cm;                                          //Returns distance value.
}

double pri_vel=0.01;
double pos_ant = 0;

int PID_Compute()
{
	int distance_error;
	double atual_vel;
	double pos_atual;
	int corrigi=0;
	
	if (millis() > time+period)
  {
    time = millis();    
    distance =readPosition((T2-T1));  
    distance_error = Setpoint - distance;   
	atual_vel=(double)((double)(distance_error - distance_previous_error)/period);
	if (atual_vel >0.1 || atual_vel < -0.1)
	{
		atual_vel=pri_vel;
		pos_atual=pos_ant+period*atual_vel;
		corrigi=1;
		
	}
    
	if (i_vel<=size_p && data_save==1)
	{
		p_velo[i_vel]=(double)((double)(distance_error - distance_previous_error)/period);
		pc_velo[i_vel]= atual_vel;
		if (corrigi==1)
			pc_pos[i_pos-1]= pos_atual;
		i_vel=i_vel+1;
	}
	if (corrigi ==1)
	{
		distance_error = Setpoint - pos_atual;
	}

	PID_p = Kp * distance_error;
    //float dist_diference = distance_error - distance_previous_error;     

    PID_d = Kd*atual_vel;
	//dtostrf((double)((double)(distance_error - distance_previous_error)/period), 6, 7, str);
	//printf("%s\n",str);
	if(-1 < distance_error && distance_error < 1)
    {
      PID_i = PID_i + (Ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 180);
  
    if(PID_total < 30){PID_total = 30;}
    if(PID_total > 150) {PID_total = 150; } 
	distance_previous_error=distance_error;
	pri_vel = atual_vel;
	pos_ant = pos_atual;
	return PID_total;
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
				//Active_ADC();
				clear_buffer();
			}
			else if (strncmp(rx_buffer,"OFF",3) == 0)
			{
				printf("The ADC is OFF");
				//OFF_ADC();
			}
			else if (strncmp(rx_buffer,"GA",2) == 0)
			{
				frist_time=1;
				//ON_AG();
				GA_ON=1;
			}
			else if (strncmp(rx_buffer,"PAG",3) == 0)
			{
				PAG=1-PAG;
			}
			else if (strncmp(rx_buffer,"EQ",2) == 0)
			{
				Equi_ON=1-Equi_ON;
				printf("OKOK\n\r");
			}
			else if (strncmp(rx_buffer,"SERVOS",6) == 0)
			{
				myServo.write(ang);
			}
			else if (strncmp(rx_buffer,"SERVO+",6) == 0)
			{
				ang = ang+5;
				myServo.write(ang);
			}
			else if (strncmp(rx_buffer,"SERVO-",6) == 0)
			{
				ang = ang-5;
				myServo.write(ang);
				
			}
		}
		else if (!(PINB & (1 << PINB0)))
		{
				//printf("Ola");
				ang = ang-5;
				myServo.write(ang);
				_delay_ms(105);
		}
		else if (!(PINB & (1 << PINB1)))
		{
				ang = ang+5;
				myServo.write(ang);
				_delay_ms(105);
		}
		else if(!(PINB & (1 << PINB2)))
		{
				frist_time=1;
				//ON_AG();
				GA_ON=1;
				_delay_ms(10);
				Equi_ON=1;
				_delay_ms(100);
				ang_cent=ang;
				printf("Saving data, wait a bit\n");
				data_save=1;
		}
		else
		{
			clear_buffer();
		}
		
	}


/*ISR(TIMER2_COMPA_vect)
{
	if (pulse_60==6000)
	{
		PORTH |= 1<<PORTH6;
		pulse_60=0;
	}
	PORTH &= ~(1<<PORTH6);
	pulse_60=pulse_60+1;
}*/

ISR(TIMER4_CAPT_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	
	if (TCCR4B & (1 << ICES1))
  	{
	 		T1 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B &=~(1<<ICES4); //CHANGE CAPTURE ON FALLING EDGE
			// TCNT4=0;
			over_f=0;

    }
    else
	{    	s_over=over_f;
	 		T2 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B|=(1<<ICES4); //CHANGING CAPTURE ON RISING EDGE
			FT=1;
	 		TIFR4 &= ~(1 << ICF4);
    }

}

ISR(TIMER4_OVF_vect)
{
	if (TCCR4B & ~(1 << ICES1))
	{
		over_f = over_f +1;
	}
}

ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;

	rx_buffer[rx_pointer++] = (char) c;	
}


int main()
{
	int duration_1=0;
	int delay_me = 70;
	stdout=&mystdout;
	init_UART();
	set_timers();
	set_sonar();
	init_AG();
	DDRB &= ~(1<<DDB0 |1<<DDB1 | 1<<DDB2);
	DDRF = 0b11111111;
	DDRK = 0b00001111;
	
	printf("OLA\n");
	myServo.attach(servoPin);                                          //Attach Servo

	//Input = readPosition();                                            //Calls function readPosition() and sets the balls
																		//  position as the input to the PID algorithm
																		
	//myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
	//myPID.SetOutputLimits(-50,50);                                     //Set Output limits to -80 and 80 degrees.
	myServo.write(85);
	
	PORTH |= 1<<PORTH6;
	_delay_us(10);
	PORTH &= ~(1<<PORTH6);


	while(1)
	{
		Controle();
		//printf("FT antes do while %d\n",FT);
		//printf("T1 = %d\n",T1);
		//printf("T2 = %d\n",T2);
		
		Show_val(ang,delay_me);
		//_delay_ms(70);
		Setpoint = 26;
		if (FT)
		{
			readAngle();

			Show_val(ang,5);
			// printf("aX = ");
			// dtostrf(accelerometer_x, 3+3, 3, str);
			// printf(str);
			// printf(" | aY = ");
			// dtostrf(accelerometer_y, 3+3, 3, str);
			// printf(str);
			// printf(" | aZ = ");
			// dtostrf(accelerometer_z, 3+3, 3, str);
			// printf(str);
			// printf("\n\r");
			// printf("AngleX = ");
			// dtostrf(accAngleX, 3+3, 3, str);
			// printf(str);
			// printf("\n\r");
			//Input = readPosition((T2-T1)); 
	
			//dtostrf(accAngleX, 6, 3, str);
			//printf("T1 = %s\n",str);
			//printf("milles = %d\n",millis());
			//myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
			ServoOutput= PID_Compute();
			//printf("Output = %d\n",(int)ServoOutput);                                            // 150 degrees is my horizontal
			myServo.write(ServoOutput);
			TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE INTERRUPTS
       		TCNT4 = 0; //Clear the counter

			//printf("passou");

			ang=(int)((accAngleX+90)*10);
			//printf("\n%d\n\n",ang);
			Show_val(ang,5);

			count_show=count_show+1;

			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4 = 0; //Clear the counter

			

			FT=0;
		}
		//printf("t = %d\n",i_vel);                                      //Writes value of Output to servo 
		if (i_vel >=350 && i_pos >=350 && i_ang >=350)
			{
				printf("Pos velocidade pos_c velo_c ang\n");
				for(int i =0; i<=350-1;i++)
				{
					
					dtostrf(p_pos[i], 6, 3, str );
					printf("%s ",str);
					dtostrf(p_velo[i], 6, 7, str);
					printf("%s ",str);
					dtostrf(pc_pos[i], 6, 3, str );
					printf("%s ",str);
					dtostrf(pc_velo[i], 6, 7, str);
					printf("%s ",str);
					dtostrf(p_ang[i], 6, 1, str);
					printf("%s\n",str);
					

				}
				data_save=0;
				i_pos=0;
				i_vel=0;
				i_ang=0;
			}

	}

	return 0;
}

#endif

#ifdef __Main_test
/* define baud rate as BAUD before this (can be defined externaly)*/
#define BAUD  9600
#define F_OSC 16000000
#define UBRRVAL F_CPU/8/BAUD-1
#define RX_BUFFER_SIZE 32
#define RXB_MASK 0x1f

#define MPU_ADDR 0x68

#undef FDEV_SETUP_STREAM
#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }
char rx_buffer[RX_BUFFER_SIZE];

long time;
long period=70;
long PID_total;
long PID_p, PID_i, PID_d;
int distance_previous_error;

uint8_t rx_pointer;
uint8_t ind;
uint8_t received = 0;


const int servoPin = 10;                                               //Servo Pin

float Kp = 1.5;                                                    //Initial Proportional Gain
float Ki = 0.2;                                                      //Initial Integral Gain
float Kd = 50000;

double i = 0;


double Setpoint, Input, Output, ServoOutput;                                       

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
																	
//Servo myServo;                                                       //Initialize Servo.
const int echoPin = 7;
const int trigPin = 9;
unsigned long lastTime =0; 
unsigned long now = 0;
unsigned long timeChange =0;
int pulse_60=0;
char str[20];
int over_f=0;
int s_over=0;

int distance;
int servo_flag = 0;
int max_count_servo=0;
int count_servo=0;
int PR=1999;

int FT =0;
float T1,T2;

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


void clear_buffer()
{
	static uint8_t i;
	for (i = 0; i < RX_BUFFER_SIZE; i++)
		rx_buffer[i] = '\0';
	rx_pointer = 0;
}

void set_timers()
{


	// timer que mando um pulso para o servo
	/*TCCR1A = 0;// set entire TCCR2A register to 0
	TCCR1B = 0;// same for TCCR2B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR1A = 50000; // este parametro vai ser mudado ao longo da simulação 
	OCR1B = PR;
	// turn on CTC mode
	TCCR1A |= (1 << COM1A1) |(1 << COM1B1) |(1 << WGM11)| (1 << WGM10); 
	//TCCR1B = (1<< WGM12);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)| (1 << CS11);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);*/

	// timer responsavel em medir o tempo do sinal do echo
	TCCR4A = 0; //Disable compare interrupts
	TCCR4B = 0;
	TCNT4 = 0; //SETTING INTIAL TIMER VALUE

	TCCR4B|=(1<<ICES4); //SETTING FIRST CAPTURE ON RISING EDGE
	TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE
	TIMSK4|=(1<<TOIE4);
	TIFR4 &= ~(1 << ICF4); //Clear the flag
	TCCR4B|=(1<<CS40);
	TCCR4B &= ~(1<<CS42);
	TCCR4B &= ~(1<<CS41);//STARTING TIMER WITH NO PRESCALER


	TCCR0A = 0; //Disable compare interrupts
	TCCR0B = 0;
	TCNT0 = 0; //SETTING INTIAL TIMER VALUE

	TCCR0B|=(1<<ICES1); //SETTING FIRST CAPTURE ON RISING EDGE
	TIMSK0|=(1<<TOIE1);
	TIFR0 &= ~(1 << ICF1); //Clear the flag
	TCCR0B|=(1<<CS10);
	TCCR0B |= (1<<CS11);
	TCCR0B &= ~(1<<CS12);//STARTING TIMER WITH NO PRESCALER
	sei();
}

void set_sonar()
{
	DDRH &=  ~(1<<DDH4);  //input (echo)
	DDRH |=  1<<DDH6;    // Output (triger)
}

void set_servo()
{
	DDRB |=  1<<DDB6;
	DDRB |=  1<<DDB4;
}
		
float readPosition(float duration) 
{                                                           

	float cm;
	printf("OLADQ\n");
	cm = (duration+65535*(float)s_over)/(928);
	//if(cm > 70)     // 35 cm is the maximum position for the ball
	//{cm=70;}
	/*dtostrf(T1, 6, 3, str);
	p1rintf("T1 = %s\n",str);
	dtostrf(T2, 6, 3, str);
	printf("T2 = %s\n",str);
	
	dtostrf(duration, 8, 3, str);
	printf("duration = %s\n",str);
	printf("overf = %d\n", s_over);*/
	dtostrf(cm, 6, 3, str);
	printf("cm = %s\n",str);
	FT = 0;
	return cm;                                          //Returns distance value.
}


int PID_Compute()
{
	int distance_error;
	
	if (millis() > time+period)
  {
    time = millis();    
    distance =readPosition((T2-T1));  
    distance_error = Setpoint - distance;   
    PID_p = Kp * distance_error;
    //float dist_diference = distance_error - distance_previous_error;     
    PID_d = Kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (Ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 180);
  
    if(PID_total < 30){PID_total = 30;}
    if(PID_total > 150) {PID_total = 150; } 
	distance_previous_error=distance_error;
	return PID_total;
	}
}

void set_ang_valeu(int ang)
{
	
	PR=1087+20.6*ang;
	OCR1B = PR;
	//servo_flag=0;
	
}



ISR(TIMER1_COMPA_vect)
{
	/*TIFR1 &= ~(1 << ICF1);
	OCR1B=(unsigned)PR+1000;
	*/
	servo_flag=0;
	/*if (servo_flag==0)
	{
		PORTB &= ~(1<<PORTB4);
		servo_flag = 1;
		//OCR1A =(int)(4999-PR);
		//TCNT1=0;
	}
	else
	{
		servo_flag=0;
	}*/
}

ISR(TIMER4_CAPT_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	
	if (TCCR4B & (1 << ICES1))
  	{
	 		T1 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B &=~(1<<ICES4); //CHANGE CAPTURE ON FALLING EDGE
			// TCNT4=0;
			over_f=0;

    }
    else
	{    	s_over=over_f;
	 		T2 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B|=(1<<ICES4); //CHANGING CAPTURE ON RISING EDGE
			FT=1;
	 		TIFR4 &= ~(1 << ICF4);
    }

}

ISR(TIMER4_OVF_vect)
{
	if (TCCR4B & ~(1 << ICES1))
	{
		over_f = over_f +1;
	}
}

ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;

	rx_buffer[rx_pointer++] = (char) c;	
}


int main()
{
	int duration_1=0;
	stdout=&mystdout;
	init_UART();
	set_timers();
	set_sonar();
	
	printf("OLA\n");
	set_servo();
	int ang =0;
	int count =0;
	//set_ang_valeu(90);

	while(1)
	{
		if (ang >= 180)
			ang=0;
		//_delay_ms(20);
		if (servo_flag==0)
		{
			servo_flag=1;

			//ang=ang+1;
			//printf("OCR1B = %d\n", (int)OCR1B);
			//printf("ang = %d\n", (int)ang);
			//printf("PR = %d\n", (int)PR);
			//set_ang_valeu(90);
			
			printf("time = %d\n", (int)millis());
			
		}
		if (count ==5)  // fazer o mesmo para a PWM e ver se melhora 
		{
			ang=ang+10;
			count=0;
		}
		else
		{
			PORTB |= 1<<PORTB6;
			i=(12*ang+544);
			delayMicroseconds((int)i);
			PORTB &= ~(1<<PORTB6); 
			delay(20);
			
		}
		count=count +1;
			
		printf("i = %d\n",(int)i);
		printf("ang = %d\n",ang);
		
		

	}
	return 0;
}
#endif