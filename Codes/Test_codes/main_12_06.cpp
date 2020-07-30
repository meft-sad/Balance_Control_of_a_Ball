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
int distance;


const int servoPin = 10;                                               //Servo Pin

float Kp = 1.5;                                                    //Initial Proportional Gain
float Ki = 0.5;                                                      //Initial Integral Gain
float Kd = 50000;
;                                                    //Intitial Derivative Gain
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
    	distance = readPosition((T2-T1));  
		// Calculation of the error
    	distance_error = Setpoint - distance; 
		// Calculation of the P 
    	PID_p = Kp * distance_error; 
    	// Calculation of the D  
   		PID_d = Kd*(double)((double)distance_error - distance_previous_error)/period); 
		// Calulation of the I  
    	if(-1 < distance_error && distance_error < 1)
    	{
      		PID_i = PID_i + (Ki * distance_error);
   		}
    	else // if the ball is in setpoint-1 and setpoint+1 range the integral part stops
    	{
      		PID_i = 0;
    	}
    	PID_total = PID_p + PID_i + PID_d; 
		// Mapping the value of the PID to a range of angles
    	PID_total = map(PID_total, -100, 100, 0, 180);
		// Limiting the angles because in the setup the servo can not go to lower angles them 30º,
    	if(PID_total < 30)
		{
			PID_total = 30;
		}
		// same as higher them 150º otherwise it will get stuck
    	if(PID_total > 150) 
		{
			PID_total = 150; 
		} 
		distance_previous_error=distance_error;
		return PID_total;
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
	stdout=&mystdout;
	init_UART();
	set_timers();
	set_sonar();
	
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
		//printf("FT antes do while %d\n",FT);
		//printf("T1 = %d\n",T1);
		//printf("T2 = %d\n",T2);
		_delay_ms(70);
		
		Setpoint = 28;
		if (FT)
		{
			
			//Input = readPosition((T2-T1)); 
	
			//dtostrf(T1, 6, 3, str);
			//printf("T1 = %s\n",str);
			//printf("milles = %d\n",millis());
			//myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
			ServoOutput= PID_Compute();
			printf("Output = %d\n",(int)ServoOutput);                                            // 150 degrees is my horizontal
			myServo.write(ServoOutput);
			TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE INTERRUPTS
       		TCNT4 = 0; //Clear the counter
			
			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4 = 0; //Clear the counter

			FT=0;
		}                                      //Writes value of Output to servo 
	}
	return 0;
}
