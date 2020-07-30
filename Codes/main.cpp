#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include<Servo.h>




/* define baud rate as BAUD before this (can be defined externaly)*/
#define BAUD  9600
#define F_OSC 16000000
#define UBRRVAL F_CPU/8/BAUD-1
#define RX_BUFFER_SIZE 32
#define RXB_MASK 0x1f

#define MPU_ADDR 0x68

// UART variables & defines

#undef FDEV_SETUP_STREAM
#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }
char rx_buffer[RX_BUFFER_SIZE];

uint8_t rx_pointer;
uint8_t ind;
uint8_t received = 0;
/*________________________________________*/

// Variables for printing the data
#define size_p 350

int data_save=0;
double p_velo[size_p];
double p_pos[size_p];
double p_ang[size_p];
double pc_velo[size_p]; // Save the value of the correction made to the velocity 
double pc_pos[size_p];	// Save the value of the correction made to the position
int num_saves=350;
int i_pos=0;
int i_vel=0;
int i_ang=0;

/*________________________________________*/

// Variables for the PID 
long time;
long period=70;
long PID_total;
long PID_p, PID_i, PID_d;
double distance_previous_error;

double Setpoint, Input, Output, ServoOutput; 

float Kp = 1.5;			//Initial Proportional Gain [1/cm]
float Ki = 0.05;		//Initial Integral Gain  [1/cm]
float Kd = 65;			//Intitial Derivative Gain [ms/cm]

double pri_vel=0.01; 	// For the corretion of the velocity 
double pri_pos = 0;		// For the corretion of the poaition 

/*________________________________________*/


// Varies for the servo SG90

Servo myServo;//Initialize Servo.
const int servoPin = 10;	//Servo Pin

/*________________________________________*/

// Varies for the Sonar

int over_f=0;
int FT =0;
float T1,T2;
double distance;
int s_over=0;

/*________________________________________*/

// Varies for the MPU-6500

float accelerometer_x;
float accAngleX;
float accelerometer_y;		
float accelerometer_z;
float temperature;
float gyro_x;
float gyro_y;
float gyro_z;

/*________________________________________*/

// For the four digit seven segment display 

int ang = 90;
int dezenas = 0;
int centenas = 0;
int unidades = 0;
int centezimas = 0;
int DONT = 0;

/*________________________________________*/


char str[20];





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
	do 
	{
		uputc(*c++,stream);
	}
	 while (*c != 0);
}

// converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
char* convert_to_str(int16_t i) 
{ 
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
	// Timer responsable to measur the echo signal
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

	// This timer is importante to make the fuction milles() work.
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



// Set the pins for the sonar
void set_sonar()
{
	DDRH &=  ~(1<<DDH4);  //input (echo)
	DDRH |=  1<<DDH6;    // Output (triger)
}

// Iniciate the MPU-6500 
void init_AG(void)
{
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
	Wire.write(0x6B); // PWR_MGMT_1 register
	
	Wire.write(0); // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
}

// The displays of the numbers 
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

// Cycle of rotation of the numbers
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
	}
	
}

// Show values of the angle 
void Show_val(int atual_ang,int de)
{
	if (DONT == 0)
	{
		// separate the number by units 	
    	dezenas = (atual_ang%1000)/100;
		centenas = (atual_ang-dezenas*100)/1000;
		unidades = (atual_ang-centenas*1000-dezenas*100)/10;
		centezimas=atual_ang-centenas*1000-dezenas*100-unidades*10;
	}
	code_numeros(centenas,dezenas,unidades,centezimas,de);
}


void readAngle() 
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR, 7*2, true);
	// "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
	accelerometer_x =  (int)(Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)  
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
	//Calculation of the distance by using the sonar 
	cm = (duration+65535*(float)s_over)/(928);
	if (i_pos<=size_p && data_save ==1)
	{
		p_pos[i_pos]= cm;
		pc_pos[i_pos]=p_pos[i_pos];
		i_pos=i_pos+1;
	}
	FT = 0;
	return cm;                                          //Returns distance value.
}



int PID_Compute()
{
	double distance_error;
	double atual_vel;
	double pos_atual;
	int corrigi=0;
	
	if (millis() > time+period)
  	{
    	time = millis();    
		// Calculation of the distance read by the sonar 
    	distance =readPosition((T2-T1));  
		// Calculation of the error
    	distance_error = Setpoint - distance;   
		atual_vel=(double)((double)(distance_error - distance_previous_error)/period);
		// Correction if the velocity is bigger them 0.14 cm/ms is an error of position so 
		//the velocity and position read will be ignored and it will assume that the ball is at 
		//the same velocity so with that we get a some correction
		if (atual_vel >0.14|| atual_vel < -0.14)
		{
			atual_vel=pri_vel;
			pos_atual=pri_pos+period*atual_vel;
			corrigi=1;
		}
		// Save the data in vectors and at the end print them on the to analise the data
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
		// Calculation of the P 
		PID_p = Kp * distance_error;
		// Calculation of the D 
    	PID_d = Kd*atual_vel;
		// Calulation of the I
		if(-1 > distance_error || distance_error > 1)
   		{
      		PID_i = PID_i + (Ki * distance_error);
    	}
    	else // if the ball is in setpoint-1 and setpoint+1 range the integral part stops
    	{
     		PID_i = 0;
    	}
    	PID_total = PID_p + PID_i + PID_d;  
		// Mapping the value of the PID to a range of angles
    	PID_total = map(PID_total, -150, 150, 0, 180);
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
		pri_vel = atual_vel;
		pri_pos = pos_atual;	
	}
	return PID_total;
}


// Control station: Wrinte on the terminal the words in "" to do what is explaned in the manual
void Controle(void)
	{
		if (rx_buffer[rx_pointer-1]=='\r' && rx_pointer>0)
		{
			rx_buffer[rx_pointer-1]='\0';
			if (strncmp(rx_buffer,"Print_T",7) == 0)
			{
				data_save=1;
				num_saves = 2;
			}
			else if (strncmp(rx_buffer,"Print_All",9) == 0)
			{
				printf("Saving data, wait a bit\n");
				data_save=1;
				num_saves = 350;
			}
		}
		// Button connected to the port PB0 of the Arduino MEGA
		else if (!(PINB & (1 << PINB0)))
		{
				data_save=1;
				num_saves = 2;
		}
		// Button connected to the port PB1 of the Arduino MEGA
		else if (!(PINB & (1 << PINB1)))
		{
				printf("Saving data, wait a bit\n");
				data_save=1;
				num_saves = 350;
		
		}
		else if(!(PINB & (1 << PINB2)))
		{

		}		
		else
		{
			clear_buffer();
		}
		
	}


// Timer4 interrupting to measure the time of the pulse sended by the sonar Trigger
ISR(TIMER4_CAPT_vect)
{
	
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

//Takes in count the overflow of the timer 4 important when measure the distance
ISR(TIMER4_OVF_vect)
{
	if (TCCR4B & ~(1 << ICES1))
	{
		over_f = over_f +1;
	}
}

//Uart interrupt
ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;

	// Writes in information send to a vector of chars
	rx_buffer[rx_pointer++] = (char) c;	
}


int main()
{
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

	myServo.write(85);
	
	PORTH |= 1<<PORTH6;
	_delay_us(10);
	PORTH &= ~(1<<PORTH6);


	while(1)
	{
		Controle();

		Show_val(ang,delay_me);

		Setpoint = 27;

		if (FT)
		{
			readAngle();

			Show_val(ang,5);

			ServoOutput= PID_Compute();
                                        
			myServo.write(ServoOutput);

			TIMSK4|=(1<<ICIE4); 
       		TCNT4 = 0; 



			ang=(int)((accAngleX+90)*10);
			Show_val(ang,5);

			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4 = 0; //Clear the counter

			

			FT=0;
		}

		if (i_vel >=num_saves && i_pos >=num_saves && i_ang >=num_saves)
			{
				printf("Pos velocidade pos_c velo_c ang\n");
				for(int i =0; i<=num_saves-1;i++)
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