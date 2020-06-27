#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define __Main_8



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
 
//    servo.write(0);      // Turn SG90 servo Left to 45 degrees
//    delay(1000);          // Wait 1 second
//    servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
//    delay(1000);          // Wait 1 second
//    servo.write(180);     // Turn SG90 servo Right to 135 degrees
//    delay(1000);          // Wait 1 second
//    servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
//    delay(1000);
 
//end control the servo's direction and the position of the motor
 
 
//control the servo's speed  
 
//if you change the delay value (from example change 50 to 10), the speed of the servo changes
  for(servoAngle = 0; servoAngle <60; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
  {                                  
    servo.write(servoAngle);              
    delay(50);                  
  }
  servo.detach();
  delay(10000);
  servo.attach(servoPin);
 
//   for(servoAngle = 180; servoAngle > 0; servoAngle--)  //now move back the micro servo from 0 degrees to 180 degrees
//   {                                
//     servo.write(servoAngle);          
//     delay(10);      
//   }
  //end control the servo's speed  
}
#endif

#ifdef __Main_4
/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS: https://www.youtube.com/channel/UCjiVhIvGmRZixSzupD0sS9Q
 *  Tutorial: http://electronoobs.com/eng_arduino_tut100.php
 *  Code: http://electronoobs.com/eng_arduino_tut100_code1.php
 *  Scheamtic: http://electronoobs.com/eng_arduino_tut100_sch1.php 
 *  3D parts: http://electronoobs.com/eng_arduino_tut100_stl1.php   
 */
#include <Wire.h>
#include <Servo.h>



///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////


////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=0.2; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 21;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////


float get_dist(int n)
{
  long sum=0;
  for(int i=0;i<n;i++)
  {
    sum=sum+analogRead(Analog_in);
  }  
  float adc=sum/n;
  //float volts = analogRead(adc)*0.0048828125;  // value from sensor * (5/1024)
  //float volts = sum*0.003222656;  // value from sensor * (3.3/1024) EXTERNAL analog refference

  float distance_cm = 17569.7 * pow(adc, -1.2062);
  //float distance_cm = 13*pow(volts, -1); 
  return(distance_cm);
}


int main()
{
  //analogReference(EXTERNAL);
  Serial.begin(9600);  
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo.write(90); //Put the servco at angle 125, so the balance is in the middle
  pinMode(Analog_in,INPUT);  
  time = millis();



  if (millis() > time+period)
  {
    time = millis();    
    distance = get_dist(100);   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 160) {PID_total = 160; } 
  
    myservo.write(PID_total+30);  
    distance_previous_error = distance_error;
  }
}
#endif

#ifdef __Main_5 

//#include<Servo.h>
#include<PID_v1.h>

const int servoPin = 10;                                               //Servo Pin
 
float Kp = 5.5;                                                    //Initial Proportional Gain
float Ki = 0.1;                                                      //Initial Integral Gain
float Kd = 0.5;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;      
float over_f=0;                                 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                     
//Servo myServo;                                                       //Initialize Servo.
const int echoPin = 6;
const int trigPin = 7;
int toggle1=0;
long duration=0;
unsigned int FT=0;
float T1,T2;

float readPosition() 
{                                                          
	float cm;
	char str[20];

  	cm = (T2+65535*over_f)/10000;
 
  	//if(cm > 12)     // 35 cm is the maximum position for the ball
  	//{
	//	cm=12;
   //	}
	
	dtostrf(cm, 6, 3, str);

  	Serial.print(str);
	  Serial.print("\n");
	over_f=0;  
	
  	return cm;                                          //Returns distance value.
}

void Set_timer()
{
	TCCR4B = 0; //Disable compare interrupts
  	//TCCR4B = 0;
  	TCNT4=0; //inicial timer valeu to 0
	//OCR4B = 1024;

  	TCCR4B|=(1<<ICES1); //rising edge triger
  	TIFR4 &= ~(1 << ICF4); //Clear the flag
  	TCCR4B|=(1<<CS10);
  	TCCR4B &= ~(1<<CS12);
  	TCCR4B &= ~(1<<CS11);//STARTING TIMER WITH NO PRESCALER

	TIMSK4|=(1<<ICIE4); //enable input capture
	TIMSK4|=(1<<TOIE4); 

	/*TCCR3A = 0;// set entire TCCR2A register to 0
	TCCR3B = 0;// same for TCCR2B
	TCNT3  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR3A = 17499;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// turn on CTC mode
	TCCR3A |= (1 << WGM21);
	// Set CS21 bit for 8 prescaler
	TCCR3B |= (1 << CS01) | (1 << CS00);    
	// enable timer compare interrupt
	TIMSK3 |= (1 << OCIE2A);*/

  	sei(); //ENABLING GLOBAL INTERRUPTS
}

/*ISR(TIMER3_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
 	TIFR3 &= ~(1 << ICF3);
	FT=0;
	PORTH |= 1<<PORTH4;
	_delay_us(10);
	PORTH &= ~(1<<PORTH4);
	TCNT1=0;
	

}*/



int main() 
{

  	Serial.begin(9600);      //Begin Serial
	Set_timer();
   	DDRH &=  ~(1<<DDH4);
	DDRH |=  1<<DDH6;

	PORTH |= 1<<PORTH6;
	_delay_us(10);
	PORTH &= ~(1<<PORTH6);
	TCNT4=0;
	//myServo.attach(servoPin);                                          //Attach Servo
							                                           //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  	//myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
 	//myPID.SetOutputLimits(-60,60);                                     //Set Output limits to -80 and 80 degrees.
	
	//Serial.println(FT);
	while (1)
	{
		
		
		if (FT == 2)
		{
			//Serial.println(FT);
			Serial.println(over_f);
			Serial.print("T1 = ");
			Serial.println(T1);

			Serial.print("T2 = ");
			Serial.println(T2);
			Input = readPosition();  
			
			_delay_ms(200);
			FT=0;
			TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE INTERRUPTS
			TCNT4 = 0; //Clear the counter
			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4=0;
			
		}
		

		
  		Setpoint = 8;
  		                                         
 
 	 	//myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  		//ServoOutput=100+Output;                                            // 150 degrees is my horizontal
  		//Serial.println(Output);
  		//myServo.write(ServoOutput);                                        //Writes value of Output to servo 	
	}
	return 0;
}

ISR(TIMER4_CAPT_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	
	if (FT == 0)
		{
	 		T1 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B &=~(1<<ICES4); //CHANGE CAPTURE ON FALLING EDGE
			 over_f=0;
			 TCNT4=0;
			 

    	}
    else if (FT == 1)
		{    
	 		T2 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B|=(1<<ICES4); //CHANGING CAPTURE ON RISING EDGE
			TIFR4 &= ~(1 << ICF4);
	 		
    	}
     FT = FT + 1;

}
ISR(TIMER4_OVF_vect)
{
	if (FT==1)
	{
		over_f = over_f +1;
		FT=1;
	}
	else if (FT==2)
	FT=2;
	else
	FT=0;
}



#endif

#ifdef __Main_6
#include<Servo.h>
#include<PID_v1.h>

const int servoPin = 10;                                               //Servo Pin
 
float Kp = 5.5;                                                    //Initial Proportional Gain
float Ki = 0.1;                                                      //Initial Integral Gain
float Kd = 0.2;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                     
Servo myServo;                                                       //Initialize Servo.
const int echoPin = 7;
const int trigPin = 9;
unsigned long lastTime =0; 
		unsigned long now = 0;
   		unsigned long timeChange =0;
		  
float readPosition() {
  delay(30);                                                           
 
long duration, cm;
unsigned long now = millis();
 digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
 
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  cm = duration/(29*2);
 
  if(cm > 70)     // 35 cm is the maximum position for the ball
  {cm=70;}
 
  Serial.println(cm);
 
  return cm;                                          //Returns distance value.
}

void setup() {

  Serial.begin(9600);      //Begin Serial
   pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
  myPID.SetOutputLimits(-50,50);                                     //Set Output limits to -80 and 80 degrees.
}

void loop()
{
 
  Setpoint = 25;
  Input = readPosition();   

  now = millis();
	timeChange = (now - lastTime);
	Serial.print("Now = ");
	Serial.println(now);
	Serial.print("\n");
	Serial.print("TC = ");
	Serial.println(timeChange);
	Serial.print("\n");
lastTime=now;
					                                         
 
  myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  ServoOutput=90+Output;
  Serial.print("Output = ");                                            // 150 degrees is my horizontal
  Serial.println(ServoOutput);
  myServo.write(ServoOutput);                                        //Writes value of Output to servo 
}


#endif
#ifdef __Main_7
//#include<Servo.h>
#include<PID_v1.h>

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

#undef FDEV_SETUP_STREAM
#define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }
const int servoPin = 10;                                               //Servo Pin
 
float Kp = 5.5;                                                    //Initial Proportional Gain
float Ki = 0.1;                                                      //Initial Integral Gain
float Kd = 0.5;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;      
float over_f=0;                                 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                     
//Servo myServo;                                                       //Initialize Servo.
const int echoPin = 6;
const int trigPin = 7;
int toggle1=0;
long duration=0;
unsigned int FT=0;
float T1=0,T2=0;
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



float readPosition() 
{                                                          
	float cm;
	

  	cm = ((T2+65535*over_f)-T1)/10000;
 
  	//if(cm > 12)     // 35 cm is the maximum position for the ball
  	//{
	//	cm=12;
   //	}
	
	dtostrf(cm, 8, 3, str);

  	printf("%s\n",str);

	dtostrf((T2+65535*over_f), 8, 3, str);

  	printf("T2+over = %s\n",str);

	dtostrf(((T2+65535*over_f)-T1), 8, 3, str);

  	printf("T2+over-T1= %s\n",str);


	dtostrf(((T2+65535*over_f)-T1)/10000, 8, 3, str);

  	printf("T2+over-T1/1000 = %s\n",str);

	
	over_f=0;  
	
  	return cm;                                          //Returns distance value.
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
	//uart receive isr
	ISR(USART0_RX_vect)
	{
		uint8_t c = UDR0;

		rx_buffer[rx_pointer++] = (char) c;	
	}

void Set_timer()
{
	TCCR4B = 0; //Disable compare interrupts
  	//TCCR4B = 0;
  	TCNT4=0; //inicial timer valeu to 0
	//OCR4B = 1024;

  	TCCR4B|=(1<<ICES1); //rising edge triger
  	TIFR4 &= ~(1 << ICF4); //Clear the flag
  	TCCR4B|=(1<<CS10);
  	TCCR4B &= ~(1<<CS12);
  	TCCR4B &= ~(1<<CS11);//STARTING TIMER WITH NO PRESCALER

	TIMSK4|=(1<<ICIE4); //enable input capture
	TIMSK4|=(1<<TOIE4); 

	/*TCCR3A = 0;// set entire TCCR2A register to 0
	TCCR3B = 0;// same for TCCR2B
	TCNT3  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR3A = 17499;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// turn on CTC mode
	TCCR3A |= (1 << WGM21);
	// Set CS21 bit for 8 prescaler
	TCCR3B |= (1 << CS01) | (1 << CS00);    
	// enable timer compare interrupt
	TIMSK3 |= (1 << OCIE2A);*/

  	sei(); //ENABLING GLOBAL INTERRUPTS
}

/*ISR(TIMER3_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
 	TIFR3 &= ~(1 << ICF3);
	FT=0;
	PORTH |= 1<<PORTH4;
	_delay_us(10);
	PORTH &= ~(1<<PORTH4);
	TCNT1=0;
	

}*/



int main() 
{
	stdout=&mystdout;
  	init_UART();
	Set_timer();
   	DDRH &=  ~(1<<DDH4);
	DDRH |=  1<<DDH6;

	PORTH |= 1<<PORTH6;
	_delay_us(10);
	PORTH &= ~(1<<PORTH6);
	TCNT4=0;
	//myServo.attach(servoPin);                                          //Attach Servo
							                                           //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  	//myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
 	//myPID.SetOutputLimits(-60,60);                                     //Set Output limits to -80 and 80 degrees.
	
	//printf(""%lf\n",FT);
	while (1)
	{		
		if (FT == 2)
		{
			//printf(""%lf\n",FT);
			dtostrf(over_f, 1, 0, str);
			printf("%s\n",str);
			printf("T1 = ");
			dtostrf(T1, 6, 0, str);
			printf("%s\n",str);

			printf("T2 = ");
			dtostrf(T2, 6, 0, str);
			printf("%s\n",str);
			Input = readPosition();  
			
			_delay_ms(60);
			FT=0;
			TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE INTERRUPTS
			TCNT4 = 0; //Clear the counter
			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4=0;
			
		}

		TIMSK4|=(1<<ICIE4); //ENABLING INPUT CAPTURE INTERRUPTS
			TCNT4 = 0; //Clear the counter
			PORTH |= 1<<PORTH6;
			_delay_us(10);
			PORTH &= ~(1<<PORTH6);
			TCNT4=0;
			_delay_ms(60);
			

		
  		Setpoint = 8;
  		                                         
 
 	 	//myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  		//ServoOutput=100+Output;                                            // 150 degrees is my horizontal
  		//Serial.println(Output);
  		//myServo.write(ServoOutput);                                        //Writes value of Output to servo 	
	}
	return 0;
}

ISR(TIMER4_CAPT_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	TIFR4 &= ~(1 << ICF4);
	if (FT == 0)
		{
	 		T1 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B &=~(1<<ICES4); //CHANGE CAPTURE ON FALLING EDGE
			 over_f=0;
			// TCNT4=0;

    	}
    else if (FT == 1)
		{    
	 		T2 = ICR4; //SAVING CAPTURED TIMESTAMP
	 		TCCR4B|=(1<<ICES4); //CHANGING CAPTURE ON RISING EDGE
			
	 		
    	}

     FT = FT + 1;

}
ISR(TIMER4_OVF_vect)
{
	if (FT==1)
	{
		over_f = over_f +1;
		FT=1;
	}
	else if (FT==2)
	FT=2;
	else
	{
		FT=0;
		//over_f=0;
	}
	
}



#endif

#ifdef __Main_8
#include<Servo.h>
#include<PID_v1.h>

const int servoPin = 10;                                               //Servo Pin
 
float Kp = 10;                                                    //Initial Proportional Gain
float Ki = 2;                                                      //Initial Integral Gain
float Kd = 11;                                                    //Intitial Derivative Gain1
double Setpoint, Input, Output, ServoOutput;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.
                                                                     
Servo myServo;                                                       //Initialize Servo.
const int echoPin = 7;
const int trigPin = 9;
unsigned long lastTime =0; 
unsigned long now = 0;
unsigned long timeChange =0;

float accelerometer_x;
float accAngleX;
float accelerometer_y;		
float accelerometer_z;
float temperature;
float gyro_x;
float gyro_y;
float gyro_z;

#define MPU_ADDR 0x68

	

void init_AG(void)
{
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
	Wire.write(0x6B); // PWR_MGMT_1 register
	
	Wire.write(0); // set to zero (wakes up the MPU-6050)
	printf("Esta ok!!\n\r");
	printf("Esta ok!!!\n\r");
}

		  
float readPosition() {
  delay(30);                                                           
 
long duration, cm;
unsigned long now = millis();
 digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
 
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  cm = duration/(29*2);
 
  if(cm > 70)     // 35 cm is the maximum position for the ball
  {cm=70;}
 
  Serial.println(cm);
 
  return cm;                                          //Returns distance value.
}
float posicao()
{
	float mean=0;
	for (int i=0; i<=20; i++)
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
		mean +=accAngleX;
	}
	
	return mean/20;
}

void setup() {

  Serial.begin(9600);      //Begin Serial
   pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.attach(servoPin);  
  init_AG();                                        //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC
  myPID.SetOutputLimits(-25,25);                                     //Set Output limits to -80 and 80 degrees.
}

void loop()
{
 
  Setpoint = 0;
  //Input = readPosition();   
  Input = posicao();
  Serial.print("Input = ");
	Serial.println(Input);
	Serial.print("\n");

  now = millis();
	timeChange = (now - lastTime);
	Serial.print("Now = ");
	Serial.println(now);
	Serial.print("\n");
	Serial.print("TC = ");
	Serial.println(timeChange);
	Serial.print("\n");
	lastTime=now;
					                                         
	if (Input >3 || Input < -3)
	{
		 myServo.attach(servoPin);
		myPID.Compute();                                                   //computes Output in range of -80 to 80 degrees
  		ServoOutput=90+Output;
  		Serial.print("Output = ");                                            // 150 degrees is my horizontal
  		Serial.println(ServoOutput);
 	 	myServo.write(ServoOutput); 
	}
	else
	{
		myServo.detach();
	} 
}


#endif