// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>
// #include <stdio.h>

// #define RX_BUFFER_SIZE 32
// char rx_buffer[RX_BUFFER_SIZE];

// /* define baud rate as BAUD before this (can be defined externaly)*/
// #define BAUD 9600
// #define UBRRVAL F_CPU/8/BAUD-1

// static int uputc(char,FILE*);

// // #undef FDEV_SETUP_STREAM
// // #define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }

// static FILE mystdout = FDEV_SETUP_STREAM(uputc, NULL, _FDEV_SETUP_WRITE);


// static int uputc(char c,FILE *stream)
// {
// 	if (c == '\n')
// 		uputc('\r',stream);
// 	loop_until_bit_is_set(UCSR0A, UDRE0);	/* wait until we can send a new byte */
// 	UDR0 = (uint8_t) c;
// 	return 0;
// }




// ISR(USART0_RX_vect)
// {
// 	uint8_t c = UDR0;
// 	putchar(c);
// }

// int main(void)
// {
// 	stdout=&mystdout;

// 	/* pin config */
// 	DDRB = (1 << DDB5);

// 	/* uart config */
// 	UCSR0A = (1 << U2X0); /* this sets U2X0 to 1 */
// 	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
// 	UCSR0C = (3 << UCSZ00); /* this sets UCSZ00 to 3 */

// 	/* baudrate setings (variable set by macros) */
// 	UBRR0H = (UBRRVAL) >> 8;
// 	UBRR0L = UBRRVAL;

// 	/* ADC cfg */
// 	sei();							  /* enable interrupts */
// 	puts("hi!\n\r");

// 	for (;;) {
// 		;/*nothing here, all is done on the uart interrupt*/
// 	}
// }


/*____________________________Outro programa ____________________*/

// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// #define NCOMS 1

// #define BAUD 9600

// #define UBRRVAL F_CPU/8/BAUD-1

// #define TMR_F 5UL
// #define PS_VAL 1024UL
// #define OSC1A_VAL F_CPU/PS_VAL/TMR_F

// #define RX_BUFFER_SIZE 32
// #define RXB_MASK 0x1f

// char rx_buffer[RX_BUFFER_SIZE];

// uint8_t rx_pointer;
// uint8_t ind;
// uint8_t received = 0;


// void uputs(const char *c,FILE *stream);
// void up_8bits(uint8_t num,FILE *stream);

// static int uputc(char,FILE*);

// // #undef FDEV_SETUP_STREAM
// // #define FDEV_SETUP_STREAM(p, g, f) { 0, 0, f, 0, 0, p, g, 0 }

// static FILE mystdout = FDEV_SETUP_STREAM(uputc, NULL, _FDEV_SETUP_WRITE);

// void clear_buffer()
// {
// 	static uint8_t i;
// 	for (i = 0; i < RX_BUFFER_SIZE; i++)
// 		rx_buffer[i] = 0;
// 	rx_pointer = 0;
// }

// static int uputc(char c,FILE *stream)
// {
// 	if (c == '\n')
// 		uputc('\r',stream);
// 	loop_until_bit_is_set(UCSR0A, UDRE0);	/* wait until we can send a new byte */
// 	UDR0 = (uint8_t) c;
// 	return 0;
// }

// void uputs(const char *c,FILE *stream)
// {
// 	do {
// 		uputc(*c++,stream);
// 	} while (*c != 0);
// }

// void up_8bits(uint8_t num,FILE *stream) // UART print uint8 as binary string
// {
// 	uint8_t i = 8;
// 	uputs("0xb",stream);
// 	do {
// 		uputc((num & 0x80) ? '1' : '0',stream);
// 		num = (uint8_t) (num << 1);
// 	} while (--i > 0);
// 	uputc('\n',stream);
// }

// /* timer interrupt */
// ISR(TIMER1_COMPA_vect)
// {
// 	PORTB ^= _BV(PB5);			  /* toggle led on pin 13 (PORTB5) */
// 	// up_8bits(ind);
// 	ind++;
// }

// //uart receive isr
// ISR(USART0_RX_vect)
// {
// 	uint8_t c = UDR0;

// 	rx_buffer[rx_pointer++] = (char) c;	

// 	if (c == 'P')
// 		received = 1;
// }

// int main(void)
// {
// 	stdout=&mystdout;
// 	uint8_t i;
// 	/* pin config */
// 	DDRB = (1 << DDB1);

// 	/* uart config */
// 	UCSR0A = (1 << U2X0); /* this sets U2X0 to 1 */
// 	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
// 	UCSR0C = (3 << UCSZ00); /* this sets UCSZ00 to 3 */

// 	/* baudrate setings (variable set by macros) */
// 	UBRR0H = (UBRRVAL) >> 8;
// 	UBRR0L = UBRRVAL;

// 	/* timer cfg */
// 	TCCR1A = (2 << COM1A0)| (2 << WGM10) ;						  /* CTC mode */
// 	TCCR1B = (3 << WGM12) | (1 << CS10);	/* internal /1 prescaler */
// 	ICR1 = OSC1A_VAL;
// 	OCR1A = OSC1A_VAL/2;
// 	TIMSK1 = (0 << OCIE1A);

// 	/* ADC cfg */
// 	sei();							  /* enable interrupts */
// 	uputs("hi!\n",stdout);

// 	clear_buffer();

// 	while (1) {		
// 		if (received) {
// 			received = 0;
// 			uputs("Echo: ",stdout);
// 			uputs(rx_buffer,stdout);
// 			uputs("\n",stdout);
// 			clear_buffer();
// 		}
// 		else
// 		{
// 			// putchar(received);
// 			// putchar('\n\r');
// 		}
		
// 	}
// }


/*____________________________Outro programa ____________________*/
/*  https://www.youtube.com/watch?v=3_omxGIL0kw */

// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>
// #include <stdio.h>

// #define BUAD 9600
// #define BRC ((F_CPU/16/BUAD)-1)

// int main(void)
// {
// 	UBRR0H = (BRC >> 8);
// 	UBRR0H = BRC;

// 	UCSR0B = (1 << TXEN0);
// 	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

// 	while (1)
// 	{
// 		UDR0 = 'H';
// 		_delay_ms(1000);
// 	}
	

// 	return 0;
// }
/*__________________Blink led________________*/
// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>
// #include <stdio.h>

// int main(void)
// {
// 	DDRB=0xFF;
// 	//DDRB=(1<<DDB7);
// 	while (1) 
// 	{
// 		//PORTB = ; 
// 		PORTB = (1<<PB7);
// 		_delay_ms(1000);
// 		PORTB = 0b00000000; 
// 		//PORTB = (0<<PB7);
// 		_delay_ms(1000);
// 	}
// 		return (0);
// }
/*___________________________ Para ver isto______________*/
#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

/* define baud rate as BAUD before this (can be defined externaly)*/
#define BAUD  9600
#define UBRRVAL F_CPU/8/BAUD-1


static int uputc(char,FILE*);

static int uputc(char c,FILE *stream)
{
	if (c == '\n')
		uputc('\r',stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);	/* wait until we can send a new byte */
	UDR0 = (uint8_t) c;
	return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uputc, NULL,_FDEV_SETUP_WRITE);


void init_adc(void)
{
	ADMUX = _BV(REFS0) | _BV(MUX1); // PIN A2
	ADCSRA = _BV(ADEN)|_BV(ADATE)|_BV(ADSC)|_BV(ADPS2);
}

//uart receive isr
ISR(USART0_RX_vect)
{
	uint8_t c = UDR0;
	putchar(c);
}

int main(void)
{
	int v;
	int count=0;
	float tempC ;
  byte precision = 4;
	char floatBuffer[20];
  char printBuffer[80];

	stdout=&mystdout;
	init_adc();
	/* pin config */
	DDRB = (1 << DDB5);
	PORTB=(1<<PB5);
	DDRF = 0b00000000;
	/* uart config */
	UCSR0A = (1 << U2X0); /* this sets U2X0 to 1 */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (3 << UCSZ00); /* this sets UCSZ00 to 3 */

	/* baudrate setings (variable set by macros) */
	UBRR0H = (UBRRVAL) >> 8;
	UBRR0L = UBRRVAL;

	/* ADC cfg */
	sei();							  /* enable interrupts */
	puts("Buongiorno");

	

	for (;;) {
		
    	 v=(ADCL | ADCH<<8);
		 if (v >=100)
		 {
	 		tempC = (float)v/1024*5.0;
             dtostrf(tempC, precision+3, precision, floatBuffer);
            sprintf(printBuffer, "With %%.%df precision, x = %s", precision, floatBuffer);
  			puts(printBuffer);
		 }
		 _delay_ms(1000);
		
    
		//nothing here, all is done on the uart interrupt/
	}
}