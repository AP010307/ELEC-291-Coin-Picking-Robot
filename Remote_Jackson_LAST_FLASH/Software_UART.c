#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Software_UART.h"

/* Pinout for DIP28 ATMega328P:

                           -------
     (PCINT14/RESET) PC6 -|1    28|- PC5 (ADC5/SCL/PCINT13)
       (PCINT16/RXD) PD0 -|2    27|- PC4 (ADC4/SDA/PCINT12)
       (PCINT17/TXD) PD1 -|3    26|- PC3 (ADC3/PCINT11)
      (PCINT18/INT0) PD2 -|4    25|- PC2 (ADC2/PCINT10)
 (PCINT19/OC2B/INT1) PD3 -|5    24|- PC1 (ADC1/PCINT9)
    (PCINT20/XCK/T0) PD4 -|6    23|- PC0 (ADC0/PCINT8)
                     VCC -|7    22|- GND
                     GND -|8    21|- AREF
(PCINT6/XTAL1/TOSC1) PB6 -|9    20|- AVCC
(PCINT7/XTAL2/TOSC2) PB7 -|10   19|- PB5 (SCK/PCINT5)
   (PCINT21/OC0B/T1) PD5 -|11   18|- PB4 (MISO/PCINT4)
 (PCINT22/OC0A/AIN0) PD6 -|12   17|- PB3 (MOSI/OC2A/PCINT3)
      (PCINT23/AIN1) PD7 -|13   16|- PB2 (SS/OC1B/PCINT2)
  (PCINT0/CLKO/ICP1) PB0 -|14   15|- PB1 (OC1A/PCINT1)
                           -------
*/

volatile unsigned char RXD_state=0, TXD_state=0;
volatile unsigned char RXD_FLAG=0, RXD_SR=0, RXD_DATA=0;
volatile unsigned char TXD_DATA=0;

// 'Timer 1 output compare A' Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
{
	OCR1A = OCR1A + OCR1_RELOAD;
	
	if(RXD_state>0) RXD_state++;
	if(TXD_state>0) TXD_state++;

	// Receive data state machine
	switch(RXD_state)
	{
		case 0:
			if ((PINB&BIT2)==0) RXD_state=1; // Start bit?
			break;
		case 4:
			// Middle of the start bit
			if ((PINB&BIT2)!=0)
			{
				RXD_state=0; // Bad start bit
			}
			else
			{
				// Good start bit, initialize shift register to zero
				RXD_SR=0;
			}
			break;
		case 12: // Check middle of bit 0
			RXD_SR|=(PINB&BIT2)?BIT0:0x00;
			break;
		case 20: // Check middle of bit 1
			RXD_SR|=(PINB&BIT2)?BIT1:0x00;
			break;
		case 28: // Check middle of bit 2
			RXD_SR|=(PINB&BIT2)?BIT2:0x00;
			break;
		case 36: // Check middle of bit 3
			RXD_SR|=(PINB&BIT2)?BIT3:0x00;
			break;
		case 44: // Check middle of bit 4
			RXD_SR|=(PINB&BIT2)?BIT4:0x00;
			break;
		case 52: // Check middle of bit 5
			RXD_SR|=(PINB&BIT2)?BIT5:0x00;
			break;
		case 60: // Check middle of bit 6
			RXD_SR|=(PINB&BIT2)?BIT6:0x00;
			break;
		case 68: // Check middle of bit 7
			RXD_SR|=(PINB&BIT2)?BIT7:0x00;
			break;
		case 76: // Check middle of stop bit
			if ((PINB&BIT2)!=0) // If valid stop bit, store data
			{
				RXD_DATA=RXD_SR;
				RXD_FLAG=0x01;
			}
			RXD_state=0; // Ready for another byte			
			break;
		default:
			break;
	}

	// Transmit data state machine
	switch(TXD_state)
	{
		case 0: // waiting for data to transmit
			break;
		case 2: // send start bit
			PORTB &= ~(BIT1);
			break;
		case 10: // send bit 0
			if(TXD_DATA&BIT0) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 18: // send bit 1
			if(TXD_DATA&BIT1) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 26: // send bit 2
			if(TXD_DATA&BIT2) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 34: // send bit 3
			if(TXD_DATA&BIT3) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 42: // send bit 4
			if(TXD_DATA&BIT4) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 50: // send bit 5
			if(TXD_DATA&BIT5) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 58: // send bit 6
			if(TXD_DATA&BIT6) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 66: // send bit 7
			if(TXD_DATA&BIT7) PORTB |= BIT1; else PORTB &= ~(BIT1);
			break;
		case 74: // send stop bit
			PORTB |= BIT1;
			break;
		case 82: // Done, get ready for another byte
			TXD_state=0;
			break;
		default:
		break;
	}
}

void Reset_Software_Uart (void)
{
	RXD_state=0;
	TXD_state=0;
	RXD_FLAG=0;
	RXD_SR=0;
	RXD_DATA=0;
	TXD_DATA=0;
}

void Init_Software_Uart(void)
{
	DDRB|=BIT1; // PB1 (pin 15) is our software UART TXD pin (output).
	PORTB |= BIT1;
	
	DDRD|=BIT4; // PD4 (pin 6) is the 'SET' pin of the JDY40
	PORTD |= BIT4;
	
	TCCR1B |= _BV(CS10);   // set prescaler to Clock/1
	TIMSK1 |= _BV(OCIE1A); // output compare match interrupt for register A
	sei(); // enable global interupt
}

void SendByte1 (unsigned char c)
{
	while(TXD_state!=0);
	TXD_DATA=c;
	TXD_state=1; // Start transmission
}

void SendString1(char * s)
{
	while(*s != 0) SendByte1(*s++);
}

unsigned char GetByte1 (void)
{
	while(RXD_FLAG!=0x01); // Wait for data to arrive
	RXD_FLAG=0;
	return RXD_DATA;
}

void GetString1(char * s, int nmax)
{
	unsigned char c;
	int n=0;
	
	while(1)
	{
		c=GetByte1();
		if( (c=='\n') || n==(nmax-1) )
		{
			*s=0;
			return;
		}
		else
		{
			*s=c;
			s++;
			n++;
		}
	}
}

void GetString1_timeout(char * s, int nmax)
{
	unsigned char c;
	int n=0;
	int timeout_cnt;
	
	while(1)
	{
		timeout_cnt=0;
		
		while(1)
		{
			if(RXD_FLAG==1)
			{
				c=GetByte1();
				break;
			}
			else
			{
				if(++timeout_cnt==100) // 100us*100 = 10ms timeout
				{
					*s=0;
					return;
				}
				_delay_us(100);
			}
		}
		timeout_cnt=0;
		
		if( (c=='\n') || n==(nmax-1) )
		{
			*s=0;
			return;
		}
		else
		{
			*s=c;
			s++;
			n++;
		}
	}
}
