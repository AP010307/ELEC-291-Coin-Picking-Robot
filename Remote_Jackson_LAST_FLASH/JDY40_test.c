
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include "usart.h"
#include "Software_UART.h"
#include "lcd.h"

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define TIMER_0_FREQ 1000L
#define BUZZER_PIN (1 << PC5)

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
//lcd

void Configure_Pins(void)
{
	DDRB|=0b00000001; // PB0 is output.
	DDRD|=0b11100000; //  PD5, PD6, and PD7 are outputs.
	DDRC|=0b00000011; // PC0, PC1 is output.
	PORTC |=0b00011100;
	PORTD |= 0b00000100;
	
	DDRC |= BUZZER_PIN;
}
void LCD_pulse (void)
{
	LCD_E_1;
	LCD_E_0;
}

void LCD_byte (unsigned char x)
{
	//Send high nible
	if(x&0x80) LCD_D7_1; else LCD_D7_0;
	if(x&0x40) LCD_D6_1; else LCD_D6_0;
	if(x&0x20) LCD_D5_1; else LCD_D5_0;
	if(x&0x10) LCD_D4_1; else LCD_D4_0;
	LCD_pulse();
	_delay_us(40);
	//Send low nible
	if(x&0x08) LCD_D7_1; else LCD_D7_0;
	if(x&0x04) LCD_D6_1; else LCD_D6_0;
	if(x&0x02) LCD_D5_1; else LCD_D5_0;
	if(x&0x01) LCD_D4_1; else LCD_D4_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS_1;
	LCD_byte(x);
	_delay_ms(1);
}

void WriteCommand (unsigned char x)
{
	LCD_RS_0;
	LCD_byte(x);
	_delay_ms(1);
}

void LCD_4BIT (void)
{
	LCD_E_0; // Resting state of LCD's enable is zero
	//LCD_RW=0; // We are only writing to the LCD in this program
	_delay_ms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	_delay_ms(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	_delay_ms(1);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}


//radio

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	PORTD &= ~(BIT4); // 'set' pin to 0 is 'AT' mode.
	_delay_ms(10);
	SendString1(s);
	GetString1(buff, 40);
	PORTD |= BIT4; // 'set' pin to 1 is normal operation mode.
	_delay_ms(10);
	printf("Response: %s\r\n", buff);
}

void ReceptionOff (void)
{
	PORTD &= ~(BIT4); // 'set' pin to 0 is 'AT' mode.
	_delay_ms(10);
	SendString1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing from the radio.
	_delay_ms(10);
	PORTD |= BIT4; // 'set' pin to 1 is normal operation mode.
}


//ADC read

unsigned int ReadChannel(unsigned char mux)
{
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // frequency prescaler
	ADMUX = mux; // channel select
	ADMUX |= (1<<REFS1) | (1<<REFS0); 
	ADCSRA |= (1<<ADSC); // Start conversion
	while ( ADCSRA & (1<<ADSC) ) ;
	ADCSRA |= (1<<ADSC); // a transformation �single conversion�
	while ( ADCSRA & (1<<ADSC) );
	ADCSRA &= ~(1<<ADEN); // Disable ADC
	return ADCW;
}

//speaker
void Timer0_init(void){
    TCNT0 = 0x00;  
    TCCR0A = 0x00;   
    TCCR0B = (1<<CS01) | (1<<CS00);
    TIMSK0 = (1<<TOIE0);

    sei(); // set global interrupts
}



int beep_flag = 0;
int pitch = 1;
int count_1 = 0;
int count_2 = 0;
int count_3 = 0;
int speaker_flag = 0;
long pause = 300;
int sound_countdown = 0;

ISR(TIMER0_OVF_vect) {
    // TCNT0 = 0x00;  // Not necessary, it is done by hardware
	if(sound_countdown > 0){
		sound_countdown --;
		if (beep_flag == 1) {
			if(count_3 >= pause){
				beep_flag = 0;
				count_3 = 0;

			}
			else{
				if(count_1 >= pitch){
					PORTC ^= BUZZER_PIN;
					count_1 = 0;
				}
				else{
					count_1++;
				}
				count_3++;
			}
		}
		else {
			if(count_2 >= pause){
				beep_flag = 1;
				count_2 = 0;
			}
			else{
				count_2++;
			}
		}
	}


}


void main (void)
{

	Configure_Pins();
	LCD_4BIT();
	Timer0_init();
	
	char c;
	char buff[80];
	char buff2[17];
	char buff3[4];
	char buff4[4];
    int timeout_cnt=0;
    int cont1=0, cont2=100;

	unsigned int adc_x;
	unsigned long int v;

	unsigned int adc_y;
	unsigned long int v2;

	int toggle_flag = 0;

	int j=0;
	int strength = 0;

	
	usart_init();   // configure the hardware usart and baudrate
	Init_Software_Uart(); // Configure the sorftware UART
	_delay_ms(500); // Give putty a chance to start before we send information...
	printf("\r\nATMega328 JDY-40 Master Test\r\n");
	
	ReceptionOff();

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDABAB\r\n"); // changed to ABAB
	SendATCommand("AT+RFC044\r\n");


	while(1)
	{

		// future joystick
		adc_x=ReadChannel(2);
		v=(adc_x*5000L)/1023L;
	//	printf("ADCx=%03x, %ld.%03ldV\r\n", adc_x, v/1000, v%1000);
		adc_y = ReadChannel(3);
		v2 = (adc_y * 5000L) / 1023L;
	//	printf("ADCy=%03x, %ld.%03ldV\r\n", adc_y, v2 / 1000, v2 % 1000);

	

		// future automode toggle button // make sure that robot recieved signal by getting a reply back that it got it
		if((PIND & (1 << 2)) == 0){
			//Start of Command
			SendString1("!");
			SendString1("TO");
			printf("toggle\n");
			_delay_ms(50); 
			timeout_cnt=0;
			while(1)
			{
				if(RXD_FLAG) break; // Something has arrived
				if(++timeout_cnt>3500) break; // Wait up to 15ms for the repply
				_delay_us(100); // 100us*250=25ms
			}
			
			if(RXD_FLAG) // Something has arrived from the slave
			{
				c = GetByte1(); // header to recieve correct signal
				if (c == '!') {
					GetString1_timeout(buff4, sizeof(buff4)-1);
					printf(buff4);
					if(strcmp(buff4,"TD")==0) 
					{
						toggle_flag = 1;
					}
					else if(strcmp(buff4,"TF")==0)
					{
						toggle_flag = 0;
					}
				}
			}
			else // Timed out waiting for reply
			{
				printf("NO RESPONSE\r\n");
			}
			_delay_ms(100); 

		}

		if((PINC & (1 << 4)) == 0){

			//Start of Command
			SendString1("!");
			SendString1("PU"); // pick up coin
			printf("PU\n");

		}
		////JOYSTICK LOGIC/////

		SendString1("!");
		
		if(adc_x > 700){
			if (adc_y > 700) {
				SendString1("BL"); // Left
				printf("BL\r\n");
				LCDprint("Back Left", 2, 1);
			}
			
			else if (adc_y < 300) {
				SendString1("BR"); // Right
				printf("BR\r\n");
				LCDprint("Back Right", 2, 1);

			}
			else {
				SendString1("BN"); // Backward
				printf("BN\r\n");
				LCDprint("Back", 2, 1);
			}
		}
		else if(adc_x < 300){
			if (adc_y > 700) {
				SendString1("FL"); // Left
				printf("FL\r\n");
				LCDprint("Forward Left", 2, 1);
			}
			else if (adc_y < 300) {
				SendString1("FR"); // Right
				printf("FR\r\n");
				LCDprint("Forward Right", 2, 1);


			}
			else {
				SendString1("FN"); // Backward
				printf("FN\r\n");
				LCDprint("Forward", 2, 1);
			}
		}
		else{
			if (adc_y > 700) {
				SendString1("HL"); // Left
				printf("HL\r\n");
				LCDprint("Hard Left", 2, 1);
			}
			else if (adc_y < 300) {
				SendString1("HR"); // Right
				printf("HR\r\n");
				LCDprint("Hard Right", 2, 1);


			}
			else {
				SendString1("ST"); // Stop
				printf("ST\r\n");
				LCDprint("Stop", 2, 1);
			}
		}

		
		/////END OF JOYSTICK LOGIC/////
	
		timeout_cnt=0;
		while(1)
		{
			if(RXD_FLAG) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 15ms for the repply
			_delay_us(100); // 100us*250=25ms
		}
		
		if(RXD_FLAG) // Something has arrived from the slave
		{
			
			c = GetByte1(); // header to recieve correct signal
			if (c == '!') {

					GetString1(buff3, sizeof(buff3)-1);
					printf(buff3);
					if(strcmp(buff3,"DI") == 0){ // dime
						pause = 300;
						strength = 1;
						sound_countdown = 500;
					}
					else if(strcmp(buff3,"QU") == 0){ // quarter 
						pause = 250;
						strength = 2;
						sound_countdown = 500;
					}
					else if(strcmp(buff3,"NI") == 0){ // Nickle
						pause = 200;
						strength = 3;
						sound_countdown = 500;
					}
					else if(strcmp(buff3,"TW") == 0){ //two dollar
						pause = 150;
						strength = 4;
						sound_countdown = 500;
					}
					else if(strcmp(buff3,"DO") == 0){ //dollar 
						pause = 100;
						strength = 5;
						sound_countdown = 500;
					}
					else{
						strength = 0;
				
					}
					if(strcmp(buff3,"TF") == 0){
						toggle_flag = 0;
					}
				
				
			}
		}
		else // Timed out waiting for reply
		{
			strength = 0;
		}
	
		if(toggle_flag){
			sprintf(buff2,"Toggle On %d",strength); // need to add signal strength
		
		}
		else{
			sprintf(buff2,"Toggle Off %d",strength); // need to add signal strength
		}
		LCDprint(buff2, 1, 1);
		_delay_ms(5);  // Set the information interchange pace: communicate about every 50ms
	}

}

