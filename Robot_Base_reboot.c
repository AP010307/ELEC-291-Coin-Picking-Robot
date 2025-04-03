///////USE THIS - FINALN- USE THIS FOR DEMO//////////

#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

volatile unsigned int servo_counter = 0;
volatile unsigned char servo1 = 150, servo2 = 150;

#define SERVO1     		P1_6 //servo1 
#define SERVO2     		P1_7 //servo2
#define PERIOD_PIN 		P1_5 //metal detector
#define OUTPIN1    		P1_0 //M2-right
#define OUTPIN2    		P1_1 //M2-right
#define OUTPIN3    		P1_2 //M1-left
#define OUTPIN4    		P1_3 //M1-left
#define OUTPIN5    		P1_4 //electromagnet
#define BOOT       		P3_7 

#define SYSCLK 72000000 // SYSCLK frequency in Hz
#define BAUDRATE 115200L
#define SARCLK 18000000L

int perimeter_flag = 0;
long ambient_freq = 0;
int coin_count = 0;

//timer5 reload for a 10us period
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate
xdata char buff[20];
char _c51_external_startup(void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key

	VDM0CN = 0x80;
	RSTSRC = 0x02 | 0x04;

#if (SYSCLK == 48000000L)	
	SFRPAGE = 0x10;
	PFE0CN = 0x10; // SYSCLK < 50 MHz.
	SFRPAGE = 0x00;
#elif (SYSCLK == 72000000L)
	SFRPAGE = 0x10;
	PFE0CN = 0x20; // SYSCLK < 75 MHz.
	SFRPAGE = 0x00;
#endif

#if (SYSCLK == 12250000L)
	CLKSEL = 0x10;
	CLKSEL = 0x10;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 24500000L)
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 48000000L)	
	// Before setting clock to 48 MHz, must transition to 24.5 MHz first
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
	CLKSEL = 0x07;
	CLKSEL = 0x07;
	while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 72000000L)
	// Before setting clock to 72 MHz, must transition to 24.5 MHz first
	CLKSEL = 0x00;
	CLKSEL = 0x00;
	while ((CLKSEL & 0x80) == 0);
	CLKSEL = 0x03;
	CLKSEL = 0x03;
	while ((CLKSEL & 0x80) == 0);
#else
#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
#endif

	// Configure the pins used as outputs
	P1MDOUT |= 0b_1101_1111; // SERVO2, SERVO1, OUPTUT1 to OUTPUT5
	P0MDOUT |= 0x11; // Configure UART0 TX (P0.4) as push-pull output
	P2MDOUT |= 0x01; // P2.0 in push-pull mode
	XBR0 = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1 = 0X00; // 
	XBR2 = 0x41; // Enable crossbar and weak pull-ups

#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100 - ((SYSCLK / BAUDRATE) / (2L * 12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |= 0x20;
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	P2_0 = 1;

	// Initialize timer 5 for periodic interrupts
	SFRPAGE = 0x10;
	TMR5CN0 = 0x00;
	TMR5 = 0xffff;   // Set to reload immediately
	EIE2 |= 0b_0000_1000; // Enable Timer5 interrupts
	TR5 = 1;         // Start Timer5 (TMR5CN0 is bit addressable)

	EA = 1;

	SFRPAGE = 0x00;

	return 0;
}

void InitADC(void)
{
	SFRPAGE = 0x00;
	ADEN = 0; // Disable ADC

	ADC0CN1 =
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
		(0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0); // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32

	ADC0CF0 =
		((SYSCLK / SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.

	ADC0CF1 =
		(0 << 7) | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)

	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0); // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2 =
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)

	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0); // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN = 1; // Enable ADC
}

void InitPinADC(unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;

	mask = 1 << pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
	case 0:
		P0MDIN &= (~mask); // Set pin as analog input
		P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	case 1:
		P1MDIN &= (~mask); // Set pin as analog input
		P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	case 2:
		P2MDIN &= (~mask); // Set pin as analog input
		P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
	default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

void Timer5_ISR(void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE = 0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	TMR5RL = RELOAD_10us;
	servo_counter++;
	if (servo_counter == 2000)
	{
		servo_counter = 0;
	}
	if (servo1 >= servo_counter)
	{
		SERVO1 = 1;
	}
	else
	{
		SERVO1 = 0;
	}
	if (servo2 >= servo_counter)
	{
		SERVO2 = 1;
	}
	else
	{
		SERVO2 = 0;
	}
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter

	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0 |= 0b_0100_0000;

	TMR3RL = (-(SYSCLK) / 1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow

	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0;                   // Stop Timer3 and clear overflow flag
}

void waitms(unsigned int ms)
{
	unsigned int j;
	for (j = ms; j != 0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

// Measure the period of a square signal at PERIOD_PIN
unsigned long GetPeriod(int n)
{
	unsigned int overflow_count;
	unsigned char i;

	TR0 = 0; // Stop Timer/Counter 0
	TMOD &= 0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD |= 0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer

	// Reset the counter
	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow_count = 0;
	TR0 = 1;
	while (PERIOD_PIN != 0) // Wait for the signal to be zero
	{
		if (TF0 == 1) // Did the 16-bit timer overflow?
		{
			TF0 = 0;
			overflow_count++;
			if (overflow_count == 10) // If it overflows too many times assume no signal is present
			{
				TR0 = 0;
				return 0; // No signal
			}
		}
	}

	// Reset the counter
	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow_count = 0;
	TR0 = 1;
	while (PERIOD_PIN != 1) // Wait for the signal to be one
	{
		if (TF0 == 1) // Did the 16-bit timer overflow?
		{
			TF0 = 0;
			overflow_count++;
			if (overflow_count == 10) // If it overflows too many times assume no signal is present
			{
				TR0 = 0;
				return 0; // No signal
			}
		}
	}

	// Reset the counter
	TR0 = 0;
	TL0 = 0; TH0 = 0; TF0 = 0; overflow_count = 0;
	TR0 = 1; // Start the timer
	for (i = 0; i < n; i++) // Measure the time of 'n' periods
	{
		while (PERIOD_PIN != 0) // Wait for the signal to be zero
		{
			if (TF0 == 1) // Did the 16-bit timer overflow?
			{
				TF0 = 0;
				overflow_count++;
			}
		}
		while (PERIOD_PIN != 1) // Wait for the signal to be one
		{
			if (TF0 == 1) // Did the 16-bit timer overflow?
			{
				TF0 = 0;
				overflow_count++;
			}
		}
	}
	TR0 = 0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!

	return (overflow_count * 65536 + TH0 * 256 + TL0);
}

void eputs(char* String)
{
	while (*String)
	{
		putchar(*String);
		String++;
	}
}

void PrintNumber(long int val, int Base, int digits)
{
	code const char HexDigit[] = "0123456789ABCDEF";
	int j;
#define NBITS 32
	xdata char buff[NBITS + 1];
	buff[NBITS] = 0;

	if (val < 0)
	{
		putchar('-');
		val *= -1;
	}

	j = NBITS - 1;
	while ((val > 0) | (digits > 0))
	{
		buff[j--] = HexDigit[val % Base];
		val /= Base;
		if (digits != 0) digits--;
	}
	eputs(&buff[j + 1]);
}

// WARNING: do not use printf().  It makes the program big and slow!

///////SLAVE///////

void UART1_Init(unsigned long baudrate)
{
	SFRPAGE = 0x20;
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	SBCON1 = 0x00;   // disable baud rate generator
	SBRL1 = 0x10000L - ((SYSCLK / baudrate) / (12L * 2L));
	TI1 = 1; // indicate ready for TX
	SBCON1 |= 0x40;   // enable baud rate generator
	SFRPAGE = 0x00;
}

void putchar1(char c)
{
	SFRPAGE = 0x20;
	while (!TI1);
	TI1 = 0;
	SBUF1 = c;
	SFRPAGE = 0x00;
}

void sendstr1(char* s)
{
	while (*s)
	{
		putchar1(*s);
		s++;
	}
}

char getchar1(void)
{
	char c;
	SFRPAGE = 0x20;
	while (!RI1);
	RI1 = 0;
	// Clear Overrun and Parity error flags 
	SCON1 &= 0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}


char getchar1_with_timeout(void)
{
	char c;
	unsigned int timeout;
	SFRPAGE = 0x20;
	timeout = 0;
	while (!RI1)
	{
		SFRPAGE = 0x00;
		Timer3us(20);
		SFRPAGE = 0x20;
		timeout++;
		if (timeout == 25000)
		{
			SFRPAGE = 0x00;
			return ('\n'); // Timeout after half second
		}
	}
	RI1 = 0;
	// Clear Overrun and Parity error flags 
	SCON1 &= 0b_0011_1111;
	c = SBUF1;
	SFRPAGE = 0x00;
	return (c);
}

void getstr1(char* s, unsigned char n)
{
	char c;
	unsigned char cnt;

	cnt = 0;
	while (1)
	{
		c = getchar1_with_timeout();
		if (c == '\n')
		{
			*s = 0;
			return;
		}

		if (cnt < n)
		{
			cnt++;
			*s = c;
			s++;
		}
		else
		{
			*s = 0;
			return;
		}
	}
}

// RXU1 returns '1' if there is a byte available in the receive buffer of UART1
bit RXU1(void)
{
	bit mybit;
	SFRPAGE = 0x20;
	mybit = RI1;
	SFRPAGE = 0x00;
	return mybit;
}

void waitms_or_RI1(unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for (j = 0; j < ms; j++)
	{
		for (k = 0; k < 4; k++)
		{
			if (RXU1()) return;
			Timer3us(250);
		}
	}
}

void SendATCommand(char* s)
{

	printf("Command: %s", s);
	P2_0 = 0; // 'set' pin to 0 is 'AT' mode.
	waitms(5);
	sendstr1(s);
	getstr1(buff, sizeof(buff) - 1);
	waitms(10);
	P2_0 = 1; // 'set' pin to 1 is normal operation mode.
	printf("Response: %s\r\n", buff);
}

void ReceptionOff(void)
{
	P2_0 = 0; // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	printf("\r\Test3.\r\n");
	sendstr1("AT+DVID0000\r\n");
	// Some unused id, so that we get nothing in RXD1.
	printf("\r\Test4.\r\n");
	waitms(10);

	// Clear Overrun and Parity error flags 
	SCON1 &= 0b_0011_1111;
	P2_0 = 1; // 'set' pin to 1 is normal operation mode.
}
void move_robot(char* command)
{

	if (strcmp(command, "FN") == 0 || strcmp(command,"!F") == 0) 
	{
		OUTPIN1 = 1;
		OUTPIN2 = 0;
		OUTPIN3 = 1;
		OUTPIN4 = 0;
		eputs("Forward\r\n");
	}
	else if (strcmp(command, "BN") == 0 || strcmp(command,"!B") == 0)
	{

		OUTPIN1 = 0;
		OUTPIN2 = 1;
		OUTPIN3 = 0;
		OUTPIN4 = 1;
		eputs("Backward\r\n");
	}
	else if (strcmp(command, "HL") == 0 || strcmp(command,"!H") == 0)
	{
		OUTPIN1 = 1;
		OUTPIN2 = 0;
		OUTPIN3 = 0;
		OUTPIN4 = 1;
		eputs("Hard Left\r\n");
	}
	else if (strcmp(command, "HR") == 0 || strcmp(command,"!H") == 0)
	{
		OUTPIN1 = 0;
		OUTPIN2 = 1;
		OUTPIN3 = 1;
		OUTPIN4 = 0;
		eputs("Hard Right\r\n");
	}
	else if (strcmp(command, "FL") == 0 || strcmp(command,"!F") == 0)
	{
		OUTPIN1 = 1;
		OUTPIN2 = 0;
		OUTPIN3 = 0;
		OUTPIN4 = 0;
		eputs("Gradual Left\r\n");
	}
	else if (strcmp(command, "FR") == 0 || strcmp(command,"!F") == 0)
	{
		OUTPIN1 = 0;
		OUTPIN2 = 0;
		OUTPIN3 = 1;
		OUTPIN4 = 0;
		eputs("Gradual Right\r\n");
	}
	else if (strcmp(command, "BL") == 0 || strcmp(command,"!B") == 0)
	{
		OUTPIN1 = 0;
		OUTPIN2 = 1;
		OUTPIN3 = 0;
		OUTPIN4 = 0;
		eputs("Backward Left\r\n");
	}
	else if (strcmp(command, "BR") == 0 || strcmp(command,"!B") == 0)
	{
		OUTPIN1 = 0;
		OUTPIN2 = 0;
		OUTPIN3 = 0;
		OUTPIN4 = 1;
		eputs("Backward Right\r\n");
	}
	else if (strcmp(command, "ST") == 0 || strcmp(command,"!S") == 0)
	{
		//servo1 = 160;
		//servo2 = 160;
		OUTPIN1 = 0;
		OUTPIN2 = 0;
		OUTPIN3 = 0;
		OUTPIN4 = 0;
		eputs("Stop\r\n");
	}
	else
	{
		eputs("Unknown command\r\n");
	}
	//waitms(200);
}

long sample_average_frequency(int samples)
{
	long freq_sum = 0;
	int valid = 0;
	int i = 0;
	for (i; i < samples; i++)
	{
		long count = GetPeriod(30);
		if (count > 0)
		{
			freq_sum += (SYSCLK * 30.0) / (count * 12);
			valid++;
		}
		waitms(50);
	}
	if (valid == 0) return 0;
	return freq_sum / valid;
}

int turn_flag = 0;

void servos_magnet()
{
	int a;

	unsigned char i;
	unsigned char j;
	//moves backward
	OUTPIN1 = 0;
	OUTPIN2 = 1;
	OUTPIN3 = 0;
	OUTPIN4 = 1;
	if (turn_flag == 1) {
		waitms(150);
	}
	else {
		waitms(250);
	}

	//stops
	OUTPIN1 = 0;
	OUTPIN2 = 0;
	OUTPIN3 = 0;
	OUTPIN4 = 0;

	for (j = 100; j < 250; j += 3)
	{
		servo2 = j;
		waitms(35);
	}
	for (i = 100; i < 250; i += 3)
	{
		servo1 = i;
		waitms(35);
	}

	OUTPIN5 = 1;

	for (j = 250; j > 140; j -= 3)
	{
		servo2 = j;
		waitms(35);
	}

	for (i = 250; i > 120; i -= 3)
	{
		servo1 = i;
		waitms(35);
	}
	for (j = 140; j > 100; j -= 3)
	{
		servo2 = j;
		waitms(35);
	}

	
	OUTPIN5 = 0;
	waitms(100);
//	a = sample_average_frequency(10);
//	if(a<=(100+ambient_freq)){
//		ambient_freq = a;
//	}
//	else{
//		coin_count--;
//	}
	
}

void perimter_detector(void) {
	long int j1, v1;
	long int j2, v2;

	j1 = ADC_at_Pin(QFP32_MUX_P2_2);
	v1 = (j1 * 33000) / 0x3fff;
	//eputs("ADC[P2.2]=0x");
	//PrintNumber(j1, 16, 4);
	//eputs(", ");
	//eputs("\r\n ");
	//PrintNumber(v1 / 10000, 10, 1);
	//putchar('.');
	//PrintNumber(v1 % 10000, 10, 4);
	//eputs("V ");;
	//eputs("\r\n ");



	j2 = ADC_at_Pin(QFP32_MUX_P2_3);
	v2 = (j2 * 33000) / 0x3fff;
	//eputs("ADC[P2.3]=0x");
	//PrintNumber(j2, 16, 4);
	//eputs(", ");
	//eputs("\r\n ");
	//PrintNumber(v2 / 10000, 10, 1);
	//putchar('.');
	//PrintNumber(v2 % 10000, 10, 4);
	//eputs("V ");
	//eputs("\r\n ");
	/*
	eputs("ADC[P2.6]=0x");
	PrintNumber(j3, 16, 4);
	eputs(", ");
	eputs("\r\n ");
	PrintNumber(v3 / 10000, 10, 1);
	putchar('.');
	PrintNumber(v3 % 10000, 10, 4);
	eputs("V ");
	eputs("\r\n ");
	*/
	/*
	j4 = ADC_at_Pin(QFP32_MUX_P0_2);
	v4 = (j4 * 33000) / 0x3fff;
	eputs("ADC[P0.2]=0x");
	PrintNumber(j4, 16, 4);
	eputs(", ");
	eputs("\r\n ");
	PrintNumber(v4 / 10000, 10, 1);
	putchar('.');
	PrintNumber(v4 % 10000, 10, 4);
	eputs("V ");
	eputs("\r\n ");
	*/
	//waitms(500);

	if (v1 > 0 || v2 > 0) {
		perimeter_flag = 1;

	}
	else {
		perimeter_flag = 0;
	}
}



void main(void)
{
	//long int j, v;
	long int count;
	long int f = 0;
	xdata char buff3[3];
	//unsigned char LED_toggle = 0; // Used to test the outputs

	//char buff[20];
	unsigned int cnt = 0;
	char c;
	//char cmd[3];

	int autoflag = 0;
	int turning_count = 0;
	int coinflag = 0;
	int turning_time = 0;
	int count_cooldown = 20;
	int temp = 0;

	int freq_diff = 0;

	int coin_tier = 0;
	int coin_radio_count = 0;


	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitPinADC(2, 3); // Configure P2.3 as analog input
	InitPinADC(2, 6);
	InitPinADC(0, 2);
	InitADC();

	waitms(1000); // Wait a second to give PuTTy a chance to start

	eputs("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.

	eputs("\r\nEFM8LB12 multi I/O example.\r\n");
	eputs("Measures the voltage from pins P2.2 and P2.3\r\n");
	eputs("Measures period on P1.5\r\n");
	eputs("Toggles pins P1.0, P1.1, P1.2, P1.3, P1.4\r\n");
	eputs("Generates servo PWMs on P1.6 and P1.7\r\n");
	eputs("Reads the BOOT push-button on pin P3.7\r\n\r\n");

	OUTPIN1 = 0;
	OUTPIN2 = 0;
	OUTPIN3 = 0;
	OUTPIN4 = 0;
	OUTPIN5 = 0;


	//waitms(250);
	printf("\r\nEFM8LB12 JDY-40 Slave Test.\r\n");
	UART1_Init(9600);
	//turn off slave reception by default
	ReceptionOff();


	//print out module info
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// Example: set an ID = 0xABBA
	SendATCommand("AT+DVIDABAB\r\n");
	SendATCommand("AT+RFC044\r\n");
	cnt = 0;

	//servos+magnet
	ambient_freq = sample_average_frequency(10);
	//servos_magnet();


	// ------------------------------------------
	// 3)  Single main loop combining both tasks
	// ------------------------------------------
	while (1)
	{
		if (count_cooldown > 0) {
			count_cooldown--;
		}
		//j = ADC_at_Pin(QFP32_MUX_P2_2);
		//v = (j * 33000) / 0x3fff;
		//eputs("ADC[P2.2]=0x");
		//PrintNumber(j, 16, 4);
		//eputs(", ");
		//PrintNumber(v / 10000, 10, 1);
		//putchar('.');
		//PrintNumber(v % 10000, 10, 4);
		//eputs("V ");
		//waitms(1000);

		//j = ADC_at_Pin(QFP32_MUX_P2_3);
		//v = (j * 33000) / 0x3fff;
		//eputs("ADC[P2.3]=0x");
		//PrintNumber(j, 16, 4);
		//eputs(", ");
		//PrintNumber(v / 10000, 10, 1);
		//putchar('.');
		//PrintNumber(v % 10000, 10, 4);
		//eputs("V ");

		//eputs("BOOT(P3.7)=");
		//if (BOOT) {
			//eputs("1 ");
		//}
		//else
		//{
			//eputs("0 ");
		//}

		// Not very good for high frequencies because of all the interrupts in the background
		// but decent for low frequencies around 10kHz.

		//convert ADC to Voltage for perimeter detector 1 and 2

		perimter_detector();


		count = GetPeriod(30);
		if (count > 0)
		{
			f = (SYSCLK * 30.0) / (count * 12);
			/*eputs("f=");
			PrintNumber(f, 10, 7);
			eputs("Hz, count=");
			PrintNumber(count, 10, 8);
			eputs("          \r");*/
		}
		else
		{
			//eputs("NO SIGNAL                     \r");
		}

		//if metal is detected the move backward just a bit then use arm
		if (f > 1000)
		{
			freq_diff = abs(f - ambient_freq);
			coinflag = (freq_diff > 300) ? 1 : 0;
		}
		else coinflag = 0;

		if (coinflag)
		{
			if (freq_diff > 1470 && coin_tier <= 6) {
				//eputs("1 DOLLAR\r\n");
				coin_tier = 6;
			}
			else if (freq_diff > 1435 && coin_tier <= 5) {
				//eputs("2 DOLLAR\r\n");
				coin_tier = 5;
			}
			else if (freq_diff > 1222 && coin_tier <= 4) {
				//eputs("25 CENT\r\n");
				coin_tier = 4;
			}
			else if (freq_diff > 927 && coin_tier <= 3) {
				//eputs("25 CENT\r\n");
				coin_tier = 3;
			}
			else if (freq_diff > 462 && coin_tier <= 2) {
				//eputs("10 CENT\r\n");
				coin_tier = 2;
			}
			else {
				//eputs("UNKNOWN COIN DETECTED\r\n");
				coin_tier = 1;
			}
			//waitms(5);

		}
		///COIN TYPE BROADCAST///
		if (autoflag == 0) {
			if (coin_radio_count > 0) {
				coin_radio_count--;
			}
			else {
				coin_radio_count = 50;
				if (coin_tier > 5) {
					eputs("Transmitted 1 DOLLAR\r\n");
					sprintf(buff, "!DO", cnt);
					//cnt++;
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else if (coin_tier > 4) {
					eputs("Transmitted 2 DOLLAR\r\n");
					sprintf(buff, "!TW", cnt);
					//cnt++;
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else if (coin_tier > 3) {
					eputs("Transmitted 5 CENT\r\n");
					sprintf(buff, "!NI", cnt);
					//cnt++;
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else if (coin_tier > 2) {
					eputs("Transmitted 25 CENT\r\n");
					sprintf(buff, "!QU", cnt);
					//cnt++;
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else if (coin_tier > 1) {
					eputs("Transmitted 10 CENT\r\n");
					sprintf(buff, "!DI", cnt);
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else if (coin_tier > 0) {
					//eputs("Transmitted Mystery Coin\r\n");
					sprintf(buff, "!MY", cnt);
					waitms(5); // short delay for reliability
					sendstr1(buff); // send to JDY-40
				}
				else {
					eputs("No Transmission\r\n");
				}
				//Reset Coin Tiers at end of Transmission
				coin_tier = 0;
			}
			
			
		}

		////

		//
		// ----  (B) JDY-40 SLAVE TASKS  ----
		//
		// Check if we received anything from master (via UART1)
		if (RXU1()) // Something from the master?
		{
			c = getchar1(); // header to recieve correct signal
			if (c == '!') {
				getstr1(buff3, sizeof(buff3) - 1);
				eputs(buff3);
				if (strcmp(buff3, "TO") == 0 || strcmp(buff3, "!T") == 0) {

					if (autoflag == 0) {
						sprintf(buff, "!TD");
						waitms(5); // short delay for reliability
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40

					}
					else if (autoflag == 1) {
						sprintf(buff, "!TF");
						waitms(5); // short delay for reliability
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
						sendstr1(buff); // send to JDY-40
					}
					autoflag ^= 1;

				}
			}
			///AUTOMATIC MODE///
			if (autoflag) {
				if (coinflag) {
					coin_count++;
					servos_magnet();

				}
				else {
					if (turn_flag) {
						if (turning_count >= turning_time) {
							move_robot("ST");
							turn_flag = 0;
							turning_count = 0;
						}
						else {
							move_robot("FL");
							turning_count++;
						}
					}
					else {
						if (perimeter_flag) {
							turn_flag = 1;
							temp = 900;
							while (temp > 0) {
								temp--;
								move_robot("BN");

							}
							move_robot("FL");
							turning_time = rand() % 15 + 3;  //8

						}
						else {
							move_robot("FN");
						}
					}
				}
			}
			///Manual Mode///
			else {
				if (strcmp(buff3, "PU") == 0  || strcmp(buff3,"!P") == 0) {
					servos_magnet();
					strcpy(buff3, " ");
				}
				else
				{
					move_robot(buff3);
				}
			}
			if (coin_count >= 20) {
				autoflag = 0;
				coin_count = 0;
				//send signal to controller that toggle went off
				sprintf(buff, "!TF");
		
				waitms(5); // short delay for reliability
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				sendstr1(buff); // send to JDY-40
				
				
			}

			// if(strcmp(buff3,"!!")==0)
		   //  {
				 // Master sent a message
			   //  getstr1(buff,sizeof(buff)-1);
			   //  if(strlen(buff)==7)
				// {
				 //    printf("Master says: %s\r\n", buff);
				// }++++++++++++++++
				// else
			   //  {
				//     printf("*** BAD MESSAGE ***: %s\r\n", buff);
			   //  }
			// }
	  // else if(c=='@')
	 //  {
		   // Master wants data from slave
		//   sprintf(buff, "%05u\n", cnt);
		//   cnt++;
		//   waitms(5); // short delay for reliability
		 //  sendstr1(buff); // send to JDY-40
	   //}
		}
		//	waitms(200);
			// Finally, wait a little while before repeating
			// (You could reduce this delay or split tasks for more responsiveness)
	} // end while(1)


}
