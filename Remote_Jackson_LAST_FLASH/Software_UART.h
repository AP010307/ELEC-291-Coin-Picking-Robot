#define SOFTWARE_UART_BAUD 9600L
#define OCR1_RELOAD ((F_CPU/(SOFTWARE_UART_BAUD*8L))+1)
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

extern volatile unsigned char RXD_state, TXD_state;
extern volatile unsigned char RXD_FLAG, RXD_SR, RXD_DATA;
extern volatile unsigned char TXD_DATA;

void Reset_Software_Uart (void);
void Init_Software_Uart(void);
void SendByte1 (unsigned char c);
void SendString1(char * s);
unsigned char GetByte1 (void);
void GetString1(char * s, int nmax);
void GetString1_timeout(char * s, int nmax);


