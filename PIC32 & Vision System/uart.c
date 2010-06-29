/*********************************************************************
 *
 *                  UART Interrupt Example
 *
 *********************************************************************
 **********************************************************************/
#include <HardwareProfile.h>

#define DESIRED_BAUDRATE    	(111111)      // The desired BaudRate

// define setup Configuration 1 for OpenUARTx
	// Module Enable 
	// Work in IDLE mode 
	// Communication through usual pins 
	// Disable wake-up 
	// Loop back disabled 
	// Input to Capture module from ICx pin 
	// no parity 8 bit 
	// 1 stop bit 
	// IRDA encoder and decoder disabled 
	// CTS and RTS pins are enabled 
	// UxRX idle state is '1' 
	// 16x baud clock - normal speed

#define config1 	UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | \
					UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | \
					UART_1STOPBIT | UART_IRDA_DIS | UART_NORMAL_RX | UART_BRGH_SIXTEEN 

// UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL

// define setup Configuration 2 for OpenUARTx
	// IrDA encoded UxTX idle state is '0'
	// Enable UxRX pin
	// Enable UxTX pin
	// Interrupt on transfer of every character to TSR 
	// Interrupt on every char received
	// Disable 9-bit address detect
	// Rx Buffer Over run status bit clear

#define config2		UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | \
					UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | \
					UART_RX_OVERRUN_CLEAR
//UART_INT_TX | 

#define NUM_ROI 2
#define ROI_PACKET_SIZE 17
#define TOTAL_PACKET_SIZE (NUM_ROI*ROI_PACKET_SIZE)

#define R_POS 0
#define R_VAL 'R'
#define X_POS 2
#define X_VAL 'X'
#define Y_POS 7
#define Y_VAL 'Y'
#define T_POS 12
#define T_VAL 'T'

#define PIN_D1 			LATDbits.LATD1
#define PIN_D2			LATDbits.LATD2
#define PIN_D3			LATDbits.LATD3
#define PIN_D4			LATDbits.LATD4

volatile struct {
	char id;
	float x;
	float y;
	unsigned int ts;
} rois[NUM_ROI];

// expected canned data for testing
char test_buf[TOTAL_PACKET_SIZE] = {
	'R', 0x5, 'X', 0x79, 0xe9, 0xf6, 0x42,
	'Y', 0xca, 0x40,  0x45, 0x44, 'T', 0xb1, 0x68, 0xde, 0x3a,
 	'R', 0x9, 'X', 0x66, 0xe6, 0x76, 0x44, 
	'Y', 0xb4, 0x48, 0x59, 0x42, 'T', 0xd2, 0x2, 0x96, 0x49};

#define CANNED_DATA 0
#define DROPPED_PACKET 0

// destination buffer
volatile char rcvbuf[TOTAL_PACKET_SIZE];
volatile unsigned int rcv = 0;

void test_code();

int main(void)
{
	int	pbClk;

	// Configure the system performance
	pbClk = SYSTEMConfigPerformance(SYS_FREQ);
	
	//Set D1, D2, D3, and D4 as a digital output
	LATD |= 0x001E; TRISD &= 0xFFE1;
	
	mInitAllLEDs();
	mLED_0_Off();
	mLED_1_Off();
	mLED_2_Off();
	mLED_3_Off();
	PIN_D1 = 0;
	PIN_D2 = 0;

	// Open UART2 with config1 and config2, U2RX = F4, U2TX = F5, U2CTS = F12, U2RTS = F13
	OpenUART2( config1, config2, pbClk/16/DESIRED_BAUDRATE-1);	// calculate actual BAUD generate value.
	
	// Open UART1 with config1 and config2, U2RX = F4, U2TX = F5, U2CTS = F12, U2RTS = F13
	OpenUART1( config1, config2, pbClk/16/DESIRED_BAUDRATE-1);	// calculate actual BAUD generate value.
		
	// Configure UART2 RX Interrupt with priority 2
	ConfigIntUART2(UART_INT_PR2 | UART_RX_INT_EN);
	
	// Must enable glocal interrupts - in this case, we are using multi-vector mode
    INTEnableSystemMultiVectoredInt();
	
	putsUART2("\r\n*** PIC32 Online (UART2) ***\r\n");	
	putsUART1("\r\n*** PIC32 Online (UART1) ***\r\n");		
	while(1);
   
   return 0;
}

// UART 2 interrupt handler
// it is set at priority level 2
#define SEND_BUF_SIZE 100
char sendbuf[SEND_BUF_SIZE];
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
	int i, offset;
	
	// Is this an RX interrupt?
	if(mU2RXGetIntFlag()) {
		PIN_D1 = !PIN_D1;	
		rcvbuf[rcv] = (char) ReadUART2();

		// wait for start of packet
		if(rcvbuf[R_POS] == R_VAL) {
			rcv++;
			mLED_2_On();
			PIN_D2 = 1;
		}

		// is packet correct so far?
		if((rcv-1 == X_POS && rcvbuf[rcv-1] != X_VAL) ||
			(rcv-1 == Y_POS && rcvbuf[rcv-1] != Y_VAL) ||
			(rcv-1 == T_POS && rcvbuf[rcv-1] != T_VAL) ||
			(rcv-1 == ROI_PACKET_SIZE+R_POS && rcvbuf[rcv-1] != R_VAL) ||
			(rcv-1 == ROI_PACKET_SIZE+X_POS && rcvbuf[rcv-1] != X_VAL) ||
			(rcv-1 == ROI_PACKET_SIZE+Y_POS && rcvbuf[rcv-1] != Y_VAL) ||
			(rcv-1 == ROI_PACKET_SIZE+T_POS && rcvbuf[rcv-1] != T_VAL)) {
			// assume bad packet
			rcv = 0;
			memset((char *) rcvbuf, 0, TOTAL_PACKET_SIZE);	
			
			mLED_2_Off();
			PIN_D2 = 0;
		}
		else if(rcv == TOTAL_PACKET_SIZE) {
			// assemble packet into (x,y) coordinates
			for(i = 0; i < NUM_ROI; i++) {
				offset = i*ROI_PACKET_SIZE;
				rois[i].id = rcvbuf[offset + 1];
				memcpy((char *)&(rois[i].x), 
					(char *) &(rcvbuf[offset+3]), sizeof(float));	
				memcpy((char *)&(rois[i].y), 
					(char *) &(rcvbuf[offset+8]), sizeof(float));
				memcpy((char *)&(rois[i].ts), 
					(char *) &(rcvbuf[offset+13]), sizeof(unsigned int));
			}
			
			test_code();

			rcv = 0;
			memset((char *) rcvbuf, 0, TOTAL_PACKET_SIZE);
			
			mLED_2_Off();
			PIN_D2 = 0;
		}
		
		mU2RXClearIntFlag();
	}

	// We don't care about TX interrupt
	if ( mU2TXGetIntFlag() ) {
		mU2TXClearIntFlag();
	}
}


void test_code()
{
#if  CANNED_DATA
	int i, offset;
	
	// were there any errors?
	if(memcmp((char *) (&(rcvbuf[R_POS])), test_buf, TOTAL_PACKET_SIZE)) {
		putcUART2('!');
	}

	// echo back data received			
	memset(sendbuf, 0, SEND_BUF_SIZE);
	offset = sprintf(sendbuf, " %d_%0.3f_%0.4f_%u * %d_%0.1f_%0.3f_%u ", 
		rois[0].id, rois[0].x, rois[0].y, rois[0].ts,
		rois[1].id, rois[1].x, rois[1].y, rois[1].ts);
	for(i = 0; i < offset; i++) {
		putcUART2(sendbuf[i]);
	}
#elif DROPPED_PACKET
	int i, offset;
	static unsigned int last_num = 0, dropped = 0;
	offset = 0;
	memset(sendbuf, 0, SEND_BUF_SIZE);
	for(i = 0; i < NUM_ROI; i++) {
		if(rois[i].ts - last_num != 1) {
			dropped++;
			offset += sprintf(sendbuf + offset, 
						"(%d) expected %u, got %u. dropped %u\n", 
						i, last_num+1, rois[i].ts, dropped);
			if(offset >= SEND_BUF_SIZE) {
				break;
			}	
		}
		last_num++;
	}

	if(offset) {
		for(i = 0; i < offset; i++) {
			putcUART2(sendbuf[i]);
		}
	}
	else {
		// send whatever to avoid timeout
		putcUART2('-');
	}
#endif
}	
