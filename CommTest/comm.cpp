#include <windows.h>
#include <conio.h>
#include <stdio.h>

#define BYTES_TO_SEND 34
#define BYTES_TO_RX 100

#define RESPONSE 1
#define DROPPED_PACKETS 0

// roi 5: x = 123.456, y = 789.0123, timestamp = 987654321
// roi 9: x = 987.6, y = 54.321, timestamp = 1234567890;
unsigned char test[BYTES_TO_SEND] = {
	'R', 0x5, 'X', 0x79, 0xe9, 0xf6, 0x42,
	'Y', 0xca, 0x40,  0x45, 0x44, 'T', 0xb1, 0x68, 0xde, 0x3a,
	'R', 0x9, 'X', 0x66, 0xe6, 0x76, 0x44, 
	'Y', 0xb4, 0x48, 0x59, 0x42, 'T', 0xd2, 0x2, 0x96, 0x49};

char reply[BYTES_TO_RX];

static HANDLE hComm;
static unsigned char info[BYTES_TO_SEND];

int open_comm()
{
	hComm = CreateFile( TEXT("\\\\.\\COM11"),
                    GENERIC_READ | GENERIC_WRITE, 
                    0, 
                    0, 
                    OPEN_EXISTING,
                    0,
                    0);

	if (hComm == INVALID_HANDLE_VALUE) {
		return -1;
	}

	DCB dcb = {0};
	if(GetCommState(hComm, &dcb) == 0) {
		CloseHandle(hComm);
		return -3;
	}

	dcb.BaudRate = CBR_115200;
	dcb.fParity = FALSE;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	if(SetCommState(hComm, &dcb) == 0) {
		CloseHandle(hComm);
		return -4;
	}

	return 0;
}

int close_comm()
{
	CloseHandle(hComm);
	return 0;
}

int main(void)
{
	unsigned int packet = 0, ts = 2;
	DWORD bytes = 0;
	COMMTIMEOUTS timeout;

	open_comm();

	// if one of these aren't set, then read returns immediately
	timeout.ReadIntervalTimeout = 1000;
	timeout.ReadTotalTimeoutConstant = 1000;
	timeout.ReadTotalTimeoutMultiplier = 1000;

	SetCommTimeouts(hComm, &timeout);

	while( !(_kbhit() && _getch() == 'q')) {
		// send test buffer
#if DROPPED_PACKETS
		// assumes sizeof(unsigned int) == 4
		ts++;
		memcpy(&test[13], &ts, sizeof(unsigned int));
		ts++;
		memcpy(&test[30], &ts, sizeof(unsigned int));
#endif

		if(WriteFile(hComm, test, BYTES_TO_SEND, &bytes, NULL) == FALSE) {
			printf("error writing to comm port\n");
		}

#if RESPONSE
		// get response
		memset(reply, 0, BYTES_TO_RX);
		if(ReadFile(hComm, reply, BYTES_TO_RX, &bytes, NULL) == FALSE) {
			printf("error reading from comm port\n");
			break;
		}

		packet++;
		if(bytes == 0) {
			printf("no bytes sent\n");
			break;
		}
		
		printf("bytes sent: %d\n", bytes);
		printf("%u: %s\n", packet, reply);

		// did an error occur?
		if(reply[0] == '!') {
			printf("error PIC detected wrong data with package\n");
			break;
		}
#endif
	}

	close_comm();

	return 0;
}
