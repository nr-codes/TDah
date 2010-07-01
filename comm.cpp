// WinCommExample.cpp : Defines the entry point for the console application.
//

#include <windows.h>
#include <conio.h>
#include "fcdynamic.h"

#define RESPONSE 0
#define DROPPED_PACKETS 0

#define BYTES_TO_SEND 17
#define BYTES_ASSUMED 4
#define DEFAULT_PORT TEXT("COM4")

static HANDLE hComm;
static unsigned char info[BYTES_TO_SEND];
static int test_comm(char *test);

int open_comm(LPCTSTR port)
{
	if(port == NULL) {
		port = DEFAULT_PORT;
	}

	hComm = CreateFile( port,
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

	dcb.BaudRate = 111111;//CBR_115200;
	dcb.fParity = FALSE;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	if(SetCommState(hComm, &dcb) == 0) {
		CloseHandle(hComm);
		return -4;
	}

	return 0;
}

int write_comm(char roi, float x, float y, unsigned int timestamp)
{
	DWORD bytes;

	assert(sizeof(float) == BYTES_ASSUMED && 
		sizeof(unsigned int) == BYTES_ASSUMED);

	info[0] = 'R';
	info[1] = (unsigned char) roi;
	info[2] = 'X';
	memcpy(info + 3, &x, sizeof(float));
	info[7] = 'Y';
	memcpy(info + 8, &y, sizeof(float));
	info[12] = 'T';
	memcpy(info + 13, &timestamp, sizeof(unsigned int));


	if(WriteFile(hComm, info, BYTES_TO_SEND, &bytes, NULL) == FALSE) {
		return -5;
	}

#if DROPPED_PACKETS || RESPONSE
	test_comm(packet);
#endif

	return 0;
}

int write_comm(char *packet, int size)
{
	DWORD bytes;
	return !WriteFile(hComm, packet, size, &bytes, NULL);
}

int close_comm()
{
	CloseHandle(hComm);
	return 0;
}

#if RESPONSE || DROPPED_PACKETS
#define BYTES_TO_RX 59
static int test_comm(char *test)
{
	static unsigned int packet = 0;
	DWORD bytes = 0;
	COMMTIMEOUTS timeout;
	char reply[BYTES_TO_RX];

	// if one of these aren't set, then read returns immediately
	timeout.ReadIntervalTimeout = 1000;
	timeout.ReadTotalTimeoutConstant = 1000;
	timeout.ReadTotalTimeoutMultiplier = 1000;

	SetCommTimeouts(hComm, &timeout);

		// send test buffer
#if DROPPED_PACKETS
	// assumes sizeof(unsigned int) == 4
	ts++;
	memcpy(&test[13], &ts, sizeof(unsigned int));
	ts++;
	memcpy(&test[30], &ts, sizeof(unsigned int));
#endif

#if RESPONSE
	// get response
	memset(reply, 0, BYTES_TO_RX);
	if(ReadFile(hComm, reply, BYTES_TO_RX, &bytes, NULL) == FALSE) {
		printf("error reading from comm port\n");
		return -1;
	}

	packet++;
	if(bytes == 0) {
		printf("no bytes sent\n");
		return -2;
	}
	
	printf("bytes sent: %d\n", bytes);
	printf("P - %u: %s\n", packet, reply);
#endif

	return 0;
}
#endif