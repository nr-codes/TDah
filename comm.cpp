// WinCommExample.cpp : Defines the entry point for the console application.
//

#include <windows.h>
#include <conio.h>
#include "fcdynamic.h"

#define BYTES_TO_SEND 17
#define DEFAULT_PORT TEXT("COM4")

static HANDLE hComm;
static unsigned char info[BYTES_TO_SEND];

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

	dcb.BaudRate = CBR_115200;
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

int write_comm(int roi, float xx, float yy, unsigned int timestamp)
{
	DWORD bytes;
	int x = *((int *) &xx);
	int y = *((int *) &yy);

	/*
	int xc, yc;


	xc = win->roi_xoff + cvRound((win->blob_xmin + win->blob_xmax) / 2);
	yc = win->roi_yoff + cvRound((win->blob_ymin + win->blob_ymax) / 2);
	*/

	info[0] = 'R';
	info[1] = (unsigned char) roi;
	info[2] = 'X';
	info[3] = (unsigned char) (x >> 24);
	info[4] = (unsigned char) (x >> 16);
	info[5] = (unsigned char) (x >> 8);
	info[6] = (unsigned char) x;
	info[7] = 'Y';
	info[8] = (unsigned char) (y >> 24);
	info[9] = (unsigned char) (y >> 16);
	info[10] = (unsigned char) (y >> 8);
	info[11] = (unsigned char) y;
	info[12] = 'T';
	info[13] = (unsigned char) (timestamp >> 24);
	info[14] = (unsigned char) (timestamp >> 16);
	info[15] = (unsigned char) (timestamp >> 8);
	info[16] = (unsigned char) timestamp;

/*
	info[0] = 'R';
	info[1] = 0xf1;
	info[2] = 0xe2;
	info[3] = 0xe3;
	info[4] = 0xd4;
	info[5] = 0xd5;
	info[6] = 0xe6;
	info[7] = 0xa7;
	info[8] = 0xd8;
	info[9] = 0xb9;
	info[10] = 0xea;
	info[11] = 0xeb;
	info[12] = 0xfc;
*/

	for(int i = 0; i < BYTES_TO_SEND; i++) {
		printf("%x ", info[i]);
	}
	printf("\n");

	if(WriteFile(hComm, info, BYTES_TO_SEND, &bytes, NULL) == FALSE) {
		return -5;
	}

	return 0;
}

int close_comm()
{
	CloseHandle(hComm);
	return 0;
}