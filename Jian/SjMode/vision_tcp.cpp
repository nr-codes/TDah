#include "vision_tcp.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>



#define DEFAULT_PORT  3490 // 3100 - qnx to matlab, 3490 - vision to qnx
#define SERVER_IP "192.168.1.65" //qnx  192.168.1.65; host pc 192.168.1.111

VisionTCP::VisionTCP() 
{
	mInitialized = false;
}

VisionTCP::~VisionTCP()
{
	closesocket(mSessionSocket);
}

int VisionTCP::Init() 
{
	// 0. Initilize; Windows specific
	WSADATA wsaData;
	if( WSAStartup(MAKEWORD(2,2),&wsaData ) == SOCKET_ERROR ) {
		fprintf(stderr,"WSAStartup failed with error %d\n",WSAGetLastError());
		WSACleanup();
		return -91;
	}
	
	//1. Socket Creation
	mSessionSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (mSessionSocket == SOCKET_ERROR) {
		fprintf(stderr, "The socket is not created!\n");
		WSACleanup();
		return -92;
	}
	//1.1 Socket Option	
	BOOL opt = 1;
    if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char*)&opt, sizeof(BOOL)) == SOCKET_ERROR) {
		fprintf(stderr,"setsockopt failed with error %d\n",WSAGetLastError());
		WSACleanup();
		return -93;
	}
	// By this setting, send() takes A LOT OF time. e.g., ~200 ms
	/*int bufSize = 0; 
	if (setsockopt(mSessionSocket, SOL_SOCKET, SO_SNDBUF, (char*)&bufSize, sizeof(int)) == SOCKET_ERROR) {
		fprintf(stderr,"setsockopt failed with error %d\n",WSAGetLastError());
		WSACleanup();
		return -94;
	}
    */

	//printf("Client: A socket is created.\n");

	//2. Connect
	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_port = htons(DEFAULT_PORT); // Port MUST be in Network Byte Order
	mServerAddr.sin_addr.s_addr = inet_addr(SERVER_IP); // INADDR_ANY;
	if ( connect(mSessionSocket, (SOCKADDR *)&mServerAddr, sizeof(mServerAddr) ) == SOCKET_ERROR ) {
		fprintf(stderr, "The connection failed!\n");
		WSACleanup();
		return -95;
	}
	printf("Connected to the qnx server.\n");
	
	mInitialized = true;
	return 0;
}

int VisionTCP::Send(int ROI, double x, double y) 
{
	if (!mInitialized) {
		printf("The TCP/IP connection is not initialized!\n");
		return -1;
	}

	// put the data to the buffer
	double dROI = (double)ROI;

	memcpy(&mbuf[0], &dROI, 8);
	memcpy(&mbuf[8], &x, 8);
	memcpy(&mbuf[16], &y, 8);
	
	//char testBuf[1460] = {1};

	// send the data
	int n = send(mSessionSocket, mbuf, 24, 0); //sizeof(mbuf), 0);
	if (n == SOCKET_ERROR) { 
		mInitialized = false;
		closesocket(mSessionSocket);
		WSACleanup();
		
		// It seems like when the connection from server is disconnected, WASGetLastError() gives 0.
		printf("The function send() failed with error %d, so I'm closing the connection socket now.\n", WSAGetLastError());
		return n;
	}

	mROI = ROI; mx = x; my = y;
	return 1;
}