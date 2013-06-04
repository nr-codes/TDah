#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>

#include <signal.h>

#include "MatlabNet.h"
#include "FifoQ.h"

#define PORT 3100    // TCP/IP port

#define SAMPLE_RATE_DIVISOR	1

MatlabNet::MatlabNet() : AperiodicTask(){
	mInitialized = 0;
	divisorCount = 0;
}

MatlabNet::~MatlabNet(){

}

// called ine SampleLoopTask::Init()
void MatlabNet::AddSignal(int ch, double *val){

	if(!mInitialized && (ch <= MATLAB_NET_NUM_CH-1)){
		// record pointer in array
		mpValPtrArr[ch] = val;
	}
}

int MatlabNet::TcpIpInit(){

	mSocket = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket < 0){
		printf("MatlabNet:tcpIpInit: error opening socket\n");
		return -1;
	}

	int opt = 1;
	if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0){
      printf("MatlabNet:tcpIpInit: error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return -1;
	}


	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_addr.s_addr = INADDR_ANY;
	mServerAddr.sin_port = htons(PORT);
	memset(&(mServerAddr.sin_zero), '\0', 8);

	if(bind(mSocket, (struct sockaddr *)&mServerAddr, sizeof(struct sockaddr)) < 0){
		printf("MatlabNet:tcpIpInit: error binding socket: %s\n", strerror(errno));
		return -1;
	}

	if(listen(mSocket, 2) == -1){
		printf("MatlabNet:tcpIpInit: listen failed\n");
		return -1;
	}

	// register an empty signal handler for SIGPIPE
	// to pervent exiting upong client disconnect
	struct sigaction act;

	act.sa_handler = &sig_handler;
	act.sa_flags = 0;
	sigaction(SIGPIPE, &act, NULL);

	return 1;
}

int MatlabNet::Init(double rate, int priority){

	mInitialized = 1;
	mSampleRate = rate;

	mpFifo = new FifoQ(sizeof(double) * MATLAB_NET_NUM_CH, 3 , FifoQ::OVERWRITE);

	// Initialize TCP/IP
	if(TcpIpInit() == -1){
		return -1;
	}

	// Start the thread
	AperiodicTask::Init("MatlabNet Task", priority);
}

void MatlabNet::Process(){
	// check to see if connection has been established
	if(validSession && mInitialized){
		divisorCount++;
		if(divisorCount == SAMPLE_RATE_DIVISOR){
			divisorCount = 0;
			// copy values into buffer
			for(int i=0; i < MATLAB_NET_NUM_CH; i++){
				mpValBuf[i] = *mpValPtrArr[i];
			}
			
			// put buffer into thread communication fifo
			mpFifo->Put(mpValBuf);
			
			// trigger the server to read buffer and send data
			Trigger(0);
		}
	}
}

void MatlabNet::Task(){
	int bytes;

	while(1){
		validSession = 0;
		mSessionSocket = accept(mSocket, 0, 0);
		printf("\nMatlabNet:Task: accepted socket\n");
		if(mSessionSocket == -1){
			printf("MatlabNet:Task: error accepting socket\n");
			continue;
		}
		else{
			int opt = 1;
  			if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0){
    			printf("MatlabNet:Task: Error setting socket option for TCP_NODELAY using IPPROTO\n");
				close(mSessionSocket);
				continue;
			}
		}

		validSession = 1;

		mpFifo->Reset();
		while(1){
			//break;
			// wait for trigger to send signals
			if(AperiodicTask::TriggerWait() == -1){
				continue;
			}

			// pull signals from fifo
			if(mpFifo->Get(&mBuf[0])!=-1){
				bytes = 8*MATLAB_NET_NUM_CH;
				if(send(mSessionSocket, mBuf, bytes, 0) == -1){
					//printf("MatlabNet:Task: Unable to send\n");
					close(mSessionSocket);
					break;
				}
			}

		} // while
	}
}



