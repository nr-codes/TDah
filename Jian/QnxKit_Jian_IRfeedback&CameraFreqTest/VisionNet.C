#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ioctl.h>
#include <sys/time.h>

#include <signal.h>

#include "VisionNet.h"
#include "SampleLoopTask.h"
#include "FifoQ.h"

#define VPORT 3490    // TCP/IP port 

#define SAMPLE_RATE_DIVISOR	1

//extern MatlabNet * MNET;
//extern IoHardware * HW;
//extern char LEDtoggle;


VisionNet::VisionNet() : AperiodicTask(){
	mInitialized = 0;
	divisorCount = 0;
	//mIsSocketAlive = 0;
}

VisionNet::~VisionNet(){

}

// called in SampleLoopTask::Init()
void VisionNet::AddSignal(int ch, double *val){

	if(!mInitialized && (ch <= VISION_NET_NUM_CH-1)){
		// record pointer in array
		mpValPtrArr[ch] = val;
		//printf("\n#%d linked!\n", ch);
	}
}


int VisionNet::TcpIpInit(){

	// 1. socket creation
	mSocket = socket(AF_INET, SOCK_STREAM, 0);

	if(mSocket < 0){
		printf("VisionNet:tcpIpInit: error opening socket\n");
		return -1;
	}
	// 1.1 socket option
	int opt = 1;
	if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0){
      printf("VisionNet:tcpIpInit: error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return -1;
	}

	// 2. binding
	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_addr.s_addr = INADDR_ANY;
	mServerAddr.sin_port = htons(VPORT);
	memset(&(mServerAddr.sin_zero), '\0', 8);
	if(bind(mSocket, (struct sockaddr *)&mServerAddr, sizeof(struct sockaddr)) < 0){
		printf("VisionNet:tcpIpInit: error binding socket: %s\n", strerror(errno));
		return -1;
	}
	
	// 3. listen
	if(listen(mSocket, 2) == -1){
		printf("VisionNet:tcpIpInit: listen failed\n");
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

int VisionNet::Init(double rate, int priority){

	mInitialized = 1;
	mSampleRate = rate;

	IFDataRecv = 0;

	printf("\nVisionNet init\n");

	//mpFifo = new FifoQ(sizeof(double) * MATLAB_NET_NUM_CH, 3 , FifoQ::OVERWRITE);

	// Initialize TCP/IP
	if(TcpIpInit() == -1){
		return -1;
	}


	// Start the thread
	AperiodicTask::Init((char *)"VisionNet Task", priority);
}

int VisionNet::Recv() {
	// When this socket is non-blocking by option setting, if there's no data recv() will return -1 (error).

	//printf("\nRecv function called.\n");

	int n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
	if (n > 0) {
		printf("%d bytes received!\n", n);
		for(int i=0; i < VISION_NET_NUM_CH; i++){
			memcpy(mpValPtrArr[i], &mBuf[8*i], 8);
		}
		printf("\n%f %f %f\n", *mpValPtrArr[0],*mpValPtrArr[1],*mpValPtrArr[2]);
	}
	else if (n == 0) { // socket is disconnected.
		//printf("VisionNet: The connection is closed.\n");
		close(mSessionSocket);
		//mIsSocketAlive = 0;
		Trigger(0); // wait a new connection from vision. /// data initialization?
	}
	return n;
}

void VisionNet::Process(){

	//printf("\nVisionNet Process.\n");

	// check to see if connection has been established
	if(validSession && mInitialized){
		divisorCount++;
		if(divisorCount == SAMPLE_RATE_DIVISOR){
			divisorCount = 0;

			/// not much thing to do here when receiving data.
			/* copy values into buffer
			for(int i=0; i < MATLAB_NET_NUM_CH; i++){
				mpValBuf[i] = *mpValPtrArr[i];
			}
			
			//put buffer into thread communication fifo
			mpFifo->Put(mpValBuf);
			*/

			// trigger the server to read buffer and send data
			Trigger(0);

		}
	}
}

void VisionNet::Task(){
	int bytes;
	double pre_cnt = 0.;

	printf("\nVisionNet Task.\n");

	while(1){
		validSession = 0;
		// 4. accept
		mSessionSocket = accept(mSocket, 0, 0);
		if(mSessionSocket == -1){ 
			printf("VisionNet: error accepting socket\n");
			continue;
		}
		else{
			// not necessary for receiving only 
			/*int opt = 1;
  			if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0){
    			printf("VisionNet: error setting socket option for TCP_NODELAY using IPPROTO\n");
				close(mSessionSocket);
				continue;
			}*/ 
			// set the socket as non-blocking, otherwise the recv function will wait until data arrives.
			// this setting is extremely important!
			/*int on = 1;
			if (ioctl(mSessionSocket, FIONBIO, &on) < 0) { // making the socket nonblocking
				printf("TVisionNet:Task: Error setting socket nonblocking\n");
				close(mSessionSocket);
				continue;
			}*/
		}
		
		printf("\nVisionNet accepted the connection from the vision system\n");
		validSession = 1;
		//mIsSocketAlive = 1;

		//mpFifo->Reset();
		while(1){
	
			// wait for trigger to receive signals
			if(AperiodicTask::TriggerWait() == -1){
				continue;
			}
			
			int n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
			//printf("%d \n", n);
			if (n > 0) {
				//printf("%d bytes received!\n", n);
				
				for(int i=0; i < VISION_NET_NUM_CH; i++){
					memcpy(mpValPtrArr[i], &mBuf[8*i], 8);
				}
				
				VNET->IFDataRecv = 1;
				VNET->n_tcp = n;	
										
					//VNET->Process();
				//}
			}
			else if (n == 0) { // socket is disconnected.
				printf("VisionNet: The connection is closed.\n");
				close(mSessionSocket);
				//mIsSocketAlive = 0;
				break;
			}
		} // while
	}
}


