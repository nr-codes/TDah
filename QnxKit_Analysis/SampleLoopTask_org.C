#include <stdio.h>
#include <math.h>
#include "SampleLoopTask.h"
#include <string.h>
#include <unistd.h> // for delay()

#include <netinet/in.h> // TCP/IP
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ioctl.h>
#include <sys/time.h>
#define PORT 3490


#include "main.h"
#include "motor.h"
#include "macros.h"

#define M_PER_VOLT		(0.03333) // scale factor for sending ROI positions
#define MM_PER_VOLT		(300/9.0)
#define DEG2RAD			(0.01745) // 1 deg

double num[MATLAB_NET_NUM_CH];
double vnum[VISION_NET_NUM_CH];
char signame[50];
char err_msg[20];

/// TCP/IP communication
static int mSocket;
static int mSessionSocket;
static struct sockaddr_in mServerAddr;

static void TCPIP_Init() {

// 1. socket creation
	mSocket = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket < 0){
		printf("Test:tcpIpInit: error opening socket\n");
		return;
	}
	// 1.1 socket option
	int opt = 1;
	if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0){
      printf("Test:tcpIpInit: error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return;
	}
	
	// 2. binding
	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_addr.s_addr = INADDR_ANY;
	mServerAddr.sin_port = htons(PORT);
	memset(&(mServerAddr.sin_zero), '\0', 8);
	if(bind(mSocket, (struct sockaddr *)&mServerAddr, sizeof(struct sockaddr)) < 0){
		printf("Test:tcpIpInit: error binding socket: %s\n", strerror(errno));
		return;
	}
	// 3. listen
	printf("client listening ready\n");
	if(listen(mSocket, 2) == -1){
		printf("Test:tcpIpInit: listen failed\n");
		return;
	}
	// 4. accept
	printf("client waiting\n");
	mSessionSocket = accept(mSocket, 0, 0);
	if(mSessionSocket == -1){
		printf("Test:Task: error accepting socket\n");
		return;
	}
	if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0){
    	printf("Test:Task: Error setting socket option for TCP_NODELAY using IPPROTO\n");
		close(mSessionSocket);
		return;
	}
	// set timeout on the socket for recv(). For some reason, making the socket nonblocking using ioctl() does not work.
	/*struct timeval timeout = {1.0, 0.0}; // {tv_sec, tv_usec} - for some reason, tv_sec must be >= 1!
	if (setsockopt(mSessionSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval)) < 0){
    	printf("Test:Task: Error setting socket option for timeout\n");
		close(mSessionSocket);
		return;
	}
	*/
	// set the socket as non-blocking, otherwise the recv function will wait until data arrives.
	// this setting is extremely important!
	int on = 1;
	if (ioctl(mSessionSocket, FIONBIO, &on) < 0) { // making the socket nonblocking
		printf("Test:Task: Error setting socket nonblocking\n");
		close(mSessionSocket);
		return;
	}
	// This is necessary. Otherwise, when the client tries to connect again, it won't show an error.
	// --> I don't close the socket since I wrote a code so that the clinet can reconnect.
	// close(mSocket);
	

	printf("client... accepted\n");
}

SampleLoopTask::SampleLoopTask() : PeriodicTask(){

}

SampleLoopTask::~SampleLoopTask(){

}

int SampleLoopTask::Init(char *name, double rate, double *actual_sample_rate, int priority){

	PeriodicTask::Init(name, rate, actual_sample_rate, priority);

	SampleRate = *actual_sample_rate;
	
	printf("actual sample rate %lf\n", *actual_sample_rate);
	
	for(int i=0; i<MATLAB_NET_NUM_CH; i++){
		MNET->AddSignal(i, &(num[i]));
	}
	for(int i=0; i<VISION_NET_NUM_CH; i++){
		VNET->AddSignal(i, &(vnum[i]));
	}
	
	// initialize gains
	Kp = 200; //20; //300.;
	Kd = 10; //50; //10.;
	Ki = 0.;
	
	pCmd = 0.;
	
	done = 0;
	
	ncycles_prev = 0;
	cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
	
	lp.go = 0;

}


int SampleLoopTask::TimingStart(double sec){
	
	lp.mu_prev = 1./SampleRate * 1.e6;
	lp.s_sq_prev = 0.;
	lp.std = 0.;
	
	lp.min = lp.max = 1./SampleRate * 1.e6;
	
	printf("%lf %lf\n", lp.min, lp.max);
	
	sem_init(&lp.sem, 1, 0);
	
	lp.numSamples = (int)(sec * SampleRate);
	lp.i = 1;
	lp.maxIndex = -1;
	lp.minIndex = -1;
	
	lp.go = 1;
	
	// block
	sem_wait(&lp.sem);
	
	printf("mean %lf us\n", lp.mu);
	printf("std %lf us\n", lp.std);
	printf("max %lf us at %d\n", lp.max, lp.maxIndex);
	printf("min %lf us at %d\n", lp.min, lp.minIndex);
	
	sem_destroy(&lp.sem);
}

int SampleLoopTask::TimingProcess(){

	if(lp.go && lp.i <= lp.numSamples){
		// get time
		lp.t = sec * 1.e6; // microseconds
		
		if(lp.t > lp.max){
			lp.max = lp.t;
			lp.maxIndex = lp.i;
		}
		
		if(lp.t < lp.min){
			lp.min = lp.t;
			lp.minIndex = lp.i;
		}
		
		lp.mu = lp.mu_prev + (lp.t - lp.mu_prev) / (((double)lp.i) + 1.);
		lp.s_sq = (1. - 1./((double)lp.i)) * lp.s_sq_prev + (((double)lp.i) + 1.) * (lp.mu - lp.mu_prev) * (lp.mu - lp.mu_prev);
		
		lp.mu_prev = lp.mu;
		lp.s_sq_prev = lp.s_sq;
		
		if(lp.i == lp.numSamples){
			lp.std = sqrt(lp.s_sq);
			lp.go = 0;
			// unblock;
			sem_post(&lp.sem);
			
		}
		
		lp.i++;
	}
}


void SampleLoopTask::Task(){
	int cnt;
	int region;
	int attempt;
	
	static int dout = 1;
	static int last_status = 0;
	static double pos_prev = 0;
	static double pCmd_prev = 0;
	static double vCmd_prev = 0;
	static double error = 0;
	
	int temp = 0;
	
	/// TCP/IP communication
	//int mSocket;
	//int mSessionSocket;
	//struct sockaddr_in mServerAddr;

	/*
	// 1. socket creation
	mSocket = socket(AF_INET, SOCK_STREAM, 0);
	if(mSocket < 0){
		printf("Test:tcpIpInit: error opening socket\n");
		return;
	}
	// 1.1 socket option
	int opt = 1;
	if(setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(int)) < 0){
      printf("Test:tcpIpInit: error setting socket option for SO_REUSEADDR using SOL_SOCKET\n");
		return;
	}
	
	// 2. binding
	mServerAddr.sin_family = AF_INET;
	mServerAddr.sin_addr.s_addr = INADDR_ANY;
	mServerAddr.sin_port = htons(PORT);
	memset(&(mServerAddr.sin_zero), '\0', 8);
	if(bind(mSocket, (struct sockaddr *)&mServerAddr, sizeof(struct sockaddr)) < 0){
		printf("Test:tcpIpInit: error binding socket: %s\n", strerror(errno));
		return;
	}
	// 3. listen
	printf("client listening ready\n");
	if(listen(mSocket, 2) == -1){
		printf("Test:tcpIpInit: listen failed\n");
		return;
	}
	// 4. accept
	printf("client waiting\n");
	mSessionSocket = accept(mSocket, 0, 0);
	if(mSessionSocket == -1){
		printf("Test:Task: error accepting socket\n");
		return;
	}
	if (setsockopt(mSessionSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&opt, sizeof(int)) < 0){
    	printf("Test:Task: Error setting socket option for TCP_NODELAY using IPPROTO\n");
		close(mSessionSocket);
		return;
	}
	// set timeout on the socket for recv(). For some reason, making the socket nonblocking using ioctl() does not work.
	struct timeval timeout = {1.0, 0.0}; // {tv_sec, tv_usec} - for some reason, tv_sec must be >= 1!
	if (setsockopt(mSessionSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval)) < 0){
    	printf("Test:Task: Error setting socket option for timeout\n");
		close(mSessionSocket);
		return;
	}
	// set the socket as non-blocking, otherwise the recv function will wait until data arrives.
	// this setting is extremely important!
	int on = 1;
	if (ioctl(mSessionSocket, FIONBIO, &on) < 0) { // making the socket nonblocking
		printf("Test:Task: Error setting socket nonblocking\n");
		close(mSessionSocket);
		return;
	}
	// This is necessary. Otherwise, when the client tries to connect again, it won't show an error.
	// --> I don't close the socket since I wrote a code so that the clinet can reconnect.
	// close(mSocket);
	

	printf("client... accepted\n");
	*/
	
	TCPIP_Init(); // accept a connection from the client (vision)

	/// for test
	int matlab_data = 0; // counter for sending less data to Matlab.
	int mu_k = 0; // for test
	while(!lost){
		TimerWait();
		
		ncycles = ClockCycles();
		sec=(double)(ncycles - ncycles_prev)/cps;
		ncycles_prev = ncycles;
		
		TimingProcess();
		
		// Read Inputs
		HW->ProcessInput();
		
		// Get status of camera
		last_status = HW->ReadDigitalBit(IoHardware::FRAME_STATUS);
		
		// Send out pulse to trigger camera
		HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 1);
		HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 0);
		
		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) ++cnt;

		// square wave command
		/*if(cnt % (int)(SAMPLE_RATE) == 0){
			dout = !dout;
			
			if(dout){
				pCmd = 0.785398;
			}
			else{
				pCmd = -0.785398;
			}
			
			//HW->SetDigitalOutBit(IoHardware::DOUT_48, dout);
			//printf("Input: %d\n", HW->GetDigitalInBit(IoHardware::DIN_71));
			//printf("Potentiometer: %f\n", HW->GetAnalogIn(IoHardware::AIN_0));
			//printf("Error: %f\n", error);
		}*/
		
		// sinusoid command, angle position (radian) input.
		// 0.785398 - Q) where did this come from? A) 45 deg
		/*pCmd = 45*DEG2RAD*sin(2*3.141592*(cnt)/SAMPLE_RATE/1.0); // period of the input: 1 sec
		
		// controller
		enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
		
		pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		vel = (pos - pos_prev) * SAMPLE_RATE;
		error += (pos - pCmd);
		
		vCmd = (pCmd - pCmd_prev) * SAMPLE_RATE;
		aCmd = (vCmd - vCmd_prev) * SAMPLE_RATE;
		
		pos_prev = pos;
		pCmd_prev = pCmd;
		vCmd_prev = vCmd;
		
		iCmd = calcI(vCmd, aCmd); // feedforward control
		
		iCmd += (Kp * (pCmd - pos) - Kd * vel - Ki * error); // PID feedback control
		*/

		/// total test
		enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
		pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		vel = (pos - pos_prev) * SAMPLE_RATE;

		pCmd = .5*3.141592 * cnt/SAMPLE_RATE; // 4.5 2*3.141592
		vCmd = (pCmd - pCmd_prev) * SAMPLE_RATE;
		aCmd = (vCmd - vCmd_prev) * SAMPLE_RATE;
		
		error += (pos - pCmd);
		
		pos_prev = pos;
		pCmd_prev = pCmd;
		vCmd_prev = vCmd;
		
		iCmd = calcI(vCmd, aCmd); // feedforward control
		
		iCmd += (Kp * (pCmd - pos) - Kd * vel - Ki * error); // PID feedback control
		//iCmd += (Kp* (pCmd - pos) - Kd * (vCmd - vel) - Ki * error); // PID feedback control P, D, I = 300, 10, 0
		//iCmd += Kd * (vCmd - vel); // + Kd is correct?
		

			// limit current based on motor specs
		if(iCmd > MAX_CURRENT_MA){ // 1600 mA
			iCmd = MAX_CURRENT_MA;
		}
		else if(iCmd < -MAX_CURRENT_MA){
			iCmd = -MAX_CURRENT_MA;
		}
		
		ampVCmd =-iCmd * AMP_GAIN; // convert to analog signal. +-10 V.  -0.86 for 4.5 rad/s. for test use -0.8
		// sign change to agree with camera
		
		// output signal to amp
		if(!done){
			HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, ampVCmd);
			//HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.0);
		}
		else{
			HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.0);
		}
		
		/*num[0] = pCmd;
		num[1] = pos;
		num[2] = 0.;
		num[3] = ampVCmd;*/
		
		HW->ProcessOutput();
		
		if(camera) {// && !lost){
			// Wait for camera to process data, with timeout counter
			while(HW->ReadDigitalBit(IoHardware::FRAME_STATUS) == last_status){
				if(++attempt == (int)(1e8/SAMPLE_RATE)) {
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					FATAL_ERROR("Frame not received -");
					//strcpy(err_msg, "Frame not received -");
					lost = 1;
				}
			}
			attempt = 0;
			
			// Receive data from vision
			// vnum[0]: ROI, vnum[1]: x, vnum[2]: y
			//VNET->Process();
			
			// Read in analog voltages to get position/orientation data
			/*region = (int)round(HW->ReadAnalogCh(IoHardware::ROI)); // ROI = 1
			switch(region){
				case ROI_0:
					obj.x0 = HW->ReadAnalogCh(IoHardware::OBJ_X)*MM_PER_VOLT;
					obj.y0 = HW->ReadAnalogCh(IoHardware::OBJ_Y)*MM_PER_VOLT; // negative sign for sync vision coord. with the encoder.
					break;
				case ROI_1:
					obj.x1 = HW->ReadAnalogCh(IoHardware::OBJ_X)*MM_PER_VOLT;
					obj.y1 = HW->ReadAnalogCh(IoHardware::OBJ_Y)*MM_PER_VOLT;
					break;
				default:
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					FATAL_ERROR("Object lost -");
					//strcpy(err_msg, "Object lost -");
					lost = 1;
				
			}
			*/

			region = (int)vnum[0];
			switch(region){
				case 0: //ROI_0:
					obj.x0 = vnum[1];
					obj.y0 = vnum[2]; 
					break;
				case 1: //ROI_1:
					obj.x1 = vnum[1];
					obj.y1 = vnum[2];
					break;
				/*default:
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					printf("roi-vnum[0]:%d\n", region);
					FATAL_ERROR("Object lost -");
					//strcpy(err_msg, "Object lost -");
					lost = 1;
					*/
				
			}
			obj.x = (obj.x0 + obj.x1)/2;
			obj.y = (obj.y0 + obj.y1)/2;
			obj.theta = atan2(obj.y1-obj.y0, obj.x1-obj.x0);

			/// test
			char mBuf[2048] = {5};
			int n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
			int i = 0;
			//while (n < 0 && i++ < 10000) {
			//	n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
			//};
			if (n > 0) {
				//printf("%d bytes received!\n", n);
				for(int i=0; i < VISION_NET_NUM_CH; i++){
					memcpy(&vnum[i], &mBuf[8*i], 8);
				}
				//printf("received: %lf %lf %lf\n", mpValBuf[0], mpValBuf[1], mpValBuf[2]);
				vnum[1] = n;
				vnum[2] = 9;
			}
			else if (n == 0) { // socket is disconnected.
				//printf("VisionNet: The connection is closed.\n");
				camera = 0;
			}
			
		}
		else { // Because the vision system keeps sending the data, QNX must read them, otherwise the vision system will get a send error.
			//VNET->Process();
			char mBuf[2048] = {5};
			int n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
			int i = 0;
			/*while (n < 0) { // && i++ < 3) {
				n = recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
			};*/
			if (n > 0) {
				//printf("%d bytes received!\n", n);
				for(int i=0; i < VISION_NET_NUM_CH; i++){
					memcpy(&vnum[i], &mBuf[8*i], 8);
				}
				//printf("received: %lf %lf %lf\n", mpValBuf[0], mpValBuf[1], mpValBuf[2]);
				vnum[1] = n;
				vnum[2] = 7;
			}
			else if (n == 0) { // socket is disconnected.
				//printf("VisionNet: The connection is closed.\n");
				camera = 0;
			}
		}
		
		/// test
		//printf("1");
		//char mBuf[24] = {-9999};
		//double ch[4] = {-9999};
		//int n = 0; //recv(mSessionSocket, mBuf, sizeof(mBuf), 0);
		/*if( n < 0 ){
			FATAL_ERROR("recv error");
			close(mSessionSocket);
			return;
		}*/
		//printf("2");
		/*if (n>0) {
			//printf("received bytes:%d ", n);
			//FATAL_ERROR("recv error"); return;
			for (int i=0; i < 3; i++) {
					memcpy(&vnum[i], &mBuf[sizeof(double)*i], sizeof(double));

			}
			//printf("received:%lf", vnum[0]);
		}
		*/
		//VNET->Process();
		
		if ( pos > (mu_k+1)*2*3.141592 )
			mu_k++;

		pos = pos - mu_k*2*3.141592;
		
		// set outputs
		num[0] = pos; // angle in rad
		num[1] = vnum[0]; //obj.theta; //vel; //obj.theta;
		num[2] = vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = vnum[2]; //obj.y1;

		vnum[0] = 5;
		vnum[1] = 5;
		vnum[2] = 5;
		
		//if (matlab_data++ % 5 == 0) { // sending data at every other sampling time			
			// Send Outputs to MATLAB
			MNET->Process();
		//}
	} // while()
	
	// for some reason, this does not work.
	/*HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
	*/ 
}

