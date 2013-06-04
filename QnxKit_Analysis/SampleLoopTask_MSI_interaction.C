// For control thetah, actually thatao
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

const double M_H = 0.4745; // mass of the hand
const double M_O = 0.142; // mass of the object
const double RHO_H = 0.15; //0.15; // radius of the hand disk
const double RHO_HI = 0.0095; // inner radius of the hand
const double RHO_O = 0.081; // radius of the object disk + thickness of the rubber band
const double GRAV = 9.8 * sin(3.142592/180*78.3); //77.6); //85.2); //82.09//45.8); // gravitational acceleration
const double K_R = (1.0/RHO_H + 1.0/RHO_O);
const double I_H = 0.5 * M_H * (RHO_H*RHO_H + RHO_HI*RHO_HI) + INERTIA_MOTOR;
const double I_O = 0.5 * M_O * RHO_O*RHO_O;
const double M11 = I_H + M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O) + I_O;
const double M12 = M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O)/RHO_H + I_O*K_R;
const double M22 = K_R*K_R*(M_O*RHO_O*RHO_O + I_O);
// naming changes according to the notation used in my paper.
const double COEFF_sig1 = M_O*GRAV*(RHO_H+RHO_O)/RHO_H;
const double COEFF_sig2 = 1/(RHO_H*M22);
const double COEFF_sig3 = 1 - M12/(RHO_H*M22);

//const double OBJ_X_OFFSET = 0.102447662745 + 0.003010220603630; // in m. It may be better to get this value from constant speed control
//const double OBJ_X_OFFSET = 0.102447662745 + 0.003010220603630 - 0.0032571 + 0.0035112; // in m. It may be better to get this value from constant speed control
//const double OBJ_X_OFFSET = 0.1065958 - 0.0028644;
//const double OBJ_Y_OFFSET = -0.0894877;  // in m. currently not use this.
//const double OBJ_X_OFFSET = 0.08009; // in m. It may be better to get this value from constant speed control
const double OBJ_X_OFFSET = 0.081437; // in m. It may be better to get this value from constant speed control
const double OBJ_Y_OFFSET = 0.03346; // in m. currently not use this.
const double TARGET_THETAHD = -3.0; // in radian/sec

// target angular velocity of the object
const double TARGET_THETAOD = 3.0; // in radian/sec. curretly not use this.

// repeated root of -4
#define GAIN_K1			(16.0) 
#define GAIN_K2			(96.0) 
#define GAIN_K3			(256.0)  
#define GAIN_K4			(256.0) 

double num[MATLAB_NET_NUM_CH];
double vnum[VISION_NET_NUM_CH];
char signame[50];
char err_msg[20];

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
	Kp = 350; //150.0; //150.0; //0.2; //200; //20; //300.;
	Kd = 1.0; //0.9; //0.01; //10; //50; //10.;
	Ki = 0.0;
	
	pCmd = 0.;
	iCmd = 0.;
	currentI = 0.;
	
	// initialization
	enc = 0;
	pos = 0;
	vel = 0;
	vCmd = 0;
	aCmd = 0;
	iCmd = 0;
	ampVCmd = 0;
	handTheta = 0;
	handTheta_prev = 0;
	handVel = 0;
	handVel_prev = 0;
	sh = 0;  
	shD = 0;
	eta1 = 0;
	eta2 = 0;
    alpha_eta = 0; 
	DalphaDeta1 = 0;
	DalphaDeta2 = 0;
	vInp = 0; 
	
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
		
	double analogIn = 0;
	while(!lost){
		TimerWait();
		
		ncycles = ClockCycles();
		sec=(double)(ncycles - ncycles_prev)/cps;
		ncycles_prev = ncycles;
		
		TimingProcess();
		
		// Read Inputs
		HW->ProcessInput(); // all digital & analog, encoders reading.
		
		analogIn = HW->ReadAnalogCh(IoHardware::AIN_0) - 2.5; // reading value range 0 to 5 V, so for 2.5 to be centered.
		
		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) {
			iCmd = analogIn /2.5*.5; // maximum allowed current 0.5 A  
		}
		else {
			iCmd = 0;
		}

		//limit current based on motor specs
		if(iCmd > (MAX_CURRENT_MA) ){ // 1.6 mA
			iCmd = MAX_CURRENT_MA;
		}
		else if(iCmd < (-MAX_CURRENT_MA) ){
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
		//*******************************************************

		//**************************************************
	
		HW->ProcessOutput(); // all digital & analog writing.

		// set outputs
		/*num[0] = obj.theta; 
		num[1] = obj.angVel;
		*/
		num[0] = analogIn; //correctedAcc; //eta1; //handVel; //pos; //handTheta; //pos; // angle in rad
		num[1] = eta1; //feedforwardVal; //pCmd; //eta2; //obj.theta; //vnum[0]; //obj.theta; //vel; //obj.theta;
		
		num[2] = pos; //feedbackVal; //vCmd; //obj.angVel; //obj.x; //vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = eta2; //handTheta; //iCmd; //sec; //elapsedSec; //sec; //vnum[2]; //obj.y1;
		
		MNET->Process();

	} // while()
	
	// for some reason, this does not work.
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
	
}

