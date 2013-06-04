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
const double RHO_H = 0.15; // radius of the hand disk
const double RHO_HI = 0.0095; // inner radius of the hand
const double RHO_O = 0.08; // radius of the object disk
const double GRAV = 9.8 * sin(3.142592/180*6.6); // gravitational acceleration
const double K_R = (1.0/RHO_H + 1.0/RHO_O);
const double I_H = 0.5 * M_H * (RHO_H*RHO_H + RHO_HI*RHO_HI) + INERTIA_MOTOR;
const double I_O = 0.5 * M_O * RHO_O*RHO_O;
const double M11 = I_H + M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O) + I_O;
const double M12 = M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O)/RHO_H + I_O*K_R;
const double M22 = K_R*K_R*(M_O*RHO_O*RHO_O + I_O);
const double COEFF_A = 1/RHO_H/M22;
const double COEFF_B = RHO_H*M22/(M12-RHO_H*M22);
const double COEFF_D = M_O*GRAV*(RHO_H+RHO_O)/RHO_H;

const double COEFF_H_POS = (M11 - M12*M12/M22) / KM_POS;
const double COEFF_J_POS = (M_O * GRAV * (M12/M22*RHO_O*K_R - (RHO_H+RHO_O)) ) / KM_POS;
const double COEFF_MUD_POS = DYN_FRIC_POS / KM_POS;
const double COEFF_FS_POS = STAT_FRIC_POS / KM_POS;
const double COEFF_H_NEG = (M11 - M12*M12/M22) / KM_NEG;
const double COEFF_J_NEG = (M_O * GRAV * (M12/M22*RHO_O*K_R - (RHO_H+RHO_O)) ) / KM_NEG;
const double COEFF_MUD_NEG = DYN_FRIC_NEG / KM_NEG;
const double COEFF_FS_NEG = STAT_FRIC_NEG / KM_NEG;
	
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
	Kp = 0.16;//0.25; //0.2 //200; //20; //300.;
	Kd = 0;//0.01; //0.01 //10; //50; //10.;
	Ki = 0.015;//0.0012;
	
	pCmd = 0.;
	iCmd = 0.;
	currentI = 0.;
	
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
		
	int cnt = 0;
	int region;
	int attempt;
	
	static int dout = 1;
	static int last_status = 0;
	static double pos_prev = 0;
	static double pCmd_prev = 0;
	static double vCmd_prev = 0;
	static double error = 0;
	
	int temp = 0;
	
	int mu_k = 0; // for test
	int obj_ang_index = 0; // to keep continuous angle value when acrossing pi(or -pi).
	double obj_ang_offset = 0.0; // to subtract the object's angle at equilibrium position when inputting 'c' (camera)
	double obj_x_offset = 0;
	double obj_y_offset = 0;
	uint64_t ncycles1, ncycles2;
	//test
	double eta1_prev = 0;
	
	// for inner loop
	int innerLoopCycle = 10; 
	double targetVel = 0;
	double handAcc = 0;
	double handAcc_prev = 0;
	while(!lost){
		TimerWait();
		
		ncycles = ClockCycles();
		sec=(double)(ncycles - ncycles_prev)/cps;
		ncycles_prev = ncycles;
		
		TimingProcess();
		
		// Read Inputs
		HW->ProcessInput();
		
		// Get status of camera
		//last_status = HW->ReadDigitalBit(IoHardware::FRAME_STATUS);
		
		// Send out pulse to trigger camera
		//HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 1);
		//HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 0);
		
		// feedback
		enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
		pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		vel = (pos - pos_prev) * SAMPLE_RATE;

		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) cnt++;
		
		// constant speed input

		pCmd = .5*3.141592 * cnt/SAMPLE_RATE; // 4.5 2*3.141592
		
		// for constant speed controller
		vCmd = (pCmd - pCmd_prev) * SAMPLE_RATE;
		aCmd = (vCmd - vCmd_prev) * SAMPLE_RATE;
		//error += (pos - pCmd);

		error += (vel - vCmd); 
		if (error > 10.0) error = 10.0;
		else if (error < -10.0) error = -10.0;
		if (isnan(error)) error = 0.0;
				
		iCmd = calcI(vCmd, aCmd); // feedforward control
		//iCmd += ( Kp* (pCmd - pos) + Kd * (vCmd - vel) - Ki * error); // Ki is currently 0
		iCmd += ( Kp* (vCmd - vel) + Kd * (vCmd - vel) - Ki * error);
			
		pos_prev = pos;
		pCmd_prev = pCmd;
		vCmd_prev = vCmd;

		// for safety
		if (fabs(vel) > 4.5 ) {//rad/sec
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			printf("pCmd:%f, pos:%f, vCmd:%f, vel:%f, aCmd:%f ,iCmd:%f, error:%f\n", 
					pCmd, pos, vCmd, vel, aCmd, iCmd, error);
			FATAL_ERROR("MOTOR RUNS TOO FAST");
		}
			
		//limit current based on motor specs
		if(iCmd > MAX_CURRENT_MA){ // 1.6 A
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
		
		HW->ProcessOutput();

		// set outputs
		num[0] = pos; 
		num[1] = vel;
		num[2] = iCmd; 
		num[3] = cnt; 
		
		MNET->Process();

	} // while()
	
	// for some reason, this does not work.
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
	
}

