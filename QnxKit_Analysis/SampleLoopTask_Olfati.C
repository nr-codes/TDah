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
const double RHO_O = 0.08; // radius of the object disk
const double GRAV = 9.8 * sin(3.142592/180*85.2); //45.8); // gravitational acceleration
const double K_R = (1.0/RHO_H + 1.0/RHO_O);
const double I_H = 0.5 * M_H * (RHO_H*RHO_H + RHO_HI*RHO_HI) + INERTIA_MOTOR;
const double I_O = 0.5 * M_O * RHO_O*RHO_O;
const double M11 = I_H + M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O) + I_O;
const double M12 = M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O)/RHO_H + I_O*K_R;
const double M22 = K_R*K_R*(M_O*RHO_O*RHO_O + I_O);
// naming changes according to the notation used in Olfati's thesis.
const double COEFF_m11 = RHO_H*M22;
const double COEFF_m12 = M12-RHO_H*M22;
const double COEFF_m0 = M_O*GRAV*(RHO_H+RHO_O)/RHO_H;

const double COEFF_H_POS = (M11 - M12*M12/M22) / KM_POS;
const double COEFF_J_POS = (M_O * GRAV * (M12/M22*RHO_O*K_R - (RHO_H+RHO_O)) ) / KM_POS;
const double COEFF_MUD_POS = DYN_FRIC_POS / KM_POS;
const double COEFF_FS_POS = STAT_FRIC_POS / KM_POS;
const double COEFF_H_NEG = (M11 - M12*M12/M22) / KM_NEG;
const double COEFF_J_NEG = (M_O * GRAV * (M12/M22*RHO_O*K_R - (RHO_H+RHO_O)) ) / KM_NEG;
const double COEFF_MUD_NEG = DYN_FRIC_NEG / KM_NEG;
const double COEFF_FS_NEG = STAT_FRIC_NEG / KM_NEG;

// hard-coding for eq. point
const double OBJ_X_OFFSET = 0.0800147; //0.0848480; // in m
const double OBJ_Y_OFFSET = -0.0093650; //-0.0091731; // in m

#define GAIN_C0			(-0.7) // -1.5 -pi/2 < C0 < 0
#define GAIN_C1			(7.5)  // 10
#define GAIN_C2			(12.25)  // 16
#define GAIN_C3			(7.0)  // 8


/*// good for ~45 deg
#define GAIN_C0			(-1.0) // -pi/2 < C0 < 0
#define GAIN_C1			(5.0) 
#define GAIN_C			(2.5)
#define GAIN_K			(2.5) 
*/

/*// good for 6.6 deg tilting
#define GAIN_C0			(-1.5) // -pi/2 < C0 < 0
#define GAIN_C1			(5.0) 
#define GAIN_C			(4.0)
#define GAIN_K			(4.0) 
*/

/*#define GAIN_C0			(-1.5) // -pi/2 < C0 < 0
#define GAIN_C1			(3.0) 
#define GAIN_C			(1.5)
#define GAIN_K			(10.0) 
*/

/*
#define GAIN_C			(3.0)
#define GAIN_C0			(1.5)
#define GAIN_C1			(2.0) 
#define GAIN_C2_FAR		(0.5) 
#define GAIN_C2_NEAR	(5.0)
#define GAIN_C3			(1.0) 
*/

/*#define RHO_H			(0.15) // radius of the hand disk
#define RHO_O			(0.08) 
#define K_R				(19.1667)   // (1/RHO_H + 1/RHO_O)
#define M22				(0.5008) // must be changed with different RHO_H or RHO_O
#define M12				(0.05879) // must be changed with different RHO_H or RHO_O
#define M11				(0.01703) // combined with the motor inertia
#define COEFF_A			(13.3124) 
#define COEFF_B			(-4.6000)
#define COEFF_D			(0.2453) //(0.1384)
*/
//#define COEFF_H_POS		(0.02201)
//#define COEFF_J_POS		(-0.0174) //(-0.0098) //wrong(-0.1513)
//#define COEFF_MUD_POS	(0.01196)
//#define COEFF_FS_POS	(0.1052) //*.85)
//#define COEFF_H_NEG		(0.02317)
//#define COEFF_J_NEG		(-0.0183) //(-0.0103) //wrong(-0.1592)
//#define COEFF_MUD_NEG	(0.01396)
//#define COEFF_FS_NEG	(0.1066)
/*
#define COEFF_H_POS		(0.0054)
#define COEFF_J_POS		(-0.00427)
#define COEFF_MUD_POS	(0.0270)
#define COEFF_FS_POS	(0.0626) 
#define COEFF_H_NEG		(0.00619)
#define COEFF_J_NEG		(-0.00489)
#define COEFF_MUD_NEG	(0.02612)
#define COEFF_FS_NEG	(0.06135)
*/

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
	static double error = 0;
	/*static int last_status = 0;
	static double pos_prev = 0;
	static double pCmd_prev = 0;
	static double vCmd_prev = 0;
	*/
	int last_status = 0;
	double pos_prev = 0;
	double pCmd_prev = 0;
	double vCmd_prev = 0;
		
	int obj_ang_index = 0; // to keep continuous angle value when acrossing pi(or -pi).
	//double obj_x_offset = 0;
	//double obj_y_offset = 0;
	uint64_t cycle, cycle_prev;

	// for acceleration calculation
	double ds = 0; 
	double sum_h = 0; 
	double s = 0;
	double u = 0;
	double alpha1 = 0;
	double k1 = 0; //!
	double k1D = 0; //!
	double k1DD = 0; //!
	double mu1 = 0; // for the change of coordinate
	double mu2 = 0; // for the change of coordinate
	double Dalpha1Deta2 =0;

	//test
	double eta1_prev = 0;
	double feedforwardVal = 0;
	double feedbackVal = 0;
	double test_v_prev = 0;
	//double vCmd_prev = 0;
	double elapsedSec = 0;
	uint64_t cycle1, cycle2;
	double elapsedSec1 = 0;
	double elapsedSec2 = 0;
	double aCmd_sin = 0;
	double obj_vel_uf = 0;
	double vInp_prev = 0;

	while(!lost){
		TimerWait();
		
		ncycles = ClockCycles();
		sec=(double)(ncycles - ncycles_prev)/cps;
		ncycles_prev = ncycles;
		
		TimingProcess();
		
		// Read Inputs
		HW->ProcessInput(); // all digital & analog, encoders reading.
		
		// Get status of camera
		last_status = HW->ReadDigitalBit(IoHardware::FRAME_STATUS);
		
		// Send out pulse to trigger camera
		HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 1);
		HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 0);
		
		// test
		//cycle1 = ClockCycles();

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
			// test
			//cycle2 = ClockCycles();
			//elapsedSec1 = (double)(cycle2 - cycle1)/cps;
			
			
			vnum[0] = -99;
			int n = VNET->Recv();
			
			//test
			//cycle1 = ClockCycles();
			//elapsedSec2 = (double)(cycle1 - cycle2)/cps;

			region = (int)vnum[0];
			switch(region){
				case 0: //ROI_0:
					obj.x0 = vnum[1]; //mm
					obj.y0 = vnum[2]; //mm
					break;
				case 1: //ROI_1:
					obj.x1 = vnum[1]; //mm
					obj.y1 = vnum[2]; //mm
					break;
				default:
					//HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					//HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					printf("roi-vnum[0]:%d\n", region);
					FATAL_ERROR("Object lost -");
					//strcpy(err_msg, "Object lost -");
					lost = 1;
			}
			obj.x = (obj.x0 + obj.x1)/2/1000; // convert to m
			obj.y = (obj.y0 + obj.y1)/2/1000;
			double obj_raw_angle = atan2(obj.y1-obj.y0, obj.x1-obj.x0); // within -pi to pi
			// compensating angle offset
			// set only once at the beginning
			/*if (obj_x_offset == 0 && obj.x0 != 0 && obj.x1 != 0) {
				obj_x_offset = obj.x;
				obj_y_offset = obj.y;
			}*/

			//uint64_t cycle1 = ClockCycles();
			
			// to keep continuous angle value when acrossing pi(or -pi).
			obj.theta = obj_raw_angle + obj_ang_index*2*3.141592; // converted angle
			if ( fabs(obj.theta - obj.theta_prev) > 3.141592 ) {
				if (obj.theta_prev > obj.theta) // pi to -pi region change
					obj_ang_index++;
				else
					obj_ang_index--;
				obj.theta = obj_raw_angle + obj_ang_index*2*3.141592; // newly converted angle
			}
				
			// calculation of the object angular velocity
			//obj.angVel = (obj.theta - obj.theta_prev) * SAMPLE_RATE;
			obj.angVel = (obj.theta - obj.theta_prev) / sec; 
			obj_vel_uf = obj.angVel;
			// low pass filter
			double alpha = 0.038; //0.02
			obj.angVel = alpha*obj.angVel + (1-alpha)*obj.angVel_prev;
			
			// calculation of the hand angular velocity
			handTheta = -(HW->GetEncoderCount(IoHardware::ENC_0)) * ENC_RAD_PER_CNT / GR / 4; // just access the data previously read by GetEnc...()
			handVel = (handTheta - handTheta_prev) /sec; //* SAMPLE_RATE;
			//alpha = 0.1;
			handVel = alpha*handVel + (1-alpha)*handVel_prev;
			
			// calculation of sh and \dot sh (=shD)
			sh = (obj.theta - handTheta) / K_R;
			shD = (obj.angVel - handVel) / K_R;
			
			// coordinate transformation
			eta1 = M22*shD + M12*handVel;
			//test filtering
			//alpha = 0.5;
			//eta1 = alpha*eta1 + (1-alpha)*eta1_prev;
			//eta1_prev = eta1;

			//eta2 = handTheta + sh/RHO_H;
			eta2 = asin(-(obj.x-OBJ_X_OFFSET)/(RHO_H+RHO_O)); // alternative way to get eta2;
			// xi = handVel;

			/*************************************************************/
			// calculation of the control input (acceleration command)
			double TANH = tanh(GAIN_C1*eta1);
			
			k1 = GAIN_C0 * TANH; 
			k1D = GAIN_C0*GAIN_C1*COEFF_m0*sin(eta2)*(1-TANH*TANH);
			k1DD = GAIN_C0*GAIN_C1*COEFF_m0*(1-TANH*TANH)*( cos(eta2)*(eta1/COEFF_m11-COEFF_m12/COEFF_m11*handVel) - 2*GAIN_C1*COEFF_m0*TANH*sin(eta2)*sin(eta2) );
			
			// change of coordinate
			mu1 = eta2 - k1;
			mu2 = (eta1 - COEFF_m12*handVel)/COEFF_m11 - k1D;

			vInp = COEFF_m11/COEFF_m12*(GAIN_C2*mu1 + GAIN_C3*mu2 + COEFF_m0/COEFF_m11*sin(eta2) - k1DD);
			aCmd = vInp; // calculated acceleration command
			
			obj.theta_prev = obj.theta;
			obj.angVel_prev = obj.angVel;
			handTheta_prev = handTheta;
			handVel_prev = handVel;

			//uint64_t cycle2 = ClockCycles();
			//elapsedSec=(double)(cycle2 - cycle1)/cps;
		}
		else { // Because the vision system keeps sending the data, QNX must read them, otherwise the vision system will get a send error.
			VNET->Process();
			//obj_ang_offset = 0.0; // reset the angle offset
			//aCmd = 0;
			//vCmd_prev = 0;
			//pCmd_prev = 0;
		}
		
		// test
		if (fabs(handVel) > 7.0 ) {//rad/sec
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			FATAL_ERROR("MOTOR RUNS TOO FAST");
			lost = 1;
		}
		
		//**************************************************************
		// reading & calculating output
		//
		// feedback
		cycle = ClockCycles();
		elapsedSec = (double)(cycle - cycle_prev)/cps;
		cycle_prev = cycle;
		

		//HW->ProcessInput(); // because of the encoder reading. Do not use this again. This takes a long time.
		//cycle2 = ClockCycles();
		enc = -(HW->ReadEncoder(IoHardware::ENC_0)); // direct read. Not working?
		//cycle1 = ClockCycles();
		//elapsedSec2 = (double)(cycle1 - cycle2)/cps;
		//enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
		pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		//vel = (pos - pos_prev) * SAMPLE_RATE;
		vel = (pos - pos_prev) / elapsedSec; // to calculate more exact velocity.
				
		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) {
			cnt++;
		
			/*double T_period = 0.2; //2;
			// sinusoidal wave
			aCmd = -2*DEG2RAD*(2*3.141592/T_period)*(2*3.141592/T_period)*sin(2*3.141592*(cnt)/SAMPLE_RATE/T_period);
			//aCmd_sin = aCmd;
			if (cnt == 1) { // at the beginning
				// because of the sinusoidal wave
				vCmd_prev = 2*DEG2RAD*(2*3.141592/T_period);
			}
			*/

			// acceleration inner loop
			// Notice that using no filtering velocity
			vCmd = vCmd_prev + aCmd / SAMPLE_RATE; 
			pCmd = pCmd_prev + vCmd_prev / SAMPLE_RATE + 0.5 * aCmd / (SAMPLE_RATE*SAMPLE_RATE);
			//pCmd = pCmd_prev + vCmd / SAMPLE_RATE; 
			//vCmd = vCmd_prev + aCmd * elapsedSec; 
			//pCmd = pCmd_prev + vCmd * elapsedSec; 
			//test
			test_v_prev = vCmd_prev;

			iCmd = calcI(vCmd, aCmd); // feedforward control. 
			feedforwardVal = iCmd;
			
			//iCmd += (150* (pCmd - pos) + 0.6 * (vCmd - vel) + Ki * error); // Ki 0
			//iCmd += (200* (pCmd - pos) + 0.6 * (vCmd - vel) + Ki * error); // Ki 0
			//iCmd += (350* (pCmd - pos) + 1.0 * (vCmd - vel) + Ki * error); // Ki 0
			iCmd += (150* (pCmd - pos) + 0.9 * (vCmd - vel) + Ki * error); // Ki 0
					
			pos_prev = pos;
			pCmd_prev = pCmd;
			vCmd_prev = vCmd;
		}
		else {
			iCmd = 0;
		}

		//**************************************************

		//**************************************************
		// control output
		// 
		//limit current based on motor specs
		if(iCmd > (MAX_CURRENT_MA) ){ // 1.6 mA
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, aCmd:%f feedforwardVal:%f cnt:%d\n", iCmd, pCmd, pos, vCmd, vel, aCmd, feedforwardVal, cnt);
			FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			lost = 1;
			iCmd = MAX_CURRENT_MA;
		}
		else if(iCmd < (-MAX_CURRENT_MA) ){
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, aCmd:%f feedforwardVal:%f cnt:%d%f\n", iCmd, pCmd, pos, vCmd, vel, aCmd, feedforwardVal, cnt);
			FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			lost = 1;
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

		// keep the position angle within 2pi
		//if ( pos > (mu_k+1)*2*3.141592 )
		//	mu_k++;
		//pos = pos - mu_k*2*3.141592;
		
		/*num[0] = sec; //aCmd; //correctedAcc; //eta1; //handVel; //pos; //handTheta; //pos; // angle in rad
		num[1] = elapsedSec1; //test_v_prev; //feedforwardVal; //pCmd; //eta2; //obj.theta; //vnum[0]; //obj.theta; //vel; //obj.theta;
		num[2] = elapsedSec2; //feedbackVal; //vCmd; //obj.angVel; //obj.x; //vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = vCmd; //handTheta; //iCmd; //sec; //elapsedSec; //sec; //vnum[2]; //obj.y1;
		*/
		
		// set outputs
		num[0] = vInp; //correctedAcc; //eta1; //handVel; //pos; //handTheta; //pos; // angle in rad
		num[1] = eta1; //feedforwardVal; //pCmd; //eta2; //obj.theta; //vnum[0]; //obj.theta; //vel; //obj.theta;
		num[2] = pos; //feedbackVal; //vCmd; //obj.angVel; //obj.x; //vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = eta2; //handTheta; //iCmd; //sec; //elapsedSec; //sec; //vnum[2]; //obj.y1;
		

		// acceleration innerloop
		/*num[0] = aCmd; //aCmd; 
		num[1] = pCmd;
		num[2] = pos;
		//num[1] = feedforwardVal; 
		//num[2] = feedbackVal; 
		num[3] = vCmd; 
		*/
		
		// motor modeling
		/*num[0] = pos; 
		num[1] = iCmd; 
		num[2] = cnt; 
		num[3] = 0; 
		*/
		MNET->Process();

	} // while()
	
	// for some reason, this does not work.
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
	
}

