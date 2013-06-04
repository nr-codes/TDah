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

#define GAIN_C			(3.0)
#define GAIN_C0			(1.5)
#define GAIN_C1			(2.0) 
#define GAIN_C2_FAR		(0.5) 
#define GAIN_C2_NEAR	(5.0)
#define GAIN_C3			(1.0) 

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
	static int last_status = 0;
	static double pos_prev = 0;
	static double pCmd_prev = 0;
	static double vCmd_prev = 0;
	static double error = 0;
	
	int temp = 0;
	
	int mu_k = 0; // for test
	int obj_ang_index = 0; // to keep continuous angle value when acrossing pi(or -pi).
	//double obj_ang_offset = 0.0; // to subtract the object's angle at equilibrium position when inputting 'c' (camera)
	double obj_x_offset = 0;
	double obj_y_offset = 0;
	uint64_t ncycles1, ncycles2;
	
	//test
	double eta1_prev = 0;
	double feedforwardVal = 0;
	double feedbackVal = 0;
	double test_v_prev = 0;
	//double vCmd_prev = 0;

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
		
		// feedback
		//enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
		//pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		//vel = (pos - pos_prev) * SAMPLE_RATE;
				
		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) cnt++;
		
		// for motor modeling
		/*if (cnt == 1) {
			iCmd = 0.275;
		}
		else if ( (cnt % 500) == 1 ) {
			iCmd = -iCmd;
		}
		if (cnt >= 3001)
			iCmd = 0;
		*/
		
		// square wave for motor modeling
		/*int duration = (int)SAMPLE_RATE;
		if ( cnt == 0 ) {
			iCmd = .175;
		}
		else if ( cnt > 6*duration ) {
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			FATAL_ERROR("Three Periods Done!");
		}
		else if ( cnt % duration == 0 ) {
			iCmd = -iCmd;
		}*/

		// square wave position command
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
		
		/* 
		// sinusoid command, angle position (radian) input.
		pCmd = 45*DEG2RAD*sin(2*3.141592*(cnt)/SAMPLE_RATE/2.0); // period of the input: 1 sec , 45deg -> 20 deg
		
		// constant speed
		//pCmd = .5*3.141592 * cnt/SAMPLE_RATE; // 4.5 2*3.141592
		
		// for constant speed controller
		vCmd = (pCmd - pCmd_prev) * SAMPLE_RATE;
		aCmd = (vCmd - vCmd_prev) * SAMPLE_RATE;
		error += (pCmd - pos);
		*/

		// desired acceleration input
		//double T_period = 0.2; //2;
		// sinusoidal wave
		//aCmd = 2*DEG2RAD*(2*3.141592/T_period)*(2*3.141592/T_period)*sin(2*3.141592*(cnt)/SAMPLE_RATE/T_period);
		
		// square wave. 
		/*if (cnt == 1) {
			aCmd = ( 1*DEG2RAD*(2*3.141592/T_period)*(2*3.141592/T_period) ) * 2/3.141592; // corresponding to the same integration area of the acc.
			//aCmd = 2; // the value measured in the primary experiment w/ Fabio's controller
		}
		else if ( (cnt % int(T_period*SAMPLE_RATE/2) ) == 1 ) {
		//else if ( (cnt % 1000) == 1 ) {
			aCmd = -aCmd;
		}
		*/
		// This part is necessary so that vCmd, pCmd is not accumulated when MOTOR_ON is not true.
		/*if(HW->motorStatus != MOTOR_ON) {
			aCmd = 0;
			// not necessary, rather cause some error
			//vCmd_prev = 0; 
			//pCmd_prev = pos;
		} 

		vCmd = vCmd_prev + aCmd / SAMPLE_RATE; 
		pCmd = pCmd_prev + vCmd / SAMPLE_RATE; 
		//test
		test_v_prev = vCmd_prev;

		iCmd = calcI(vCmd, aCmd); // feedforward control. Not being used currently.
		feedforwardVal = iCmd;
		
		//iCmd = (Kp* (pCmd - pos) + Kd * (vCmd - vel) + Ki * error); // Ki 0
		iCmd = (200* (pCmd - pos) + 0.6 * (vCmd - vel) + Ki * error); // Ki 0
		feedbackVal = (Kp* (pCmd - pos) + Kd * (vCmd - vel) + Ki * error);
		
		pos_prev = pos;
		pCmd_prev = pCmd;
		vCmd_prev = vCmd;
		*/

		double elapsedSec = 0;
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
			
			vnum[0] = -99;
			int n = VNET->Recv();
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
			if (obj_x_offset == 0 && obj.x0 != 0 && obj.x1 != 0) {
				obj_x_offset = obj.x;
				obj_y_offset = obj.y;
			}

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
			obj.angVel = (obj.theta - obj.theta_prev) * SAMPLE_RATE;
			// low pass filter
			double alpha = 0.02; //0.02
			obj.angVel = alpha*obj.angVel + (1-alpha)*obj.angVel_prev;
			
			//test. moved from the top
			enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
			pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
			vel = (pos - pos_prev) * SAMPLE_RATE;

			// calculation of the hand angular velocity
			handTheta = pos; 
			handVel = (handTheta - handTheta_prev) * SAMPLE_RATE;
			//alpha = 0.05;
			handVel = alpha*handVel + (1-alpha)*handVel_prev;
			
			// calculation of sh and \dot sh (=shD)
			sh = (obj.theta - handTheta) / K_R;
			shD = (obj.angVel - handVel) / K_R;
			
			// coordinate transformation
			eta1 = M22*shD + M12*handVel;
			//test filtering
			//eta1 = alpha*eta1 + (1-alpha)*eta1_prev;
			//eta1_prev = eta1;

			//eta2 = handTheta + sh/RHO_H;
			eta2 = asin(-(obj.x-obj_x_offset)/(RHO_H+RHO_O)); // alternative way to get eta2;

			// calculation of the control input (acceleration command)
			//xi := handVel; 
			//eta1D := COEFF_D * sin(eta2); // COEFF_D
			//eta2D := COEFF_A * eta1 - xi/COEFF_B; 
			double GAIN_C2 = 0.5; //GAIN_C2_FAR;   0.5, 5
			//if ( abs(eta2) < 0.03 && abs(handVel) < 0.45) { //0.015 ) {
			//	GAIN_C2 = GAIN_C2_NEAR;
			//}
			double TANH = tanh(GAIN_C1*eta1 + GAIN_C2*eta2);
			alpha_eta = COEFF_B * (COEFF_A*eta1 + GAIN_C0*TANH + GAIN_C3*eta2);
			DalphaDeta1 = COEFF_B * ( COEFF_A + GAIN_C0*(1-TANH*TANH)*GAIN_C1 );
			DalphaDeta2 = COEFF_B * ( GAIN_C0*(1-TANH*TANH)*GAIN_C2 + GAIN_C3 );
			vInp = -GAIN_C * (handVel - alpha_eta) + DalphaDeta1*(COEFF_D * sin(eta2)) + DalphaDeta2*(COEFF_A * eta1 - handVel/COEFF_B);
			aCmd = vInp; // calculated acceleration command
			
			// calculation of the original control input
			// for practical purpose
			/*double SIGN_VEL = sign(handVel);
			if ( abs(handVel) < 0.01 ) {
				SIGN_VEL = sign(vInp);
				//SIGN_VEL = 0;
			}
			
			if(vInp < 0){ // acceleration
			//if(handVel < 0){ // acceleration.  Because the motor rotaiting direction was opposite when the modeling was done.
				iCmd = COEFF_H_NEG * vInp + COEFF_J_NEG * sin(eta2) + COEFF_MUD_NEG * handVel + COEFF_FS_NEG * SIGN_VEL; 
			}
			else{
				iCmd = COEFF_H_POS * vInp + COEFF_J_POS * sin(eta2) + COEFF_MUD_POS * handVel + COEFF_FS_POS * SIGN_VEL; 
			}
			*/

			//test
			//double T_period = 0.2; //2;
			// sinusoidal wave
			//aCmd = 2*DEG2RAD*(2*3.141592/T_period)*(2*3.141592/T_period)*sin(2*3.141592*(cnt)/SAMPLE_RATE/T_period);

			// This part is necessary so that vCmd, pCmd is not accumulated when MOTOR_ON is not true.
			if(HW->motorStatus != MOTOR_ON) {
				iCmd = 0; // for the high level control law

				aCmd = 0;
				//vCmd_prev = 0;
				//pCmd_prev = pos;
			} // for the safety
			
			// acceleration inner loop
			// Notice that using no filtering velocity
			vCmd = vCmd_prev + aCmd / SAMPLE_RATE; 
			pCmd = pCmd_prev + vCmd / SAMPLE_RATE; 
			//test
			test_v_prev = vCmd_prev;

			iCmd = calcI(vCmd, aCmd); // feedforward control. Not being used currently.
			feedforwardVal = iCmd;
			
			/*if(vInp < 0){ // acceleration
				iCmd = COEFF_H_NEG * vInp + COEFF_J_NEG * sin(eta2) + COEFF_MUD_NEG * handVel + COEFF_FS_NEG * sign(handVel); 
				//iCmd = COEFF_H_NEG * vInp + COEFF_J_NEG * sin(eta2) + COEFF_MUD_NEG * vCmd + COEFF_FS_NEG * sign(vCmd); 
			}
			else{
				iCmd = COEFF_H_POS * vInp + COEFF_J_POS * sin(eta2) + COEFF_MUD_POS * handVel + COEFF_FS_POS * sign(handVel); 
				//iCmd = COEFF_H_POS * vInp + COEFF_J_POS * sin(eta2) + COEFF_MUD_POS * vCmd + COEFF_FS_POS * sign(vCmd); 
			}*/

			//iCmd = (Kp* (pCmd - pos) + Kd * (vCmd - vel) + Ki * error); // Ki 0
			iCmd += (80* (pCmd - pos) + 0.3 * (vCmd - vel) + Ki * error); // Ki 0
			//iCmd = (10* (pCmd - pos) + 0.0 * (vCmd - vel) + Ki * error); // Ki 0
			feedbackVal = (Kp* (pCmd - pos) + Kd * (vCmd - vel) + Ki * error);
		
			pos_prev = pos;
			pCmd_prev = pCmd;
			vCmd_prev = vCmd;
			
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
		if (fabs(handVel) > 5.0 ) {//rad/sec
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			FATAL_ERROR("MOTOR RUNS TOO FAST");
		}
		
		//limit current based on motor specs
		if(iCmd > (MAX_CURRENT_MA/1.6*1.2) ){ // 1.6 mA
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, feedforwardVal:%f\n", iCmd, pCmd, pos, vCmd, vel, feedforwardVal);
			FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			iCmd = MAX_CURRENT_MA/1.6*1.2;
		}
		else if(iCmd < (-MAX_CURRENT_MA/1.6*1.2) ){
			HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			HW->motorStatus = MOTOR_OFF;
			printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, feedforwardVal:%f\n", iCmd, pCmd, pos, vCmd, vel, feedforwardVal);
			FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			iCmd = -MAX_CURRENT_MA/1.6*1.2;
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

		// keep the position angle within 2pi
		//if ( pos > (mu_k+1)*2*3.141592 )
		//	mu_k++;
		//pos = pos - mu_k*2*3.141592;
		
		num[0] = aCmd; //correctedAcc; //eta1; //handVel; //pos; //handTheta; //pos; // angle in rad
		num[1] = eta1; //test_v_prev; //feedforwardVal; //pCmd; //eta2; //obj.theta; //vnum[0]; //obj.theta; //vel; //obj.theta;
		num[2] = pos; //feedbackVal; //vCmd; //obj.angVel; //obj.x; //vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = eta2; //handTheta; //iCmd; //sec; //elapsedSec; //sec; //vnum[2]; //obj.y1;

		
		// set outputs
		/*num[0] = vInp; //correctedAcc; //eta1; //handVel; //pos; //handTheta; //pos; // angle in rad
		num[1] = eta1; //feedforwardVal; //pCmd; //eta2; //obj.theta; //vnum[0]; //obj.theta; //vel; //obj.theta;
		num[2] = eta2; //feedbackVal; //vCmd; //obj.angVel; //obj.x; //vnum[1]; //obj.x1; //ampVCmd; //obj.x1; // position in m, *10 for cm
		num[3] = pos; //handTheta; //iCmd; //sec; //elapsedSec; //sec; //vnum[2]; //obj.y1;
		*/

		// acceleration innerloop
		/*num[0] = aCmd; 
		num[1] = pCmd;
		num[2] = vCmd;
		//num[1] = feedforwardVal; 
		//num[2] = feedbackVal; 
		num[3] = pos; 
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

