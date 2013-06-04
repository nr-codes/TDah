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
const double RHO_O = 0.08; // radius of the object disk
const double GRAV = 9.8 * sin(3.142592/180*85.2); //82.09//45.8); // gravitational acceleration
const double K_R = (1.0/RHO_H + 1.0/RHO_O);
const double I_H = 0.5 * M_H * (RHO_H*RHO_H + RHO_HI*RHO_HI) + INERTIA_MOTOR;
const double I_O = 0.5 * M_O * RHO_O*RHO_O;
const double M11 = I_H + M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O) + I_O;
const double M12 = M_O*(RHO_H+RHO_O)*(RHO_H+RHO_O)/RHO_H + I_O*K_R;
const double M22 = K_R*K_R*(M_O*RHO_O*RHO_O + I_O);
// naming changes according to the notation used in my paper.
const double COEFF_sig1 = M_O*GRAV*(RHO_H+RHO_O)/RHO_H;
const double COEFF_sig2 = 1.0/(RHO_H*M22);
const double COEFF_sig3 = 1 - M12/(RHO_H*M22);

// hard-coding for eq. point
const double OBJ_X_OFFSET = 0.0909338 + 0.00184 - 0.000621; // in m
const double OBJ_Y_OFFSET = -0.0894877; // in m

// target angular position of the object
const double TARGET_THETAO = 3.142592/180*90; //90 degree
const double TARGET_THETAH = -3.142592/180*45; //-RHO_O/RHO_H*TARGET_THETAO;

// repeated root of -3
const double GRT = 3; // in the form of (lambda + GRT) to give the repeated root at -3
const double GAIN_K4 = 4*GRT;  
const double GAIN_K3 = -GRT*GRT*GRT*GRT/(COEFF_sig1*COEFF_sig2);
const double GAIN_K2 = (6*GRT*GRT + COEFF_sig1*COEFF_sig2-GAIN_K3)/COEFF_sig3; 
const double GAIN_K1 = (GAIN_K4*COEFF_sig1*COEFF_sig2 + 4*GRT*GRT*GRT)/(COEFF_sig1*COEFF_sig3);	

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
	double z = 0; //!
	double h = 0; //! linearizing output
	double hD = 0; //!
	double hDD = 0; //!
	double hDDD = 0; //!
	double u =0; //

	double eta1_prev = 0; //!
	double eta1D = 0; //! to use eta1D instead of sigma1 * sin(eta2)
	double eta1D_prev = 0; //! to use eta1D instead of sigma1 * sin(eta2)

	//test
	double feedforwardVal = 0;
	double feedbackVal = 0;
	//double test_v_prev = 0;
	//double vCmd_prev = 0;
	double elapsedSec = 0;
	uint64_t cycle1, cycle2;
	double elapsedSec1 = 0;
	double elapsedSec2 = 0;
	double obj_vel_uf = 0;
	double vInp_prev = 0;
	//double aCmd_sin = 0;
	//int update_done = 0; // for automatic x_offset update
	//double x_avg = 0;  // for automatic x_offset update
	//double obj_x_offset = 0; // for automatic x_offset update
	//double x_max = 0;
	//double x_min = 0;

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
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					HW->motorStatus = MOTOR_OFF;
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
			// eta1D
			eta1D = (eta1 - eta1_prev) / sec;
			eta1D = alpha*eta1D + (1-alpha)*eta1D_prev;

			//eta2 = handTheta + sh/RHO_H;
			eta2 = asin(-(obj.x-OBJ_X_OFFSET)/(RHO_H+RHO_O)); // alternative way to get eta2;
			//eta2 += 0.008;
			//eta2 = atan(-(obj.x-OBJ_X_OFFSET)/(obj.y-OBJ_Y_OFFSET+RHO_H+RHO_O)); // alternative way to get eta2;
			// xi = handVel;

			/*************************************************************/
			// calculation of the control input (acceleration command)
			z = handTheta - TARGET_THETAH;
			//vInp = -GAIN_K1*eta1 - GAIN_K2*eta2 - GAIN_K3*z - GAIN_K4*handVel;
			vInp = -1533.6*eta1 - 613.4*eta2 - (-9.04)*z - 16.0*handVel;

			/*************************************************************/
			aCmd = vInp; // calculated acceleration command
			
			obj.theta_prev = obj.theta;
			obj.angVel_prev = obj.angVel;
			handTheta_prev = handTheta;
			handVel_prev = handVel;
			eta1_prev = eta1;
			eta1D_prev = eta1D;
			
			// For resetting of the upright eq. position 
			// not working very well. don't do this.
			/*x_avg += (obj.x - OBJ_X_OFFSET);
			if ( (cnt%int(SAMPLE_RATE)) == 799 ) { // just to have 1 sec (800 cnts)
				if ( cnt > int(SAMPLE_RATE)*6 ) { // cnt grows up after Motor ON
					obj_x_offset = x_avg / SAMPLE_RATE;
				}
				x_avg = 0;
			}*/

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

