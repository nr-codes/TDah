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
const double RHO_O = 0.081; // radius of the object disk
const double GRAV = 9.8 * sin(3.142592/180*78.3); //77.6); //82.09); //85.2); //45.8); // gravitational acceleration
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

double TARGET_THETAHD = -3.0; // hand target velocity. in radian/sec
const double COEFF_ETA1_STAR = (-M22*RHO_H + M12)*TARGET_THETAHD;

// hard-coding for eq. point from speed control making average eta2 zero.
//const double OBJ_X_OFFSET = 0.0827712 - (RHO_H+RHO_O)*(-0.008805015605812); //0.1039268; //0.0827712; //0.1039268; // in m
//const double OBJ_Y_OFFSET = -0.009247; // in m. currently not use this.
const double OBJ_X_OFFSET = 0.0783; // in m. It may be better to get this value from constant speed control
const double OBJ_Y_OFFSET = 0.03346; // in m. currently not use this.

// For position control
/*const double OBJ_X_OFFSET_0 = OBJ_X_OFFSET - (-0.0008); //(RHO_H+RHO_O)*(-0.003513559715127) ; //- 0.0006;
const double OBJ_X_OFFSET_m90 = OBJ_X_OFFSET - (0.0005); // + 0.001;// + 0.0005;   
const double OBJ_X_OFFSET_m180 = OBJ_X_OFFSET - (-0.001 - 0.0005);  //-0.0016;  - 0.0007; // + 0.0003;
const double OBJ_X_OFFSET_m90b = OBJ_X_OFFSET - (-0.0014 - 0.0003 + 0.0006); // + 0.0020;
*/
const double OBJ_X_OFFSET_0 = OBJ_X_OFFSET + 0.00164; //(RHO_H+RHO_O)*(-0.003513559715127) ; //- 0.0006;
const double OBJ_X_OFFSET_m90 = OBJ_X_OFFSET; // 0.00160; // + 0.001;// + 0.0005;   
const double OBJ_X_OFFSET_m180 = OBJ_X_OFFSET + 0.0023; //0.00028;  //-0.0016;  - 0.0007; // + 0.0003;
const double OBJ_X_OFFSET_m90b = OBJ_X_OFFSET + 0.00160; // + 0.0020;

// for stabilization control
#define GAIN_K1			(4.2) 
#define GAIN_K2			(6.0) 
#define GAIN_C			(10.0)  

// for speed control
// repeated root of -4
#define GAIN_K1S			(16.0) 
#define GAIN_K2S			(96.0) 
#define GAIN_K3S			(256.0)  
#define GAIN_K4S			(256.0) 

// for position control
// repeated root of -4.5. This is normal range of the control gains for position control.
#define GAIN_K1P			(18.0) 
#define GAIN_K2P			(121.5) 
#define GAIN_K3P			(364.5)  
#define GAIN_K4P			(410.1) 

// repeated root of -6. For a certain target angle. I need a higher gain. 
#define GAIN_K1P2			(24.0) 
#define GAIN_K2P2			(216.0) 
#define GAIN_K3P2			(864.0)  
#define GAIN_K4P2			(1296.0) 

// for position control, root -7. near the target position, the higher gains the better performance.
#define GAIN_K1P3			(28.0) 
#define GAIN_K2P3			(294.0) 
#define GAIN_K3P3			(1372.0)  
#define GAIN_K4P3			(2401.0) 


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
	
	// Do we need static variables?
	static int dout = 1;
	static double error = 0;
	
	int last_status = 0;
	double pos_prev = 0;
	double pCmd_prev = 0;
	double vCmd_prev = 0;
		
	int obj_ang_index = 0; // to keep continuous angle value when acrossing pi(or -pi).
	uint64_t cycle, cycle_prev;

	// for stabilization control
	double u = 0; 
	double alpha = 0;
	double DuDeta1 =0;
	double DuDeta2 =0;
	double DalphaDeta1 =0;
	double DalphaDeta2 =0;
	
	// for speed control
	double z = 0; 
	double h = 0; // linearizing output
	double hD = 0; 
	double hDD = 0; 
	double hDDD = 0;
	
	// for acceleration calculation
	double DuDeta1Bar =0;
	double DalphaDeta1Bar =0;
	double eta1Bar = 0; 
	
	double target_thetahD = 0;
	double target_thetah = 0;
	double base_thetah = 0;

	// to use eta2D in calculation of shD
	double eta2_prev = 0; 
	double eta2D = 0; 
	double eta2D_prev = 0; 

	int controller = 1; // for different control objective
	double time = 0;
	double time_offset = 0;
	double thetah_offset = 0;
	
	// for position control
	double err_sum = 0;
	int posControl_flag = 0;
	int integral_flag = 0;
	
	double elapsedSec = 0;

	double cnt_new = 0;

	//test
	double feedforwardVal = 0;
	double feedbackVal = 0;
	uint64_t cycle1, cycle2;
	
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
		
		if(camera) { 
			// Wait for camera to process data, with timeout counter
			while(HW->ReadDigitalBit(IoHardware::FRAME_STATUS) == last_status){
				if(++attempt == (int)(6.0e5/SAMPLE_RATE)) { // 5.0e5 must be found out by experiments to give the smallest time to determine an error status
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
					FATAL_ERROR("Frame not received -");
					//strcpy(err_msg, "Frame not received -"); // not work this way.
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
					lost = 1;
			}
			obj.x = (obj.x0 + obj.x1)/2/1000; // convert to m
			obj.y = (obj.y0 + obj.y1)/2/1000;
			double obj_raw_angle = atan2(obj.y1-obj.y0, obj.x1-obj.x0); // within -pi to pi
			
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
			obj.angVel = (obj.theta - obj.theta_prev) / sec; // the use of SAMPLE_RATE may not be a big difference.
			// low pass filter
			double alpha = 0.038;
			obj.angVel = alpha*obj.angVel + (1-alpha)*obj.angVel_prev;
			
			// calculation of the hand angular velocity
			handTheta = -(HW->GetEncoderCount(IoHardware::ENC_0)) * ENC_RAD_PER_CNT / GR / 4; // just access the data previously read by GetEnc...()
			handVel = (handTheta - handTheta_prev) /sec; 
			handVel = alpha*handVel + (1-alpha)*handVel_prev;
			
			// calculation of sh and \dot sh (=shD)
			sh = (obj.theta - handTheta) / K_R;

			//obj.x = obj.x - 0.0007*cos(obj.theta + 3.0223 - 0.3491); // center position compensation
			//obj.x = obj.x + 0.0005*sin(obj.theta + 1.0);

			eta2 = asin(-(obj.x-OBJ_X_OFFSET)/(RHO_H+RHO_O)); // alternative way to get eta2;
			// xi = handVel;
			
			// controller = 1 for stabilization is default
			
			//time = cnt/SAMPLE_RATE; //time (sec) = cnt/SAMPLE_RATE
			if (cnt_new == 10*SAMPLE_RATE) {
				cnt_new = 0;
			}
			time = cnt_new/SAMPLE_RATE;
			cnt_new = cnt_new + 1;
			if ( time >=5 && time < 10 ) {	
				controller = 2;
				//target_thetahD = -3.0;
				TARGET_THETAHD = -4.0;
				time_offset = 7;
			} 
			else if ( time >=15 && time < 20 ) {
				controller = 2;
				//target_thetahD = 3.0;
				TARGET_THETAHD = 3.0;
				time_offset = 17;	
			}
			else if ( time >=27 && time < 32 ) {	
				controller = 2;
				//target_thetahD = -3.0;
				TARGET_THETAHD = -3.0;
			} 
			else if ( time >=37 && time < 42 ) {
				controller = 2;
				//target_thetahD = 3.0;
				TARGET_THETAHD = 3.0;
			}
			/*else if ( time >= 27 && time < 32 ) {
				controller = 3;
				if (posControl_flag == 0) {
					//base_thetah = floor(handTheta/1.570796327)*1.570796327; // nearest smaller target in terms of PI.
					target_thetah = base_thetah;
					err_sum = 0;
					integral_flag = 0;
					posControl_flag = 1;
				}
				eta2 = asin(-(obj.x-OBJ_X_OFFSET_0)/(RHO_H+RHO_O)); // use different offset for the specific angle.
			} 
			else if ( time >= 32 && time < 37) {
				controller = 3;
				if (posControl_flag == 1) {
					target_thetah = base_thetah - 1.570796327;
					err_sum = 0;
					integral_flag = 0;
					posControl_flag = 2;
					
				}
				eta2 = asin(-(obj.x-OBJ_X_OFFSET_m90)/(RHO_H+RHO_O)); // use different offset for the specific angle.
			} 
			else if ( time >= 37 && time < 42 ) {
				controller = 3;
				if (posControl_flag == 2) {
					target_thetah = base_thetah - 1.570796327*2;
					err_sum = 0;
					integral_flag = 0;
					posControl_flag = 3;

				}
				eta2 = asin(-(obj.x-OBJ_X_OFFSET_m180)/(RHO_H+RHO_O)); // use different offset for the specific angle.
			}
			else if ( time >= 42 && time < 60 ) {
				controller = 3;
				if (posControl_flag == 3) {
					target_thetah = base_thetah - 1.570796327;
					err_sum = 0;
					integral_flag = 0;
					posControl_flag = 4;
				}
				eta2 = asin(-(obj.x-OBJ_X_OFFSET_m90b)/(RHO_H+RHO_O));  // use different offset for the specific angle.
			} */
			else {
				controller = 1;
				thetah_offset = handTheta;
				
				base_thetah = ceil(handTheta/3.141592)*3.141592; // nearest smaller target in terms of PI.
				err_sum = 0;
				integral_flag = 0;
			}
			//eta2 = eta2 - 0.004*sin(obj.theta + 1.885); 
			// new method for shD, thus eta1 feedback
			eta2D = (eta2 - eta2_prev) / sec;
			eta2D = alpha*eta2D + (1-alpha)*eta2D_prev;
			shD = RHO_H*(eta2D - handVel);
			eta1 = M22*shD + M12*handVel;
				
			switch (controller) {
				case 1: // stabilization
					if (eta2 == 0) {
						u = -GAIN_K1*eta1;
					}
					else {
						u = -GAIN_K1*eta1*sin(eta2)/eta2 - GAIN_K2*eta2;
					}
					alpha = (u - COEFF_sig2*eta1)/COEFF_sig3;
					if (eta2 == 0) {
						DuDeta1 = -GAIN_K1;
						DuDeta2 = -GAIN_K2;
					}
					else {
						DuDeta1 = -GAIN_K1*sin(eta2)/eta2;
						DuDeta2 = -GAIN_K1*eta1*( (cos(eta2)*eta2 - sin(eta2))/(eta2*eta2) ) - GAIN_K2;
					}
					DalphaDeta1 = (DuDeta1 - COEFF_sig2)/COEFF_sig3;
					DalphaDeta2 = DuDeta2/COEFF_sig3;

					vInp = DalphaDeta1*COEFF_sig1*sin(eta2) + DalphaDeta2*(COEFF_sig2*eta1+COEFF_sig3*handVel) - eta2*COEFF_sig3 - GAIN_C*(handVel - alpha);
					break;
				
				case 2: // speed control
					/*z = (handTheta - thetah_offset) - target_thetahD*(time - time_offset); // the difference is here. time (sec) = cnt/SAMPLE_RATE
					h = eta2 - COEFF_sig3*z;
					hD = eta2D - COEFF_sig3*(handVel - target_thetahD); 
					hDD = COEFF_sig2*COEFF_sig1*sin(eta2); 
					hDDD = COEFF_sig2*COEFF_sig1*cos(eta2)*eta2D; 
					
					u = -GAIN_K1S*hDDD - GAIN_K2S*hDD - GAIN_K3S*hD - GAIN_K4S*h;
					vInp = ( u - COEFF_sig2*COEFF_sig1*sin(eta2)*(-eta2D + COEFF_sig1*COEFF_sig2*cos(eta2)) )/(COEFF_sig1*COEFF_sig2*COEFF_sig3*cos(eta2)); // slightly better
					*/
					/*************************************************************/
					// calculation of the control input (acceleration command)
					eta1Bar = eta1 - COEFF_ETA1_STAR;
					z = handVel - TARGET_THETAHD;
					if (eta2 == 0) {
						u = -GAIN_K1*eta1Bar;
					}
					else {
						u = -GAIN_K1*eta1Bar*sin(eta2)/eta2 - GAIN_K2*eta2;
					}
					alpha = (u - COEFF_sig2*eta1Bar)/COEFF_sig3;
					if (eta2 == 0) {
						DuDeta1Bar = -GAIN_K1;
						DuDeta2 = -GAIN_K2;
					}
					else {
						DuDeta1Bar = -GAIN_K1*sin(eta2)/eta2;
						DuDeta2 = -GAIN_K1*eta1Bar*( (cos(eta2)*eta2 - sin(eta2))/(eta2*eta2) ) - GAIN_K2;
					}
					DalphaDeta1Bar = (DuDeta1Bar - COEFF_sig2)/COEFF_sig3;
					DalphaDeta2 = DuDeta2/COEFF_sig3;

					vInp = DalphaDeta1Bar*COEFF_sig1*sin(eta2) + DalphaDeta2*(COEFF_sig2*eta1Bar+COEFF_sig3*z) - eta2*COEFF_sig3 - GAIN_C*(z - alpha);
					break;

				case 3:  // position control
					z = handTheta - target_thetah;
					h = eta2 - COEFF_sig3*z;
					hD = eta2D - COEFF_sig3*handVel;  
					hDD = COEFF_sig2*COEFF_sig1*sin(eta2); 
					hDDD = COEFF_sig2*COEFF_sig1*cos(eta2)*eta2D; 
					
					if (integral_flag == 0) {
						if ( fabs(z) < 0.4 && fabs(handVel) < 0.1 ) { // to find when the pure FL action ends
							integral_flag = 1; // turn on the integral action
						}
					}
					/* For addition of integral control */
					//integral_flag = 1;
					if(HW->motorStatus == MOTOR_ON && integral_flag == 1) {
						err_sum = err_sum + z*sec;
					}
					else {
						err_sum = 0;
					} 
					// when no integral action is needed.
					err_sum = 0;
					
					/*if (posControl_flag == 2) { // need a larger control gain.
						u = -GAIN_K1P2*hDDD - GAIN_K2P2*hDD - GAIN_K3P2*hD - GAIN_K4P2*h + 80*err_sum;
					}
					else {
						u = -GAIN_K1P*hDDD - GAIN_K2P*hDD - GAIN_K3P*hD - GAIN_K4P*h + 80*err_sum;
					}
					*/

					u = -GAIN_K1P*hDDD - GAIN_K2P*hDD - GAIN_K3P*hD - GAIN_K4P*h + 80*err_sum;
					//u = -GAIN_K1P2*hDDD - GAIN_K2P2*hDD - GAIN_K3P2*hD - GAIN_K4P2*h + 80*err_sum;
					
					//integral_flag = 0;
					if (integral_flag == 1) { 
						u = -GAIN_K1P3*hDDD - GAIN_K2P3*hDD - GAIN_K3P3*hD - GAIN_K4P3*h + 80*err_sum;
						//u = -GAIN_K1P2*hDDD - GAIN_K2P2*hDD - GAIN_K3P2*hD - GAIN_K4P2*h + 80*err_sum;
					}
					
					vInp = ( u - COEFF_sig2*COEFF_sig1*sin(eta2)*(-eta2D + COEFF_sig1*COEFF_sig2*cos(eta2)) )/(COEFF_sig1*COEFF_sig2*COEFF_sig3*cos(eta2)); // slightly better
					break;

				default:
					break;
			}
			
			/*************************************************************/
			aCmd = vInp; // calculated acceleration command
			
			obj.theta_prev = obj.theta;
			obj.angVel_prev = obj.angVel;
			handTheta_prev = handTheta;
			handVel_prev = handVel;
			eta2D_prev = eta2D;
			eta2_prev = eta2;
		}
		else { // Because the vision system keeps sending the data, QNX must read them, otherwise the vision system will get a send error.
			VNET->Process();
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
		enc = -(HW->ReadEncoder(IoHardware::ENC_0)); // direct read. Not working?
		pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
		vel = (pos - pos_prev) / elapsedSec; // to calculate more exact velocity.
				
		// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
		if(HW->motorStatus == MOTOR_ON) {
			cnt++;
		
			// acceleration inner loop
			// Notice that using no filtering velocity
			vCmd = vCmd_prev + aCmd / SAMPLE_RATE; 
			pCmd = pCmd_prev + vCmd_prev / SAMPLE_RATE + 0.5 * aCmd / (SAMPLE_RATE*SAMPLE_RATE);
			
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
		if(iCmd > (MAX_CURRENT_MA) ){ // 1.6 A
			//if (iCmd > 2.2 ) { 
			//	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			//	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			//	HW->motorStatus = MOTOR_OFF;
			//	printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, aCmd:%f feedforwardVal:%f cnt:%d\n", iCmd, pCmd, pos, vCmd, vel, aCmd, feedforwardVal, cnt);
			//	FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			//	lost = 1;
			//}
			iCmd = MAX_CURRENT_MA;
		}
		else if(iCmd < (-MAX_CURRENT_MA) ){
			//if (iCmd < -2.2 ) { 
			//	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
			//	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
			//	HW->motorStatus = MOTOR_OFF;
			//	printf("iCmd:%f, pCmd:%f, pos:%f, vCmd:%f, vel:%f, aCmd:%f feedforwardVal:%f cnt:%d\n", iCmd, pCmd, pos, vCmd, vel, aCmd, feedforwardVal, cnt);
			//	FATAL_ERROR("MOTOR CURRENT TOO HIGH");
			//	lost = 1;
			//}
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
		num[0] = vInp; 
		num[1] = eta1; 
		num[2] = pos; 
		num[3] = eta2;
		
		//MNET->Process();

	} // while()
	
	// for some reason, this does not work.
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
}

