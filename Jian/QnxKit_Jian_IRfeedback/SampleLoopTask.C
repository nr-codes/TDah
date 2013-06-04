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
#define pi 3.1415926
#define acc_sac 2
#define err_sac 0.03

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

	printf("\nSampleloop Init.\n");

	PeriodicTask::Init(name, rate, actual_sample_rate, priority);

	SampleRate = *actual_sample_rate;
	
	printf("actual sample rate %lf\n", *actual_sample_rate);

	printf("\ncpu_frequency:%dHz\n", SYSPAGE_ENTRY( qtime )->cycles_per_sec);
	
	for(int i=0; i<MATLAB_NET_NUM_CH; i++){
		MNET->AddSignal(i, &(num[i]));
	}
	for(int i=0; i<VISION_NET_NUM_CH; i++){
		VNET->AddSignal(i, &(vnum[i]));
	}
	
	// initialize gains
	Kp = 0.26; //0.32; //0.4; //0.2
	Kd = 0.00; //0.01;
	Ki = 1.5; //4.2; //0.25;

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
		
	printf("\nSampleloop Task.\n");

	int cnt = 0;
	int region;
	int attempt;
	
	static int dout = 1;
	static int last_status = 0;
	static double pos_prev = 0;
	static double vel_prev = 0;
	static int cnt_prev = 0;
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
	double enc_vel;
	double enc_vel_prev;

	double hwc, hwsec, hwc_prev;
	double tt, tt_prev = 0.;

	while(!lost){

		
		TimerWait();
		
		ncycles = ClockCycles();
		sec=(double)(ncycles - ncycles_prev)/cps;
		ncycles_prev = ncycles;
		
		TimingProcess();
		
			
		if (VNET->IFDataRecv){
		//if (1){
			
		

			//vnum[0] = -99;
			//int n = VNET->Recv();
			//VNET->n_tcp = n;

			ncycles_cl = ClockCycles();
			sec_cl =(double)(ncycles_cl - ncycles_prev_cl)/cps;
			ncycles_prev_cl = ncycles;

			VNET->IFDataRecv = 0;
			VNET->Process();

			// Read Inputs
			//hwc_prev = ClockCycles();

			HW->ProcessInput();

			//hwc = ClockCycles();
			//hwsec =(double)(hwc - hwc_prev)/cps;

		
			if (VNET->n_tcp > 0) {

				if (abs(vnum[0]) < .1){
					HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_ON);
					HW->motorStatus = MOTOR_ON;
					cnt = 0;
					error = 0.;
					vel = 0.;
					printf("\n\n\n\nMotor is ON!\n\n");
				}
				
				cnt = int(vnum[0]);

				tt_prev = tt;
				tt = vnum[2];

				//printf("\n n:%d cnt:%d runcnt:%f w:%f tt:%f\n", VNET->n_tcp ,cnt, vnum[0], vnum[1], vnum[2]);

				if (abs(VNET->n_tcp - 24) > 1){

					if (cnt < 80){
						iCmd = 0;
						error = 0.;
						HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0);
						
					}

					printf("\n\nData size is incorrect!!! cnt:%d\n", cnt);
					continue;
				}
			}
			else if (VNET->n_tcp == -1) {
			  printf("No data!!\n");
			  continue;
			}

			
		
			// Get status of camera
			//last_status = HW->ReadDigitalBit(IoHardware::FRAME_STATUS);
		
			// Send out pulse to trigger camera
			//HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 1);
			//HW->WriteDigitalBit(IoHardware::CAMERA_TRIGGER, 0);
		
			
			
		
			if (isnan(vnum[1])){
				vel = 0;
				if (cnt < 80){
					iCmd = 0;
					error = 0.;
					HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0);
				}
				printf("Vel error! cnt:%d\n", cnt);
				continue;
			}
			else{
				vel = vnum[1];
			}

			if (cnt == cnt_prev && cnt != 0){
				printf("\nRecv same frame# twice!! cnt:%d\n", cnt);
				continue;
			}

			
			// This is necessary for not having motor run unexpectedly when swtiching from motor off to motor on.
			//if(HW->motorStatus == MOTOR_ON) cnt++;
		
			// feedback
			
			//This part is getting feedback from encoder
			enc = -(HW->GetEncoderCount(IoHardware::ENC_0)); // sign change to agree with camera
			pos = (enc * ENC_RAD_PER_CNT) / GR / 4; // convert to rad. GR:gear ratio (50). 4?
			//enc_vel = (pos - pos_prev) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;
			enc_vel = (pos - pos_prev) / sec_cl;
			
			if (abs(enc_vel) > 2.5 || (cnt - cnt_prev)>1){
				/*printf("\nshit!  cnt - cnt_prev = %d.\n", (cnt - cnt_prev));
				printf("pcmd:%f, pos:%f, pos_prev:%f, vcmd:%f, vel:%f, enc_vel:%f, cnt:%d, cnt_prev:%d\n", 
					pcmd, pos, pos_prev, vcmd, vel, enc_vel, cnt, cnt_prev);
				printf("control loop time: %f", sec_cl);*/
			}


			//if ((cnt - cnt_prev) > 1) printf("\n\nwocao! d = %d.\n\n", (cnt - cnt_prev));
			//attempt = 0;
			//while (abs((enc_vel - enc_vel_prev)/(cnt - cnt_prev)) * CAMERA_SAMPLE_RATE > 10 
			//	  && attempt < 20)
			//{
			//	//enc_vel = (pos - pos_prev) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;
			//	//enc_vel_prev = enc_vel;
			//	if (cnt <= 81){
			//		enc_vel_prev = enc_vel;
			//		break;
			//	}
			//	else{
			//		/*enc = -(HW->ReadEncoder(IoHardware::ENC_0));
			//		pos = (enc * ENC_RAD_PER_CNT) / GR / 4;
			//		enc_vel = (pos - pos_prev) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;
			//		attempt++;*/

			//		enc_vel = enc_vel_prev;
			//	}
			//}
		
			//printf("%d;", attempt);
			//if (attempt) printf("%f\n", abs(enc_vel - enc_vel_prev));

			//enc_vel_prev = enc_vel;
			//num[2] = vel; 
			//vel = enc_vel;

			// constant speed input

			pCmd = .5*3.141592 * cnt/CAMERA_SAMPLE_RATE; // 4.5 2*3.141592
			//pCmd = pi + .5*pi * sin(.1*pi*cnt/CAMERA_SAMPLE_RATE);
		
			// for constant speed controller
			vCmd = (pCmd - pCmd_prev) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;
			aCmd = (vCmd - vCmd_prev) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;

			if (abs(aCmd) > 10) aCmd = aCmd/500;
			//aCmd = 0;
			//error += (pos - pCmd);
			error += (vCmd - vel) * (cnt - cnt_prev) / CAMERA_SAMPLE_RATE; 
			if (error > err_sac) error = err_sac;
			else if (error < -err_sac) error = -err_sac;
			if (isnan(error)) error = 0.0;
		
			//error = 0;
			//acc = ((vCmd - vel) - (vCmd_prev - vel_prev)) / (cnt - cnt_prev) * CAMERA_SAMPLE_RATE;

			acc = ((vCmd - vel) - (vCmd_prev - vel_prev)) / sec_cl;

			if (abs(acc) > acc_sac){
				if (acc > 0) acc = acc_sac;
				if (acc < 0) acc = -acc_sac;
			}
				
		
			iCmd = calcI(vCmd, aCmd); // feedforward control
			//iCmd = 0;
			iCmd += ( Kp* (vCmd - vel) + Kd * (acc) + Ki * error); 

			//printf("cnt:%d\n",cnt);
			if (cnt < 80) iCmd = 0;
			
			//iCmd = 0.1;
		
			/*if ( abs(error) >= err_sac || abs(vel) > 2.5 ){
				printf("\ncnt:%d\n",cnt);
				printf("pCmd:%f, pos:%f, vCmd:%f, vel:%f, acc:%f, aCmd:%f ,iCmd:%f, error:%f\n", 
					pCmd, pos, vCmd, vel, acc, aCmd, iCmd, error);
				
			}*/

			/*if (abs(sec_cl - 1.0/double(CAMERA_SAMPLE_RATE)) > 0.002){
				printf("\n!!sec_cl:%f, cnt:%d, prev_cnt:%d\n", sec_cl, cnt, cnt_prev);
			}*/

			pos_prev = pos;
			vel_prev = vel;
			cnt_prev = cnt;

			pCmd_prev = pCmd;
			vCmd_prev = vCmd;
		
			//limit current based on motor specs
			if(iCmd > MAX_CURRENT_MA){ // 1.6 A
				iCmd = MAX_CURRENT_MA;
			}
			else if(iCmd < -MAX_CURRENT_MA){
				iCmd = -MAX_CURRENT_MA;
			}

			// for safety
			if (fabs(vel) > 10 ) {//rad/sec
				HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
				HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
				HW->motorStatus = MOTOR_OFF;
				FATAL_ERROR("MOTOR RUNS TOO FAST");
			}
			
		
			

			ampVCmd = -iCmd * AMP_GAIN; // convert to analog signal. +-10 V.  -0.86 for 4.5 rad/s. for test use -0.8
			// sign change to agree with camera
		
			//output signal to amp
			if(!done){
				HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, ampVCmd);
				//HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.0);
			}
			else{
				HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.0);
			}
		
			HW->ProcessOutput();

			//printf("Output Control! cnt=%d\n", cnt);
			

			/*hwc = clockcycles();
			hwsec =(double)(hwc - hwc_prev)/cps;*/

			// set outputs
			//num[0] = pos; 
			num[0] = sec;
			num[1] = sec_cl;
			num[2] = vel; 
			num[3] = enc_vel;
			//num[2] = enc_vel;
			//num[3] = hwsec; 
			//num[3] = enc;
			/*if (abs(tt - tt_prev) > 1) tt = tt_prev + 0.004;
			num[3] = tt - tt_prev; */
		
			//MNET->Process();

			//tt_prev = tt;

		} //if
		else {

			VNET->Process();

			if (cnt < 80){
				iCmd = 0;
				HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0);
				HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.0);
			}

			//printf("skip!\n");
		}


	} // while()
	
	// for some reason, this does not work.
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	delay(10);
	FATAL_ERROR(err_msg);
	
}

