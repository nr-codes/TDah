#ifndef SampleLoopTask_h
#define SampleLoopTask_h

#include <sys/neutrino.h>
#include <inttypes.h>
#include <sys/syspage.h>

#include "PeriodicTask.h"
#include <semaphore.h>

#define CAMERA_SAMPLE_RATE 250

class SampleLoopTask : public PeriodicTask{
    public:
		// Constuctor
		SampleLoopTask();
		// Destructor
		~SampleLoopTask();

		int Init(char *name, double rate, double *actual_sample_rate, int priority);
		
		// feedback gains
		double Kp;
		double Kd;
		double Ki;
		
		double pCmd;
		double currentI; // test for checking static friction
		
		// camera commands
		int done;
		int lost;
		int camera;
		
		double SampleRate;
			
		typedef struct {
			int numSamples;
			double mu;
			double mu_prev;
			double s_sq;
			double s_sq_prev;
			double std;
			double min;
			double max;
			int maxIndex;
			int minIndex;
			int i;
			char go;
			double t;
			sem_t sem;
		}loopTiming;
		
		// region 0 is -5V, region 1 is +5V,
		// analog input was used to allow for more than 2 regions to be used in the future
		// voltages are currently rounded to the nearest integer
		enum RoiNum{
			ROI_0 = -5, // analog voltage corresponding to a particular region
			ROI_1 = 5
		};
		  
		typedef struct {
			double x0; // location of region 0
			double y0;
			double x1; // location of region 1
			double y1;
			
			double x; // location and orientation of centroid
			double y;
			double theta;

			// added for angular velocity calculation by Ji-Chul
			double angVel; // angular velocity
			double theta_prev;
			double angVel_prev;

		}objectInfo;
			
		  
		uint64_t cps;
		
		uint64_t ncycles;
		uint64_t ncycles_prev;
		double sec;
		//These three are for timing the control loop, which should be run as the same freq as camera
		double sec_cl;
		double ncycles_cl;
		double ncycles_prev_cl;

		loopTiming lp;
	  
		int TimingStart(double sec);
		int TimingProcess();
		
		objectInfo obj;

    private:
		void Task();
	  
		double enc;
		double pos;
		double vel;
		double acc;
		
		double vCmd;
		double aCmd;
		double iCmd;
		double ampVCmd;

		// for controller implementation
		double handTheta;
		double handTheta_prev;
		double handVel;
		double handVel_prev;
		double sh;  
		double shD;
		double eta1;
		double eta2;
		//double xi;
		double alpha_eta; 
		double DalphaDeta1;
		double DalphaDeta2;
		double vInp; 
		// for some reason, I cannot have more variables - just one more double now - maybe because a memory problem?
		// double eta1D; 
		// double eta2D;
		
		
		


};

#endif // SampleLoopTask_h


