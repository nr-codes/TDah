#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>

#include <unistd.h>

#include "PeriodicTask.h"
#include "Qnx.h"

#define TIMER_PULSE_CODE	_PULSE_CODE_MINAVAIL

PeriodicTask::PeriodicTask(){
	mRunning = 0;
	mOverRun = 0;
	mChannelId = mConnectId = -1;
}

int PeriodicTask::Init(char *name, double rate, double *actual_rate, int priority){
	mTaskName = name;

	// PeriodicTask::Timer
	if(InitTimer(priority, rate, actual_rate) == -1){
		printf("%s:InitThread:InitTimer failed\n", mTaskName);
		exit(1);
	}
	
	// Execute PeriodicTask::TimerThread as a thread. See Qnx.C for InitThread(). 
	if(InitThread(&mTimerThreadId, priority-1, TimerThread, (void *)(this))==-1){
		printf("%s:InitThread:TimerThread failed\n", mTaskName);
		exit(1);
	}

	//  Execute PeriodicTask::TimerControlThread as a thread. See Qnx.h for InitThread().
	if(InitThread(&mTimerControlThreadId, priority, TimerControlThread, (void *)(this))==-1){
		printf("%s:InitThread:TimerControlThread failed\n", mTaskName);
		exit(1);
	}

	return 1;
}

PeriodicTask::~PeriodicTask(){

}

int PeriodicTask::InitTimer(int priority, double rate, double *actual_rate){
	sigevent timerEvent;
	timer_t timerId;
	itimerspec timerSpec;
	struct _clockperiod period;
	struct timespec clockResSpec;
	long actual_period_ns;

	// Set qnx kernel tick size
	period.nsec = (long) (1./rate * 1.e9);
	period.fract = 0;
	ClockPeriod(CLOCK_REALTIME, &period, NULL, NULL);
	clock_getres(CLOCK_REALTIME, &clockResSpec);

	// Main clock frequency is 1.1931816 Mhz and only integer divisors are possible
	// get the actual ticksize
	actual_period_ns = clockResSpec.tv_nsec;
	*actual_rate = 1./((double)actual_period_ns / 1.e9);
	

	// Create a pulse channel
	if((mChannelId = ChannelCreate(NULL)) == -1){
		return -1;
	}

	// Create a pulse channel connection
	if((mConnectId = ConnectAttach(0, 0, mChannelId, _NTO_SIDE_CHANNEL, 0)) == -1){
		return -1;
	}

	// Initialize the pulse structure
	SIGEV_PULSE_INIT(&timerEvent, mConnectId, priority, TIMER_PULSE_CODE, 0);

	// Create the timer
	if(timer_create(CLOCK_REALTIME, &timerEvent, &timerId) ==  -1){
		return -1;
	}

	// Set timer parameters
	timerSpec.it_value.tv_sec 		= 0;
	timerSpec.it_value.tv_nsec 	= actual_period_ns;
	timerSpec.it_interval.tv_sec 	= 0;
	timerSpec.it_interval.tv_nsec = actual_period_ns;

	if(timer_settime(timerId, 0, &timerSpec, NULL) == -1){
		return -1;
	}
		

	if(sem_init(&mTimerSemaphore, 1, 0) == -1){
		return -1;
	}

	return 1;
}

void *PeriodicTask::TimerControlThread(void *arg){
	_pulse pulseMsg;
	PeriodicTask *taskInst;

	taskInst = (PeriodicTask *)arg;

	while(1){
		// wait for the pulse from the timer
		if(MsgReceivePulse(taskInst->mChannelId, &pulseMsg, sizeof(_pulse), NULL) == -1){
			printf("%s:TimerControlThread:MsgReceivePulse failed.\n", taskInst->mTaskName);
			exit(1);
		}

		// verify that this is the correct pulse
		if(pulseMsg.code == TIMER_PULSE_CODE){
			if(taskInst->mRunning){
				taskInst->mOverRun++;
			}
			else{
				// unblock task thread
				if(sem_post(& (taskInst->mTimerSemaphore) ) == -1){
					printf("$s:TimerControlThread:sem_post failed.\n", taskInst->mTaskName);
					exit(1);
				}
			}
		}
		else{
			printf("%s:TimerControlThread:Received unknown pulse code.\n", taskInst->mTaskName);
			exit(1);
		}

	}
}

int PeriodicTask::TimerWait(){

	mRunning = 0;

	if(sem_wait(&(mTimerSemaphore))== -1){
		printf("%s:TimerThread:sem_wait failed.\n", mTaskName);
		return -1;
	}

	mRunning = 1;

	return 1;
}

void *PeriodicTask::TimerThread(void *arg){
	PeriodicTask *taskInst;

	taskInst = (PeriodicTask *)arg;

	taskInst->Task();

}
