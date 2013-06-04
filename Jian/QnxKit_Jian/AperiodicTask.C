#include<stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>

#include "AperiodicTask.h"
#include "Qnx.h"

#define TASK_PULSE_CODE	_PULSE_CODE_MINAVAIL

AperiodicTask::AperiodicTask(){
	mChannelId = mConnectId = -1;
	mPriority = 0;
}

AperiodicTask::~AperiodicTask(){

}

void AperiodicTask::Init(char *name, int priority){
	mTaskName = name;
	mPriority = priority;

	// Create a pulse channel
	if((mChannelId = ChannelCreate(NULL)) == -1){
		printf("%s:Init:ChannelCreate failed\n", mTaskName);
		exit(1);
	}

	// Connect to channel
	if((mConnectId = ConnectAttach(0, 0, mChannelId, _NTO_SIDE_CHANNEL, 0)) == -1){
		printf("%s:Init:ConnectAttach failed\n", mTaskName);
		exit(1);
	}

	if(InitThread(&mThreadId, priority, TaskThread, (void *)(this))==-1){
		printf("%s:Init:InitThread:TaskThread failed\n", mTaskName);
		exit(1);
	}
}

int AperiodicTask::Trigger(int value){

	if(MsgSendPulse(mConnectId, mPriority, TASK_PULSE_CODE, value) == -1){
		printf("%s:Trigger:MsgSendPulse failed.\n", mTaskName);
		exit(1);
	}

}

int AperiodicTask::TriggerWait(){
		_pulse pulseMsg;

		if(MsgReceivePulse(mChannelId, &pulseMsg, sizeof(_pulse), NULL) == -1){
			printf("%s:TaskThread:MsgReceivePulse failed.\n", mTaskName);
			exit(1);
		}

		if(pulseMsg.code == TASK_PULSE_CODE){
			return pulseMsg.value.sival_int;
		}
		else{
			return -1;
		}


}

void *AperiodicTask::TaskThread(void *arg){
	AperiodicTask *taskInst;

	taskInst = (AperiodicTask *)arg;

	taskInst->Task();

}



