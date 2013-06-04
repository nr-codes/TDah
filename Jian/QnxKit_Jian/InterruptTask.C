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

#include "InterruptTask.h"
#include "Qnx.h"


InterruptTask::InterruptTask(){
	mRunning = 0;

}

int InterruptTask::Init(char *name, int priority, int intNum){
	mTaskName = name;

	mIntNum = intNum;

	if(InitThread(&mIntThreadId, priority, IntThread, (void *)(this))==-1){
		printf("%s:InitThread:IntThread failed\n", mTaskName);
		exit(1);
	}

	return 1;
}

InterruptTask::~InterruptTask(){

}

void *InterruptTask::IntThread(void *arg){
	static int interruptId;
	struct sigevent event;
	int s;

	InterruptTask *taskInst;
	taskInst = (InterruptTask *)arg;

	event.sigev_notify = SIGEV_INTR;
	interruptId = InterruptAttachEvent(taskInst->mIntNum, &event, 0);

	while(1){
		if(InterruptWait(0, NULL) == -1){
			continue;
		}
		
		InterruptUnmask(taskInst->mIntNum, interruptId);
		taskInst->mRunning = 1;
		taskInst->IntTask();
		taskInst->mRunning = 0;
		
	}

}

