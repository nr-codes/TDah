#include "Qnx.h"


int InitThread(pthread_t *threadId, int prio, void *(*func)(void *), void *param){
	pthread_attr_t attr;
	sched_param sched;

	if(pthread_attr_init(&attr) != EOK)
		return -1;

	if(pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != EOK)
		return -1;

	if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != EOK)
		return -1;

	if(pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != EOK)
		return -1;

	if(pthread_attr_getschedparam(&attr, &sched) != EOK)
		return -1;

	sched.sched_priority = prio;

 	if(pthread_attr_setschedparam(&attr, &sched) != EOK)
		return -1;


	if(pthread_create(threadId, &attr, func, param) != EOK)
		return -1;

	if(pthread_attr_destroy(&attr) != 0)
		return -1;

	return 1;
}

