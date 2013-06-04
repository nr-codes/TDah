#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <sys/neutrino.h>

int InitThread(pthread_t *threadId, int prio, void *(*func)(void *), void *param);

