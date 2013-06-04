///////////////////////////////////////////////////////////////////////////////
// Periodic Task Class Definition
//
// A periodic task is invoked by means of a timer with a period and priority
// specified in the constructor.  It is defined by subclassing the PeriodicTask
// taking care to call the base class constructor and defining the Task()
// function.  The task is started with a call to Init() function.
//
// PeriodicTask class starts two threads: a timer control thread and a task
// thread.  The control thread is triggered by the timer and controls the
// execution of the task thread through a semaphore.  The mOverRun counter
// is incremented every time the task thread is not ready to run
// when the timer pulse comes in.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef PeriodicTask_h
#define PeriodicTask_h

#include <semaphore.h>
#include <pthread.h>

class PeriodicTask{
    public:
        // Constuctor
        PeriodicTask();
        // Destructor
        ~PeriodicTask();

		  int Init(char *name, double rate, double *actual_rate, int priority);

		  int mOverRun;

    private:
		unsigned char mRunning;

		sem_t mTimerSemaphore;

		int mChannelId;
		int mConnectId;

		char *mTaskName;

		pthread_t mTimerThreadId;
		pthread_t mTimerControlThreadId;

		int InitTimer(int priority, double rate, double *actual_rate);
		static void *TimerControlThread(void *arg);
		static void *TimerThread(void *arg);

		virtual void Task()=0;

	protected:
		int TimerWait();

};

#endif // PeriodicTask_h


