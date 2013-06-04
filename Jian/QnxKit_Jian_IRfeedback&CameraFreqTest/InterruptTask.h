///////////////////////////////////////////////////////////////////////////////
// Interrupt Task Class Definition
//
// Interrupt triggered task
// 
///////////////////////////////////////////////////////////////////////////////

#ifndef InterruptTask_h
#define InterruptTask_h

#include <semaphore.h>
#include <pthread.h>

class InterruptTask{
    public:
        // Constuctor
        InterruptTask();
        // Destructor
        ~InterruptTask();

		int Init(char *name, int priority, int intNum);

    private:
		unsigned char mRunning;
		int mIntNum;
		char *mTaskName;

		pthread_t mIntThreadId;

		static void *IntThread(void *arg);

		virtual void IntTask()=0;
};

#endif // InterruptTask_h


