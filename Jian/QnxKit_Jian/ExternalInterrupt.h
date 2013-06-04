#ifndef ExternalInterrupt_h
#define ExternalInterrupt_h

#include "InterruptTask.h"

class ExternalInterrupt : public InterruptTask{
    public:
        // Constuctor
        ExternalInterrupt();
        // Destructor
        ~ExternalInterrupt();

		int Init(char *name, int priority, int intNum);

    private:
		void IntTask();
};

#endif // ExternalInterrupt_h


