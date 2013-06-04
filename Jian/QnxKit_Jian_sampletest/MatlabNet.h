#ifndef MatlabNet_h
#define MatlabNet_h

#include <sys/types.h>
#include <netinet/in.h>
#include "AperiodicTask.h"

class FifoQ;
class MatlabNet;

extern MatlabNet 	*MNET;

#define MATLAB_NET_NUM_CH	4

class MatlabNet : public AperiodicTask{
    public:
        // Constuctor
        MatlabNet();
        // Destructor
        ~MatlabNet();
		
		int Init(double rate, int priority);
		void Process();
		void AddSignal(int ch, double *val);

    private:
		double mSampleRate;
		FifoQ *mpFifo;

		unsigned char mBuf[MATLAB_NET_NUM_CH*8];

		double *mpValPtrArr[MATLAB_NET_NUM_CH];
		double mpValBuf[MATLAB_NET_NUM_CH];

		int mInitialized;

		// TCP/IP
		int mSocket;
		int mSessionSocket;
		struct sockaddr_in mServerAddr;
		unsigned char validSession;

		int TcpIpInit();

		// Task function
		void Task();

		static void sig_handler(int signo){};

		int divisorCount;

};

#endif // MatlabNet_h


