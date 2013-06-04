#ifndef VisionNet_h
#define VisionNet_h

#include <sys/types.h>
#include <netinet/in.h>
#include "AperiodicTask.h"

//class FifoQ;
class VisionNet;

extern VisionNet 	*VNET;


#define VISION_NET_NUM_CH	3

// This is a modification from MatlabNet.C/h 
// Whereas MatlabNet is for sending data to display at a host computer,
// This is for receiving data from the vision computer.
//
// Basically, we don't have to create another "server" socket because MatlabNet already has one.
// However, creating another one is not also a problem at all and this is easier in coding than using MatlabNet's.
class VisionNet : public AperiodicTask{
    public:
        // Constuctor
        VisionNet();
        // Destructor
        ~VisionNet();
		
		int Init(double rate, int priority);
		void Process();
		void AddSignal(int ch, double *val);
		int Recv(); // for direction data receoption.

    private:
		double mSampleRate;
		//FifoQ *mpFifo;

		//char mBuf[VISION_NET_NUM_CH*8];
		//when the program is in the camera mode, only [VISION_NET_NUM_CH*8] might be enough.
		//However, when recv() is called on the thread, we may need a larger size of buffer.
		char mBuf[128]; 

		double *mpValPtrArr[VISION_NET_NUM_CH];
		//double mpValBuf[VISION_NET_NUM_CH]; // not necessary for vision TCP/IP

		int mInitialized;
		//int mIsSocketAlive;

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

#endif // VisionNet_h


