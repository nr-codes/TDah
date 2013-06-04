#ifndef VISION_TCP_H_
#define VISION_TCP_H_

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib") // linking to the library. necessary!

class VisionTCP;

#define VISION_NET_NUM_CH	3

class VisionTCP 
{
    public:
        // Constuctor
        VisionTCP();
        // Destructor
        ~VisionTCP();
		
		int Init();
		int Send(int ROI, double x, double y);
		//int Send(int ROI, double x);

    private:
		int mROI; 
		double mx;
		double my;

		char mbuf[8*VISION_NET_NUM_CH];

		bool mInitialized;

		// TCP/IP
		int mSocket;
		int mSessionSocket;
		//struct sockaddr_in mServerAddr;
		SOCKADDR_IN mServerAddr;

};

#endif /* VISION_TCP_H_ */

