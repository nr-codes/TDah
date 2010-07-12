#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <math.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024

#define NUM_ROI 2
#define ROI_PACKET_SIZE 17
#define TOTAL_PACKET_SIZE (NUM_ROI*ROI_PACKET_SIZE)

#define R_POS 0
#define R_VAL 'R'
#define X_POS 2
#define X_VAL 'X'
#define Y_POS 7
#define Y_VAL 'Y'
#define T_POS 12
#define T_VAL 'T'
#define P_POS 17
#define P_VAL 'P'

#define QUIT (_kbhit() && _getch() == 'q')


static char reply[TOTAL_PACKET_SIZE];
static char text[100];
static HANDLE hComm;
static struct {
	char id;
	float x;
	float y;
	unsigned int ts;
} rois[NUM_ROI];

int open_comm()
{
	hComm = CreateFile( TEXT("\\\\.\\COM12"),
                    GENERIC_READ | GENERIC_WRITE, 
                    0, 
                    0, 
                    OPEN_EXISTING,
                    0,
                    0);

	if (hComm == INVALID_HANDLE_VALUE) {
		return -1;
	}

	DCB dcb = {0};
	if(GetCommState(hComm, &dcb) == 0) {
		CloseHandle(hComm);
		return -3;
	}

	dcb.BaudRate = CBR_115200;
	dcb.fParity = FALSE;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	if(SetCommState(hComm, &dcb) == 0) {
		CloseHandle(hComm);
		return -4;
	}

	return 0;
}

int close_comm()
{
	CloseHandle(hComm);
	return 0;
}

int main(void)
{
	unsigned int i, offset;
	DWORD bytes = 0;
	float theta;
	int u[NUM_ROI], v[NUM_ROI];

	IplImage *rgb = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 3);

	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );
	CvMat *world_R = (CvMat *)cvLoad( "Rotation.xml" );
	CvMat *world_Rvec = cvCreateMat(3, 1, CV_32FC1);
	CvMat *world_T = (CvMat*)cvLoad( "Translation.xml" );
	CvMat *pp = cvCreateMat(1,1, CV_32FC3);
	CvMat *wp = cvCreateMat(1,1, CV_32FC3);

	if(!rgb || !intrinsic || !distortion || !world_Rvec || !world_T || !pp || !wp) {
		return -1;
	}
	cvRodrigues2(world_R, world_Rvec);
	cvNamedWindow("PuckTracker", CV_WINDOW_AUTOSIZE);

	open_comm();

	while(!QUIT) {
		// get response
		memset(reply, 0, TOTAL_PACKET_SIZE);

		while(reply[0] != R_VAL && !QUIT) {
			if(ReadFile(hComm, reply, 1, &bytes, NULL) == FALSE) {
				printf("error reading from comm port\n");
				break;
			}
		}

		int sent = bytes;
		while(sent < TOTAL_PACKET_SIZE && !QUIT) {
			if(ReadFile(hComm, reply + sent, 1, &bytes, NULL) == FALSE) {
				printf("error reading from comm port\n");
				break;
			}

			// is packet correct so far?
			if((sent == X_POS && reply[sent] != X_VAL) ||
				(sent == Y_POS && reply[sent] != Y_VAL) ||
				(sent == T_POS && reply[sent] != T_VAL) ||
				(sent == ROI_PACKET_SIZE+R_POS && reply[sent] != P_VAL) ||
				(sent == ROI_PACKET_SIZE+X_POS && reply[sent] != X_VAL) ||
				(sent == ROI_PACKET_SIZE+Y_POS && reply[sent] != Y_VAL) ||
				(sent == ROI_PACKET_SIZE+T_POS && reply[sent] != T_VAL)) {
				// assume bad packet			
				printf("bad packet\n");
				break;
			}

			sent += bytes;
		}

		if(sent == TOTAL_PACKET_SIZE) {
			// assemble packet into (x,y) coordinates
			for(i = 0; i < NUM_ROI; i++) {
				offset = i*ROI_PACKET_SIZE;
				rois[i].id = reply[offset + 1];
				memcpy((char *)&(rois[i].x), 
					(char *) &(reply[offset+3]), sizeof(float));	
				memcpy((char *)&(rois[i].y), 
					(char *) &(reply[offset+8]), sizeof(float));
				memcpy((char *)&(rois[i].ts), 
					(char *) &(reply[offset+13]), sizeof(unsigned int));

				if(rois[i].id < 0) {
					printf("roi %d lost\n", ~rois[i].id);
					break;
				}

				// world to image plane
				wp->data.fl[0] = rois[i].x;
				wp->data.fl[1] = rois[i].y;
				wp->data.fl[2] = 0;

				cvProjectPoints2(wp, world_Rvec, world_T, 
					intrinsic, distortion, pp);

				u[i] = cvRound(pp->data.fl[0]);
				v[i] = cvRound(pp->data.fl[1]);
				cvDrawCircle(rgb, cvPoint(u[i], v[i]), 2, 
					CV_RGB(255,255, 255), CV_FILLED);
				
				memset(text, 0, 100);
				sprintf_s(text, 100, "(%d,%d)", u[i], v[i]);
				cvPutText(rgb, text, cvPoint(u[i], v[i]), &font, CV_RGB(0,255,0));
			}

			printf("%d_%0.3f_%0.4f_%u * %d_%0.3f_%0.4f_%u\n", 
					rois[0].id, rois[0].x, rois[0].y, rois[0].ts,
					rois[1].id, rois[1].x, rois[1].y, rois[1].ts);

			theta = atan2(rois[1].y - rois[0].y, rois[1].x - rois[0].x) *
						180/acos(-1.0f);
			
			sprintf_s(text, 100, "angle: %0.3f", theta);
			cvPutText(rgb, text, cvPoint(20, 20), &font, CV_RGB(0,255,0));

			cvShowImage("PuckTracker", rgb);
			cvWaitKey(1);
			cvZero(rgb);
		}
	}

	close_comm();

	return 0;
}
