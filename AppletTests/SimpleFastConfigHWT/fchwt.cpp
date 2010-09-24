#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

#define APPLET "FastConfig.dll"
#define NUM_BUFFERS 16
#define NUM_IMAGES 1
#define TIMEOUT 3
#define WINDOW "SimpleAppletTest"
#define FRAME_TIME 50000
#define EXPOSURE 20000
#define WIDTH 512
#define HEIGHT 512

#define DAQ_PIN "Dev4/ctr0"
#define BUF_SIZE 2048
// min = 2.5e-8 s, max = 53.6871 s for SIG_DELAY, HI_TIME, LO_TIME
#define SIG_DELAY 0
#define HI_TIME 4e-3//120e-9
#define LO_TIME 235e-6//120e-9

#include <stdio.h>
#include <NIDAQmx.h>

int FCInit(Fg_Struct *fg, int w, int h)
{
    // setup ROI 0
	int rc;
	FastConfigSequence mFcs;
	FC_ParameterSet lRoiParameterSet;

	mFcs.mLengthOfSequence = 1;
	mFcs.mRoiPagePointer = new int[mFcs.mLengthOfSequence];
	mFcs.mRoiPagePointer[0]	= 0;

    memset(&lRoiParameterSet, 0, sizeof(FC_ParameterSet));
    setParameterSetRoi(&lRoiParameterSet, 0, w, 0, h);
    setParameterSetTime(&lRoiParameterSet, EXPOSURE, FRAME_TIME);
    setParameterSetLinlog(&lRoiParameterSet, 0, 0, 0, 0);

    rc = FastConfigInit(PORT_A);
    if(rc != FG_OK) {
        printf("FC Init: %s\n", Fg_getLastErrorDescription(fg));
        rc = Fg_getLastErrorNumber(fg);
        Fg_FreeGrabber(fg);
        return rc;
    }

	rc = Fg_setParameter(fg, FG_FASTCONFIG_SEQUENCE, &mFcs,PORT_A);
    if(rc != FG_OK) {
        printf("FC SEQ: %s\n", Fg_getLastErrorDescription(fg));
        rc = Fg_getLastErrorNumber(fg);
        Fg_FreeGrabber(fg);
        return rc;
    }

    writeParameterSet(fg, &lRoiParameterSet, 0, 0xfab, 1, PORT_A);

	return FG_OK;
}

int main(void)
{
	char errBuff[BUF_SIZE]={'\0'};
	TaskHandle taskHandle = 0;
	int i, rc;
	unsigned __int64 freq, loop_start, loop_stop;
	double dur;
	Fg_Struct *fg = NULL;
	IplImage *cvDisplay = NULL;
	unsigned char *img;
	int j, k;
	int mode, w, h, nr;

	/*
	rc = SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS );
	if(rc == FALSE) {
      return GetLastError();
   } 
   */

	/*
	rc = SetThreadPriority(GetCurrentThread(), HIGH_PRIORITY_CLASS);
	if(rc == FALSE) {
      return GetLastError();
   }
   */

	w = WIDTH;
	h = HEIGHT;

	fg = Fg_Init(APPLET, PORT_A);
	if(fg == NULL) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	mode = ASYNC_TRIGGER;
	if(Fg_setParameter(fg, FG_TRIGGERMODE, &mode, PORT_A) < 0) {
		printf("mode: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) < 0) {
		printf("dual tap: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	// the following pinouts pertain to PORTA (sub-D15 labelled L1 on the meIV TTL output card)
	rc = TRGINSRC_1; // pin 12 on meIV TTL output card (compatible with meIII FG board)
	rc = TRGINSRC_0; // pin 11 on meIV TTL output card (compatible with meIII FG board)
	if(Fg_setParameter(fg, FG_TRIGGERINSRC, &rc, PORT_A) < 0) {
		printf("trig in: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_setExsync(fg, FG_ON, PORT_A) < 0) {
		printf("sync on: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

    rc = FG_ON;
	if(Fg_setParameter(fg, FG_EXSYNCINVERT, &rc, PORT_A) < 0) {
		printf("sync invert: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_AllocMem(fg, w*h*NUM_BUFFERS, NUM_BUFFERS, PORT_A) == NULL){
		printf("mem: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

    if(FCInit(fg, w, h) != FG_OK) {
		printf("FCInit: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
    }

	if(Fg_Acquire(fg, PORT_A, NUM_IMAGES) < 0){
		printf("acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
	}

	cvDisplay = cvCreateImage(cvSize(w, h), 8, 1);
	cvNamedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

	rc = DAQmxCreateTask("", &taskHandle);
	if(rc < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
		printf("create daq task %s\n", errBuff);
		return rc;
	}

	rc = DAQmxCreateCOPulseChanTime(taskHandle, DAQ_PIN, "", DAQmx_Val_Seconds, DAQmx_Val_Low, SIG_DELAY, LO_TIME, HI_TIME);
	if(rc < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
		printf("create pulse task %s\n", errBuff);
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		return rc;
	}

	/*
	rc = DAQmxCfgImplicitTiming(taskHandle, DAQmx_Val_FiniteSamps , 2000);
	if(rc < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
		printf("implicit timing %s\n", errBuff);
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		return rc;
	}
	*/

	printf("actual \t expected \t error \n");
	QueryPerformanceFrequency((LARGE_INTEGER *) &freq);
	nr = 0;
	for(i = 0; i < 30; i++) {
		QueryPerformanceCounter((LARGE_INTEGER *) &loop_start);
		if(DAQmxStartTask(taskHandle) < 0) {
			DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
			printf("start pulse task %s\n", errBuff);
			DAQmxStopTask(taskHandle);
			DAQmxClearTask(taskHandle);
			return rc;
		}

		nr++;
		rc = Fg_getLastPicNumberBlocking(fg, nr, PORT_A, TIMEOUT);
		if(rc <= FG_OK) {
			printf("get images: %s\n", Fg_getLastErrorDescription(fg));
			rc = Fg_getLastErrorNumber(fg);
			break;
		}

		if(DAQmxWaitUntilTaskDone(taskHandle, TIMEOUT) < 0) {
			DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
			printf("wait pulse task %s\n", errBuff);
			DAQmxStopTask(taskHandle);
			DAQmxClearTask(taskHandle);
			return rc;
		}

		if(DAQmxStopTask(taskHandle) < 0) {
			DAQmxGetExtendedErrorInfo(errBuff, BUF_SIZE);
			printf("stop pulse task %s\n", errBuff);
			DAQmxStopTask(taskHandle);
			DAQmxClearTask(taskHandle);
			return rc;
		}
		QueryPerformanceCounter((LARGE_INTEGER *) &loop_stop);

		dur = (loop_stop - loop_start) / (freq * 1.0);
		printf("%gs \t %gs \t %gs \n", dur, LO_TIME + HI_TIME, dur - LO_TIME - HI_TIME);

		img = (unsigned char *) Fg_getImagePtr(fg, rc, PORT_A);
		for(k = 0; k < h; k++) {
			for(j = 0; j < w; j++) {
				cvDisplay->imageData[j + (k * w)] = img[j + (k * w)];
				cvDisplay->imageDataOrigin[j + (k * w)] = img[j + (k * w)];
			}
		}

		cvShowImage(WINDOW, cvDisplay);
		cvWaitKey(200);
	}
	cvReleaseImage(&cvDisplay);

	if(Fg_setExsync(fg, FG_OFF, PORT_A) < 0) {
		printf("sync off: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(FastConfigFree(PORT_A) != FG_OK) {
		printf("FC free: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_FreeGrabber(fg) != FG_OK) {
		printf("free grabber: %s\n", Fg_getLastErrorDescription(fg));
	}

	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);

	return 0;
}


#if 0

int main(void)
{
	Fg_Struct *fg = NULL;
	IplImage *cvDisplay = NULL;
	unsigned char *img;
	int rc, i, j;
	int mode, w, h, nr;

	fg = Fg_Init(APPLET, PORT_A);
	if(fg == NULL) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	mode = ASYNC_TRIGGER;
	if(Fg_setParameter(fg, FG_TRIGGERMODE, &mode, PORT_A) < 0) {
		printf("mode: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) < 0) {
		printf("dual tap: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	// the following pinouts pertain to PORTA (sub-D15 labelled L1 on the meIV TTL output card)
	rc = TRGINSRC_1; // pin 12 on meIV TTL output card (compatible with meIII FG board)
	rc = TRGINSRC_0; // pin 11 on meIV TTL output card (compatible with meIII FG board)
	if(Fg_setParameter(fg, FG_TRIGGERINSRC, &rc, PORT_A) < 0) {
		printf("trig in: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_setExsync(fg, FG_ON, PORT_A) < 0) {
		printf("sync on: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

    rc = FG_ON;
	if(Fg_setParameter(fg, FG_EXSYNCINVERT, &rc, PORT_A) < 0) {
		printf("sync invert: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_getParameter(fg, FG_WIDTH, &w, PORT_A) < 0) {
		printf("width: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_getParameter(fg, FG_HEIGHT, &h, PORT_A) < 0) {
		printf("height: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}	

	if(Fg_AllocMem(fg, w*h*NUM_BUFFERS, NUM_BUFFERS, PORT_A) == NULL){
		printf("mem: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

    if(FCInit(fg, w, h) != FG_OK) {
		printf("FCInit: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
    }


	if(Fg_Acquire(fg, PORT_A, NUM_IMAGES) < 0){
		printf("acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
	}

	nr = 0;
	cvDisplay = cvCreateImage(cvSize(w, h), 8, 1);
	cvNamedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	while(nr < NUM_IMAGES)
	{
		nr++;
		rc = Fg_getLastPicNumberBlocking(fg, nr, PORT_A, TIMEOUT);
		if(rc <= FG_OK) {
			printf("get images: %s\n", Fg_getLastErrorDescription(fg));
			rc = Fg_getLastErrorNumber(fg);
			break;
		}

		img = (unsigned char *) Fg_getImagePtr(fg, rc, PORT_A);		
		for(i = 0; i < h; i++) {
			for(j = 0; j < w; j++) {
				cvDisplay->imageData[j + (i * w)] = img[j + (i * w)];
				cvDisplay->imageDataOrigin[j + (i * w)] = img[j + (i * w)];
			}
		}
		cvShowImage(WINDOW, cvDisplay);
		cvWaitKey(200);
	}
	cvReleaseImage(&cvDisplay);

	if(Fg_setExsync(fg, FG_OFF, PORT_A) < 0) {
		printf("sync off: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(FastConfigFree(PORT_A) != FG_OK) {
		printf("FC free: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_FreeGrabber(fg) != FG_OK) {
		printf("free grabber: %s\n", Fg_getLastErrorDescription(fg));
	}

	return 0;
}
#endif