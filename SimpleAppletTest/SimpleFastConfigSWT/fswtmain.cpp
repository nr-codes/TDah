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

	mode = ASYNC_SOFTWARE_TRIGGER;
	if(Fg_setParameter(fg, FG_TRIGGERMODE, &mode, PORT_A) < 0) {
		printf("mode: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_getParameter(fg, FG_TRIGGERMODE, &mode, PORT_A) < 0) {
		printf("mode: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	switch (mode) {
		case FREE_RUN:
			printf("free run\n");
			break;
		case GRABBER_CONTROLLED:
			printf("grabber controlled\n");
			break;
		case ASYNC_TRIGGER:
			printf("async trigger\n");
			break;
		case GRABBER_CONTROLLED_SYNCHRON:
			printf("gr controlled sync\n");
			break;
		case ASYNC_SOFTWARE_TRIGGER:
			printf("aysnc software trigger\n");
			break;
		default:
			printf("unknown trigger signal\n");
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) < 0) {
		printf("dual tap: %s\n", Fg_getLastErrorDescription(fg));
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
		if(Fg_sendSoftwareTrigger(fg, PORT_A) != FG_OK) {
			printf("send trigger 1: %s\n", Fg_getLastErrorDescription(fg));
			rc = Fg_getLastErrorNumber(fg);
			Fg_FreeGrabber(fg);
			return rc;
		}

		rc = FG_ON;
		if(Fg_setParameter(fg, FG_SENDSOFTWARETRIGGER, &rc, PORT_A) != FG_OK) {
			printf("send trigger 2: %s\n", Fg_getLastErrorDescription(fg));
			rc = Fg_getLastErrorNumber(fg);
			Fg_FreeGrabber(fg);
			return rc;
		}

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
