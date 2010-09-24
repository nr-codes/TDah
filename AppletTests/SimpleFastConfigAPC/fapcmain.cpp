#include <conio.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

#define APPLET "FastConfig.dll"
#define NUM_BUFFERS 16
#define NUM_IMAGES 100
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

static void *mem;
int testAPC(int i, void* b)
{
	Fg_Struct *fg = (Fg_Struct *) b;
	CvMat cvDisplay;
	unsigned char *img;
	int rc;
	int w, h;

	if(fg == NULL) {
		return -1;
	}

	img = (unsigned char *) Fg_getImagePtrEx(fg, i, PORT_A, mem);
	if(img == NULL) {
		printf("image is NULL\n");
		return -1;
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

	cvDisplay = cvMat(h, w, CV_8UC1, img);
	cvNamedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	cvShowImage(WINDOW, &cvDisplay);
	cvWaitKey(2);

	printf("testAPC: %d\n", i);
	return FG_OK;
}


int main(void)
{
	Fg_Struct *fg = NULL;
	IplImage *cvDisplay = NULL;
	int rc;
	int mode, w, h, nr;

	fg = Fg_Init(APPLET, PORT_A);
	if(fg == NULL) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	mode = GRABBER_CONTROLLED;
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

    if(FCInit(fg, w, h) != FG_OK) {
		printf("FCInit: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
    }

	if((mem = Fg_AllocMemEx(fg, w*h*NUM_BUFFERS, NUM_BUFFERS)) == NULL){
		printf("mem: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_AcquireAPCEx(fg, PORT_A, NUM_IMAGES, ACQ_STANDARD, mem, testAPC, fg) < 0){
		printf("acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        FastConfigFree(PORT_A);
		Fg_FreeGrabber(fg);
		return rc;
	}

	nr = 0;
	while(nr < NUM_IMAGES)
	{
		rc = Fg_getLastPicNumberBlockingEx(fg, nr, PORT_A, TIMEOUT, mem);
		//rc = Fg_getLastPicNumberEx(fg, PORT_A, mem);
		nr = rc;
		if(rc < FG_OK) {
			printf("get images: %s\n", Fg_getLastErrorDescription(fg));
			rc = Fg_getLastErrorNumber(fg);
			break;
		}
	}

	if(Fg_setExsync(fg, FG_OFF, PORT_A) < 0) {
		printf("sync off: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_stopAcquireEx(fg, PORT_A, mem, STOP_ASYNC) != FG_OK) {
		printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
	}

	// need to ensure testAPC doesn't get called before freeing fg
	// otherwise program will crash
	cvWaitKey(TIMEOUT * 1000);
	printf("done sleeping\n");

	if(Fg_FreeMemEx(fg, mem) != FG_OK) {
		printf("free mem: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(FastConfigFree(PORT_A) != FG_OK) {
		printf("FC free: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(Fg_FreeGrabber(fg) != FG_OK) {
		printf("free grabber: %s\n", Fg_getLastErrorDescription(fg));
	}

	_getch();
	return 0;
}
