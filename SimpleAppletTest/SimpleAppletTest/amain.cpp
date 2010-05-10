#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

#define APPLET "SingleAreaGray.dll"
#define NUM_BUFFERS 16
#define NUM_IMAGES 10
#define TIMEOUT 3
#define WINDOW "SimpleAppletTest"


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

	if(Fg_setExsync(fg, FG_ON, PORT_A) < 0) {
		printf("sync on: %s\n", Fg_getLastErrorDescription(fg));
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

	if(Fg_Acquire(fg, PORT_A, NUM_IMAGES) < 0){
		printf("acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
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
		cvWaitKey(1);
	}
	cvReleaseImage(&cvDisplay);

	if(Fg_setExsync(fg, FG_OFF, PORT_A) < 0) {
		printf("sync on: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}


	rc = Fg_stopAcquire(fg, PORT_A);
	if(rc != FG_OK) {
		printf("stop acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	rc = Fg_FreeMem(fg, PORT_A);
	if(rc != FG_OK) {
		printf("free mem: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	rc = Fg_FreeGrabber(fg);
	if(rc != FG_OK) {
		printf("free grabber: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	return 0;
}