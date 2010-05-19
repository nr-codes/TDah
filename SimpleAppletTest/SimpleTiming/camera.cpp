#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

#define APPLET "FastConfig.dll"
#define WINDOW "SimpleAppletTest"
#define SEQ_LEN 1
#define MAX_ROI 8
#define DO_INIT 1
#define PAUSE 10 // ms
#define SHOW_DISP 0

int close_cam(Fg_Struct *fg);
int update_roi(Fg_Struct *fg, int w, int h, double e, double f);
int open_cam(Fg_Struct **gr, int mode);
int get_images(Fg_Struct *fg, int num_imgs);
int show_images(Fg_Struct *fg, int *nr, int num_images, int w, int h);

int update_roi(Fg_Struct *fg, int label, int w, int h, double e, double f, int init)
{
    // setup one ROI, ROI0
	int rc;
	FC_ParameterSet lRoiParameterSet;

    memset(&lRoiParameterSet, 0, sizeof(FC_ParameterSet));
	
	if(setParameterSetRoi(&lRoiParameterSet, 0, w, 0, h) < 0) {
		printf("set roi: %s\n", Fg_getLastErrorDescription(fg));
	}
    
	if(setParameterSetTime(&lRoiParameterSet, e, f) < 0) {
		printf("set time: %s\n", Fg_getLastErrorDescription(fg));
	}

	if(setParameterSetLinlog(&lRoiParameterSet, 0, 0, 0, 0) < 0) {
		printf("set linlog: %s\n", Fg_getLastErrorDescription(fg));
	}

    rc = writeParameterSet(fg, &lRoiParameterSet, 0, label, init, PORT_A);
	if(rc < 0) {
		printf("write params: %s\n", Fg_getLastErrorDescription(fg));
	}

	return rc;
}

int open_cam(Fg_Struct **gr, int mode, int num_images, int w, int h)
{
	Fg_Struct *fg = NULL;
	FastConfigSequence mFcs;
	int rc;

	fg = Fg_Init(APPLET, PORT_A);
	if(fg == NULL) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

	if(Fg_setParameter(fg, FG_TRIGGERMODE, &mode, PORT_A) < 0) {
		printf("mode: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) < 0) {
		printf("dual tap: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

	if(Fg_setExsync(fg, FG_ON, PORT_A) < 0) {
		printf("sync on: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

    rc = FG_ON;
	if(Fg_setParameter(fg, FG_EXSYNCINVERT, &rc, PORT_A) < 0) {
		printf("sync invert: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

    rc = FG_ON;
	if(Fg_setParameter(fg, FG_EXSYNCPOLARITY, &rc, PORT_A) < 0) {
		printf("sync polarity: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

	if(Fg_AllocMem(fg, w*h*num_images, num_images, PORT_A) == NULL) {
		printf("mem: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		close_cam(fg);
		return rc;
	}

////////////////////////////////////////////
	/*
	if(Fg_setParameter(fg, FG_WIDTH, &w, PORT_A) < 0) {
		printf("width: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}

	if(Fg_setParameter(fg, FG_HEIGHT, &h, PORT_A) < 0) {
		printf("height: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
		Fg_FreeGrabber(fg);
		return rc;
	}
	*/
////////////////////////////////////////////


	// FastConfig parameters

	if(FastConfigInit(PORT_A) != FG_OK) {
        printf("fc init: %s\n", Fg_getLastErrorDescription(fg));
        rc = Fg_getLastErrorNumber(fg);
        close_cam(fg);
        return rc;
    }

	// only one ROI
	mFcs.mLengthOfSequence = SEQ_LEN;
	mFcs.mRoiPagePointer = new int[SEQ_LEN];
	mFcs.mRoiPagePointer[0]	= 0;

    if(Fg_setParameter(fg, FG_FASTCONFIG_SEQUENCE, &mFcs, PORT_A) != FG_OK) {
        printf("fc seq: %s\n", Fg_getLastErrorDescription(fg));
        rc = Fg_getLastErrorNumber(fg);
        close_cam(fg);
        return rc;
    }

	*gr = fg;
	return FG_OK;
}

int get_images(Fg_Struct *fg, int num_imgs)
{
	int rc;

	if(Fg_Acquire(fg, PORT_A, GRAB_INFINITE) < 0) {
		printf("acq: %s\n", Fg_getLastErrorDescription(fg));
		rc = Fg_getLastErrorNumber(fg);
        close_cam(fg);
		return rc;
	}

	return FG_OK;
}

int show_images(Fg_Struct *fg, int *nr, int num_imgs, int w, int h)
{
#if SHOW_DISP
	int i;
	int disp;
	unsigned long *img;

	disp = CreateDisplay(8, w, h);
	SetBufferWidth(disp, w, h);
	for(i = 0; i < num_imgs; i++) {
		if(nr[i] > FG_OK) {
			img = Fg_getImagePtr(fg, nr[i], PORT_A);
			DrawBuffer(disp, img, nr[i], "SimpleTiming");
			Sleep(PAUSE);
		}
	}
	CloseDisplay(disp);
#endif
	return FG_OK;
}

int close_cam(Fg_Struct *fg)
{
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

	return FG_OK;
}