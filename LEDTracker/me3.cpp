/**
* @file cam.cpp contains functions for initiatlizing, acquiring, and deinitializing the 
* frame grabber and camera.
*/

#include "me3.h"

static const unsigned long *mem;
static int fc;

/**
* Returns a pointer to buffer memory allocated by the frame buffer.
*/

const unsigned long *get_mem()
{
	return mem;
}

int is_fastconfig()
{
	return fc;
}

/**
* Initializes the framegrabber and camera.
* 
* init_cam performs the necessary initialization routines prior to
* acquiring images.  <code>memsize</code> should equal the image
* width x image height x <code>buffers</code>.  Although this is
* not enforced, undesired behavior may result.
* 
* @param grabber an uninitialized Fg_Struct object defined in the Silicon Software API
* @param memsize the image buffer memory size in bytes
* @param buffers the number of buffer to divide the memsize bytes into
* @param camlink the camera link type as defined in the Silicon Software API
*
* @note camlink is typically set to <code>FG_CL_DUALTAP_8_BIT</code>
* in order to maximize the number of pixel information that can be sent
* over the cable (physically) connecting the framegrabber and camera.
*/

int me3_init(Fg_Struct **grabber, char *applet, int memsize, int buffers)
{
	int rc;

	Fg_Struct *fg = NULL;

	fg = Fg_Init(applet, PORT_A);
	if(fg == NULL) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = GRABBER_CONTROLLED;
	if(Fg_setParameter(fg, FG_TRIGGERMODE, &rc, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(Fg_setExsync(fg, FG_ON, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	fc = !strncmp(applet, FC_APPLET, sizeof(FC_APPLET));
	if(fc) {
		if(FastConfigInit(PORT_A) != FG_OK) {
			return Fg_getLastErrorNumber(fg);
		}
	}

	mem = Fg_AllocMem(fg, memsize, buffers, PORT_A);
	if(mem == NULL) {
		return Fg_getLastErrorNumber(fg);
	}

	*grabber = fg;
	return FG_OK;
}

/**
* Transfers the initial region of interest (ROI) information to the frame grabber.
* 
* acquire_imgs tells the frame grabber to start grabbing an infinite number of images.
* This function will also write the ROI parameters to the frame grabber.  The active
* ROIs will become active on the camera in the order that <code>seq</code> lists
* them.  After cycling through the list, the sequence repeats itself from the beginning.
* A <code>seq</code> cannot exceed 4,096 entires and <code>seq_len</code> must equal 
* the number of entries in <code>seq</code>.
*
* @param grabber an initialized Fg_Struct object defined in the Silicon Software API
* @param seq the sequence specifying when a ROI is active
* @param seq_len the length of <code>seq</code>.
*
* @see roi.cpp
*/

int me3_acquire(Fg_Struct *fg, int *seq, int seq_len)
{
	int i;

	if(fc) {
		if(roi_sequence(fg, seq, seq_len) != FG_OK) {
			return Fg_getLastErrorNumber(fg);
		}

		for(i = 0; i < MAX_ROI; i++) {
			if(write_roi(fg, i, i, !DO_INIT) != FG_OK) {
				return Fg_getLastErrorNumber(fg);
			}
		}
	}

	if(Fg_Acquire(fg, PORT_A, GRAB_INFINITE) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}

/** 
* Stops grabbing images and frees resources associated with the frame grabber
*
* deinit_cam stops the frame grabber from acquiring images and frees the resources
* used by the frame grabber.

* @param grabber an initialized Fg_Struct object defined in the Silicon Software API
*/

int me3_deinit(Fg_Struct *fg)
{
	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(Fg_FreeMem(fg, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(fc) {
		if(FastConfigFree(PORT_A) != FG_OK) {
			return Fg_getLastErrorNumber(fg);
		}
	}

	if(Fg_FreeGrabber(fg) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}