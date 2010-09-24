/**
* @file cam.cpp contains functions for initiatlizing, acquiring, and deinitializing the 
* frame grabber and camera.
*/

#include "fcdynamic.h"

static const unsigned long *mem;

/**
* Returns a pointer to buffer memory allocated by the frame buffer.
*/

const unsigned long *get_mem()
{
	return mem;
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

int init_cam(Fg_Struct **grabber, int memsize, int buffers, int camlink)
{
	int rc;
	Fg_Struct *fg = NULL;

	fg = Fg_Init("FastConfig.dll", 0);
	if(fg == NULL) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &camlink, PORT_A);
	if(rc != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = FastConfigInit(PORT_A);
	if(rc != FG_OK) {
		return Fg_getLastErrorNumber(fg);
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

int acquire_imgs(Fg_Struct *fg, int *seq, int seq_len)
{
	int rc, i;

	rc = roi_sequence(fg, seq, seq_len);
	if(rc != FG_OK) {
		printf("init of roi failed\n");
		return Fg_getLastErrorNumber(fg);
	}

	for(i = 0; i < MAX_ROI; i++) {
		rc = write_roi(fg, i, 0, !DO_INIT);
		if(rc != FG_OK) {
			printf("init of roi %d failed\n", i);
			return Fg_getLastErrorNumber(fg);
		}
	}

	rc = Fg_Acquire(fg, PORT_A, GRAB_INFINITE);
	if(rc != FG_OK){ 
		printf("acquire failed\n");
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

int deinit_cam(Fg_Struct *fg)
{
	int rc;

	rc = Fg_stopAcquire(fg, PORT_A);
	if(rc != FG_OK) {
		printf("stop acquire failed\n");
		return Fg_getLastErrorNumber(fg);
	}

	rc = Fg_FreeMem(fg, PORT_A);
	if(rc != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = FastConfigFree(PORT_A);
	if(rc != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = Fg_FreeGrabber(fg);
	if(rc != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}