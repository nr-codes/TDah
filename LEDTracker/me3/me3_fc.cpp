/**
* @file cam.cpp contains functions for initiatlizing, acquiring, and deinitializing the 
* frame grabber and camera.
*/

#include "me3_fc.h"

static const unsigned long *mem;

/**
* Returns a pointer to buffer memory allocated by the frame buffer.
*/

const unsigned long *get_mem()
{
	return mem;
}

int me3_fc_init(Fg_Struct **grabber, int trig, int memsize, int nbufs)
{
	int rc;
	Fg_Struct *fg = NULL;

	fg = Fg_Init(FC_APPLET, PORT_A);
	if(fg == NULL) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = FG_CL_DUALTAP_8_BIT;
	if(Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &rc, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	rc = trig;
	if(Fg_setParameter(fg, FG_TRIGGERMODE, &rc, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(trig != FREE_RUN) {
		if(Fg_setExsync(fg, FG_ON, PORT_A) != FG_OK) {
			return Fg_getLastErrorNumber(fg);
		}
	}

	if(FastConfigInit(PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	mem = Fg_AllocMem(fg, memsize, nbufs, PORT_A);
	if(mem == NULL) {
		return Fg_getLastErrorNumber(fg);
	}

	*grabber = fg;
	return FG_OK;
}

int me3_fc_acquire(Fg_Struct *fg, int *seq, int seq_len)
{
	int i;

	if(roi_sequence(fg, seq, seq_len) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	for(i = 0; i < MAX_ROI; i++) {
		if(write_roi(fg, i, i, !DO_INIT) != FG_OK) {
			return Fg_getLastErrorNumber(fg);
		}
	}

	if(Fg_Acquire(fg, PORT_A, GRAB_INFINITE) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}

int me3_fc_deinit(Fg_Struct *fg)
{
	if(Fg_setExsync(fg, FG_OFF, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(Fg_FreeMem(fg, PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(FastConfigFree(PORT_A) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	if(Fg_FreeGrabber(fg) != FG_OK) {
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}