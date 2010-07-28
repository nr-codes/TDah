#include "t_dah.h"

#define BGR_CHANS 3
#define PYR_LVL 2
#define PYR_OFFSET 5

bool TDahMe3Fc::open(char *conf_file)
{
	return false;
}

bool TDahMe3Fc::open(int tt, double e, double f, int iw, int ih, int b)
{
	int seq[] = {ROI_0};
	int seq_len = sizeof(seq)/sizeof(int);

	// initialize certain class members
	img_w = iw;
	img_h = ih;
	img = cvCreateImage(cvSize(iw, ih), IPL_DEPTH_8U, BGR_CHANS);
	if(!img) return false;

	// initialize me3 and fastconfig
	if(me3_fc_init(&fg, tt, iw*ih*b, b) != FG_OK) {
		me3_err("open");
		return false;
	}

	for(int i = 0; i < MAX_ROI; i++) {
		// set all ROIs at full frame
		if(roi_window(i, 0, FC_MAX_WIDTH, 0, FC_MAX_HEIGHT) != FG_OK) {
			me3_err("open");
			return false;
		}

		// set all ROIs at same exposure and frame time
		if(roi_exposure(i, e, f) != FG_OK) {
			me3_err("open");
			return false;
		}
	}

	// start grabbing images in order of ROIs specified in seq
	if(me3_fc_acquire(fg, seq, seq_len) != FG_OK) {
		me3_err("open");
		return false;
	}

	return true;
}

void TDahMe3Fc::close()
{
	cvReleaseImage(&img);

	if(me3_fc_deinit(fg) != FG_OK) {
		me3_err("close");
	}
}

bool TDahMe3Fc::grabFrame()
{
	int trigger;

	if(Fg_getParameter(fg, FG_TRIGGERMODE, &trigger, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return false;
	}

	if(trigger == ASYNC_SOFTWARE_TRIGGER 
		&& Fg_sendSoftwareTrigger(fg, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return false;
	}

	return true;
}

IplImage *TDahMe3Fc::retrieveFrame(int img_nr)
{
	int act_nr;
	CvMat imgData;

	act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	if(act_nr < FG_OK) {
		me3_err("retrieveFrame");
		return NULL;
	}

	imgData = cvMat(img_h, img_w, CV_8UC1, 
		Fg_getImagePtr(fg, act_nr, PORT_A));

	cvResetImageROI(img);
	cvMerge(&imgData, &imgData, &imgData, NULL, img);

	return img;
}

int TDahMe3Fc::getROILoc(int index, ROILoc *r)
{
#if 0
	
	// get image tag, tag == X => ROI_X
	cur_roi = act_nr;
	if(Fg_getParameter(fg, FG_IMAGE_TAG, &cur_roi, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return NULL;
	}
	cur_roi = cur_roi >> 16;

	r.img_nr = act_nr;
	r.ts = (double) act_nr;
	if(Fg_getParameter(fg, FG_TIMESTAMP_LONG, &dots[cur_roi].r.ts, PORT_A)
		!= FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}
	
	// get roi associated with image and point to image data
	img->imageData = img->imageDataOrigin = 
		(char *) Fg_getImagePtr(fg, img_nr, PORT_A);

	cvSetImageROI(img, cvGetImageROI(dots[cur_roi].gr));
	cvCopyImage(img, dots[cur_roi].gr);

	return act_nr;
#endif

	return CV_OK;
}
