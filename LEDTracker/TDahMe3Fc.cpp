#include "t_dah.h"

#define BGR_CHANS 3
#define GR_CHAN 1
#define PYR_LVL 2
#define PYR_OFFSET 5

bool TDahMe3Fc::open(char *conf_file)
{
	return false;
}

bool TDahMe3Fc::open(int tt, double e, double f, int b)
{
	int seq[] = {ROI_0};
	int seq_len = sizeof(seq)/sizeof(int);

	// initialize certain class members
	img_w = FC_MAX_WIDTH;
	img_h = FC_MAX_HEIGHT;
	roi_w = img_w;
	roi_h = img_h;
	frame_time = f;
	exposure = e;

	// bgr_img is associated with the entire full resolution frame
	bgr_img = cvCreateImage(cvSize(img_w, img_h), IPL_DEPTH_8U, BGR_CHANS);
	if(!bgr_img) return false;

	// gr_img is associated with the ROI that is sent back from the camera
	gr_img = cvCreateImageHeader(cvSize(roi_w, roi_h), IPL_DEPTH_8U, GR_CHAN);
	if(!gr_img) return false;

	// initialize me3 and fastconfig
	if(me3_fc_init(&fg, tt, img_w*img_h*b, b) != FG_OK) {
		me3_err("open");
		return false;
	}

	for(int i = 0; i < MAX_ROI; i++) {
		// set all ROIs to full image frame
		if(roi_window(i, 0, img_w, 0, img_h) != FG_OK) {
			me3_err("setupFullFrame");
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
	cvReleaseImage(&bgr_img);
	cvReleaseImageHeader(&gr_img);

	if(me3_fc_deinit(fg) != FG_OK) {
		me3_err("close");
	}
}

int TDahMe3Fc::setupNextFrame(int roi_nr, IplImage *img)
{
	// TODO fix centroid being incorrect at edges of image frame
	CvRect win = cvGetImageROI(img);

	win.width = roi_w;
	win.height = roi_h;
	if(win.x + roi_w > img_w) {
		win.x = img_w - roi_w;
	}

	if(win.y + roi_h > img_h) {
		win.y = img_h - roi_h;
	}

	win.x &= MULT_OF_FOUR_MASK;

	if(roi_window(roi_nr, win.x, win.width, win.y, win.height) != FG_OK) {
		me3_err("setupFullFrame");
		return Fg_getLastErrorNumber(fg);
	}

	// TODO: understand when DO_INIT is necessary
	if(write_roi(fg, roi_nr, roi_nr, !DO_INIT) != FG_OK) {
		me3_err("setupFullFrame");
		return Fg_getLastErrorNumber(fg);
	}

	cvSetImageROI(img, win);
	return CV_OK;
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
	int act_nr, dma_trf_len;
	CvMat imgData;

	// get newest image
	img_nr = std::max(img_nr, 
		Fg_getImage(fg, SEL_NEW_IMAGE, 0, PORT_A, TIMEOUT));
	if(img_nr < FG_OK) {
			me3_err("retrieveFrame");
			return NULL;
	}

	// get image data from buffer
	act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	if(act_nr < FG_OK) {
		me3_err("retrieveFrame");
		return NULL;
	}

	dma_trf_len = act_nr;
	if(Fg_getParameter(fg, FG_TRANSFER_LEN, &dma_trf_len, PORT_A) != FG_OK) {
		me3_err("retrieveFrame");
		return NULL;
	}

	// make sure that the image we asked for is full resolution
	cvZero(bgr_img);
	if(dma_trf_len != img_w*img_h) {
		//return NULL;
		CvMat submat;
		//cvResetImageROI(bgr_img);
		imgData = cvMat(roi_h, roi_w, CV_8UC1, Fg_getImagePtr(fg, act_nr, PORT_A));
		cvGetSubRect(bgr_img, &submat, ctrd2roi(512, 512, roi_w, roi_h));
		cvMerge(&imgData, &imgData, &imgData, NULL, &submat);
	}
	else {
		imgData = cvMat(img_h, img_w, CV_8UC1, Fg_getImagePtr(fg, act_nr, PORT_A));
		cvMerge(&imgData, &imgData, &imgData, NULL, bgr_img);
	}

	// copy image data to bgr image
	//imgData = cvMat(img_h, img_w, CV_8UC1, Fg_getImagePtr(fg, act_nr, PORT_A));
	//cvMerge(&imgData, &imgData, &imgData, NULL, bgr_img);

	return bgr_img;
}

// TODO make sure to note that camera must be acquiring before calling initROIs
int TDahMe3Fc::initROIs(int n, int rw, int rh, char *s, bool uk, bool ut)
{
	int img_nr;
	int seq[] = {ROI_0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

	rw = GET_CLOSEST_ROI_WIDTH(rw);
	if(n > MAX_ROI || rw < MIN_ROI_WIDTH) {
		return CV_BADROI_ERR;
	}

	// make full frame ROIs
	for(int i = 0; i < MAX_ROI; i++) {
		if(roi_window(i, 0, img_w, 0, img_h) != FG_OK) {
			me3_err("initROIs");
			return Fg_getLastErrorNumber(fg);
		}

		if(write_roi(fg, i, i, !DO_INIT) != FG_OK) {
			me3_err("initROIs");
			return Fg_getLastErrorNumber(fg);
		}
	}
	img_nr = Fg_getStatus(fg, NUMBER_OF_ACT_IMAGE, 0, PORT_A);
	img_nr = Fg_getLastPicNumberBlocking(fg, img_nr + 2, PORT_A, TIMEOUT);
	if(img_nr < FG_OK) {
		me3_err("initROIs");
		return Fg_getLastErrorNumber(fg);
	}

	if(TDah::initROIs(n, rw, rh, s, uk, ut) != CV_OK) {
		return !CV_OK;
	}

	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		me3_err("initROIs");
		return Fg_getLastErrorNumber(fg);
	}

	for(int i = 0; i < n_roi; i++) {
		if(setupNextFrame(i, gr[i]) != CV_OK) return !CV_OK;
	}

	if(me3_fc_acquire(fg, seq, n_roi) != FG_OK) {
		me3_err("initROIs");
		return Fg_getLastErrorNumber(fg);
	}

	return CV_OK;
}

void TDahMe3Fc::showROILoc(void)
{
	/*
	cvZero(bgr_img);
	cvResetImageROI(gr_img);
	cvResetImageROI(bgr_img);
	cvCvtColor(gr_img, bgr_img, CV_GRAY2BGR);
	show_position(gr, n_roi, kal, wr, NULL, bgr_img);
	*/
	show_position(gr, n_roi, kal, wr);
}

int TDahMe3Fc::getROILoc(int img_nr, ROILoc *r)
{
	int act_nr, cur_roi;
	uint64 ts;

	act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	if(act_nr < FG_OK) {
		me3_err("retrieveFrame");
		return Fg_getLastErrorNumber(fg);
	}

	// get image tag, tag == X => ROI_X
	cur_roi = act_nr;
	if(Fg_getParameter(fg, FG_IMAGE_TAG, &cur_roi, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}
	cur_roi = cur_roi >> 16;

	ts = act_nr;
	if(Fg_getParameter(fg, FG_TIMESTAMP_LONG, &ts, PORT_A)
		!= FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}

	cvSetData(gr_img, Fg_getImagePtr(fg, act_nr, PORT_A), CV_AUTOSTEP);
	cvSetImageROI(gr_img, cvGetImageROI(gr[cur_roi]));
	cvCopyImage(gr_img, gr[cur_roi]);

	r->ts = (double) ts;
	r->img_nr = act_nr;
	r->roi_nr = cur_roi;

	updateROILoc(cur_roi, gr_img, r, (float) frame_time);
	if(setupNextFrame(cur_roi, gr[cur_roi]) != CV_OK) return CV_BADROI_ERR;
	return act_nr;
}
