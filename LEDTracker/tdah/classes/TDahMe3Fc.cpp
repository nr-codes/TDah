#include "TDahMe3Fc.h"

#define FULLFRAME 0
#define REACQUIRE 1

#define BGR_CHANS 3
#define GR_CHAN 1
#define PYR_LVL 2
#define PYR_OFFSET 5
#define ROI_MASK 0xf
#define TMPLT_W FC_MAX_WIDTH
#define TMPLT_H FC_MAX_HEIGHT

//////////////// CAMERA CODE ////////////////

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
	int act_nr, dma_trf_len;
	CvMat imgData, roiRect;

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

	// copy image data to bgr image
	if(dma_trf_len != img_w*img_h) {
		// use current ROI setting
		cvZero(bgr_img);
		imgData = cvMat(roi_h, roi_w, CV_8UC1, 
			Fg_getImagePtr(fg, act_nr, PORT_A));
		cvGetSubRect(bgr_img, &roiRect, 
			ctrd2roi(bgr_img->width/2, bgr_img->height/2, roi_w, roi_h));
		cvMerge(&imgData, &imgData, &imgData, NULL, &roiRect);
	}
	else {
		// received a full resolution image, ignore ROI setting
		imgData = cvMat(img_h, img_w, CV_8UC1, 
			Fg_getImagePtr(fg, act_nr, PORT_A));
		cvMerge(&imgData, &imgData, &imgData, NULL, bgr_img);
	}

	return bgr_img;
}

//////////////// TRACKING CODE ////////////////
int TDahMe3Fc::toggle_dio0()
{
	static int toggle = 0;

	toggle = DOUT0_HIGH - toggle;
	return set_dio0(toggle);
}

int TDahMe3Fc::set_dio0(int value)
{
	if(Fg_setParameter(fg, FG_DIGIO_OUTPUT, &value, PORT_A) != FG_OK) {
		me3_err("set_dio0");
		return Fg_getLastErrorNumber(fg);;
	}
	return CV_OK;
}

int TDahMe3Fc::make_img_tag(int roi_nr, int mode)
{
	return (mode << 4) | roi_nr;
}

int TDahMe3Fc::get_roi_nr(int img_tag)
{
	return (img_tag >> 16) & ROI_MASK;
}

int TDahMe3Fc::get_tracking_mode(int img_tag)
{
	return (img_tag >> 20);
}

int TDahMe3Fc::setupNextROIFrame(int roi_nr, int mode)
{
	CvRect win = cvGetImageROI(gr[roi_nr]);
	int tag = make_img_tag(roi_nr, mode);

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
		me3_err("setupNextROIFrame");
		//return Fg_getLastErrorNumber(fg);
	}

	// TODO: understand when DO_INIT is necessary

	if(write_roi(fg, roi_nr, tag, !DO_INIT) != FG_OK) {
		me3_err("setupNextROIFrame");
		return Fg_getLastErrorNumber(fg);
	}

	cvSetImageROI(gr[roi_nr], win);
	return CV_OK;
}

int TDahMe3Fc::setupFullFrameROI(int roi_nr, int mode)
{
	// TODO fix centroid being incorrect at edges of image frame
	CvRect win = cvRect(0, 0, img_w, img_h);
	int tag = make_img_tag(roi_nr, mode);

	if(roi_window(roi_nr, win.x, win.width, win.y, win.height) != FG_OK) {
		me3_err("setupFullFrameROI");
		return Fg_getLastErrorNumber(fg);
	}

	// TODO: understand when DO_INIT is necessary
	if(write_roi(fg, roi_nr, tag, !DO_INIT) != FG_OK) {
		me3_err("setupFullFrameROI");
		return Fg_getLastErrorNumber(fg);
	}

	cvSetImageROI(gr[roi_nr], win);
	return CV_OK;
}

int TDahMe3Fc::initHelper(int mode)
{
	int img_nr;
	int seq[] = {ROI_0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

	if(mode == FULLFRAME) {
		// set full frame ROIs by going straight to camera
		for(int i = 0; i < MAX_ROI; i++) {
			if(roi_window(i, 0, img_w, 0, img_h) != FG_OK) {
				me3_err("initHelper");
				return Fg_getLastErrorNumber(fg);
			}

			if(write_roi(fg, i, i, !DO_INIT) != FG_OK) {
				me3_err("initHelper");
				return Fg_getLastErrorNumber(fg);
			}
		}

		img_nr = Fg_getStatus(fg, NUMBER_OF_ACT_IMAGE, 0, PORT_A);
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr + 2, PORT_A, TIMEOUT);
		if(img_nr < FG_OK) {
			me3_err("initHelper");
			return Fg_getLastErrorNumber(fg);
		}
	}
	else if(mode == REACQUIRE) {
		//DELETE
		for(int i = 0; i < n_roi; i++) {
			CvPoint p = roi2ctrd(gr[i]);
			printf("%d (%d, %d) box: %d %d %d %d ih\n", i, p.x, p.y, gr[i]->roi->xOffset, gr[i]->roi->yOffset, gr[i]->roi->width, gr[i]->roi->height);
		}

		// start acquire with gr ROIs
		if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
			me3_err("initHelper");
			return Fg_getLastErrorNumber(fg);
		}

		for(int i = 0; i < n_roi; i++) {
			if(setupNextROIFrame(i, CTRD) != CV_OK) return !CV_OK;
		}

		if(me3_fc_acquire(fg, seq, n_roi) != FG_OK) {
			me3_err("initHelper");
			return Fg_getLastErrorNumber(fg);
		}
	}
	else {
		return !CV_OK;
	}

	return CV_OK;
}

int TDahMe3Fc::initROIs(int n, char *c, bool k, bool t, char *in, char *ex)
{
	if(n < MIN_ROI || n > MAX_ROI) {
		return CV_BADROI_ERR;
	}
	else if(n == MIN_ROI) {
		printf("warning: only using 1 ROI, ");
		printf("you must ask for every other image.  See documentation\n");
	}

	if(initHelper(FULLFRAME) != CV_OK) return !CV_OK;

	// setup ROIs
	if(TDah::initROIs(n, c, k, t, in, ex) != CV_OK) {
		return !CV_OK;
	}

	roi_w = GET_CLOSEST_ROI_WIDTH(roi_w);
	if(roi_w < MIN_ROI_WIDTH) {
		return CV_BADROI_ERR;
	}

	return initHelper(REACQUIRE);
}

// TODO make sure to note that camera must be acquiring before calling initROIs
int TDahMe3Fc::initROIs(int n, int rw, int rh, char *s, bool uk, bool ut,
						char *intr, char *extr)
{
	rw = GET_CLOSEST_ROI_WIDTH(rw);
	if(n > MAX_ROI || rw < MIN_ROI_WIDTH) {
		return CV_BADROI_ERR;
	}

	// make full frame ROIs for manual_acquire dots function
	if(initHelper(FULLFRAME) != CV_OK) return !CV_OK;

	// setup ROIs
	if(TDah::initROIs(n, rw, rh, s, uk, ut, intr, extr) != CV_OK) {
		return !CV_OK;
	}

	return initHelper(REACQUIRE);
}

void TDahMe3Fc::showROILoc(void)
{
	cvZero(bgr_img);
	for(int i = 0; i < n_roi; i++) {
		cvSetImageROI(bgr_img, cvGetImageROI(gr[i]));
		cvCvtColor(gr[i], bgr_img, CV_GRAY2BGR);
	}
	cvResetImageROI(bgr_img);
	show_position(gr, n_roi, kal, wr, NULL, bgr_img);
}

bool TDahMe3Fc::find_ctrd(int j)
{
	double score = track_ctrd(gr[j], roi_w, roi_h, threshold, &wr[j]);
	return (score > 0 && score <= max_radius);
}

bool TDahMe3Fc::find_tmplt(int j)
{
	double score;

	// try downsampled template matching
	score = track_tmplt_pyr(gr[j], tplt[j], pyr_temp, PYR_LVL, PYR_OFFSET);
	if(score < min_match) {
		// try full image template matching
		score = track_tmplt(gr[j], tplt[j], tplt_temp);
		if(score < min_match) {
			return false;
		}
	}

	return true;
}

int TDahMe3Fc::grabROIImage(int img_nr, ROILoc *r)
{
	int img_tag, cur_roi, mode;
	CvMat imgData;
	uint64 ts;

	img_tag = img_nr;
	if(Fg_getParameter(fg, FG_IMAGE_TAG, &img_tag, PORT_A) != FG_OK) {
		me3_err("grabROIImage");
		return Fg_getLastErrorNumber(fg);
	}
	cur_roi = get_roi_nr(img_tag);
	mode = get_tracking_mode(img_tag);

	ts = img_nr;
	if(Fg_getParameter(fg, FG_TIMESTAMP_LONG, &ts, PORT_A) != FG_OK) {
		me3_err("grabROIImage");
		return Fg_getLastErrorNumber(fg);
	}

	int dma_trf_len = img_nr;
	if(Fg_getParameter(fg, FG_TRANSFER_LEN, &dma_trf_len, PORT_A) != FG_OK) {
		me3_err("retrieveFrame");
		return NULL;
	}

	if(dma_trf_len != roi_w*roi_h) {
		printf("dma len: %d, expected: %d (%d x %d)\n", dma_trf_len, roi_w*roi_h, roi_w, roi_h);
		assert(false);
	}

	imgData = cvMat(gr[cur_roi]->roi->height, 
					gr[cur_roi]->roi->width, 
					CV_8UC1, 
					Fg_getImagePtr(fg, img_nr, PORT_A));
	
	cvZero(gr[cur_roi]);
	cvCopy(&imgData, gr[cur_roi]);

	r->ts = (double) ts;
	r->img_nr = img_nr;
	r->roi_nr = cur_roi;

	if(r->img) {
		cvSetImageROI(r->img, cvGetImageROI(gr[cur_roi]));
		cvCopyImage(gr[cur_roi], r->img);
	}

	return mode;
}

int TDahMe3Fc::updateROILoc(int mode, ROILoc *r)
{
	CvPoint c;
	CvPoint2D32f w;
	float z[Z_DIM];
	int j;

	j = r->roi_nr;

	if(mode == CTRD) {
		r->obj_found = find_ctrd(j);

		if(!r->obj_found) {
			cvNamedWindow("blob", 0);
			show_seqs(wr, roi_w, roi_h, 2, 1, 2);
			cvShowImage("blob", gr[j]);
			cvWaitKey(0);
		}
		//r->loc = roi2ctrd(gr[j]); // DELETE
		//cvSetImageROI(r->img, cvGetImageROI(gr[j])); // DELETE
		//draw_ctrd(gr[j], gr[j], wr[j].seq, j);
		//cvCopyImage(gr[j], r->img);
		//cvNamedWindow("sequences", 0);
		//show_seqs(wr, roi_w, roi_h, 1, 1, n_roi);
		if(!r->obj_found && tplt) {
			mode = TMPLT;
			// TODO: think carefully what happens b/c of IMG+2 delay
			setupFullFrameROI(j, mode);
			return mode;
		}
	}
	else if(mode == TMPLT) {
		r->obj_found = find_tmplt(j);
		if(r->obj_found) {
			mode = CTRD;
		}
		else {
			setupFullFrameROI(j, mode);
			return mode;
		}
	}
	else {
		OPENCV_ASSERT(false, __FUNCTION__, "tracking mode not supported");
	}

	if(kal && cam_mat) {
		c = roi2ctrd(gr[j]);
		w = pixel2world(c, cam_mat, cam_dist, world_r, world_t);
		z[0] = w.x;
		z[1] = w.y;

		// use the prediction for step k+1
		estimate_and_predict(kal[j], (float) frame_time, 
			r->obj_found ? z : NULL);
		kal_assert(gr[j], kal[j]->state_pre, roi_w, roi_h);

		w.x = kal[j]->state_pre->data.fl[0];
		w.y = kal[j]->state_pre->data.fl[1];
		c = world2pixel(w, cam_mat, cam_dist, world_r, world_t);
		ctrd2roi(gr[j], c.x, c.y, roi_w, roi_h);
	}

	if(setupNextROIFrame(j, mode) != CV_OK) {
		return CV_BADROI_ERR;
	}

	c = roi2ctrd(gr[j]);
	if(cam_mat) {
		r->loc = pixel2world(c, cam_mat, cam_dist, world_r, world_t);
	}
	else {
		r->loc.x = (float) c.x;
		r->loc.y = (float) c.y;
	}

	return mode;
}

int TDahMe3Fc::getROILoc(int img_nr, ROILoc *r)
{
	int act_nr, mode;

	act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	if(act_nr < FG_OK) {
		me3_err("getROILoc");
		return Fg_getLastErrorNumber(fg);
	}

	mode = grabROIImage(act_nr, r);
	mode = updateROILoc(mode, r);

	return mode < 0 ? mode : act_nr;
}
