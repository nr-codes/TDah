#include "TDahMe3Ag.h"

#define BGR_CHANS 3
#define GR_CHAN 1
#define PYR_LVL 2
#define PYR_OFFSET 5

//////////////// CAMERA CODE ////////////////

bool TDahMe3Ag::open(char *conf_file)
{
	return false;
}

bool TDahMe3Ag::open(int tt, double e, double f, int b)
{
	// initialize certain class members
	img_w = AG_MAX_WIDTH;
	img_h = AG_MAX_HEIGHT;
	roi_w = img_w;
	roi_h = img_h;
	frame_time = f;
	exposure = e;
	trigger = tt;
	buffers = b;

	// bgr_img is associated with the entire full resolution frame
	bgr_img = cvCreateImage(cvSize(img_w, img_h), IPL_DEPTH_8U, BGR_CHANS);
	if(!bgr_img) return false;

	// initialize me3 and fastconfig
	if(me3_ag_init(&fg, tt, img_w*img_h*b, b) != FG_OK) {
		me3_err("open");
		return false;
	}

	if(roi_window(fg, 0, img_w, 0, img_h) != FG_OK) {
		me3_err("open");
		return false;
	}

	if(roi_exposure(fg, exposure, frame_time) != FG_OK) {
		me3_err("open");
		return false;
	}

	// start grabbing images in order of ROIs specified in seq
	if(me3_ag_acquire(fg) != FG_OK) {
		me3_err("open");
		return false;
	}

	return true;
}

void TDahMe3Ag::close()
{
	cvReleaseImage(&bgr_img);

	if(me3_ag_deinit(fg) != FG_OK) {
		me3_err("close");
	}
}

bool TDahMe3Ag::grabFrame()
{
	if(trigger == ASYNC_SOFTWARE_TRIGGER && 
		Fg_sendSoftwareTrigger(fg, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return false;
	}

	return true;
}

IplImage *TDahMe3Ag::retrieveFrame(int img_nr)
{
	int act_nr;
	CvMat imgData;

	if(img_nr <= 0) {
		// get newest image
		act_nr = Fg_getImage(fg, SEL_NEW_IMAGE, 0, PORT_A, TIMEOUT);
	}
	else {
		// get image data from buffer
		act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	}

	if(act_nr < FG_OK) {
		me3_err("retrieveFrame");
		return NULL;
	}

	imgData = cvMat(img_h, img_w, CV_8UC1, Fg_getImagePtr(fg, act_nr, PORT_A));
	cvMerge(&imgData, &imgData, &imgData, NULL, bgr_img);

	return bgr_img;
}

bool TDahMe3Ag::saveMe3Buffer(char *file)
{
	int i, n;
	bool rc;
	char *pixels;
	CvVideoWriter *writer;
	CvMat imgData;

	n = Fg_getStatus(fg, NUMBER_OF_ACT_IMAGE, 0, PORT_A);
	if(n < FG_OK) {
		me3_err("saveMe3Buffer");
		return false;
	}

	if(Fg_stopAcquire(fg, PORT_A) != FG_OK) {
		me3_err("saveMe3Buffer");
		return false;
	}

	writer = cvCreateVideoWriter(file, CV_FOURCC('D', 'I', 'B', ' ') , 
		1e6 / frame_time, cvSize(img_w, img_h));
	if(!writer) return false;

	rc = true; // assume no error
	for(i = std::max(1, n - buffers + 1); i <= n; i++) {
		// get image from buffer and write to file
		pixels = (char *) Fg_getImagePtr(fg, i, PORT_A);
		if(pixels == NULL) {
			rc = false;
			break;
		}

		imgData = cvMat(img_h, img_w, CV_8UC1, pixels);
		cvMerge(&imgData, &imgData, &imgData, NULL, bgr_img);
		
		if(!cvWriteFrame(writer, bgr_img)) {
			rc = false;
			break;
		}
	}
	cvReleaseVideoWriter(&writer);

	if(Fg_Acquire(fg, PORT_A, GRAB_INFINITE) != FG_OK) {
		me3_err("saveMe3Buffer");
		return false;
	}

	return rc;
}

//////////////// TRACKING CODE ////////////////

bool TDahMe3Ag::find_ctrd(int j)
{
	double score;
	CvPoint2D32f c;
	
	score = track_ctrd(gr[j], roi_w, roi_h, threshold, threshold_type,
		&wr[j], &c);
	return (score > 0 && score <= max_radius);
}

bool TDahMe3Ag::find_tmplt(int j, IplImage *img)
{
	double score;

	if(tplt) {
		// try downsampled template matching
		cvResetImageROI(img);
		cvSetImageROI(gr[j], cvGetImageROI(img));
		cvCvtColor(img, gr[j], CV_BGR2GRAY);

		score = track_tmplt_pyr(gr[j], tplt[j], pyr_temp, PYR_LVL, PYR_OFFSET);
		if(score < min_match) {
			// try full image template matching
			score = track_tmplt(gr[j], tplt[j], tplt_temp);
			if(score < min_match) {
				return false;
			}
		}
	}

	return tplt != NULL;
}

int TDahMe3Ag::getROILoc(int img_nr, ROILoc *r)
{
	static int j = 0;
	int act_nr;
	float z[Z_DIM], dt;
	IplImage *img;
	CvPoint img_pt;
	CvPoint2D32f w_pt;
	uint64 ts;

	OPENCV_ASSERT(gr[j]->roi, __FUNCTION__, "ROI not set");
	img = retrieveFrame(img_nr);

	cvSetImageROI(img, cvGetImageROI(gr[j]));
	cvCvtColor(img, gr[j], CV_BGR2GRAY);
	cvResetImageROI(img);

	// get image info
	act_nr = Fg_getStatus(fg, NUMBER_OF_LAST_IMAGE, 0, PORT_A);
	if(act_nr < FG_OK) {
		me3_err("getROILoc");
		return Fg_getLastErrorNumber(fg);
	}

	ts = act_nr;
	if(Fg_getParameter(fg, FG_TIMESTAMP_LONG, &ts, PORT_A) != FG_OK) {
		me3_err("getROILoc");
		return Fg_getLastErrorNumber(fg);
	}

	r->roi_nr = j;
	r->img_nr = act_nr;
	r->ts = (double) ts;

	r->obj_found = find_ctrd(j);
	if(!r->obj_found) {
		// centroid couldn't find object, try template matching
		r->obj_found = find_tmplt(j, img);
	}

	if(kal && cam_mat) {
		img_pt = roi2ctrd(gr[j]);
		w_pt = pixel2world(img_pt, cam_mat, cam_dist, world_r, world_t);
		z[0] = w_pt.x;
		z[1] = w_pt.y;

		// use the prediction for step k+1
		dt = (float) (r->ts - prev_ts[j]);
		estimate_and_predict(kal[j], dt, r->obj_found ? z : NULL);
		kal_assert(gr[j], kal[j]->state_pre, roi_w, roi_h);
		
		w_pt.x = kal[j]->state_pre->data.fl[0];
		w_pt.y = kal[j]->state_pre->data.fl[1];
		img_pt = world2pixel(w_pt, cam_mat, cam_dist, world_r, world_t);
		ctrd2roi(gr[j], img_pt.x, img_pt.y, roi_w, roi_h);

		prev_ts[j] = r->ts;
	}

	img_pt = roi2ctrd(gr[j]);
	if(cam_mat) {
		r->loc = pixel2world(img_pt, cam_mat, cam_dist, world_r, world_t);		
	}
	else {
		r->loc.x = (float) img_pt.x;
		r->loc.y = (float) img_pt.y;
	}

	if(r->img) {
		cvSetImageROI(r->img, cvGetImageROI(gr[j]));
		cvCopyImage(gr[j], r->img);
	}

	j++;
	j %= n_roi;

	return act_nr;
}
