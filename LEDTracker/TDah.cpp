#include <cv.h>
#include "t_dah.h"
#include "TDah.h"

#define SHOW_IMGS 0
#define CHANS 1
#define R_PAD 3
#define M_PAD .03
#define PYR_LVL 2
#define PYR_OFFSET 5

TDah::TDah()
{
	// general info
	n_roi = roi_w = roi_h = img_w = img_h = 0;

	// centroid tracking
	threshold = 0;
	max_radius = 0;
	gr = NULL;
	wr = NULL;
	mem = NULL;

	// template matching
	min_match = 1; // max value
	tplt = NULL;
	pyr_temp = tplt_temp = NULL;

	// predictive tracking
	kal = NULL;
	prev_ts = NULL;
}

int TDah::initROIs(int n, int rw, int rh, char *s, bool use_kal, bool use_tmplt)
{
	double val;
	IplImage *img;
	CvFileStorage *fs;

	img = queryFrame();
	if(!img) return !CV_OK;

	n_roi = n;
	roi_w = rw;
	roi_h = rh;
	img_w = img->width;
	img_h = img->height;
	if(alloc_mbrs(use_kal, use_tmplt) != CV_OK) return !CV_OK;

	// set ROIs for each object
	setup_kalman(kal, n_roi);
	manual_acquire(this, gr, roi_w, roi_h, &threshold, wr, n_roi, tplt, kal);

	max_radius = 0.;
	min_match = 1.;
	img = queryFrame();
	if(!img) return !CV_OK;
	for(int i = 0; i < n_roi; i++) {
		// convert to gray and calc ctrd
		cvSetImageROI(img, cvGetImageROI(gr[i]));
		cvCvtColor(img, gr[i], CV_BGR2GRAY);

		val = track_ctrd(gr[i], roi_w, roi_h, threshold, &wr[i]);
		max_radius = std::max(val, max_radius);

		if(tplt) {
			// copy entire image and tmplt match
			cvResetImageROI(gr[i]);
			cvResetImageROI(img);
			cvCvtColor(img, gr[i], CV_BGR2GRAY);

			val = track_tmplt(gr[i], tplt[i], tplt_temp);
			min_match = std::min(val, min_match);
		}

		if(kal) {
			// assume it's safe to not update on this image
		}
	}

	// add a fudge factor for safety
	max_radius += R_PAD;
	min_match -= M_PAD;

	if(s) {
		fs = cvOpenFileStorage(s, NULL, CV_STORAGE_WRITE);
		if(fs == NULL) {
			deinitROIs();
			return !CV_OK;
		}

		write_track_params(fs, threshold, min_match, 
			max_radius, roi_w, roi_h, img_w, img_h);
		write_obj_loc(fs, gr, n_roi);
		if(use_tmplt) write_templates(fs, tplt, n_roi);
		if(use_kal) write_kalman(fs, kal, n_roi);
		cvReleaseFileStorage(&fs);
	}

	return CV_OK;
}

int TDah::initROIs(int num_roi, char *conf_file, bool use_kal, bool use_tmplt)
{
	int rc;
	CvFileStorage *fs;

	fs = cvOpenFileStorage(conf_file, NULL, CV_STORAGE_READ);
	if(fs == NULL) return !CV_OK;

	// initialize parameters
	n_roi = num_roi;
	read_track_params(fs, 
		&threshold, 
		&min_match, 
		&max_radius, 
		&roi_w, 
		&roi_h, 
		&img_w, 
		&img_h);

	if(alloc_mbrs(use_kal, use_tmplt) != CV_OK) {
		cvReleaseFileStorage(&fs);
		return !CV_OK;
	}
	
	read_obj_loc(fs, gr, n_roi);
	if(use_kal)	{
		setup_kalman(kal, n_roi);
		read_kalman(fs, kal, n_roi);
	}

	if(use_tmplt) read_templates(fs, tplt, n_roi);

	// get dots
	rc = auto_acquire(this, gr, roi_w, roi_h, 
		threshold, wr, max_radius, n_roi, tplt, min_match, kal, SHOW_IMGS);
	if(rc != CV_OK) {
		rc = manual_acquire(this, gr, roi_w, roi_h, 
			&threshold, wr, n_roi, tplt, kal);
	}

	cvReleaseFileStorage(&fs);

	return CV_OK;
}

void TDah::deinitROIs(void)
{
	for(int i = 0; i < n_roi; i++) {
		if(gr) cvReleaseImage(&gr[i]);
		if(mem) cvReleaseMemStorage(&mem[i]);
		if(tplt) cvReleaseImage(&tplt[i]);
		if(kal)	cvReleaseKalman(&kal[i]);
	}

	cvFree(&tplt);
	cvFree(&gr);
	cvFree(&kal);
	cvFree(&wr);
	cvFree(&prev_ts);

	cvReleaseImage(&pyr_temp);
	cvReleaseImage(&tplt_temp);

	n_roi = 0;
}

int TDah::alloc_mbrs(bool init_kal, bool init_tmplt)
{
	int n;
	CvSize sz;
	
	sz = cvSize(img_w, img_h);
	n = n_roi;
	deinitROIs();
	n_roi = n;

__BEGIN__;
	
	gr = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	if(!gr) EXIT;

	wr = (CvSeqWriter *) cvAlloc(sizeof(CvSeqWriter) * n_roi);
	if(!wr) EXIT;

	mem = (CvMemStorage **) cvAlloc(sizeof(CvMemStorage *) * n_roi);
	if(!mem) EXIT;

	kal = NULL;
	if(init_kal) {
		prev_ts = (double *) cvAlloc(sizeof(double) * n_roi);
		if(!prev_ts) EXIT;

		kal = (CvKalman **) cvAlloc(sizeof(CvKalman *) * n_roi);
		if(!kal) EXIT;
	}
	
	tplt = NULL;
	if(init_tmplt) {
		pyr_temp = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
		if(!pyr_temp) EXIT;
		
		tplt_temp = cvCreateImage(sz, IPL_DEPTH_32F, CHANS);
		if(!tplt_temp) EXIT;

		tplt = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
		if(!tplt) EXIT;
	}

	for(int i = 0; i < n_roi; i++) {
		gr[i] = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
		if(!gr[i]) EXIT;
		cvSetImageROI(gr[i], cvRect(0, 0, sz.width, sz.height));

		mem[i] = cvCreateMemStorage(0);
		if(!mem[i]) EXIT;
		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
			sizeof(CvSeq), 
			sizeof(CvPoint), 
			mem[i], 
			&wr[i]);

		if(init_kal) {
			prev_ts[i] = cvGetTickCount()/cvGetTickFrequency();
			kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
			if(!kal[i]) EXIT;
		}

		if(init_tmplt) {
			tplt[i] = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
			if(!tplt[i]) EXIT;
			cvSetImageROI(tplt[i], cvRect(0, 0, sz.width, sz.height));
		}
	}

	return CV_OK;
__END__;
	deinitROIs();
	return !CV_OK;
}

inline bool TDah::find_ctrd(int j)
{
	double score;
	
	score = track_ctrd(gr[j], roi_w, roi_h, threshold, &wr[j]);
	return (score > 0 && score <= max_radius);
}

inline bool TDah::find_tmplt(int j, IplImage *img)
{
	double score;

	if(tplt) {
		// try downsampled template matching
		cvResetImageROI(img);
		cvSetImageROI(gr[j], cvGetImageROI(img));

		if(img->nChannels == 3) {
			cvCvtColor(img, gr[j], CV_BGR2GRAY);
		}
		else {
			cvCopyImage(img, gr[j]);
		}

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

void TDah::updateROILoc(int j, IplImage *img, ROILoc *r)
{
	float z[Z_DIM], dt;
	CvPoint c;

	OPENCV_ASSERT(gr[j]->roi, __FUNCTION__, "ROI not set");

	r->obj_found = find_ctrd(j);
	if(!r->obj_found) {
		// centroid couldn't find object, try template matching
		r->obj_found = find_tmplt(j, img);
	}

	if(kal) {
		c = roi2ctrd(gr[j]);
		z[0] = (float) c.x;
		z[1] = (float) c.y;
		printf("Mea: (%0.3g, %0.3g)\n", z[0], z[1]);
		dt = (float) ((r->ts - prev_ts[j])*1e-6);
		estimate_and_predict(kal[j], dt, r->obj_found ? z : NULL);
		
		// use the prediction for step k+1
		kal_assert(gr[j], kal[j]->state_pre, roi_w, roi_h);
		ctrd2roi(gr[j], cvRound(kal[j]->state_pre->data.fl[0]), 
				cvRound(kal[j]->state_pre->data.fl[1]), roi_w, roi_h);

		prev_ts[j] = r->ts;
	}

	r->loc = roi2ctrd(gr[j]);
	if(r->img) {
		cvSetImageROI(r->img, cvGetImageROI(gr[j]));
		cvCopyImage(gr[j], r->img);
	}
}