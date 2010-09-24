#include "TDah.h"

#define SHOW_IMGS 0
#define CHANS 1
#define R_PAD 3
#define M_PAD .03

TDah::TDah()
{
	// general info
	n_roi = roi_w = roi_h = img_w = img_h = 0;

	// centroid tracking
	threshold = 0;
	threshold_type = CV_THRESH_BINARY;
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

	// world frame conversion
	cam_mat = cam_dist = world_r = world_t = NULL;
}

int TDah::initROIs(int n, int rw, int rh, char *s, 
				   bool uk, bool ut, char *intr, char *extr)
{
	double val;
	IplImage *img;
	CvFileStorage *fs;
	CvPoint2D32f c;

	img = queryFrame();
	if(!img) return !CV_OK;

	n_roi = n;
	roi_w = rw;
	roi_h = rh;
	img_w = img->width;
	img_h = img->height;
	if(alloc_mbrs(uk, ut, intr, extr) != CV_OK) return !CV_OK;

	// set ROIs for each object
	setup_kalman(kal, n_roi);
	if(manual_acquire(this, gr, roi_w, roi_h, &threshold, threshold_type,
		wr, n_roi, tplt, kal) != CV_OK) {
		return !CV_OK;
	}

	max_radius = 0.;
	min_match = 1.;
	img = queryFrame();
	if(!img) return !CV_OK;
	for(int i = 0; i < n_roi; i++) {
		// convert to gray and calc ctrd
		cvSetImageROI(img, cvGetImageROI(gr[i]));
		cvCvtColor(img, gr[i], CV_BGR2GRAY);
		cvResetImageROI(img);

		val = track_ctrd(gr[i], roi_w, roi_h, threshold, 
			threshold_type, &wr[i], &c);
		max_radius = std::max(val, max_radius);

		if(tplt) {
			// copy entire image and tmplt match
			cvResetImageROI(gr[i]);
			cvCvtColor(img, gr[i], CV_BGR2GRAY);

			val = track_tmplt(gr[i], tplt[i], tplt_temp);
			min_match = std::min(val, min_match);
		}

		if(kal) {
			// assume it's safe to not update on this image
		}
	}

	// add a fudge factor for safety
	max_radius = std::max(roi_w, roi_h) / 2 + R_PAD;
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
		if(ut) write_templates(fs, tplt, n_roi);
		if(uk) write_kalman(fs, kal, n_roi);
		cvReleaseFileStorage(&fs);
	}

	return CV_OK;
}

int TDah::initROIs(int num_roi, char *conf_file, 
				   bool uk, bool ut, char *intr, char *extr)
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

	if(alloc_mbrs(uk, ut, intr, extr) != CV_OK) {
		cvReleaseFileStorage(&fs);
		return !CV_OK;
	}
	
	read_obj_loc(fs, gr, n_roi);
	if(uk)	{
		setup_kalman(kal, n_roi);
		read_kalman(fs, kal, n_roi);
	}

	if(ut) read_templates(fs, tplt, n_roi);

	// get dots
	rc = auto_acquire(this, gr, roi_w, roi_h, 
		threshold, threshold_type, wr, max_radius, n_roi, 
		tplt, min_match, kal, SHOW_IMGS);
	if(rc != CV_OK) {
		rc = manual_acquire(this, gr, roi_w, roi_h, 
			&threshold, threshold_type, wr, n_roi, tplt, kal);
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

	cvReleaseMat(&cam_mat);
	cvReleaseMat(&cam_dist);
	cvReleaseMat(&world_r);
	cvReleaseMat(&world_t);

	n_roi = 0;
}

int TDah::alloc_mbrs(bool init_kal, bool init_tmplt, char *intr, char *extr)
{
	int n;
	CvSize sz;
	CvFileStorage *fs;
	
	sz = cvSize(img_w, img_h);
	n = n_roi;
	deinitROIs();
	n_roi = n;

__CV_BEGIN__;
	
	gr = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	if(!gr) __CV_EXIT__;

	wr = (CvSeqWriter *) cvAlloc(sizeof(CvSeqWriter) * n_roi);
	if(!wr) __CV_EXIT__;

	mem = (CvMemStorage **) cvAlloc(sizeof(CvMemStorage *) * n_roi);
	if(!mem) __CV_EXIT__;

	kal = NULL;
	if(init_kal) {
		prev_ts = (double *) cvAlloc(sizeof(double) * n_roi);
		if(!prev_ts) __CV_EXIT__;

		kal = (CvKalman **) cvAlloc(sizeof(CvKalman *) * n_roi);
		if(!kal) __CV_EXIT__;
	}
	
	tplt = NULL;
	if(init_tmplt) {
		pyr_temp = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
		if(!pyr_temp) __CV_EXIT__;
		
		tplt_temp = cvCreateImage(sz, IPL_DEPTH_32F, CHANS);
		if(!tplt_temp) __CV_EXIT__;

		tplt = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
		if(!tplt) __CV_EXIT__;
	}

	for(int i = 0; i < n_roi; i++) {
		gr[i] = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
		if(!gr[i]) __CV_EXIT__;
		cvSetImageROI(gr[i], cvRect(0, 0, sz.width, sz.height));

		mem[i] = cvCreateMemStorage(0);
		if(!mem[i]) __CV_EXIT__;
		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
			sizeof(CvSeq), 
			sizeof(CvPoint), 
			mem[i], 
			&wr[i]);

		if(init_kal) {
			prev_ts[i] = cvGetTickCount()/cvGetTickFrequency();
			kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
			if(!kal[i]) __CV_EXIT__;
		}

		if(init_tmplt) {
			tplt[i] = cvCreateImage(sz, IPL_DEPTH_8U, CHANS);
			if(!tplt[i]) __CV_EXIT__;
			cvSetImageROI(tplt[i], cvRect(0, 0, sz.width, sz.height));
		}
	}

	if(intr && extr) {
		fs = cvOpenFileStorage(intr, NULL, CV_STORAGE_READ);
		if(!fs) __CV_EXIT__;
		read_intrinsic_params(fs, &cam_mat, &cam_dist);
		cvReleaseFileStorage(&fs);

		fs = cvOpenFileStorage(extr, NULL, CV_STORAGE_READ);
		if(!fs) __CV_EXIT__;
		read_extrinsic_params(fs, &world_r, &world_t);
		cvReleaseFileStorage(&fs);
	}

	return CV_OK;
__CV_END__;
	deinitROIs();
	return !CV_OK;
}
