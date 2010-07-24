#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "TDahOpenCV.h"

#define PYR_LVL 2
#define PYR_OFFSET 5


void TDahOpenCV::showROILoc(void)
{
	IplImage *img = cvQueryFrame(cap);
	cvResetImageROI(img);
	show_position(gr, n_roi, kal, wr, NULL, img);
}

int TDahOpenCV::getROILoc(int index, ROILoc *r)
{

	static int j = 0;
	static int cnt = 0;
	double score;
	float z[Z_DIM];
	CvPoint c;
	IplImage *img;

	OPENCV_ASSERT(gr[j]->roi, __FUNCTION__, "ROI not set");

	// TODO: interesting side effect is queryFrame() gets img ROI.  should i keep this?
	img = cvRetrieveFrame(cap);
	cvSetImageROI(img, cvGetImageROI(gr[j]));
	cvCvtColor(img, gr[j], CV_BGR2GRAY);

	r->roi_nr = j;
	r->img_nr = cnt++;
	r->ts = cvGetTickCount()/cvGetTickFrequency();
	r->obj_found = true; // assume found

	score = track_ctrd(gr[j], roi_w, roi_h, threshold, &wr[j]);
	printf("ctrd\n");
	if(score <= 0 || score > max_radius) {
		// ctrd couldn't find object
		if(tplt) {
			printf("template0\n");
			// try downsampled template matching
			cvResetImageROI(img);
			cvSetImageROI(gr[j], cvGetImageROI(img));
			cvCvtColor(img, gr[j], CV_BGR2GRAY);
			// TODO: why is pyr_temp NULL?  probably b/c of deinit in alloc_mbrs
			score = track_tmplt_pyr(gr[j], tplt[j], pyr_temp, PYR_LVL, PYR_OFFSET);
			if(score < min_match) {
				printf("template1\n");
				// try full image template matching
				score = track_tmplt(gr[j], tplt[j], tplt_temp);
				if(score < min_match) {
					r->obj_found = false;
				}
			}
		}
		else {
			r->obj_found = false;
		}
	}

	if(kal) {
		printf("kal\n");
		c = roi2ctrd(gr[j]);
		z[0] = (float) c.x;
		z[1] = (float) c.y;
		prediction(kal[j], (float) ((r->ts - prev_ts[j])*1e-6), z);

		if(!r->obj_found) {
			printf("kal lost\n");
			// only trust process dynamics
			cvCopy(kal[j]->error_cov_pre, kal[j]->error_cov_post);
			cvCopy(kal[j]->state_pre, kal[j]->state_post);
		}

		prev_ts[j] = r->ts;
		kal_assert(gr[j], kal[j], roi_w, roi_h);
		ctrd2roi(gr[j], cvRound(kal[j]->state_post->data.fl[0]), 
				cvRound(kal[j]->state_post->data.fl[1]), roi_w, roi_h);
	}
	r->loc = roi2ctrd(gr[j]);

	if(r->img) cvCopyImage(gr[j], r->img);

	j++;
	j %= n_roi;

	return cvRound(score);
}
