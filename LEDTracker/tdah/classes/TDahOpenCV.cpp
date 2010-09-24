#include "TDahOpenCV.h"

#define PYR_LVL 2
#define PYR_OFFSET 5

bool TDahOpenCV::find_ctrd(int j)
{
	double score;
	CvPoint2D32f c;
	
	score = track_ctrd(gr[j], roi_w, roi_h, threshold, threshold_type, &wr[j], &c);
	return (score > 0 && score <= max_radius);
}

bool TDahOpenCV::find_tmplt(int j, IplImage *img)
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

int TDahOpenCV::getROILoc(ROILoc *r)
{
	static int j = 0;
	static int cnt = 0;
	static double avg_frame_time = 0;
	float z[Z_DIM];
	IplImage *img;
	CvPoint img_pt;
	CvPoint2D32f w_pt;

	OPENCV_ASSERT(gr[j]->roi, __FUNCTION__, "ROI not set");

	img = cvRetrieveFrame(cap);
	cvSetImageROI(img, cvGetImageROI(gr[j]));
	cvCvtColor(img, gr[j], CV_BGR2GRAY);
	r->roi_nr = j;
	r->img_nr = cnt++;
	r->ts = cvGetTickCount()/cvGetTickFrequency();

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
		avg_frame_time += ((r->ts - prev_ts[j])*1e-6 - avg_frame_time) / cnt;
		estimate_and_predict(kal[j], (float) avg_frame_time, 
			r->obj_found ? z : NULL);
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

	return cnt;
}
