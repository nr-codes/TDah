#include "t_dah.h"

void TDahOpenCV::showROILoc(void)
{
	show_position(gr, n_roi, kal, wr, NULL, retrieveFrame(0));
}

int TDahOpenCV::getROILoc(ROILoc *r)
{
	static int j = 0;
	static int cnt = 0;
	IplImage *img;

	OPENCV_ASSERT(gr[j]->roi, __FUNCTION__, "ROI not set");

	img = cvRetrieveFrame(cap);
	cvSetImageROI(img, cvGetImageROI(gr[j]));
	cvCvtColor(img, gr[j], CV_BGR2GRAY);
	r->roi_nr = j;
	r->img_nr = cnt++;
	r->ts = cvGetTickCount()/cvGetTickFrequency();
	updateROILoc(j, img, r);

	j++;
	j %= n_roi;

	return cnt;
}
