#ifndef _T_DAH_H_
#define _T_DAH_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "acquire_dots.h"
#include "config_parser.h"
#include "dot_coords.h"
#include "dot_tracker.h"
#include "draw_dots.h"
#include "xtal_ball.h"

#if defined(WIN32) && defined(_WIN32)
#define snprintf _snprintf
#endif

#define THRESHOLD 101
#define THRESHOLD_TYPE CV_THRESH_BINARY_INV

#define WHITE 255
#define GRAY 128
#define BLACK 0
#define BACKGROUND BLACK
#define FOREGROUND WHITE
#define BORDER GRAY

#define TIME_CODE(str, code) do { \
			double time_us = cvGetTickCount()/cvGetTickFrequency(); \
			code; \
			time_us = cvGetTickCount()/cvGetTickFrequency() - time_us; \
			printf("%s: %g\n", str, time_us); \
		} while(0)

struct ROILoc {
	ROILoc() { img = 0; ts = 0.; roi_nr = img_nr = loc.x = loc.y = 0; };

	int obj_found;
	int roi_nr;
	int img_nr;
	CvPoint loc;
	double ts;
	IplImage *img;
};

CV_INLINE CvPoint roi2ctrd(CvRect r)
{
	return cvPoint(r.x + r.width/2, r.y + r.height/2);
}

CV_INLINE CvPoint roi2ctrd(IplImage *img)
{
	return roi2ctrd(cvGetImageROI(img));
}

CV_INLINE CvRect ctrd2roi(int x, int y, int w, int h)
{
	return cvRect(x - w/2, y - h/2, w, h);
}

CV_INLINE void ctrd2roi(IplImage *img, int x, int y, int w, int h)
{
	cvSetImageROI(img, ctrd2roi(x, y, w, h));
}

#endif /* _T_DAH_H_ */