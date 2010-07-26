#ifndef _T_DAH_H_
#define _T_DAH_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "config_parser.h"
#include "dot_tracker.h"
#include "draw_dots.h"
#include "me3.h"

#define SEQ {ROI_0, ROI_1}
#define SEQ_LEN 2
#define CAMLINK FG_CL_DUALTAP_8_BIT

// APPLICATION-SPECIFIC
#define THRESHOLD 101
#define X_DIM 4
#define Z_DIM 2
#define U_DIM 1

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
	ROILoc() { img = 0; ts = 0.; roi_nr = img_nr = loc.x = loc.y = 0;};

	int obj_found;
	int roi_nr;
	int img_nr;
	CvPoint loc;
	double ts;
	IplImage *img;
};

extern int auto_acquire(CvCapture *capture, IplImage **gr, 
						int roi_w, int roi_h, int t, CvSeqWriter *wr, 
						double r, int n = 1, IplImage **tplt = NULL, 
						double m = 0, CvKalman **kal = NULL, 
						int show_results = 0);
extern int manual_acquire(CvCapture *capture, IplImage **gr, 
						  int roi_w, int roi_h, int *t, CvSeqWriter *wr, 
						  int n = 1, IplImage **tplt = NULL, 
						  CvKalman **kal = NULL);

extern void setup_kalman(CvKalman **kal, int n = 1, 
						 float **x0 = NULL, float **P0 = NULL);
extern void prediction(CvKalman *kal, float dt_k, float *z_k);

extern IplImage *queryFrame();
extern int grabFrameOpenCV(int index);
extern int retrieveFrameOpenCV(int index, ROILoc *roi);
extern int queryFrameOpenCV(int index, ROILoc *roi);
extern int grabFrameMe3(int img_nr);
extern int retrieveFrameMe3(int img_nr, ROILoc *roi);
extern int queryFrameMe3(int img_nr, ROILoc *roi);
extern int init_system(int num_roi, char *yaml_file);
extern int deinit_system(void);
extern int init_system_opencv(int index, int roi_w, int roi_h, int num_roi, 
					   char *save_config_as, double exposure);

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

CV_INLINE void kal_assert(IplImage *img, CvKalman *kal, int roi_w, int roi_h)
{
	// copied from cvSetImageROI function which allows zero ROI width or height
	int x = cvRound(kal->state_post->data.fl[0]);
	int y = cvRound(kal->state_post->data.fl[1]);
	OPENCV_ASSERT(roi_w >= 0 && roi_h >= 0 && 
		x < img->width && y < img->height &&
		x + roi_w >= (int)(roi_w > 0) && y + roi_h >= (int)(roi_h > 0), 
	   __FUNCTION__, 
	"Kalman update will result in blob being out of image bounds");
}

#endif /* _T_DAH_H_ */