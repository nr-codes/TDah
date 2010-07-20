#ifndef T_DAH_H_
#define T_DAH_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <fcdynamic.h>

#define SEQ {ROI_0, ROI_1}
#define SEQ_LEN 2
#define CAMLINK FG_CL_DUALTAP_8_BIT

// CAMERA REGION OF INTEREST
#define ROI_BOX 40
#define ROI_WIDTH ROI_BOX
#define ROI_HEIGHT ROI_BOX

// APPLICATION-SPECIFIC
#define THRESHOLD 101
#define X_DIM 4
#define Z_DIM 2
#define U_DIM 1

#define TIME_CODE(str, code) do { \
			double time_us = cvGetTickCount()/cvGetTickFrequency(); \
			code; \
			time_us = cvGetTickCount()/cvGetTickFrequency() - time_us; \
			printf("%s: %g\n", str, time_us); \
		} while(0)

extern int auto_acquire(CvCapture *capture, IplImage **gr, 
						int t, double r, int n, 
						IplImage **tplt = NULL, double m = 0, 
						CvKalman **kal = NULL, int show_results = 0);
extern int manual_acquire(CvCapture *capture, IplImage **gr, int *t, int n = 1, 
				   IplImage **tplt = NULL, CvKalman **kal = NULL);

extern void setup_kalman(CvKalman **kal, int n = 1, 
						 float **x0 = NULL, float **P0 = NULL);
extern void prediction(CvKalman *kal, float dt_k, float *z_k);

extern double track_ctrd(IplImage *gray, int thresh, CvSeqWriter *wr);
extern double track_tmplt(IplImage *gray, IplImage *templ);
extern double track_tmplt_pyr(IplImage *gr, IplImage *tmplt, 
					   IplImage *temp, int lvl = 1, int offset = 0);


extern void show_position(IplImage **roiImg, int n, 
				   CvKalman **kal = NULL, CvSeqWriter *bndry = NULL, 
				   char *draw_what = NULL, IplImage *fullImg = NULL);

extern void show_tplts(IplImage **tplt, int rows, int cols, int n);
extern void show_seqs(CvSeqWriter *bndry, int rows, int cols, int n);
extern void draw_ctrd(IplImage *dst, IplImage *src, CvSeq *bndry, int i);
extern void draw_kal(IplImage *dst, CvKalman *kal);


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


#endif /* T_DAH_H_ */