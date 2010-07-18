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
#define BAD_ROI INT_MIN

// APPLICATION-SPECIFIC
#define THRESHOLD 101
#define X_DIM 4
#define Z_DIM 2
#define U_DIM 1

extern int auto_acquire(CvCapture *capture, CvRect *r, int t, int n = NULL, 
				 IplImage **tplt = NULL, CvKalman **kal = NULL);
extern int manual_acquire(CvCapture *capture, CvRect *r, int *t, int n = 1, 
				   IplImage **tplt = NULL, CvKalman **kal = NULL);

extern void setup_kalman(CvKalman **kal, int n = 1, 
						 float **x0 = NULL, float **P0 = NULL);
extern void prediction(CvKalman *kal, float dt_k, float *z_k);

extern int position(IplImage *gray, CvRect *rect, 
					int thresh, CvSeqWriter *wr);
extern int emergency(CvRect *rect, IplImage *gray, IplImage *templ);
extern void draw_position(IplImage *gray, IplImage *rgb, 
						  CvSeq *pts, CvKalman *kal);


extern void show_position(IplImage **roiImg, int n, 
				   CvKalman **kal = NULL, CvSeqWriter *bndry = NULL, 
				   char *draw_what = NULL, IplImage *fullImg = NULL);

extern void show_tplts(IplImage **tplt, int rows, int cols, int n);
extern void show_seqs(CvSeqWriter *bndry, int rows, int cols, int n);

#endif /* T_DAH_H_ */