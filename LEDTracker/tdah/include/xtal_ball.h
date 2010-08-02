#ifndef _XTAL_BALL_H_
#define _XTAL_BALL_H_

#define X_DIM 4
#define Z_DIM 2
#define U_DIM 1

extern void setup_kalman(CvKalman **kal, int n = 1, 
						 float **x0 = NULL, float **P0 = NULL);
extern void estimate_and_predict(CvKalman *kal, float dt_k, float *z_k);

CV_INLINE void kal_assert(IplImage *img, CvMat *state, int roi_w, int roi_h)
{
	int x;
	int y;
	
	x = cvRound(state->data.fl[0]);
	y = cvRound(state->data.fl[1]);

	// modified from cvSetImageROI function, 
	// which would raise the same, but less informative, error.
	OPENCV_ASSERT(roi_w >= 0 && roi_h >= 0 && 
		x < img->width && y < img->height &&
		x + roi_w >= (int)(roi_w > 0) && y + roi_h >= (int)(roi_h > 0), 
	   __FUNCTION__, 
	   "Kalman update will result in blob being out of image bounds");
}

#endif /* _XTAL_BALL_H_ */