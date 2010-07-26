#ifndef _XTAL_BALL_H_
#define _XTAL_BALL_H_

extern void setup_kalman(CvKalman **kal, int n = 1, 
						 float **x0 = NULL, float **P0 = NULL);
extern void prediction(CvKalman *kal, float dt_k, float *z_k);

CV_INLINE void kal_assert(IplImage *img, CvKalman *kal, int roi_w, int roi_h)
{
	int x = cvRound(kal->state_post->data.fl[0]);
	int y = cvRound(kal->state_post->data.fl[1]);

	// modified from cvSetImageROI function, 
	// which would raise the same, but less informative, error.
	OPENCV_ASSERT(roi_w >= 0 && roi_h >= 0 && 
		x < img->width && y < img->height &&
		x + roi_w >= (int)(roi_w > 0) && y + roi_h >= (int)(roi_h > 0), 
	   __FUNCTION__, 
	   "Kalman update will result in blob being out of image bounds");
}

#endif /* _XTAL_BALL_H_ */