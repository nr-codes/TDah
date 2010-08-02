#ifndef _ACQUIRE_DOTS_H
#define _ACQUIRE_DOTS_H

extern int auto_acquire(CvCapture *capture, IplImage **gr, 
						int roi_w, int roi_h, int t, CvSeqWriter *wr, 
						double r, int n = 1, IplImage **tplt = NULL, 
						double m = 0, CvKalman **kal = NULL, 
						int show_results = 0);

extern int manual_acquire(CvCapture *capture, IplImage **gr, 
						  int roi_w, int roi_h, int *t, CvSeqWriter *wr, 
						  int n = 1, IplImage **tplt = NULL, 
						  CvKalman **kal = NULL);

#endif /* _ACQUIRE_DOTS_H */