#ifndef DOT_TRACKER_H_
#define DOT_TRACKER_H_

extern double track_ctrd(IplImage *gray, int roi_w, int roi_h, 
						 int thresh, CvSeqWriter *wr);
extern double track_tmplt(IplImage *gray, IplImage *templ, IplImage *temp = NULL);
extern double track_tmplt_pyr(IplImage *gr, IplImage *tmplt, 
					   IplImage *temp, int lvl = 1, int offset = 0);
#endif /* DOT_TRACKER_H_ */
