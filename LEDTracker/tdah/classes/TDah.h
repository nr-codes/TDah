#ifndef TDAH_H_
#define TDAH_H_

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <_highgui.h>
#include "t_dah.h"

class TDah : public CvCapture
{
public:
	TDah();
	virtual ~TDah() { deinitROIs(); };
	virtual void showROILoc(void) { show_position(gr, n_roi, kal, wr); };
	virtual void deinitROIs(void);
	virtual int initROIs(int num_roi, int roi_w, int roi_h, char *save_conf_as = NULL, 
		bool use_kal = true, bool use_tmplt = true);
	virtual int initROIs(int num_roi, char *conf_file, 
		bool use_kal = true, bool use_tmplt = true);

protected:
	// general info
	int n_roi;
	int roi_w;
	int roi_h;
	int img_w;
	int img_h;

	// centroid tracking
	int threshold;
	double max_radius;
	IplImage **gr;
	CvSeqWriter *wr;
	CvMemStorage **mem;

	// template matching
	double min_match;
	IplImage **tplt;
	IplImage *pyr_temp;
	IplImage *tplt_temp;

	// predictive tracking
	CvKalman **kal;
	double *prev_ts;

	// helper functions
	int alloc_mbrs(bool init_kal, bool init_tmplt);
};

#endif /* TDAH_H_ */