#ifndef TDAH_ME3_H_
#define TDAH_ME3_H_

#include "me3_fc.h"
#include "TDah.h"

class TDahMe3Fc : public TDah
{
public:
	TDahMe3Fc(char *conf_file);
	TDahMe3Fc(int trigger_type, double exposure_time_us, 
		double frame_time_us, int img_w, int img_h, int buffers);

	bool open(char *conf_file);
	bool open(int trigger_type, double exposure_time_us, 
		double frame_time_us, int img_w, int img_h, int buffers);
	void close();
	virtual ~TDahMe3Fc();
	int getROILoc(int img_nr, ROILoc *r);

	// CvCapture Inherited Fcns
	virtual bool grabFrame();
	virtual IplImage *retrieveFrame(int);

protected:
	double frame_time;
	Fg_Struct *fg;
	IplImage *bgr_img;
	IplImage *gr_img;
};

#endif /* TDAH_ME3_H_ */