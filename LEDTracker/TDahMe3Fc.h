#ifndef TDAH_ME3_H_
#define TDAH_ME3_H_

#include "me3_fc.h"
#include "TDah.h"

class TDahMe3Fc : public TDah
{
public:
	virtual ~TDahMe3Fc() { close(); };
	TDahMe3Fc(char *conf_file);
	TDahMe3Fc(int trigger_type, double exposure_time_us, 
		double frame_time_us, int buffers)
	{
		open(trigger_type, exposure_time_us, frame_time_us, buffers);	
	};

	bool open(char *conf_file);
	bool open(int trigger_type, double exposure_time_us, 
		double frame_time_us, int buffers);
	void close();

	// TDah inherited fcns
	int initROIs(int num_roi, int roi_w, int roi_h, char *save_conf_as = NULL, 
		bool use_kal = true, bool use_tmplt = true);
	int getROILoc(int img_nr, ROILoc *r);
	void showROILoc(void);

	// CvCapture inherited fcns
	virtual bool grabFrame();
	virtual IplImage *retrieveFrame(int);

protected:
	double frame_time;
	double exposure;
	Fg_Struct *fg;
	IplImage *bgr_img;
	IplImage *gr_img;

	int setupFullFrame();
};

#endif /* TDAH_ME3_H_ */