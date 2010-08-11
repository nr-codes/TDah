#ifndef TDAH_ME3_H_
#define TDAH_ME3_H_

#include "me3_ag.h"
#include "TDah.h"

class TDahMe3Ag : public TDah
{
public:
	virtual ~TDahMe3Ag() { close(); };

	bool open(char *conf_file);
	bool open(int trigger_type, double exposure_time_us, 
		double frame_time_us, int buffers);
	void close();
	bool saveMe3Buffer(char *file_name);

	// TDah inherited fcns
	int getROILoc(int img_nr, ROILoc *r);
	void showROILoc(void) 
	{
		show_position(gr, n_roi, kal, wr, NULL, retrieveFrame(0));
	};

	// CvCapture inherited fcns
	virtual bool grabFrame();
	virtual IplImage *retrieveFrame(int);

protected:
	int buffers;
	int trigger;
	double frame_time;
	double exposure;
	Fg_Struct *fg;
	IplImage *bgr_img;

	bool find_ctrd(int roi_nr);
	bool find_tmplt(int roi_nr, IplImage *img);
};

#endif /* TDAH_ME3_H_ */