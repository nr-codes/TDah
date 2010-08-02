#ifndef TDAH_OPENCV_H_
#define TDAH_OPENCV_H_

#include "TDah.h"

class TDahOpenCV : public TDah
{
public:
	TDahOpenCV(int index) { is_cap_open = open(index); };
	virtual ~TDahOpenCV() { close(); };
	bool isCapOpen(void) { return is_cap_open; };
	int getROILoc(ROILoc *r);
	virtual void close() { cvReleaseCapture(&cap); };
	virtual bool open(int index) 
	{ 
		return !!(cap = cvCaptureFromCAM(index)); 
	};

	void showROILoc(void) 
	{
		show_position(gr, n_roi, kal, wr, NULL, retrieveFrame(0));
	};

	// CvCapture Inherited Fcns
	virtual bool grabFrame() { return !!cvGrabFrame(cap); };
	virtual int getCaptureDomain() { return cap->getCaptureDomain(); };
	virtual IplImage *retrieveFrame(int) 
	{ 
		IplImage *img = cvRetrieveFrame(cap);
		cvResetImageROI(img);
		return img;
	};

	virtual double getProperty(int prop) 
	{
		return cvGetCaptureProperty(cap, prop); 
	};

    virtual bool setProperty(int prop, double val) 
	{
		return !!cvSetCaptureProperty(cap, prop, val); 
	};

protected:
	bool is_cap_open;
	CvCapture *cap;

	bool find_ctrd(int roi_nr);
	bool find_tmplt(int roi_nr, IplImage *img);
};

#endif /* TDAH_OPENCV_H_ */